// GX5s base firmware — the application: power states, ignition, supply protection,
// telemetry. It is the only place that knows about more than one component: they
// post events, on_event() decides. ISRs and timers only post events.
// Architecture and power-state table: docs/adr/0001-base-firmware.md
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/uart_wakeup.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gx5.h"
#include "gx5_can.h"
#include "gx5_ext.h"
#include "gx5_io.h"
#include "gx5_modem.h"
#include "gx5_store.h"
#include "motion.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "parse.h"

#define SEC(s) ((uint64_t)(s) * 1000000ULL)
#ifdef CONFIG_GX5_LIGHT_SLEEP
#define LIGHT_SLEEP true
#else
#define LIGHT_SLEEP false
#endif

#ifdef CONFIG_GX5_EXT_PWR_ACTIVE_HIGH
#define EXT_PWR_ON 1
#else
#define EXT_PWR_ON 0
#endif
#ifdef CONFIG_GX5_MOTION_WAKE
#define MOTION_WAKE true
#else
#define MOTION_WAKE false
#endif
#ifdef CONFIG_GX5_HIBERNATE
#define HIBERNATE true
#else
#define HIBERNATE false
#endif

static const char *TAG = "gx5";

static RTC_DATA_ATTR bool s_protect; // last sleep was PROTECT (survives deep sleep)
static RTC_DATA_ATTR bool s_can_armed;    // last sleep had CAN traffic as its ext0 wake, not the ACC wire
static RTC_DATA_ATTR uint8_t s_can_wakes; // CAN-traffic wakes in a row that did not lead to ignition
static bool s_ign, s_connected, s_linger;
static RTC_DATA_ATTR bool s_vin_checked;  // false after a power loss: the device may be in another vehicle now
static bool s_was_on;       // ignition has been on since this wake
static gx5_ign_cfg_t s_igc; // the vehicle profile's ignition rule ...
static uint16_t s_pre_s;    // ... and how long to wait for ignition after a CAN or motion wake
static int s_low_count;
static bool s_chg; // charger enabled
static char s_id[18]; // "AA:BB:CC:DD:EE:FF"
static char s_pop[9]; // provisioning PIN, from the device id
static const char *s_wake = "power-on";
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cal; // eFuse-calibrated counts -> mV; NULL = use raw counts (about 1 mV each)
static esp_timer_handle_t s_t_debounce, s_t_sleep, s_t_telemetry, s_t_vcc;

// ── Measurements ────────────────────────────────────────────────────────────

static int adc_avg(adc_channel_t ch)
{
    int sum = 0, raw = 0;
    for (int i = 0; i < 8; i++) {
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
    }
    return sum / 8;
}

// Averaged pin voltage in mV, through the chip's factory ADC calibration.
static int adc_mv(adc_channel_t ch)
{
    int mv = adc_avg(ch);
    if (s_cal) adc_cali_raw_to_voltage(s_cal, mv, &mv);
    return mv;
}

static int vcc_mv(void) { return gx5_vcc_node_to_mv(adc_mv(ADC_CHANNEL_0), CONFIG_GX5_VCC_VBE_MV); }
// LiPo voltage, measured only with the charger off: charging lifts the reading by
// ~0.4 V (soak 2026-11-17), so while it charges this is the last resting value.
static int bat_mv(void)
{
    static int rest;
    if (!s_chg) rest = adc_mv(ADC_CHANNEL_7) * 22 / 12; // BAT+ - R8 1K - node - R5 1.2K - GND
    return rest;
}
static bool ignition(void) { return gpio_get_level(GX5_PIN_ACC_INT) == 0; }

// Supply is present but too low to keep loading it.
static bool supply_low(int mv)
{
    return mv >= CONFIG_GX5_VCC_ABSENT_MV && mv < CONFIG_GX5_VCC_CUTOFF_MV + (s_protect ? CONFIG_GX5_VCC_HYST_MV : 0);
}

static void store_telemetry(void);

// ── Sleep ───────────────────────────────────────────────────────────────────

static const int s_sd_in[] = {GX5_PIN_SD_CS, GX5_PIN_SD_CLK, GX5_PIN_SD_MOSI};

static void hold(int pin, int level)
{
    gpio_set_level(pin, level);
    gpio_hold_en(pin);
}

// SLEEP: modem stays registered, RI/ignition/heartbeat wake us.
// HIBERNATE (GX5_HIBERNATE): the same sleep with the modem off, so no RI wake.
// PROTECT: modem off, only ignition (if currently off) and a recheck timer wake us.
// ── Factory reset: five fast resets ─────────────────────────────────────────
// Five resets in a row (reset button or power cycle), each within about 5 s of the
// previous boot, erase the stored configuration: broker, CAN setup and the offline
// queue. The device then starts unprovisioned. Only real power-on resets count, so
// deep-sleep wakes, crashes, watchdogs and brownouts can never add up to a wipe.
#define FACTORY_RESETS 5
#define CAN_WAKES_MAX 3
static bool s_rst_open; // counting: closed (count back to 0) at the first supply tick

static void reset_count_set(uint8_t n)
{
    nvs_handle_t h;
    if (nvs_open("gx5", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "rst", n);
    nvs_commit(h);
    nvs_close(h);
}

static void reset_window_close(void)
{
    if (s_rst_open) reset_count_set(0);
    s_rst_open = false;
}

// After nvs_flash_init(). True if this boot was the fifth fast reset and everything was erased.
static bool factory_reset_check(void)
{
    nvs_handle_t h;
    uint8_t n = 0;
    if (esp_reset_reason() != ESP_RST_POWERON) return false;
    if (nvs_open("gx5", NVS_READWRITE, &h) == ESP_OK) {
        nvs_get_u8(h, "rst", &n);
        nvs_close(h);
    }
    if (++n < FACTORY_RESETS) {
        ESP_LOGI(TAG, "fast reset %d of %d", n, FACTORY_RESETS);
        reset_count_set(n);
        s_rst_open = true;
        return false;
    }
    ESP_LOGW(TAG, "factory reset: configuration erased, device is unprovisioned");
    nvs_flash_erase();
    nvs_flash_init();
    gx5_store_clear();
    return true;
}

static void deep_sleep(bool protect)
{
    reset_window_close();
    // Asleep the board lives on its LiPo and takes nothing from the harness: EXT-PWR
    // disables the 36V->5V buck (bench: 0.15 mA, against 11.4 mA with it left on).
    // Only a missing or flat LiPo keeps the buck on (6.6 mA): without either supply
    // the MCU browns out, the buck's enable floats back on and the board reboots in a loop.
    bool off = protect || HIBERNATE; // modem powered down for this sleep
    if (!s_connected && !protect) store_telemetry(); // an offline heartbeat still leaves a record
    gx5_mqtt_publish("status/connected", protect ? "protect" : off ? "hibernate" : "sleep", true);
    gx5_modem_stop(off);
    gx5_exp_set(GX5_EXP_GSM_SLP, true); // DTR high: the modem may now sleep; low again at boot (expander init)
    gx5_can_stop();
    s_protect = protect;

    // The charger stops only now, after the modem's last transmission: with it off first
    // and the LiPo at 3.4 V the modem stopped answering mid-publish (bench 2026-11-18).
    gx5_exp_set(GX5_EXP_CHRG_EN, s_chg = false); // asleep we take nothing: the charger alone drew ~200 mA at 12 V (bench)
    vTaskDelay(pdMS_TO_TICKS(200)); // ponytail: settling time after the charger stops is not measured; raise if bat reads high here
    int bat = bat_mv();
    bool feed = bat < CONFIG_GX5_BAT_MIN_MV;
    ESP_LOGW(TAG, "%s: vcc=%d mV bat=%d mV ign=%d feed=%d", protect ? "PROTECT" : off ? "HIBERNATE" : "SLEEP", vcc_mv(), bat, s_ign, feed);

    gx5_exp_set(GX5_EXP_LED_SVR | GX5_EXP_LED_IG | GX5_EXP_LED_PWR, false);

    // Freeze every rail/control pin: unheld outputs float in deep sleep.
    gx5_imu_arm();                                // drop a latched motion interrupt
    hold(GX5_PIN_3V3_PSM, 0); // regulator power-save mode: IMU/expander/RTC stay powered, ~5 mA less
    hold(GX5_PIN_SD_3V3, 0);
    hold(GX5_PIN_GSM_RST, 1);
    if (CONFIG_GX5_CAN_EN_GPIO >= 0) hold(CONFIG_GX5_CAN_EN_GPIO, 1); // CAN transceiver stays in standby
    for (int i = 0; i < 3; i++) { // SD NAND is unpowered: its inputs low, so no pin feeds it
        gpio_set_direction(s_sd_in[i], GPIO_MODE_OUTPUT);
        hold(s_sd_in[i], 0);
    }
    hold(CONFIG_GX5_MODEM_TX_GPIO, 1); // idle-high UART line, or the modem never sleeps
    if (CONFIG_GX5_EXT_PWR_GPIO >= 0) hold(CONFIG_GX5_EXT_PWR_GPIO, feed ? EXT_PWR_ON : !EXT_PWR_ON);
    gpio_deep_sleep_hold_en();

    // Drop the light-sleep GPIO wake sources, then arm deep-sleep ones. ext0/ext1
    // are level triggered: only ever arm the level a pin is NOT at right now, or
    // we would wake immediately and loop.
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    gpio_wakeup_disable(GX5_PIN_ACC_INT);
    gpio_wakeup_disable(GX5_PIN_GSM_INT);
    // ext0 is one pin: the ACC wire, or CAN traffic (CAN-RX low = a dominant bit; the
    // transceiver passes the bus through in standby) when the profile takes ignition
    // from somewhere else. This chip's ext1 only does "any high", so CAN-RX cannot go there.
    s_can_armed = false;
    if (s_igc.src == GX5_IGN_ACC) {
        esp_sleep_enable_ext0_wakeup(GX5_PIN_ACC_INT, ignition() ? 1 : 0); // next ignition change
    } else if (!protect) {
        // A bus that keeps talking, or one that woke us CAN_WAKES_MAX times for nothing,
        // is left alone for this sleep: motion and the heartbeat still wake us.
        bool busy = false;
        gpio_set_direction(GX5_PIN_CAN_RX, GPIO_MODE_INPUT);
        for (int64_t end = esp_timer_get_time() + 200000; !busy && esp_timer_get_time() < end;) busy = gpio_get_level(GX5_PIN_CAN_RX) == 0;
        GX5_DBG(TAG, "CAN wake: bus %s, %d wakes for nothing", busy ? "busy" : "quiet", s_can_wakes);
        if (busy || s_can_wakes >= CAN_WAKES_MAX) ESP_LOGW(TAG, "CAN wake not armed: %s", busy ? "bus busy" : "woke for nothing too often");
        else esp_sleep_enable_ext0_wakeup(GX5_PIN_CAN_RX, 0), s_can_armed = true;
    }
    if (!protect) {
        int pins[] = {off ? -1 : GX5_PIN_GSM_INT, MOTION_WAKE ? GX5_PIN_APS_INT : -1}; // no RI from a modem that is off
        uint64_t mask = 0;
        for (int i = 0; i < 2; i++) {
            if (pins[i] < 0) continue;
            if (gpio_get_level(pins[i]) == 0) mask |= 1ULL << pins[i];
            else ESP_LOGW(TAG, "GPIO%d stuck high, not armed as wake source", pins[i]);
        }
        if (mask) esp_sleep_enable_ext1_wakeup_io(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
        GX5_DBG(TAG, "wake sources: ext0 %s, ext1 mask %llx, timer %d s", s_igc.src == GX5_IGN_ACC ? "ACC wire" : s_can_armed ? "CAN traffic" : "none",
                (unsigned long long)mask, CONFIG_GX5_HEARTBEAT_S);
    }
    esp_sleep_enable_timer_wakeup(SEC(protect ? CONFIG_GX5_PROTECT_RECHECK_S : CONFIG_GX5_HEARTBEAT_S));
    esp_deep_sleep_start();
}

// ── Events ──────────────────────────────────────────────────────────────────

static void post_cb(void *id) { esp_event_post(GX5_EVENT, (int32_t)(intptr_t)id, NULL, 0, 0); }

static void acc_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    gpio_intr_disable(GX5_PIN_ACC_INT); // level interrupt: re-armed after debounce
    esp_event_isr_post(GX5_EVENT, GX5_EV_ACC_EDGE, NULL, 0, &woken);
    if (woken) portYIELD_FROM_ISR();
}

// Interrupt (and light-sleep wake) on the level opposite to the current state.
static void arm_acc(void)
{
    gpio_wakeup_enable(GX5_PIN_ACC_INT, s_ign ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);
    gpio_intr_enable(GX5_PIN_ACC_INT);
}

// (Re)read the ignition rule. The ACC wire is only watched when it is the source.
static void load_profile(void)
{
    gx5_can_profile(&s_igc, &s_pre_s);
    if (s_igc.src == GX5_IGN_ACC) {
        arm_acc();
    } else {
        gpio_intr_disable(GX5_PIN_ACC_INT);
        gpio_wakeup_disable(GX5_PIN_ACC_INT);
    }
}

// ts is 0 until the RTC or the network has set the clock.
static void telemetry_json(char *js, size_t size)
{
    time_t now = time(NULL);
    snprintf(js, size, "{\"ts\":%lld,\"ign\":%d,\"vcc_mv\":%d,\"bat_mv\":%d,\"ext\":%d,\"chg\":%d,\"can\":%lu,\"wake\":\"%s\",\"up_s\":%lld",
             now > 1700000000 ? (long long)now : 0LL, s_ign, vcc_mv(), bat_mv(), gpio_get_level(GX5_PIN_VCC_INT) == 0, gx5_charging(),
             (unsigned long)gx5_can_frames(), s_wake, esp_timer_get_time() / 1000000);
    size_t n = strlen(js);
    int mg[3], t, w[3];
    if (gx5_imu_read(mg, &t, w)) {
        n += snprintf(js + n, size - n, ",\"acc\":[%d,%d,%d],\"temp\":%d.%d", mg[0], mg[1], mg[2], t / 10, abs(t % 10));
        if (s_ign && n < size) // the gyro only runs with ignition on; deg/s
            n += snprintf(js + n, size - n, ",\"gyro\":[%.1f,%.1f,%.1f]", w[0] / 10.0, w[1] / 10.0, w[2] / 10.0);
    }
    if (n >= size - 1) n = strlen(js);
    gx5_gnss_fix(js + n, size - n - 1); // lat/lon/kmh when the modem has a fix
    strlcat(js, "}", size);
}

static void store_telemetry(void)
{
    char js[GX5_PUB_MAX];
    telemetry_json(js, sizeof js);
    gx5_store_push(js);
}

static bool send_stored(const char *line) { return gx5_mqtt_publish("telemetry", line, false); }
static bool send_can(const char *json)
{
    GX5_DBG(TAG, "can %s", json);
    return gx5_mqtt_publish("can", json, false);
}

// While offline, telemetry is queued on the SD NAND and replayed a few lines at a
// time once the production broker is reachable.
static void publish_telemetry(void)
{
    char js[GX5_PUB_MAX];
    telemetry_json(js, sizeof js);
    ESP_LOGI(TAG, "telemetry %s", js); // also what the bench tests read (fw/test/bench)
    if (!gx5_mqtt_publish("telemetry", js, false)) {
        gx5_store_push(js);
    } else if (!gx5_mqtt_provisioning()) {
        gx5_can_publish(send_can);
        gx5_store_drain(10, send_stored);
    }
}

// Ignition on: stay up, report periodically. Ignition off: the awake window is
// bounded by CONNECT_TIMEOUT (+ LINGER once connected) no matter what the link does.
// The CAN bus runs as the vehicle profile says for the state: before ignition,
// ignition on, after ignition off.
static void apply_ignition(void)
{
    gx5_exp_set(GX5_EXP_LED_IG, s_ign);
    if (s_ign) s_was_on = true;
    gx5_can_state(s_ign ? GX5_ST_ON : s_was_on ? GX5_ST_OFF : GX5_ST_PRE);
    if (s_ign && !s_vin_checked) gx5_can_vin_check(); // first ignition since a power loss
    gx5_imu_gyro(s_ign);
    esp_timer_stop(s_t_sleep);
    esp_timer_stop(s_t_telemetry);
    s_linger = false;
    if (s_ign) esp_timer_start_periodic(s_t_telemetry, SEC(CONFIG_GX5_TELEMETRY_S));
    else esp_timer_start_once(s_t_sleep, SEC(CONFIG_GX5_CONNECT_TIMEOUT_S));
}

// Ignition off and connected: one bounded window to exchange messages, then sleep.
static void maybe_linger(void)
{
    if (s_ign || !s_connected || s_linger) return;
    s_linger = true;
    esp_timer_stop(s_t_sleep);
    // Woken by the vehicle (CAN traffic, motion) and ignition comes from the profile's
    // rule: give the driver time to start before going back to sleep.
    bool wait = !s_was_on && s_igc.src != GX5_IGN_ACC && (!strcmp(s_wake, "can") || !strcmp(s_wake, "motion"));
    GX5_DBG(TAG, "ignition off and connected: sleep in %d s", wait ? s_pre_s : CONFIG_GX5_LINGER_S);
    esp_timer_start_once(s_t_sleep, SEC(wait ? s_pre_s : CONFIG_GX5_LINGER_S));
}

static void set_ignition(bool on)
{
    if (on == s_ign) return;
    s_ign = on;
    if (on) s_can_wakes = 0;
    ESP_LOGI(TAG, "ignition %s", on ? "on" : "off");
    apply_ignition();
    publish_telemetry();
    maybe_linger();
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case GX5_EV_ACC_EDGE:
        esp_timer_stop(s_t_debounce);
        esp_timer_start_once(s_t_debounce, 500 * 1000);
        break;
    case GX5_EV_ACC_SETTLED:
        if (s_igc.src != GX5_IGN_ACC) break;
        set_ignition(ignition());
        arm_acc();
        break;
    case GX5_EV_CAN_IGN:
        if (s_igc.src >= GX5_IGN_SIG) set_ignition(*(bool *)data);
        break;
    case GX5_EV_VIN: { // answered after the first ignition since a power loss: same vehicle?
        const char *vin = data;
        char was[18] = "", js[128];
        size_t len = sizeof was;
        nvs_handle_t h;
        time_t now = time(NULL);
        if (nvs_open("gx5", NVS_READWRITE, &h) == ESP_OK) {
            nvs_get_str(h, "vin", was, &len);
            if (strcmp(was, vin) && (nvs_set_str(h, "vin", vin) != ESP_OK || nvs_commit(h) != ESP_OK)) ESP_LOGE(TAG, "VIN not saved");
            nvs_close(h);
        }
        s_vin_checked = true;
        ESP_LOGI(TAG, "VIN %s%s%s", vin, strcmp(was, vin) ? ", was " : " (unchanged)", strcmp(was, vin) ? (was[0] ? was : "not known") : "");
        // ponytail: published once; if the broker is unreachable right now the event is lost (the VIN is still stored)
        snprintf(js, sizeof js, "{\"ts\":%lld,\"ev\":\"vin\",\"vin\":\"%s\",\"was\":\"%s\",\"changed\":%s}", now > 1700000000 ? (long long)now : 0LL, vin, was,
                 strcmp(was, vin) ? "true" : "false");
        gx5_mqtt_publish("event", js, false);
        break;
    }
    case GX5_EV_MQTT_CONNECTED:
        s_connected = true;
        gx5_exp_set(GX5_EXP_LED_SVR, true);
        gx5_mqtt_publish("status/connected", gx5_mqtt_provisioning() ? "provisioning" : "online", true);
        publish_telemetry();
        maybe_linger();
        break;
    case GX5_EV_MQTT_DISCONNECTED:
        s_connected = false;
        gx5_exp_set(GX5_EXP_LED_SVR, false);
        break;
    case GX5_EV_MQTT_DATA: { // provisioning, commands; anything else gets a telemetry reply
        gx5_msg_t *m = data;
        gx5_broker_t b;
        char ext[GX5_EXT_LINE];
        bool is_prov;
        char loc0[GX5_LOC_LEN];
        if (!strncmp(m->topic, "discovery/", 10)) { // unprovisioned: only a login matters (our own announcement echoes here too)
            if (gx5_parse_login(m->data, &b, loc0)) {
                ESP_LOGI(TAG, "login received, location %s", loc0);
                gx5_location_set(loc0, false);
                apply_ignition(); // ignition off: fresh connect window to try it
                gx5_broker_set(&b);
            }
            break;
        }
        const char *prov = gx5_provision(m->data, s_pop, &is_prov);
        if (is_prov || gx5_parse_broker(m->data, &b)) {
            // The broker only ever changes through {"provision":{"pop":..}} with this device's PIN.
            if (!prov) {
                ESP_LOGW(TAG, "provisioning rejected: wrong or missing PIN");
                gx5_mqtt_publish("event", "{\"reason\":\"provision-rejected\"}", false);
                break;
            }
            const char *cfg = gx5_json_val(prov, "config");
            char loc[GX5_LOC_LEN];
            bool new_broker = gx5_parse_broker(prov, &b);
            if (cfg && gx5_can_command(cfg)) load_profile(); // "config":{"vehicle":{...},"location":"0001"}
            if (cfg && gx5_json_val(cfg, "location")) {
                if (gx5_json_str(cfg, "location", loc, sizeof loc) && gx5_id_ok(loc)) {
                    ESP_LOGI(TAG, "new location: %s", loc);
                    gx5_location_set(loc, !new_broker);
                } else {
                    ESP_LOGW(TAG, "location ignored: 1-9 of A-Z a-z 0-9 _ -");
                }
            }
            if (new_broker) {
                ESP_LOGI(TAG, "new broker: %s:%d", b.host, b.port);
                apply_ignition(); // ignition off: fresh connect window to try it
                gx5_broker_set(&b);
            }
        } else if (gx5_json_str(m->data, "cmd", ext, sizeof ext) && !strcmp(ext, "ota")) {
            // {"cmd":"ota","version":"1.4.2"}; no version = "latest"
            char ver[16];
            if (!gx5_json_str(m->data, "version", ver, sizeof ver) || (ver[0] && !gx5_ver_ok(ver))) {
                ESP_LOGW(TAG, "ota: bad version");
            } else {
                ESP_LOGI(TAG, "ota requested: %s", ver[0] ? ver : "latest");
                gx5_ota_start(ver[0] ? ver : "latest");
            }
        } else if (gx5_json_val(m->data, "debug")) { // {"debug":true}
            nvs_handle_t h;
            gx5_debug = gx5_json_flag(m->data, "debug", false);
            if (nvs_open("gx5", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_u8(h, "dbg", gx5_debug);
                nvs_commit(h);
                nvs_close(h);
            }
            ESP_LOGI(TAG, "debug mode %s", gx5_debug ? "on" : "off");
        } else if (gx5_can_command(m->data)) {
            load_profile(); // applied and stored by can.c; the ignition source may have changed
        } else if (gx5_json_str(m->data, "ext", ext, sizeof ext) && ext[0]) {
            gx5_ext_send(ext); // {"ext":"text"}: one line out of the external UART
        } else {
            ESP_LOGI(TAG, "cmd %s: %s", m->topic, m->data);
            publish_telemetry();
        }
        break;
    }
    case GX5_EV_TIME: { // network time: system clock and RTC follow
        struct timeval tv = {.tv_sec = *(int64_t *)data};
        settimeofday(&tv, NULL);
        gx5_rtc_set(tv.tv_sec);
        break;
    }
    case GX5_EV_EXT_RX: { // a line from whatever is wired to the harness UART
        const char *line = data;
        ESP_LOGI(TAG, "ext rx: %s", line);
        if (!strcmp(line, "gx5 ping")) { // wiring check: answers without a broker
            char pong[40];
            snprintf(pong, sizeof pong, "gx5 pong %s", s_id);
            gx5_ext_send(pong);
        } else {
            gx5_mqtt_publish("ext", line, false);
        }
        break;
    }
    case GX5_EV_MOTION: { // harsh acceleration, braking, left or right turn
        const gx5_motion_t *ev = data;
        char js[96];
        ESP_LOGI(TAG, "motion %s %d mg", gx5_mot_name(ev->kind), ev->mg);
        if (ev->kind == GX5_MOT_CALIBRATED) break;
        time_t now = time(NULL);
        snprintf(js, sizeof js, "{\"ts\":%lld,\"ev\":\"%s\",\"mg\":%d}", now > 1700000000 ? (long long)now : 0LL, gx5_mot_name(ev->kind), ev->mg);
        gx5_mqtt_publish("event", js, false);
        break;
    }
    case GX5_EV_TELEMETRY:
        publish_telemetry();
        break;
    case GX5_EV_VCC_TICK: {
        reset_window_close(); // up for 5 s: this boot was not part of a fast-reset sequence
        int mv = vcc_mv();
        s_low_count = supply_low(mv) ? s_low_count + 1 : 0;
        if (!s_low_count) s_protect = false; // recovered past the hysteresis
        // Charge the LiPo only from a healthy supply. Charging pulls the reading down, so
        // it starts only above the hysteresis: without it the charger switched every tick.
        if (mv < CONFIG_GX5_VCC_CUTOFF_MV) s_chg = false;
        else if (mv >= CONFIG_GX5_VCC_CUTOFF_MV + CONFIG_GX5_VCC_HYST_MV) s_chg = true;
        gx5_exp_set(GX5_EXP_CHRG_EN, s_chg);
        GX5_DBG(TAG, "vcc=%d mV bat=%d mV chg=%d low=%d ign=%d can=%lu", mv, bat_mv(), s_chg, s_low_count, s_ign, (unsigned long)gx5_can_frames());
        if (s_igc.src == GX5_IGN_VCC) { // ignition = the alternator is charging
            // ponytail: sampled on this 5 s tick, so on/off lag by up to 5 s; give it its own timer if that matters
            static int below;
            below = mv < s_igc.off_mv ? below + 1 : 0;
            if (mv >= s_igc.on_mv) set_ignition(true);
            else if (below * 5 >= s_igc.hold_s) set_ignition(false);
        }
        if (s_low_count >= 3) deep_sleep(true);               // 3 in a row: rides through cranking dips
        break;
    }
    case GX5_EV_SLEEP_DUE:
        if (s_ign) break;
        if (gx5_ota_busy()) esp_timer_start_once(s_t_sleep, SEC(CONFIG_GX5_LINGER_S)); // let the update finish
        else deep_sleep(false);
        break;
    }
}

// Commands over the USB console: one JSON object per line, handled exactly like a
// message from the broker (so the broker itself still needs the provisioning PIN).
// The chip may be in light sleep when the first character arrives and that one is
// lost: send an empty line first. Typing then keeps it awake for 10 s at a time.
static void usb_task(void *arg)
{
    static gx5_msg_t m = {.topic = "usb"};
    esp_pm_lock_handle_t awake = NULL;
    size_t n = 0;
    bool held = false;
    uint8_t c;
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "usb", &awake);
    if (uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0) != ESP_OK) vTaskDelete(NULL);
    uart_wakeup_setup(UART_NUM_0, &(uart_wakeup_cfg_t){.wakeup_mode = UART_WK_MODE_ACTIVE_THRESH, .rx_edge_threshold = 3});
    esp_sleep_enable_uart_wakeup(UART_NUM_0);
    for (;;) {
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10000)) != 1) {
            if (held) esp_pm_lock_release(awake);
            held = false;
            n = 0;
            continue;
        }
        if (!held && awake) held = esp_pm_lock_acquire(awake) == ESP_OK;
        if (c != '\n' && c != '\r') {
            if (n < sizeof m.data - 1) m.data[n++] = (char)c;
            continue;
        }
        m.data[n] = '\0';
        if (n) GX5_DBG(TAG, "usb line, %u characters", (unsigned)n);
        if (n && m.data[0] == '{') esp_event_post(GX5_EVENT, GX5_EV_MQTT_DATA, &m, sizeof m, portMAX_DELAY);
        n = 0;
    }
}

static esp_timer_handle_t make_timer(int event)
{
    esp_timer_handle_t t;
    esp_timer_create_args_t a = {.callback = post_cb, .arg = (void *)(intptr_t)event, .name = "gx5"};
    ESP_ERROR_CHECK(esp_timer_create(&a, &t));
    return t;
}

void app_main(void)
{
    // Rails first. Pins were held through deep sleep; release before driving.
    int outs[] = {GX5_PIN_3V3_PSM, GX5_PIN_SD_3V3, GX5_PIN_GSM_RST, CONFIG_GX5_MODEM_TX_GPIO, CONFIG_GX5_EXT_PWR_GPIO};
    int levels[] = {1, 0, 1, 1, EXT_PWR_ON}; // awake: regulator in normal mode, fed from the harness; SD on per use
    gpio_deep_sleep_hold_dis();
    for (int i = 0; i < 3; i++) gpio_hold_dis(s_sd_in[i]);
    for (int i = 0; i < 5; i++) {
        if (outs[i] < 0) continue;
        gpio_config_t out = {.pin_bit_mask = 1ULL << outs[i], .mode = GPIO_MODE_OUTPUT};
        gpio_hold_dis(outs[i]);
        gpio_set_level(outs[i], levels[i]);
        gpio_config(&out); // also selects the GPIO function (GPIO12-15 boot as JTAG)
    }
    gpio_config_t in = {.pin_bit_mask = 1ULL << GX5_PIN_ACC_INT | 1ULL << GX5_PIN_VCC_INT | 1ULL << GX5_PIN_APS_INT, .mode = GPIO_MODE_INPUT};
    gpio_config(&in);
#if CONFIG_GX5_CAN_EN_GPIO >= 0
    gpio_config_t en = {.pin_bit_mask = 1ULL << CONFIG_GX5_CAN_EN_GPIO, .mode = GPIO_MODE_OUTPUT};
    gpio_hold_dis(CONFIG_GX5_CAN_EN_GPIO);
    gpio_set_level(CONFIG_GX5_CAN_EN_GPIO, 1); // CAN transceiver in standby until ignition
    gpio_config(&en);
#endif

    esp_pm_config_t pm = {.max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = LIGHT_SLEEP};
    ESP_ERROR_CHECK(esp_pm_configure(&pm));

    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    adc_oneshot_chan_cfg_t chan = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
    adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &chan);
    adc_oneshot_config_channel(s_adc, ADC_CHANNEL_7, &chan);
    adc_cali_line_fitting_config_t cal = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_cali_create_scheme_line_fitting(&cal, &s_cal) != ESP_OK) s_cal = NULL;

    uint32_t causes = esp_sleep_get_wakeup_causes();
    rtc_gpio_deinit(GX5_PIN_CAN_RX); // ext0 leaves its pin with the RTC: back to the CAN controller
    if (causes & BIT(ESP_SLEEP_WAKEUP_EXT0)) s_wake = s_can_armed ? "can" : "ignition";
    else if (causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) s_wake = esp_sleep_get_ext1_wakeup_status() & (1ULL << GX5_PIN_APS_INT) ? "motion" : "modem";
    else if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) s_wake = "timer";

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    mac[5] += 3; // device id = factory MAC + 3
    snprintf(s_id, sizeof s_id, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("############################################################\n");
    printf("## %-54s ##\n", CONFIG_GX5_MANUFACTURER " - " CONFIG_GX5_WEBSITE);
    printf("## %-54s ##\n", CONFIG_GX5_PRODUCT_NAME);
    gx5_pop(mac, s_pop);
    printf("## ID: %-50s ##\n", s_id);
    printf("## PIN: %-49s ##\n", s_pop);
    printf("## HW: %-50s ##\n", CONFIG_GX5_PRODUCT_ID);
    printf("## FW: v%-49s ##\n", esp_app_get_description()->version);
    printf("## %-54s ##\n", CONFIG_GX5_REPO);
    printf("############################################################\n");

    vTaskDelay(pdMS_TO_TICKS(300)); // regulator + ACC/VCC dividers settle
    s_ign = ignition();
    int mv = vcc_mv();
    ESP_LOGI(TAG, "id=%s wake=%s ign=%d vcc=%d mV bat=%d mV rst=%d", s_id, s_wake, s_ign, mv, bat_mv(), esp_reset_reason());

    esp_err_t err = nvs_flash_init(); // broker and CAN configuration live here
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) ESP_LOGE(TAG, "NVS unavailable: configuration will not persist");
    bool wiped = err == ESP_OK && factory_reset_check();
    nvs_handle_t nvs;
    if (nvs_open("gx5", NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t dbg = 0;
        nvs_get_u8(nvs, "dbg", &dbg);
        gx5_debug = dbg;
        nvs_close(nvs);
    }
    GX5_DBG(TAG, "debug mode on: wake causes %lx, ext1 status %llx", (unsigned long)causes, (unsigned long long)esp_sleep_get_ext1_wakeup_status());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(GX5_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gx5_i2c_init();
    if (!strcmp(s_wake, "power-on")) { // not after a wake from sleep: list what is on the I2C bus (on-board + harness)
        char found[128];
        gx5_i2c_scan(found, sizeof found);
        ESP_LOGI(TAG, "i2c devices:%s", found);
    }
    for (int i = 0; wiped && i < 10; i++) { // five blinks of the power LED: reset done
        gx5_exp_set(GX5_EXP_LED_PWR, i & 1);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    gx5_ext_init();
    gx5_modem_start(s_id);
    gx5_can_init();
    gx5_can_profile(&s_igc, &s_pre_s);
    if (s_igc.src != GX5_IGN_ACC) s_ign = false; // no ACC wire in this install: the profile's rule will tell
    if (!strcmp(s_wake, "can")) s_can_wakes++;
    else if (strcmp(s_wake, "timer")) s_can_wakes = 0; // anything else from the vehicle: CAN wake gets a fresh chance

    // Supply low and nothing says the engine is about to charge it: (back) to
    // PROTECT at once. A wake with ignition on (maybe cranking) gets the 3-sample
    // monitor instead, unless it is only the periodic PROTECT recheck.
    if (supply_low(mv) && (!s_ign || (s_protect && (causes & BIT(ESP_SLEEP_WAKEUP_TIMER))))) deep_sleep(true);

    s_t_debounce = make_timer(GX5_EV_ACC_SETTLED);
    s_t_sleep = make_timer(GX5_EV_SLEEP_DUE);
    s_t_telemetry = make_timer(GX5_EV_TELEMETRY);
    s_t_vcc = make_timer(GX5_EV_VCC_TICK);
    esp_timer_start_periodic(s_t_vcc, SEC(5));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(gpio_isr_handler_add(GX5_PIN_ACC_INT, acc_isr, NULL));
    apply_ignition();
    load_profile();
    xTaskCreate(usb_task, "usb", 3072, NULL, 4, NULL);
}
