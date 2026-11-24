// CAN bus: TWAI driver, one task that assembles responses, sends the scheduled
// requests of the current state and watches the profile's ignition rule; the
// vehicle profile (NVS) and publishing of changed values.
// Protocol logic lives in canproto.h.
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "canproto.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gx5.h"
#include "gx5_can.h"
#include "nvs.h"

#ifdef CONFIG_GX5_CAN
static const char *TAG = "can";

static gx5_can_cfg_t s_cfg, s_new; // s_new: scratch for commands and loading (2 kB, too big for a task stack)
static gx5_can_state_t s_st;
static SemaphoreHandle_t s_lock; // guards s_cfg, s_st, s_node and s_state
static QueueHandle_t s_rxq;
static twai_node_handle_t s_node;
static int s_state = GX5_ST_PRE;
static bool s_applied; // s_state is what the bus is running (or stopped) for
static volatile uint32_t s_frames;
static int64_t s_last[GX5_CAN_POLL], s_busy_until; // ms
static gx5_ign_t s_ign;
static gx5_trip_t s_trip;
static bool s_ign_told, s_ign_retell; // retell: the profile changed, report the state again
static bool s_vin_want;               // ask for the VIN until it is answered
static int64_t s_vin_last;

static volatile uint32_t s_err_flags, s_errs; // kinds of bus error seen (twai_error_flags_t) and how many

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static int mode(void) { return s_cfg.st[s_state].mode; }

// ISR: hand the frame to the task.
static bool can_rx(twai_node_handle_t node, const twai_rx_done_event_data_t *e, void *ctx)
{
    gx5_frame_t f = {0};
    twai_frame_t raw = {.buffer = f.data, .buffer_len = sizeof f.data};
    BaseType_t woken = pdFALSE;
    if (twai_node_receive_from_isr(node, &raw) != ESP_OK) return false;
    f.id = raw.header.id;
    f.ext = raw.header.ide;
    f.dlc = raw.header.dlc > 8 ? 8 : raw.header.dlc;
    s_frames = s_frames + 1;
    xQueueSendFromISR(s_rxq, &f, &woken); // full queue: frame dropped
    return woken == pdTRUE;
}

static bool can_err(twai_node_handle_t node, const twai_error_event_data_t *e, void *ctx)
{
    s_err_flags = s_err_flags | e->err_flags.val;
    s_errs = s_errs + 1;
    return false;
}

// Every 5 s: a node that went bus-off is brought back (it would stay silent until
// the next state change), and debug mode gets the health of the bus. Errors with no
// frames received point at the wiring or the bitrate; "ack" alone means nobody is
// listening at this bitrate.
static void bus_check(void)
{
    static int64_t last;
    twai_node_status_t st;
    twai_node_record_t rec;
    twai_error_flags_t f = {.val = s_err_flags};
    if (!s_node || now_ms() - last < 5000 || twai_node_get_info(s_node, &st, &rec) != ESP_OK) return;
    last = now_ms();
    s_err_flags = 0;
    GX5_DBG(TAG, "bus %s: tx errors %u, rx errors %u, bus errors %lu, callbacks %lu%s%s%s%s%s, frames %lu",
            (const char *[]){"ok", "warning", "error-passive", "bus-off"}[st.state], st.tx_error_count, st.rx_error_count, (unsigned long)rec.bus_err_num,
            (unsigned long)s_errs, f.ack_err ? " ack" : "", f.bit_err ? " bit" : "", f.form_err ? " form" : "", f.stuff_err ? " stuff" : "",
            f.arb_lost ? " arb-lost" : "", (unsigned long)s_frames);
    if (st.state == TWAI_ERROR_BUS_OFF) {
        ESP_LOGW(TAG, "bus-off, recovering");
        twai_node_recover(s_node);
    }
}

// The driver transmits asynchronously from the pointer it is given, so frames
// live in a small static ring.
static void can_tx(const gx5_frame_t *f)
{
    static twai_frame_t ring[8];
    static uint8_t data[8][8];
    static int i;
    if (!s_node || mode() < GX5_CAN_OBD2) return;
    GX5_DBG(TAG, "tx %lX %02X%02X%02X%02X%02X%02X%02X%02X", (unsigned long)f->id, f->data[0], f->data[1], f->data[2], f->data[3], f->data[4], f->data[5],
            f->data[6], f->data[7]);
    i = (i + 1) % 8;
    memcpy(data[i], f->data, 8);
    ring[i] = (twai_frame_t){.header = {.id = f->id, .dlc = f->dlc, .ide = f->ext}, .buffer = data[i], .buffer_len = f->dlc};
    if (twai_node_transmit(s_node, &ring[i], 0) != ESP_OK) ESP_LOGD(TAG, "tx %lX failed", (unsigned long)f->id);
}

// Send at most one due request of the current state. OBD-II is one request at a
// time: the next waits for the response to complete, or 300 ms.
static void schedule(void)
{
    int64_t now = now_ms();
    if (now < s_busy_until) return;
    if (s_vin_want && (!s_vin_last || now - s_vin_last > 10000)) { // every 10 s until answered; nothing goes out in listen mode
        can_tx(&s_cfg.cmd[s_cfg.vin_cmd]);
        s_vin_last = now;
        if (mode() == GX5_CAN_OBD2) s_busy_until = now + 300;
        return;
    }
    for (int i = 0; i < GX5_CAN_POLL; i++) {
        const gx5_can_poll_t *p = &s_cfg.st[s_state].poll[i];
        if (!p->ds || !s_cfg.cmd[p->cmd].dlc || (s_last[i] && now - s_last[i] < p->ds * 100LL)) continue;
        can_tx(&s_cfg.cmd[p->cmd]);
        s_last[i] = now;
        if (mode() == GX5_CAN_OBD2) s_busy_until = now + 300;
        return;
    }
}

// Debug mode: a frame per CAN id every 5 s, enough to see what is on a bus without
// drowning the console (32 ids tracked; the oldest gives way).
static void dbg_rx(const gx5_frame_t *f)
{
    static struct {
        uint32_t id;
        int64_t at;
    } seen[32];
    int slot = 0;
    int64_t now = now_ms();
    if (!gx5_debug || (!f->id && !f->dlc)) return;
    for (int i = 0; i < 32; i++) {
        if (seen[i].at && seen[i].id == f->id) {
            if (now - seen[i].at < 5000) return;
            slot = i;
            break;
        }
        if (seen[i].at < seen[slot].at) slot = i;
    }
    seen[slot].id = f->id;
    seen[slot].at = now;
    GX5_DBG(TAG, "rx %lX [%d] %02X%02X%02X%02X%02X%02X%02X%02X (frames %lu)", (unsigned long)f->id, f->dlc, f->data[0], f->data[1], f->data[2], f->data[3],
            f->data[4], f->data[5], f->data[6], f->data[7], (unsigned long)s_frames);
}

static void can_task(void *arg)
{
    gx5_frame_t f, reply;
    for (;;) {
        bool got = xQueueReceive(s_rxq, &f, s_node ? pdMS_TO_TICKS(100) : portMAX_DELAY);
        char vin[18] = "";
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (got) dbg_rx(&f);
        if (got) gx5_ign_frame(&s_cfg.ign, &s_ign, &f, now_ms());
        if (got && mode() == GX5_CAN_OBD2) {
            bool done;
            if (gx5_obd_feed(&s_st, &f, &reply, &done)) can_tx(&reply);
            if (done) {
                GX5_DBG(TAG, "answer from %lX complete", (unsigned long)f.id);
                gx5_trip_update(&s_cfg, &s_st, &s_trip);
                s_busy_until = 0;
                gx5_ign_answer(&s_cfg, &s_st, &s_ign, f.id, now_ms());
            }
        } else if (got && mode() == GX5_CAN_J1939) {
            if (gx5_j1939_feed(&s_st, &s_cfg, &f, &reply)) can_tx(&reply);
            gx5_trip_update(&s_cfg, &s_st, &s_trip);
        }
        if (got && s_vin_want) {
            const gx5_can_val_t *v = gx5_cmd_val(&s_st, &s_cfg.cmd[s_cfg.vin_cmd]);
            if (v && gx5_vin_from(v->data, v->len, vin)) s_vin_want = false;
        }
        bus_check();
        schedule();
        bool on = gx5_ign_on(&s_cfg.ign, &s_ign, now_ms()), tell = s_cfg.ign.src >= GX5_IGN_SIG && (on != s_ign_told || s_ign_retell);
        if (tell) s_ign_told = on, s_ign_retell = false;
        if (tell) GX5_DBG(TAG, "ignition rule %d: %s, last seen %lld ms ago", s_cfg.ign.src, on ? "on" : "off", s_ign.seen_ms ? now_ms() - s_ign.seen_ms : -1LL);
        xSemaphoreGive(s_lock);
        if (vin[0]) esp_event_post(GX5_EVENT, GX5_EV_VIN, vin, sizeof vin, portMAX_DELAY);
        if (tell) esp_event_post(GX5_EVENT, GX5_EV_CAN_IGN, &on, sizeof on, portMAX_DELAY);
    }
}

static void stop_locked(void)
{
    if (CONFIG_GX5_CAN_EN_GPIO >= 0) gpio_set_level(CONFIG_GX5_CAN_EN_GPIO, 1); // transceiver standby
    if (!s_node) return;
    twai_node_disable(s_node); // also releases the driver's no-light-sleep lock
    twai_node_delete(s_node);
    s_node = NULL;
}

static void start_locked(void)
{
    gx5_frame_t wake = {0}, claim;
    if (s_node || mode() == GX5_CAN_OFF) return;
    twai_onchip_node_config_t cfg = {
        .io_cfg = {.tx = GX5_PIN_CAN_TX, .rx = GX5_PIN_CAN_RX, .quanta_clk_out = -1, .bus_off_indicator = -1},
        .bit_timing.bitrate = s_cfg.bitrate,
        .tx_queue_depth = 8,
        .flags.enable_listen_only = mode() == GX5_CAN_LISTEN, // listen: never drive or acknowledge
    };
    twai_event_callbacks_t cbs = {.on_rx_done = can_rx, .on_error = can_err};

    memset(s_st.sess, 0, sizeof s_st.sess);
    memset(s_last, 0, sizeof s_last);
    s_busy_until = 0;
    if (CONFIG_GX5_CAN_EN_GPIO >= 0) gpio_set_level(CONFIG_GX5_CAN_EN_GPIO, 0); // RS low = transceiver active
    if (twai_new_node_onchip(&cfg, &s_node) != ESP_OK || twai_node_register_event_callbacks(s_node, &cbs, NULL) != ESP_OK ||
        twai_node_enable(s_node) != ESP_OK) {
        ESP_LOGW(TAG, "not started");
        stop_locked();
        return;
    }
    if (mode() == GX5_CAN_J1939) {
        gx5_j1939_claim(&claim);
        can_tx(&claim);
    }
    xQueueSend(s_rxq, &wake, 0); // get the task out of its idle wait
}

void gx5_can_stop(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    stop_locked();
    s_applied = false;
    xSemaphoreGive(s_lock);
}

// Run the bus the way the profile says for this state (which may be: not at all).
void gx5_can_state(int state)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (state != s_state || !s_applied) {
        // Same mode in both states: keep the node, or an answer in flight is lost (a
        // Cerato's odometer answer was cut off by pre -> on and came 30 s late).
        bool keep = s_applied && s_node && s_cfg.st[state].mode == mode();
        if (!keep) stop_locked(); // listen <-> active needs a new node
        s_state = state;
        s_applied = true;
        if (keep) memset(s_last, 0, sizeof s_last); // the new state's requests are all due
        else start_locked();
        ESP_LOGI(TAG, "state %s, mode %d", (const char *[]){"pre", "on", "off"}[state], mode());
    }
    xSemaphoreGive(s_lock);
}

uint32_t gx5_can_frames(void) { return s_frames; }

void gx5_can_trip(char *out, size_t size)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    gx5_trip_json(&s_cfg, &s_trip, out, size);
    xSemaphoreGive(s_lock);
}

void gx5_can_vin_check(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_vin_want = s_cfg.vin_cmd < GX5_CAN_CMDS && s_cfg.cmd[s_cfg.vin_cmd].dlc;
    s_vin_last = 0;
    xSemaphoreGive(s_lock);
}

void gx5_can_profile(gx5_ign_cfg_t *ign, uint16_t *pre_s, bool *wake_vcc)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *ign = s_cfg.ign;
    *pre_s = s_cfg.pre_s;
    *wake_vcc = s_cfg.wake_vcc;
    xSemaphoreGive(s_lock);
}

// {"vehicle":..}, {"pid":..}, {"sig":..} or {"can":..} (canproto.h): one part of the
// vehicle profile; {"profile":"clear"} puts it back as shipped. Stored in NVS and
// applied at once. False if the message is none of them.
bool gx5_can_command(const char *json)
{
    nvs_handle_t h;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    char what[8];
    s_new = s_cfg;
    if (gx5_json_val(json, "profile") && gx5_json_str(json, "profile", what, sizeof what) && !strcmp(what, "clear")) {
        gx5_can_defaults(&s_new, CONFIG_GX5_CAN_BITRATE);
    } else if (!gx5_parse_vehicle(json, &s_new) && gx5_parse_pid(json, &s_new) < 0 && gx5_parse_sig(json, &s_new) < 0 && gx5_parse_can_state(json, &s_new) < 0) {
        xSemaphoreGive(s_lock);
        return false;
    }
    bool running = s_applied;
    stop_locked();
    s_cfg = s_new;
    memset(s_st.val, 0, sizeof s_st.val); // answers to the old library mean nothing now
    s_ign.seen_ms = 0;
    s_ign_retell = true;
    s_vin_want = false;
    memset(&s_trip, 0, sizeof s_trip);
    if (nvs_open("gx5", NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_set_blob(h, "veh", &s_cfg, sizeof s_cfg) != ESP_OK || nvs_commit(h) != ESP_OK) ESP_LOGE(TAG, "profile not saved");
        nvs_close(h);
    }
    ESP_LOGI(TAG, "profile %s: bitrate=%lu ign=%d modes pre/on/off=%d/%d/%d", s_cfg.name, (unsigned long)s_cfg.bitrate, s_cfg.ign.src,
             s_cfg.st[GX5_ST_PRE].mode, s_cfg.st[GX5_ST_ON].mode, s_cfg.st[GX5_ST_OFF].mode);
    if (running) start_locked();
    xSemaphoreGive(s_lock);
    xQueueSend(s_rxq, &(gx5_frame_t){0}, 0); // the task may be idle: let it report ignition under the new rule
    return true;
}

// Hand what changed since last time to `send`: decoded signals as
// {"ts":..,"sig":{"rpm":1726,..}}, answers without a signal as text,
// {"ts":..,"obd":["7E8 01 0C 1AF8",..]} or {"ts":..,"j1939":["FEEE 00 8C7D..",..]},
// split over as many messages as needed.
void gx5_can_publish(bool (*send)(const char *json))
{
    char msg[GX5_PUB_MAX];
    time_t now = time(NULL);
    long long ts = now > 1700000000 ? (long long)now : 0LL;
    for (int next = 0; next < GX5_CAN_SIGS;) {
        xSemaphoreTake(s_lock, portMAX_DELAY); // held only while formatting, not while publishing
        int items = gx5_sig_chunk(&s_cfg, &s_st, ts, msg, sizeof msg, &next);
        if (next >= GX5_CAN_SIGS) gx5_sig_sent(&s_cfg, &s_st);
        xSemaphoreGive(s_lock);
        if (items && !send(msg)) return;
    }
    for (bool more = true; more;) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int items = gx5_can_chunk(&s_st, mode() == GX5_CAN_J1939, ts, msg, sizeof msg, &more);
        xSemaphoreGive(s_lock);
        if (!items || !send(msg)) return;
    }
}

void gx5_can_init(void)
{
    nvs_handle_t h;
    size_t len = sizeof s_new;
    gx5_can_defaults(&s_cfg, CONFIG_GX5_CAN_BITRATE);
    if (nvs_open("gx5", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_blob(h, "veh", &s_new, &len) == ESP_OK && len == sizeof s_new) s_cfg = s_new; // another layout: defaults
        nvs_close(h);
    }
    s_lock = xSemaphoreCreateMutex();
    s_rxq = xQueueCreate(64, sizeof(gx5_frame_t));
    xTaskCreate(can_task, "can", 4096, NULL, 8, NULL);
}
#else
void gx5_can_init(void) {}
void gx5_can_state(int state) {}
void gx5_can_vin_check(void) {}
void gx5_can_trip(char *out, size_t size) { out[0] = '\0'; }
void gx5_can_stop(void) {}
uint32_t gx5_can_frames(void) { return 0; }
bool gx5_can_command(const char *json) { return false; }
void gx5_can_publish(bool (*send)(const char *json)) {}
void gx5_can_profile(gx5_ign_cfg_t *ign, uint16_t *pre_s, bool *wake_vcc)
{
    *ign = (gx5_ign_cfg_t){0}; // the ACC wire
    *pre_s = 60;
    *wake_vcc = false;
}
#endif
