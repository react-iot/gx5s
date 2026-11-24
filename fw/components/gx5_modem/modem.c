// Quectel BG96 over UART: AT engine, network bring-up and the modem's own MQTT
// client (AT+QMT*). One task owns the UART; everything else talks to it via s_msgq.
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "gx5.h"
#include "gx5_modem.h"

#define UART     UART_NUM_1
// +QMTDISC: once took longer than 5 s on the bench (2026-11-17). Giving up early
// leaves the session open and the late URC wakes the board right after it sleeps.
#define DISC_MS  15000
#define LINE_LEN 640 // +QMTRECV line: topic + a 511-byte payload
#define AT(cmd)  at(cmd, NULL, NULL, NULL, 2000)
#define STR_(x)  #x
#define STR(x)   STR_(x)

static const char *TAG = "modem";

typedef struct {
    enum { M_PUB, M_RI, M_STOP, M_RECONNECT, M_OTA } type; // M_OTA: version in data
    bool retain;
    char topic[96]; // queued M_PUB: subtopic only; the modem task adds device/<id>/<location>/
    char data[GX5_PUB_MAX];
} msg_t;

static QueueHandle_t s_uartq, s_msgq;
static QueueSetHandle_t s_set;
static SemaphoreHandle_t s_parked;
static esp_pm_lock_handle_t s_awake; // held whenever UART bytes may arrive
static volatile bool s_connected;
static volatile int s_stop;          // 0 run, 1 park for SLEEP, 2 power off
static volatile bool s_abortable;    // bring-up in progress: s_stop aborts AT waits
static const char *s_id;

// Broker selection (ADR 0001 §8): candidate (just received, untested) > stored (NVS)
// > bootstrap (menuconfig). s_fails counts consecutive stored-broker failures with
// the network up; it lives in RTC memory so it survives deep sleep.
static const gx5_broker_t s_boot = {
    .host = CONFIG_GX5_MQTT_HOST,
    .port = CONFIG_GX5_MQTT_PORT, // anonymous: no user/pass, client id = MAC
#ifdef CONFIG_GX5_MQTT_TLS
    .tls = true,
#endif
};
static gx5_broker_t s_stored, s_cand;
static volatile bool s_have_cand;
static bool s_have_stored, s_provisioning, s_session; // s_session: MQTT was opened since the last park
static RTC_DATA_ATTR uint8_t s_fails;

// Location id: topics are device/<id>/<location>/..., commands arrive on device/<id>
// and location/<location>. Only the modem task reads s_loc; a change comes through NVS.
static char s_loc[GX5_LOC_LEN] = "0000";
static volatile bool s_loc_dirty = true;

static void load_loc(void)
{
    nvs_handle_t h;
    size_t len = sizeof s_loc;
    s_loc_dirty = false;
    if (nvs_open("gx5", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_str(h, "loc", s_loc, &len) != ESP_OK || !gx5_id_ok(s_loc)) strcpy(s_loc, "0000");
    nvs_close(h);
}

static void full_topic(char *out, size_t size, const char *sub)
{
    snprintf(out, size, "device/%s/%s/%s", s_id, s_loc, sub);
}

static void urc(char *line)
{
    char *topic, *payload;
    if (!strncmp(line, "+QMTRECV:", 9)) {
        gx5_msg_t m = {0};
        if (gx5_parse_qmtrecv(line, &topic, &payload)) {
            strlcpy(m.topic, topic, sizeof m.topic);
            strlcpy(m.data, payload, sizeof m.data);
            esp_event_post(GX5_EVENT, GX5_EV_MQTT_DATA, &m, sizeof m, 0);
        }
    } else if (!strncmp(line, "+QMTSTAT:", 9)) {
        ESP_LOGW(TAG, "broker link lost: %s", line);
        s_connected = false;
    } else {
        ESP_LOGD(TAG, "urc: %s", line);
    }
}

// GX5_MODEM_AT_LOG or debug mode: every command and reply line. Lines with credentials (broker login,
// APN, a received MQTT message, which may be a provisioning message) lose their arguments.
#ifdef CONFIG_GX5_MODEM_AT_LOG
#define AT_LOG true
#else
#define AT_LOG false // only in debug mode
#endif
static void at_log(const char *dir, const char *s)
{
    if (!AT_LOG && !gx5_debug) return;
    bool secret = strstr(s, "QMTCONN") || strstr(s, "QICSGP") || strstr(s, "QMTRECV");
    ESP_LOGI(TAG, "%s %.*s%s", dir, secret ? (int)strcspn(s, "=:") : (int)strlen(s), s, secret ? " ..." : "");
}

// One line (CR/LF stripped, blanks skipped) or the bare '>' data prompt.
static bool read_line(char *buf, size_t size, TickType_t deadline)
{
    size_t n = 0;
    for (;;) {
        if (s_stop && s_abortable) return false;
        int32_t left = (int32_t)(deadline - xTaskGetTickCount());
        if (left <= 0) return false;
        uint8_t c;
        if (uart_read_bytes(UART, &c, 1, left > pdMS_TO_TICKS(200) ? pdMS_TO_TICKS(200) : left) != 1) continue;
        if (c == '\r' || c == '\n') {
            if (!n) continue;
            buf[n] = '\0';
            at_log("<", buf);
            return true;
        }
        if (n == 0 && c == ' ') continue;
        if (n == 0 && c == '>') {
            strcpy(buf, ">");
            return true;
        }
        if (n < size - 1) buf[n++] = c;
    }
}

// Send cmd. Succeeds once "OK" and (if given) a line starting with `expect` have
// both arrived, in either order; that line is copied to `out` (LINE_LEN) if set.
// `payload` is written at the '>' prompt. Unrelated lines are handled as URCs.
static bool at_awake(const char *cmd, const char *expect, char *out, const char *payload, int timeout_ms)
{
    char line[LINE_LEN];
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    bool ok = false, got = !expect;
    int name = (int)strcspn(cmd, "="); // never log arguments (credentials)

    at_log(">", cmd);
    uart_write_bytes(UART, cmd, strlen(cmd));
    uart_write_bytes(UART, "\r", 1);
    while (read_line(line, sizeof line, deadline)) {
        if (!strcmp(line, "OK")) {
            ok = true;
        } else if (!strncmp(line, "ERROR", 5) || !strncmp(line, "+CME ERROR", 10) || !strncmp(line, "+CMS ERROR", 10)) {
            if (strncmp(cmd, "AT+QGPS", 7)) ESP_LOGW(TAG, "%.*s -> %s", name, cmd, line); // "no fix yet" is routine
            return false;
        } else if (payload && (!strcmp(line, ">") || !strcmp(line, "CONNECT"))) { // data prompts
            at_log(">", payload);
            uart_write_bytes(UART, payload, strlen(payload));
        } else if (expect && !strncmp(line, expect, strlen(expect))) {
            got = true;
            if (out) strcpy(out, line);
        } else {
            urc(line);
        }
        if (ok && got) return true;
    }
    ESP_LOGW(TAG, "%.*s -> no response", name, cmd);
    return false;
}

// One command and its reply. UART bytes are lost in light sleep, so the MCU stays
// awake for the whole exchange whoever calls (the lock counts, nesting is fine).
static bool at(const char *cmd, const char *expect, char *out, const char *payload, int timeout_ms)
{
    esp_pm_lock_acquire(s_awake);
    bool ok = at_awake(cmd, expect, out, payload, timeout_ms);
    esp_pm_lock_release(s_awake);
    return ok;
}

static bool sync_at(int tries)
{
    while (tries-- && !(s_stop && s_abortable))
        if (at("AT", NULL, NULL, NULL, 500)) return true;
    return false;
}

static bool bringup(void)
{
    char line[LINE_LEN];

    if (!sync_at(6)) { // powered down (after PROTECT) or hung
        ESP_LOGW(TAG, "no AT response, pulsing reset");
        gpio_set_level(GX5_PIN_GSM_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(GX5_PIN_GSM_RST, 1);
        if (!sync_at(40)) return false;
    }
    AT("ATE0");
    AT("AT+QSCLK=0");
    at("AT+CFUN=1", NULL, NULL, NULL, 15000); // radio back on after a radio-off sleep
    AT("AT+QGPSCFG=\"autogps\",0"); // GNSS starts only when asked (GX5sV2 stored auto-start in the modem)
    AT("AT+CPSMS=0"); // never PSM: this card wakes from it at once, which is a modem restart mid-session
    // Pulse RI 120 ms before every URC and hold the URC until the pulse ends, so
    // the MCU can light-sleep and still catch +QMTRECV (see ADR 0001 §7).
    AT("AT+QCFG=\"risignaltype\",\"physical\"");
    AT("AT+QCFG=\"urc/ri/other\",\"pulse\",120,1");
    AT("AT+QCFG=\"urc/ri/smsincoming\",\"pulse\",120");
    AT("AT+QCFG=\"urc/delay\",1");
    AT("AT+CTZU=1"); // modem clock follows network time
#ifdef CONFIG_GX5_GNSS
    AT("AT+QGPS=1"); // errors if already running
#endif
    // Right after AT+CFUN=1 the SIM is still starting: +CME ERROR: 14 (SIM busy). Ask again
    // rather than fail the whole bring-up and wait out the 5 s back-off (soak 2026-11-22).
    for (int i = 0; !at("AT+CPIN?", "+CPIN: READY", NULL, NULL, 5000); i++) {
        if (i >= 20 || (s_stop && s_abortable)) return false; // ~4 s: no SIM
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    AT("AT+CMGD=1,4"); // SMS is only a wake-up poke: drop it so storage never fills

    for (int i = 0;; i++) {
        if (at("AT+CEREG?", "+CEREG:", line, NULL, 2000) && gx5_reg_ok(line)) break;
        if (at("AT+CREG?", "+CREG:", line, NULL, 2000) && gx5_reg_ok(line)) break;
        if (i >= 60 || (s_stop && s_abortable)) return false; // ~2 min, then back off
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (CONFIG_GX5_APN[0]) AT("AT+QICSGP=1,1,\"" CONFIG_GX5_APN "\",\"\",\"\",0");
    at("AT+QIDEACT=1", NULL, NULL, NULL, 40000); // clear a stale context; may fail
    return at("AT+QIACT=1", NULL, NULL, NULL, 150000);
}

// Serving cell and signal, e.g.
//   lte +COPS: 0,0,"Telstra",8 +QNWINFO: "CAT-M1","50501","LTE BAND 28",9410 +QCSQ: "CAT-M1",-70,-98,150,-12
// COPS: operator name. QCSQ fields after the mode: RSSI dBm, RSRP dBm, SINR, RSRQ dB.
static void log_network(void)
{
    char op[LINE_LEN] = "", cell[LINE_LEN] = "", sig[LINE_LEN] = "";
    at("AT+COPS?", "+COPS:", op, NULL, 2000);
    at("AT+QNWINFO", "+QNWINFO:", cell, NULL, 2000);
    at("AT+QCSQ", "+QCSQ:", sig, NULL, 2000);
    ESP_LOGI(TAG, "lte %s %s %s", op, cell, sig);
}

static bool mqtt_connect(const gx5_broker_t *b)
{
    char cmd[256], line[LINE_LEN];
    int result = -1;

    // A socket left open from before sleep: opening over it fails with "client identifier
    // occupied" (+QMTOPEN: 0,2), so wait for the close to finish. ERROR = nothing open.
    at("AT+QMTCLOSE=0", "+QMTCLOSE:", NULL, NULL, 5000);

    AT("AT+QMTCFG=\"version\",0,4"); // MQTT 3.1.1
    AT("AT+QMTCFG=\"pdpcid\",0,1");
    AT("AT+QMTCFG=\"keepalive\",0," STR(CONFIG_GX5_MQTT_KEEPALIVE_S));
    AT("AT+QMTCFG=\"session\",0,0"); // persistent: broker queues QoS1 cmds while we sleep
    // Unprovisioned, the broker allows discovery/<id> and nothing else: no last will.
    // The will is first set with QoS 0 / no retain, then switched off. Switching it off
    // alone made the broker drop the CONNECT unanswered (bench 2026-11-16); probably the
    // modem kept the earlier will QoS and retain bits, which MQTT forbids without a will.
    if (b == &s_boot) {
        AT("AT+QMTCFG=\"will\",0,1,0,0,\"x\",\"x\"");
        strcpy(cmd, "AT+QMTCFG=\"will\",0,0");
    } else snprintf(cmd, sizeof cmd, "AT+QMTCFG=\"will\",0,1,1,1,\"device/%s/%s/status/connected\",\"offline\"", s_id, s_loc);
    AT(cmd);
    if (b->tls) {
        AT("AT+QSSLCFG=\"seclevel\",0," STR(CONFIG_GX5_MQTT_TLS_SECLEVEL));
        if (CONFIG_GX5_MQTT_TLS_SECLEVEL) AT("AT+QSSLCFG=\"cacert\",0,\"" CONFIG_GX5_MQTT_TLS_CACERT "\"");
        AT("AT+QMTCFG=\"ssl\",0,1,0");
    } else {
        AT("AT+QMTCFG=\"ssl\",0,0");
    }

    snprintf(cmd, sizeof cmd, "AT+QMTOPEN=0,\"%s\",%d", b->host, b->port);
    if (!at(cmd, "+QMTOPEN:", line, NULL, 75000) || gx5_last_int(line) != 0) {
        ESP_LOGE(TAG, "broker open failed: %s", line);
        return false;
    }
    if (b->user[0])
        snprintf(cmd, sizeof cmd, "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"", s_id, b->user, b->pass);
    else
        snprintf(cmd, sizeof cmd, "AT+QMTCONN=0,\"%s\"", s_id);
    if (!at(cmd, "+QMTCONN:", line, NULL, 15000) || gx5_last_int(line) != 0) {
        ESP_LOGE(TAG, "broker rejected connect: %s", line);
        return false;
    }
    // +QMTSUB: <client>,<msgid>,<result>,<value>
    if (b == &s_boot) { // the login arrives on the topic we announce on
        snprintf(cmd, sizeof cmd, "AT+QMTSUB=0,1,\"discovery/%s\",1", s_id);
        return at(cmd, "+QMTSUB:", line, NULL, 15000) && sscanf(line, "+QMTSUB: %*d,%*d,%d", &result) == 1 && result == 0;
    }
    // Commands: device/<id> (this device) and location/<location> (every device there).
    snprintf(cmd, sizeof cmd, "AT+QMTSUB=0,1,\"device/%s\",1", s_id);
    if (!at(cmd, "+QMTSUB:", line, NULL, 15000) || sscanf(line, "+QMTSUB: %*d,%*d,%d", &result) != 1 || result) return false;
    snprintf(cmd, sizeof cmd, "AT+QMTSUB=0,2,\"location/%s\",1", s_loc);
    return at(cmd, "+QMTSUB:", line, NULL, 15000) && sscanf(line, "+QMTSUB: %*d,%*d,%d", &result) == 1 && result == 0;
}

#ifdef CONFIG_GX5_GNSS
static char s_fix[128]; // ,"lat":..,"lon":..,"kmh":..,"cog":..,"sat":..   ("" = no fix at the last look)
static portMUX_TYPE s_fix_mux = portMUX_INITIALIZER_UNLOCKED;

// Ask the modem where it is and keep the answer for gx5_gnss_fix(). The BG96 runs
// GNSS alongside LTE; this is refreshed on every bring-up and telemetry publish.
static void refresh_fix(void)
{
    char line[LINE_LEN] = "", fix[sizeof s_fix] = "";
    long long t = 0;
    if (!at("AT+QGPSLOC=2", "+QGPSLOC:", line, NULL, 2000) || !gx5_fix_json(line, fix, sizeof fix, &t)) {
        fix[0] = '\0';
        at("AT+QGPS?", "+QGPS:", line, NULL, 2000); // 1 = receiver running, just no fix yet
    }
    ESP_LOGI(TAG, "gnss %s", fix[0] ? fix + 1 : line[0] ? line : "no answer");
    if (fix[0] && t) { // GPS time: the clocks follow every fix
        int64_t utc = t;
        esp_event_post(GX5_EVENT, GX5_EV_TIME, &utc, sizeof utc, 0);
    }
    portENTER_CRITICAL(&s_fix_mux);
    strcpy(s_fix, fix);
    portEXIT_CRITICAL(&s_fix_mux);
}

size_t gx5_gnss_fix(char *out, size_t size)
{
    size_t n;
    portENTER_CRITICAL(&s_fix_mux);
    n = strlen(s_fix);
    if (n < size) memcpy(out, s_fix, n + 1);
    portEXIT_CRITICAL(&s_fix_mux);
    return n < size ? n : 0;
}
#else
static void refresh_fix(void) {}
size_t gx5_gnss_fix(char *out, size_t size) { return 0; }
#endif

// Network time -> GX5_EV_TIME; the application sets the clocks. GPS time (refresh_fix)
// does the same whenever there is a fix.
static void sync_time(void)
{
    char line[LINE_LEN];
    long long t;
    bool ok = at("AT+CCLK?", "+CCLK:", line, NULL, 2000) && gx5_parse_cclk(line, &t);
    ESP_LOGI(TAG, "time %s%s", line, ok ? "" : " (not used)");
    if (!ok) return;
    int64_t utc = t;
    esp_event_post(GX5_EVENT, GX5_EV_TIME, &utc, sizeof utc, 0);
}

static void publish(const char *topic, const msg_t *m)
{
    static uint16_t id;
    char cmd[160];
    int qos = m->retain ? 1 : 0;

    if (qos && ++id == 0) id = 1; // BG96: msgid must be non-zero for QoS > 0
    snprintf(cmd, sizeof cmd, "AT+QMTPUB=0,%u,%d,%d,\"%s\",%u", qos ? id : 0, qos, m->retain, topic, (unsigned)strlen(m->data));
    if (!at(cmd, "+QMTPUB:", NULL, m->data, 15000)) s_connected = false; // reconnect
    if (strstr(topic, "/telemetry")) refresh_fix();                    // for the next one
}

// ── OTA ─────────────────────────────────────────────────────────────────────
// The modem downloads the image into its own file system (AT+QHTTPGET +
// QHTTPREADFILE), then it is pulled across in 1 KB pieces (AT+QFREAD) and written
// to the other app slot. Pulling keeps the UART from overrunning during flash writes.
static volatile bool s_ota;

// AT+QFREAD=<fh>,<n> -> "CONNECT <got>\r\n" <got raw bytes> "\r\nOK". Returns got, -1 on error.
static int file_read(int fh, uint8_t *buf, int n)
{
    char cmd[40], line[LINE_LEN];
    int got = -1;
    uint8_t lf;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    snprintf(cmd, sizeof cmd, "AT+QFREAD=%d,%d\r", fh, n);
    uart_write_bytes(UART, cmd, strlen(cmd));
    while (read_line(line, sizeof line, deadline)) {
        if (sscanf(line, "CONNECT %d", &got) == 1) break;
        if (strstr(line, "ERROR")) return -1;
    }
    if (got < 0 || got > n || uart_read_bytes(UART, &lf, 1, pdMS_TO_TICKS(500)) != 1 || lf != '\n') return -1;
    if (uart_read_bytes(UART, buf, got, pdMS_TO_TICKS(3000)) != got) return -1;
    while (read_line(line, sizeof line, deadline))
        if (!strcmp(line, "OK")) return got;
    return -1;
}

// Returns why it failed; on success the device restarts into the new image.
static const char *ota(const char *ver)
{
    static uint8_t buf[1024];
    char url[200], cmd[64], line[LINE_LEN];
    int err = -1, code = 0, size = 0, fh = -1, done = 0;
    esp_ota_handle_t h;
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    const char *why = NULL;

    if (!part) return "no-ota-slot";
    int n = snprintf(url, sizeof url, CONFIG_GX5_OTA_URL "/fw/" CONFIG_GX5_PRODUCT_ID "-%s.bin?id=%s&v=%s", ver, s_id, esp_app_get_description()->version);
    if (n >= (int)sizeof url) return "url-too-long";
    ESP_LOGI(TAG, "ota %s", url);
    AT("AT+QHTTPCFG=\"contextid\",1");
    AT("AT+QHTTPCFG=\"responseheader\",0"); // body only
    snprintf(cmd, sizeof cmd, "AT+QHTTPURL=%d,30", n);
    if (!at(cmd, NULL, NULL, url, 35000)) return "url-rejected";
    // +QHTTPGET: <err>[,<http status>[,<content length>]]
    if (!at("AT+QHTTPGET=80", "+QHTTPGET:", line, NULL, 90000) || sscanf(line, "+QHTTPGET: %d,%d,%d", &err, &code, &size) < 2 || err || code != 200) {
        ESP_LOGE(TAG, "ota: %s", line);
        return code == 404 ? "no-such-firmware" : "download-failed";
    }
    if (size <= 0 || size > (int)part->size) return "bad-size";
    at("AT+QFDEL=\"ota.bin\"", NULL, NULL, NULL, 5000); // left over from a failed try; may fail
    if (!at("AT+QHTTPREADFILE=\"ota.bin\",80", "+QHTTPREADFILE:", line, NULL, 900000) || gx5_last_int(line) != 0) return "download-failed";
    if (!at("AT+QFOPEN=\"ota.bin\",2", "+QFOPEN:", line, NULL, 5000) || sscanf(line, "+QFOPEN: %d", &fh) != 1) return "file-open-failed";

    if (esp_ota_begin(part, size, &h) != ESP_OK) {
        why = "flash-begin-failed";
    } else {
        while (done < size) {
            int got = file_read(fh, buf, size - done < (int)sizeof buf ? size - done : (int)sizeof buf);
            if (got <= 0 || esp_ota_write(h, buf, got) != ESP_OK) break;
            done += got;
            if (done % (64 * 1024) < got) ESP_LOGI(TAG, "ota %d / %d", done, size);
        }
        if (done != size) {
            esp_ota_abort(h);
            why = "transfer-failed";
        } else if (esp_ota_end(h) != ESP_OK) { // checks the image (magic, checksum, SHA-256)
            why = "bad-image";
        } else if (esp_ota_set_boot_partition(part) != ESP_OK) {
            why = "set-boot-failed";
        }
    }
    snprintf(cmd, sizeof cmd, "AT+QFCLOSE=%d", fh);
    AT(cmd);
    AT("AT+QFDEL=\"ota.bin\"");
    return why;
}

// 'm' = message in *m, 'u' = UART activity, 0 = timeout. Queues are only ever
// read through the set (FreeRTOS requirement).
static int wait_any(msg_t *m, TickType_t ticks)
{
    QueueSetMemberHandle_t h = xQueueSelectFromSet(s_set, ticks);
    uart_event_t e;
    if (h == s_msgq) return xQueueReceive(s_msgq, m, 0) ? 'm' : 0;
    if (h == s_uartq) return xQueueReceive(s_uartq, &e, 0) ? 'u' : 0;
    return 0;
}

static const char *why_here(void);
static void announce(const char *reason);

// Connected: sleep until the modem (RI pulse / UART) or the app wants something.
static void serve(void)
{
    msg_t m;
    char line[LINE_LEN];
    bool held = false;

    gpio_intr_enable(GX5_PIN_GSM_INT);
    while (s_connected) {
        int w = wait_any(&m, held ? pdMS_TO_TICKS(3000) : portMAX_DELAY);
        if (!w) { // 3 s quiet: allow light sleep again
            esp_pm_lock_release(s_awake);
            held = false;
            gpio_intr_enable(GX5_PIN_GSM_INT);
            continue;
        }
        if (!held) {
            esp_pm_lock_acquire(s_awake);
            held = true;
        }
        if (w == 'u') {
            while (read_line(line, sizeof line, xTaskGetTickCount() + pdMS_TO_TICKS(100))) urc(line);
        } else if (m.type == M_PUB && s_provisioning) {
            announce(why_here()); // telemetry tick: still here, still waiting for a login
        } else if (m.type == M_PUB) {
            char topic[96];
            full_topic(topic, sizeof topic, m.topic);
            publish(topic, &m);
        } else if (m.type == M_OTA) {
            char topic[96];
            msg_t r = {.type = M_PUB};
            s_ota = true;
            const char *why = ota(m.data);
            s_ota = false;
            ESP_LOGW(TAG, "ota %s", why ? why : "done, restarting");
            snprintf(r.data, sizeof r.data, "{\"ev\":\"ota\",\"version\":\"%.15s\",\"result\":\"%s\"}", m.data, why ? why : "ok");
            full_topic(topic, sizeof topic, "event");
            publish(topic, &r);
            if (!why) {
                at("AT+QMTDISC=0", "+QMTDISC:", NULL, NULL, 5000);
                esp_restart();
            }
        } else if (m.type == M_STOP || m.type == M_RECONNECT) {
            break;
        }
    }
    if (held) esp_pm_lock_release(s_awake);
}

// GNSS off, confirmed: left on it searches at ~42 mA and the modem never sleeps. The
// first AT+QGPSEND after the last publish got no answer (bench 2026-11-19), so ask again.
static void gnss_off(void)
{
    char line[LINE_LEN];
    for (int i = 0; i < 5; i++) {
        at("AT+QGPSEND", NULL, NULL, NULL, 3000);
        line[0] = '\0';
        if (at("AT+QGPS?", "+QGPS:", line, NULL, 2000) && strstr(line, "+QGPS: 0")) return;
    }
    ESP_LOGE(TAG, "GNSS still on");
}

static void park(void)
{
    esp_pm_lock_acquire(s_awake);
    vTaskDelay(pdMS_TO_TICKS(300)); // let an aborted command finish talking
    uart_flush_input(UART);
    // End the session first, also before the radio goes off: a link that just drops makes
    // the broker publish our will ("offline") over the status we sent last.
    if (s_session) log_network(); // signal at the end too: just after registering it reads -125,0,8,0 (not measured yet)
    if (s_session) at("AT+QMTDISC=0", "+QMTDISC:", NULL, NULL, DISC_MS); // never connected: no URC comes, time wasted
    s_session = false;
    gnss_off();
    if (s_stop == 2) {
        // Not AT+QPOWD: the card confirms it, then starts again by itself ~6 s later and
        // runs awake all through the sleep (bench 2026-11-19: LiPo flat in 5.5 h).
        AT("AT+QCFG=\"ledmode\",1"); // net LED off while the radio is off; the default mode blinks it every 2 s all through the sleep
        if (!at("AT+CFUN=0", NULL, NULL, NULL, 15000)) ESP_LOGE(TAG, "modem radio not off"); // deregisters
    }
    AT("AT+QSCLK=1"); // modem sleeps when idle; registered unless the radio is off
    uart_wait_tx_done(UART, pdMS_TO_TICKS(200));
    esp_pm_lock_release(s_awake);
}

static void save_broker(void)
{
    nvs_handle_t h;
    if (nvs_open("gx5", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, "broker", &s_stored, sizeof s_stored) != ESP_OK || nvs_commit(h) != ESP_OK) ESP_LOGE(TAG, "broker not saved");
    nvs_close(h);
}

static void load_broker(void)
{
    nvs_handle_t h;
    size_t len = sizeof s_stored;
    if (nvs_open("gx5", NVS_READONLY, &h) != ESP_OK) return;
    s_have_stored = nvs_get_blob(h, "broker", &s_stored, &len) == ESP_OK && len == sizeof s_stored && s_stored.host[0];
    nvs_close(h);
}

static const char *why_here(void) { return s_have_stored ? "broker-unreachable" : "unprovisioned"; }

// Tell whoever runs this broker who we are and why we are here:
// discovery/<id> on the bootstrap broker, device/<id>/<location>/event otherwise.
static void announce(const char *reason)
{
    msg_t m = {.type = M_PUB};
    char topic[96];
    if (s_provisioning) snprintf(topic, sizeof topic, "discovery/%s", s_id);
    else full_topic(topic, sizeof topic, "event");
    snprintf(m.data, sizeof m.data, "{\"id\":\"%s\",\"hw\":\"" CONFIG_GX5_PRODUCT_ID "\",\"fw\":\"%s\",\"reason\":\"%s\"}", s_id, esp_app_get_description()->version, reason);
    publish(topic, &m);
}

static void modem_task(void *arg)
{
    msg_t m;
    int backoff_s = 5;
    const char *note = NULL; // reported on the next connection

    for (;;) {
        bool cand = s_have_cand, retry_now = false;
        const gx5_broker_t *b = cand ? &s_cand
                              : s_have_stored && s_fails < CONFIG_GX5_PROVISION_AFTER_FAILS ? &s_stored
                              : &s_boot;
        if (!s_stop) {
            if (s_loc_dirty) load_loc();
            esp_pm_lock_acquire(s_awake);
            s_abortable = true;
            // Reconnecting with a session still open (a login arrived, the broker changed):
            // end it before bringup() drops the data context under it. After that the modem
            // refuses DISC, never finishes CLOSE, and the next open fails (bench 2026-11-17).
            if (s_session) at("AT+QMTDISC=0", "+QMTDISC:", NULL, NULL, DISC_MS);
            s_session = false;
            bool net = bringup(); // the network comes first: no broker is reachable without it
            ESP_LOGI(TAG, "network %s", net ? "up" : "down");
            if (net) {
                log_network();
                sync_time(); // network time needs the cell, not a broker
                refresh_fix();
            }
            bool up = false;
            if (!b->host[0]) ESP_LOGE(TAG, "not provisioned and no bootstrap broker set (GX5_MQTT_HOST)");
            else up = net && mqtt_connect(b);
            s_abortable = false;
            esp_pm_lock_release(s_awake);

            if (cand) { // commit only a broker that was actually reached
                s_have_cand = false;
                if (up) {
                    s_stored = s_cand;
                    s_have_stored = true;
                    s_fails = 0;
                    save_broker();
                    note = "broker-changed";
                } else if (net) {
                    ESP_LOGW(TAG, "new broker unreachable, keeping the old one");
                    note = "broker-change-failed";
                    retry_now = true;
                }
            } else if (b == &s_stored) {
                if (up) s_fails = 0;
                else if (net && s_fails < 255) s_fails++;
            } else {
                s_fails = 0; // bootstrap tried: give the stored broker another round
            }

            if (up) {
                ESP_LOGI(TAG, "MQTT connected to %s%s", b->host, b == &s_boot ? " (bootstrap)" : "");
                backoff_s = 5;
                s_provisioning = b == &s_boot;
                s_connected = s_session = true;
                if (note) announce(note);
                else if (s_provisioning) announce(why_here());
                note = NULL;
                esp_event_post(GX5_EVENT, GX5_EV_MQTT_CONNECTED, NULL, 0, portMAX_DELAY);
                serve();
                s_connected = false;
                esp_event_post(GX5_EVENT, GX5_EV_MQTT_DISCONNECTED, NULL, 0, portMAX_DELAY);
            }
        }
        if (s_stop) {
            park();
            xSemaphoreGive(s_parked);
            vTaskSuspend(NULL); // deep sleep follows
        }
        if (s_have_cand || retry_now) continue;         // broker change: act on it now
        // Retry later. Only a message (M_STOP, M_RECONNECT) cuts it short; UART
        // events left over from bringup must not, or this loop spins.
        TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(backoff_s * 1000);
        for (TickType_t now; (now = xTaskGetTickCount()) < end && wait_any(&m, end - now) != 'm';) {}
        if (backoff_s < 60) backoff_s *= 2;
    }
}

static void ri_isr(void *arg)
{
    static const msg_t ri = {.type = M_RI};
    BaseType_t woken = pdFALSE;
    gpio_intr_disable(GX5_PIN_GSM_INT); // level interrupt: re-armed by serve()
    xQueueSendFromISR(s_msgq, &ri, &woken);
    if (woken) portYIELD_FROM_ISR();
}

bool gx5_mqtt_publish(const char *subtopic, const char *payload, bool retain)
{
    msg_t m = {.type = M_PUB, .retain = retain};
    if (!s_connected) return false;
    // Unprovisioned, only discovery/<id> may be published: each telemetry tick becomes
    // a repeat of the announcement (serve) and everything else is dropped.
    if (s_provisioning && strcmp(subtopic, "telemetry")) return false;
    strlcpy(m.topic, subtopic, sizeof m.topic);
    strlcpy(m.data, payload, sizeof m.data);
    return xQueueSend(s_msgq, &m, pdMS_TO_TICKS(2000)) == pdTRUE; // brief wait rides out a burst
}

bool gx5_mqtt_provisioning(void) { return s_provisioning; }

void gx5_ota_start(const char *version)
{
    msg_t m = {.type = M_OTA};
    strlcpy(m.data, version, sizeof m.data);
    if (s_connected) xQueueSend(s_msgq, &m, pdMS_TO_TICKS(1000));
}

bool gx5_ota_busy(void) { return s_ota; }

void gx5_broker_set(const gx5_broker_t *b)
{
    msg_t m = {.type = M_RECONNECT};
    s_cand = *b;
    if (!s_cand.host[0]) { // a login only: it is for the bootstrap broker itself
        strcpy(s_cand.host, s_boot.host);
        s_cand.port = s_boot.port;
        s_cand.tls = s_boot.tls;
    }
    s_have_cand = true; // set last: the modem task reads s_cand only after this
    xQueueSend(s_msgq, &m, pdMS_TO_TICKS(1000));
}

void gx5_location_set(const char *loc, bool reconnect)
{
    msg_t m = {.type = M_RECONNECT};
    nvs_handle_t h;
    if (nvs_open("gx5", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_str(h, "loc", loc) != ESP_OK || nvs_commit(h) != ESP_OK) ESP_LOGE(TAG, "location not saved");
    nvs_close(h);
    s_loc_dirty = true; // the modem task picks it up before its next connect
    if (reconnect) xQueueSend(s_msgq, &m, pdMS_TO_TICKS(1000));
}

void gx5_modem_stop(bool power_off)
{
    msg_t m = {.type = M_STOP};
    s_stop = power_off ? 2 : 1;
    xQueueSend(s_msgq, &m, pdMS_TO_TICKS(1000));
    if (!xSemaphoreTake(s_parked, pdMS_TO_TICKS(20000))) ESP_LOGE(TAG, "modem park timed out");
}

void gx5_modem_start(const char *device_id)
{
    s_id = device_id;
    load_broker();
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART, 2048, 0, 8, &s_uartq, 0));
    ESP_ERROR_CHECK(uart_param_config(UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART, CONFIG_GX5_MODEM_TX_GPIO, CONFIG_GX5_MODEM_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_msgq = xQueueCreate(8, sizeof(msg_t));
    s_set = xQueueCreateSet(8 + 8);
    s_parked = xSemaphoreCreateBinary();
    xQueueAddToSet(s_uartq, s_set);
    xQueueAddToSet(s_msgq, s_set);
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "modem", &s_awake));

    // RI wakes the MCU from light sleep and tells serve() to stay awake for the URC.
    gpio_config_t ri = {.pin_bit_mask = 1ULL << GX5_PIN_GSM_INT, .mode = GPIO_MODE_INPUT};
    gpio_config(&ri);
    gpio_wakeup_enable(GX5_PIN_GSM_INT, GPIO_INTR_HIGH_LEVEL);
    gpio_isr_handler_add(GX5_PIN_GSM_INT, ri_isr, NULL);
    gpio_intr_disable(GX5_PIN_GSM_INT);

    xTaskCreate(modem_task, "modem", 8192, NULL, 10, NULL);
}
