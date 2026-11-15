// CAN bus: TWAI driver, one task that assembles responses and sends scheduled
// requests, runtime configuration (NVS) and publishing of changed values.
// Protocol logic lives in canproto.h. Runs only while ignition is on.
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

static gx5_can_cfg_t s_cfg = {.mode = GX5_CAN_LISTEN, .bitrate = CONFIG_GX5_CAN_BITRATE};
static gx5_can_state_t s_st;
static SemaphoreHandle_t s_lock; // guards s_cfg and s_st
static QueueHandle_t s_rxq;
static twai_node_handle_t s_node;
static volatile uint32_t s_frames;
static int64_t s_last[GX5_CAN_REQS], s_busy_until; // ms

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

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

// The driver transmits asynchronously from the pointer it is given, so frames
// live in a small static ring.
static void can_tx(const gx5_frame_t *f)
{
    static twai_frame_t ring[8];
    static uint8_t data[8][8];
    static int i;
    if (!s_node || s_cfg.mode < GX5_CAN_OBD2) return;
    i = (i + 1) % 8;
    memcpy(data[i], f->data, 8);
    ring[i] = (twai_frame_t){.header = {.id = f->id, .dlc = f->dlc, .ide = f->ext}, .buffer = data[i], .buffer_len = f->dlc};
    if (twai_node_transmit(s_node, &ring[i], 0) != ESP_OK) ESP_LOGD(TAG, "tx %lX failed", (unsigned long)f->id);
}

// Send at most one due request. OBD-II is one request at a time: the next waits
// for the response to complete, or 300 ms.
static void schedule(void)
{
    int64_t now = now_ms();
    if (now < s_busy_until) return;
    for (int i = 0; i < GX5_CAN_REQS; i++) {
        const gx5_can_req_t *r = &s_cfg.req[i];
        if (!r->interval_s || (s_last[i] && now - s_last[i] < r->interval_s * 1000LL)) continue;
        can_tx(&r->frame);
        s_last[i] = now;
        if (s_cfg.mode == GX5_CAN_OBD2) s_busy_until = now + 300;
        return;
    }
}

static void can_task(void *arg)
{
    gx5_frame_t f, reply;
    for (;;) {
        bool got = xQueueReceive(s_rxq, &f, s_node ? pdMS_TO_TICKS(100) : portMAX_DELAY);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (got && s_cfg.mode == GX5_CAN_OBD2) {
            bool done;
            if (gx5_obd_feed(&s_st, &f, &reply, &done)) can_tx(&reply);
            if (done) s_busy_until = 0;
        } else if (got && s_cfg.mode == GX5_CAN_J1939) {
            if (gx5_j1939_feed(&s_st, &s_cfg, &f, &reply)) can_tx(&reply);
        }
        schedule();
        xSemaphoreGive(s_lock);
    }
}

void gx5_can_stop(void)
{
    if (CONFIG_GX5_CAN_EN_GPIO >= 0) gpio_set_level(CONFIG_GX5_CAN_EN_GPIO, 1); // transceiver standby
    if (!s_node) return;
    twai_node_disable(s_node); // also releases the driver's no-light-sleep lock
    twai_node_delete(s_node);
    s_node = NULL;
}

void gx5_can_start(void)
{
    gx5_frame_t wake = {0}, claim;
    if (s_node || s_cfg.mode == GX5_CAN_OFF) return;
    twai_onchip_node_config_t cfg = {
        .io_cfg = {.tx = GX5_PIN_CAN_TX, .rx = GX5_PIN_CAN_RX, .quanta_clk_out = -1, .bus_off_indicator = -1},
        .bit_timing.bitrate = s_cfg.bitrate,
        .tx_queue_depth = 8,
        .flags.enable_listen_only = s_cfg.mode == GX5_CAN_LISTEN, // listen: never drive or acknowledge
    };
    twai_event_callbacks_t cbs = {.on_rx_done = can_rx};

    memset(s_st.sess, 0, sizeof s_st.sess);
    memset(s_last, 0, sizeof s_last);
    s_busy_until = 0;
    if (CONFIG_GX5_CAN_EN_GPIO >= 0) gpio_set_level(CONFIG_GX5_CAN_EN_GPIO, 0); // RS low = transceiver active
    if (twai_new_node_onchip(&cfg, &s_node) != ESP_OK || twai_node_register_event_callbacks(s_node, &cbs, NULL) != ESP_OK ||
        twai_node_enable(s_node) != ESP_OK) {
        ESP_LOGW(TAG, "not started");
        gx5_can_stop();
        return;
    }
    if (s_cfg.mode == GX5_CAN_J1939) {
        gx5_j1939_claim(&claim);
        can_tx(&claim);
    }
    xQueueSend(s_rxq, &wake, 0); // get the task out of its idle wait
}

uint32_t gx5_can_frames(void) { return s_frames; }

static void save_cfg(void)
{
    nvs_handle_t h;
    if (nvs_open("gx5", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, "can", &s_cfg, sizeof s_cfg) != ESP_OK || nvs_commit(h) != ESP_OK) ESP_LOGE(TAG, "config not saved");
    nvs_close(h);
}

// {"can":{..}} sets mode/bitrate/filter, {"can_req":{..}} sets or clears a request
// slot. Stored in NVS and applied at once. False if the message is neither.
bool gx5_can_command(const char *json)
{
    gx5_can_cfg_t cfg;
    gx5_can_req_t req;
    bool running = s_node != NULL;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    cfg = s_cfg;
    xSemaphoreGive(s_lock);
    int slot = gx5_parse_can_req(json, &req);
    if (slot >= 0) cfg.req[slot] = req;
    else if (!gx5_parse_can_cfg(json, &cfg)) return false;

    gx5_can_stop();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = cfg;
    if (slot < 0) memset(s_st.val, 0, sizeof s_st.val); // another protocol: old values mean nothing
    xSemaphoreGive(s_lock);
    save_cfg();
    ESP_LOGI(TAG, "config: mode=%d bitrate=%lu filter=%d", cfg.mode, (unsigned long)cfg.bitrate, cfg.filter);
    if (running) gx5_can_start();
    return true;
}

// Hand values that changed since last time to `send`, as
// {"ts":..,"obd":["7E8 01 0C 1AF8",..]} or {"ts":..,"j1939":["FEEE 00 8C7D..",..]},
// split over as many messages as needed.
void gx5_can_publish(bool (*send)(const char *json))
{
    char msg[GX5_PUB_MAX];
    time_t now = time(NULL);
    for (bool more = true; more;) {
        xSemaphoreTake(s_lock, portMAX_DELAY); // held only while formatting, not while publishing
        int items = gx5_can_chunk(&s_st, s_cfg.mode == GX5_CAN_J1939, now > 1700000000 ? (long long)now : 0LL, msg, sizeof msg, &more);
        xSemaphoreGive(s_lock);
        if (!items || !send(msg)) return;
    }
}

void gx5_can_init(void)
{
    nvs_handle_t h;
    gx5_can_cfg_t cfg;
    size_t len = sizeof cfg;
    if (nvs_open("gx5", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_blob(h, "can", &cfg, &len) == ESP_OK && len == sizeof cfg && cfg.mode <= GX5_CAN_J1939) s_cfg = cfg;
        nvs_close(h);
    }
    s_lock = xSemaphoreCreateMutex();
    s_rxq = xQueueCreate(64, sizeof(gx5_frame_t));
    xTaskCreate(can_task, "can", 4096, NULL, 8, NULL);
}
#else
void gx5_can_init(void) {}
void gx5_can_start(void) {}
void gx5_can_stop(void) {}
uint32_t gx5_can_frames(void) { return 0; }
bool gx5_can_command(const char *json) { return false; }
void gx5_can_publish(bool (*send)(const char *json)) {}
#endif
