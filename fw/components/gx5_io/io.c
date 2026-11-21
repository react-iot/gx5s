// gx5_io — devices on the I2C bus: TCA9555 expander, MPU-6050 motion wake, PCF8563 RTC.
// Each is optional at runtime: one that does not answer is logged once and skipped.
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gx5.h"
#include "gx5_io.h"
#include "motion.h"
#include "parse.h"

static const char *TAG = "io";

// ── I2C ─────────────────────────────────────────────────────────────────────

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_exp, s_imu, s_rtc;

static i2c_master_dev_handle_t i2c_dev(uint8_t addr)
{
    i2c_device_config_t cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 100000};
    i2c_master_dev_handle_t h = NULL;
    if (s_bus) i2c_master_bus_add_device(s_bus, &cfg, &h);
    return h;
}

#define I2C_TRIES 3

// One transfer, retried: the harness shares this bus, so another master (the bench Pi)
// or noise can spoil a single transfer. A device that fails I2C_TRIES times in a row
// is dropped for this boot (no log spam).
static bool xfer(i2c_master_dev_handle_t *d, const char *name, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    if (!*d) return false;
    for (int i = 0; i < I2C_TRIES; i++) {
        if (i) vTaskDelay(pdMS_TO_TICKS(10));
        if ((r ? i2c_master_transmit_receive(*d, w, wn, r, rn, 50) : i2c_master_transmit(*d, w, wn, 50)) == ESP_OK) return true;
    }
    ESP_LOGW(TAG, "%s not responding, disabled", name);
    *d = NULL;
    return false;
}

static bool wr(i2c_master_dev_handle_t *d, const char *name, const uint8_t *b, size_t n)
{
    return xfer(d, name, b, n, NULL, 0);
}

static bool rd(i2c_master_dev_handle_t *d, const char *name, uint8_t reg, uint8_t *b, size_t n)
{
    return xfer(d, name, &reg, 1, b, n);
}

// Every address that answers, as " 0x20 0x51 ...". The harness I2C pins are this same
// bus, so an external sensor shows up here next to the on-board devices.
void gx5_i2c_scan(char *out, size_t size)
{
    size_t n = 0;
    out[0] = '\0';
    for (int a = 0x08; s_bus && a < 0x78 && n + 6 < size; a++)
        if (i2c_master_probe(s_bus, a, 20) == ESP_OK) n += snprintf(out + n, size - n, " 0x%02x", a);
}

// ── TCA9555 expander ────────────────────────────────────────────────────────

static uint16_t s_exp_out;

static void exp_write(uint8_t reg, uint16_t v)
{
    uint8_t b[3] = {reg, v & 0xff, v >> 8};
    wr(&s_exp, "TCA9555", b, sizeof b);
}

void gx5_exp_set(uint16_t mask, bool on)
{
    uint16_t out = on ? s_exp_out | mask : s_exp_out & ~mask;
    if (out == s_exp_out) return;
    s_exp_out = out;
    exp_write(0x02, s_exp_out);
}

bool gx5_charging(void)
{
    uint8_t in[2];
    return rd(&s_exp, "TCA9555", 0x00, in, 2) && !(in[0] & 0x10); // P04 = TP4056 CHRG#, low while charging
}

// ── MPU-6050: wake on motion ────────────────────────────────────────────────

// Reading INT_STATUS releases the latched APS-INT line.
void gx5_imu_arm(void)
{
    uint8_t st;
    rd(&s_imu, "MPU-6050", 0x3A, &st, 1);
}

// Acceleration in mg (+-8 g range, 4096 counts/g), die temperature in 0.1 degC and
// rotation rate in 0.1 deg/s (+-250 deg/s range, 131 counts per deg/s; zero while
// the gyro is in standby). Also releases a latched motion interrupt, like any read.
// False without an IMU.
bool gx5_imu_read(int mg[3], int *temp_dc, int ddps[3])
{
    uint8_t b[14];
    if (!rd(&s_imu, "MPU-6050", 0x3B, b, sizeof b)) return false; // ACCEL_XOUT_H .. GYRO_ZOUT_L
    for (int i = 0; i < 3; i++) {
        mg[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]) * 1000 / 4096;
        ddps[i] = (int16_t)(b[8 + 2 * i] << 8 | b[9 + 2 * i]) * 10 / 131;
    }
    *temp_dc = (int16_t)(b[6] << 8 | b[7]) / 34 + 365; // degC = raw / 340 + 36.53
    return true;
}

// ── Driving events: gyro + accelerometer fusion at 50 Hz while the gyro is on ──

// "Left" as learned from turns (motion.h); with it accelerating and braking can be
// told apart from the first metre of the next drive.
// ponytail: RTC memory survives deep sleep but not a power loss; the first few
// turns relearn it. Move to NVS if that gap matters.
static RTC_DATA_ATTR float s_left[3];
static TaskHandle_t s_motion;
static volatile bool s_gyro_on;

static void motion_task(void *arg)
{
    static const gx5_mot_cfg_t cfg = {CONFIG_GX5_HARSH_ACCEL_MG, CONFIG_GX5_HARSH_BRAKE_MG, CONFIG_GX5_HARSH_TURN_MG};
    static gx5_mot_t m;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // gyro switched on: calibrate again
        gx5_mot_reset(&m, s_left);
        TickType_t t = xTaskGetTickCount();
        int mg[3], temp, w[3];
        while (s_gyro_on && gx5_imu_read(mg, &temp, w)) {
            float a[3] = {mg[0], mg[1], mg[2]}, g[3] = {w[0] / 10.0f, w[1] / 10.0f, w[2] / 10.0f};
            gx5_motion_t ev = {0};
            int peak = 0;
            ev.kind = gx5_mot_step(&m, &cfg, a, g, &peak);
            if (ev.kind != GX5_MOT_NONE) {
                ev.mg = peak;
                memcpy(s_left, m.left, sizeof s_left);
                esp_event_post(GX5_EVENT, GX5_EV_MOTION, &ev, sizeof ev, 0);
            }
            vTaskDelayUntil(&t, pdMS_TO_TICKS(1000 / GX5_MOT_HZ));
        }
    }
}

// Gyro on (about 3.6 mA) or in standby. On, it needs ~35 ms before its data is valid;
// then driving-event detection starts with a fresh calibration.
void gx5_imu_gyro(bool on)
{
    uint8_t b[2] = {0x6C, on ? 0x00 : 0x07}; // PWR_MGMT_2: gyro X/Y/Z standby bits
    bool was = s_gyro_on;
    s_gyro_on = on && wr(&s_imu, "MPU-6050", b, 2);
    if (!on) wr(&s_imu, "MPU-6050", b, 2);
    if (s_gyro_on && !was) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (!s_motion) xTaskCreate(motion_task, "motion", 4096, NULL, 5, &s_motion);
        xTaskNotifyGive(s_motion);
    }
}

#ifdef CONFIG_GX5_MOTION_WAKE
static void imu_w(uint8_t reg, uint8_t v)
{
    uint8_t b[2] = {reg, v};
    wr(&s_imu, "MPU-6050", b, 2);
}
#endif

static void imu_init(void)
{
#ifdef CONFIG_GX5_MOTION_WAKE
    s_imu = i2c_dev(0x68);
    gx5_imu_arm();      // first: APS-INT is the GPIO12 boot strap, do not leave it high
    imu_w(0x6B, 0x80);  // PWR_MGMT_1: device reset
    vTaskDelay(pdMS_TO_TICKS(100));
    imu_w(0x6B, 0x00);  // wake, internal oscillator
    // ponytail: accel always on, gyro in standby (~0.5 mA). Cycle mode (PWR_MGMT_1
    // CYCLE + LP_WAKE_CTRL) gets to tens of uA; needs threshold re-tuning on the bench.
    imu_w(0x6C, 0x07);  // PWR_MGMT_2: gyro X/Y/Z standby
    imu_w(0x1A, 0x03);  // DLPF 44 Hz
    imu_w(0x1C, 0x10);  // +-8 g, high-pass reset
    vTaskDelay(pdMS_TO_TICKS(50));
    imu_w(0x1C, 0x17);  // +-8 g, high-pass HOLD: compare against the attitude at rest
    imu_w(0x37, 0x30);  // INT active high, push-pull, latched, cleared by any read
    imu_w(0x1F, CONFIG_GX5_MOTION_THRESHOLD);
    imu_w(0x20, 10);    // motion must last 10 ms
    imu_w(0x38, 0x40);  // motion interrupt only
#endif
}

// ── PCF8563 RTC ─────────────────────────────────────────────────────────────

static uint8_t bcd(int v) { return (uint8_t)((v / 10) << 4 | v % 10); }
static int dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0f); }

void gx5_rtc_set(time_t utc)
{
    struct tm t;
    gmtime_r(&utc, &t);
    uint8_t b[8] = {0x02, bcd(t.tm_sec), bcd(t.tm_min), bcd(t.tm_hour), bcd(t.tm_mday), t.tm_wday, bcd(t.tm_mon + 1), bcd(t.tm_year % 100)};
    wr(&s_rtc, "PCF8563", b, sizeof b); // seconds bit 7 = 0 also clears the low-voltage flag
}

// System clock from the RTC, unless the RTC lost power (VL flag) or was never set.
static void rtc_init(void)
{
    uint8_t d[7];
    s_rtc = i2c_dev(0x51);
    if (!rd(&s_rtc, "PCF8563", 0x02, d, sizeof d) || (d[0] & 0x80)) return;
    int mo = dec(d[5] & 0x1f), day = dec(d[3] & 0x3f);
    if (mo < 1 || mo > 12 || day < 1) return;
    struct timeval tv = {.tv_sec = gx5_epoch(2000 + dec(d[6]), mo, day, dec(d[2] & 0x3f), dec(d[1] & 0x7f), dec(d[0] & 0x7f))};
    if (tv.tv_sec <= 1700000000) return;
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "clock from RTC: %lld", (long long)tv.tv_sec);
}

void gx5_i2c_init(void)
{
    i2c_master_bus_config_t bus = {
        .i2c_port = -1,
        .sda_io_num = GX5_PIN_I2C_SDA,
        .scl_io_num = GX5_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus, &s_bus) != ESP_OK) s_bus = NULL;
    s_exp = i2c_dev(0x20);
    s_exp_out = GX5_EXP_APS_RST | GX5_EXP_LED_PWR; // MPU-6050 out of reset, power LED on
    exp_write(0x02, s_exp_out);
    exp_write(0x06, (uint16_t)~GX5_EXP_OUTPUTS); // 0 = output
    rtc_init();
    imu_init();
}
