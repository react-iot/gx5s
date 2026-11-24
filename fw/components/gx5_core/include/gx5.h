// gx5_core — what every component shares: the board description (pins, expander
// map) and the event bus. Architecture: docs/adr/0001-base-firmware.md
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Pins. Where docs/gx5s-board-spec.md disagrees (SD CS/CLK), these are right (bench).
// GPIO34-39 are input-only.
#define GX5_PIN_ADC_VCC 36 // ADC1_CH0, harness supply through divider
#define GX5_PIN_BAT_ADC 35 // ADC1_CH7, LiPo / 2
#define GX5_PIN_ACC_INT 34 // LOW = ignition on (inverted by transistor stage)
#define GX5_PIN_GSM_INT 25 // modem RI, inverted: HIGH = active
#define GX5_PIN_GSM_RST 17 // modem PERST#, active low
#define GX5_PIN_I2C_SCL 27
#define GX5_PIN_I2C_SDA 14
#define GX5_PIN_3V3_PSM 15 // regulator mode: HIGH = normal, LOW = power save (boot-log strap)
#define GX5_PIN_SD_3V3  2  // HIGH = SD NAND rail on (strap: low at reset)
#define GX5_PIN_VCC_INT 39 // LOW = harness supply present
#define GX5_PIN_APS_INT 12 // MPU-6050 INT, HIGH = motion (strap: low at reset)
#define GX5_PIN_CAN_RX  32
#define GX5_PIN_CAN_TX  33
#define GX5_PIN_SD_CS   18
#define GX5_PIN_SD_CLK  5
#define GX5_PIN_SD_MISO 16
#define GX5_PIN_SD_MOSI 4

// TCA9555 @0x20 outputs (docs/components/gpio-expander-tca9555.md).
#define GX5_EXP_CHRG_EN 0x0020 // P05, high = charge the LiPo
#define GX5_EXP_GSM_SLP 0x0800 // P13 = modem DTR (mPCIe pin 31): low = awake, high = may sleep (QSCLK=1)
#define GX5_EXP_LED_SVR 0x0400 // P12, green "connected"
#define GX5_EXP_LED_IG  0x1000 // P14, yellow "ignition"
#define GX5_EXP_LED_PWR 0x2000 // P15, red "running"
#define GX5_EXP_APS_RST 0x8000 // P17, low = MPU-6050 held in reset
#define GX5_EXP_OUTPUTS (GX5_EXP_CHRG_EN | GX5_EXP_GSM_SLP | GX5_EXP_LED_SVR | GX5_EXP_LED_IG | GX5_EXP_LED_PWR | GX5_EXP_APS_RST)

// Components never call each other: they post here (default event loop) and the
// application in main/ decides what happens.
ESP_EVENT_DECLARE_BASE(GX5_EVENT);
enum {
    GX5_EV_ACC_EDGE,          // ACC-INT changed level (from ISR)
    GX5_EV_ACC_SETTLED,       // debounce elapsed
    GX5_EV_VCC_TICK,          // time to sample the supply
    GX5_EV_TELEMETRY,         // time to publish telemetry
    GX5_EV_SLEEP_DUE,         // ignition-off awake window elapsed
    GX5_EV_MQTT_CONNECTED,
    GX5_EV_MQTT_DISCONNECTED,
    GX5_EV_MQTT_DATA,         // data: gx5_msg_t
    GX5_EV_TIME,              // network time, data: int64_t UTC epoch
    GX5_EV_MOTION,            // driving event or calibration done, data: gx5_motion_t
    GX5_EV_EXT_RX,            // line from the external UART, data: char[]
    GX5_EV_CAN_IGN,           // ignition as the vehicle profile's CAN rule sees it, data: bool
    GX5_EV_VIN,               // the vehicle answered the VIN request, data: char[18]
};

typedef struct {
    int kind; // gx5_mot_ev_t (motion.h)
    int mg;   // peak acceleration of the event
} gx5_motion_t;

// Debug mode: {"debug":true} over MQTT or USB, kept in NVS. Adds the detail needed to
// work out what a vehicle or the modem is doing: CAN frames, answers, ignition
// decisions, supply samples, wake sources, every modem command.
extern bool gx5_debug;
#define GX5_DBG(tag, fmt, ...) do { if (gx5_debug) ESP_LOGI(tag, "dbg " fmt, ##__VA_ARGS__); } while (0)

#define GX5_PUB_MAX 256 // largest payload the device publishes

typedef struct {
    char topic[64];
    char data[512]; // fits the provisioning message (broker + config + PIN)
} gx5_msg_t;
