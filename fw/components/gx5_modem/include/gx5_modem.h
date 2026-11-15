// gx5_modem — Quectel BG96: AT engine, network bring-up, the modem's own MQTT client,
// broker provisioning (NVS) and GNSS. Posts GX5_EV_MQTT_* and GX5_EV_TIME.
// Needs nvs_flash_init() and the default event loop before gx5_modem_start().
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "parse.h"

void gx5_modem_start(const char *device_id);
// Queue a publish to <prefix>/<device-id>/<subtopic>. retain => QoS 1 retained.
bool gx5_mqtt_publish(const char *subtopic, const char *payload, bool retain);
// Copy the modem's last GNSS fix as a JSON fragment (,"lat":..,"lon":..,"kmh":..,"cog":..,"sat":..)
// into out; returns its length, 0 if there was no fix at the last look or no room.
size_t gx5_gnss_fix(char *out, size_t size);
// True while connected to the bootstrap (provisioning) broker.
bool gx5_mqtt_provisioning(void);
// Try a new broker now; it is stored only if the connection succeeds.
void gx5_broker_set(const gx5_broker_t *b);
// Blocks until the modem is parked for SLEEP (registered, QSCLK=1) or powered off.
void gx5_modem_stop(bool power_off);
