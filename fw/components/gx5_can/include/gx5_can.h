// gx5_can — CAN bus (OBD-II / J1939), configured at runtime and kept in NVS.
// Protocol logic is in gx5_proto/canproto.h. Needs nvs_flash_init() before gx5_can_init().
#pragma once
#include <stdbool.h>
#include <stdint.h>

void gx5_can_init(void);
void gx5_can_start(void);                // blocks light sleep while running
void gx5_can_stop(void);
uint32_t gx5_can_frames(void);
bool gx5_can_command(const char *json);  // {"can":..} / {"can_req":..}; false if neither
// Hand changed values to `send` as JSON messages ({"ts":..,"obd":[..]} or "j1939").
void gx5_can_publish(bool (*send)(const char *json));
