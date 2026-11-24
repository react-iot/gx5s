// gx5_can — CAN bus (OBD-II / J1939) and the vehicle profile: a library of requests
// and decoded signals, a request list per state and the ignition rule, set at
// runtime and kept in NVS. Protocol logic and the message formats are in
// gx5_proto/canproto.h. Needs nvs_flash_init() before gx5_can_init().
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "canproto.h"

void gx5_can_init(void);
void gx5_can_state(int state);           // GX5_ST_*: run the bus as the profile says for it; blocks light sleep while it runs
void gx5_can_stop(void);
uint32_t gx5_can_frames(void);
bool gx5_can_command(const char *json);  // {"vehicle":..} / {"pid":..} / {"sig":..} / {"can":..} / {"profile":"clear"}; false if none
// Hand changed values to `send` as JSON messages ({"ts":..,"sig":{..}}, "obd":[..] or "j1939":[..]).
void gx5_can_publish(bool (*send)(const char *json));
void gx5_can_profile(gx5_ign_cfg_t *ign, uint16_t *pre_s, bool *wake_vcc); // what the application needs of the profile
// With a "sig" or "frame" ignition rule the component posts GX5_EV_CAN_IGN (data: bool) when it changes.
// Ask the vehicle for its VIN (the profile's "vin_pid") until it answers; the answer
// arrives as GX5_EV_VIN. Needs a state whose mode transmits.
void gx5_can_vin_check(void);
// The profile's trip signals as JSON members, each with a leading comma
// (,"odo_km":78882,"fuel_l":10.5): their last good values, for the trip events.
void gx5_can_trip(char *out, size_t size);
