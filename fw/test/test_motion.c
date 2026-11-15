// Host check for motion.h:  cc -Icomponents/gx5_proto/include test/test_motion.c -lm -o test/test_motion && test/test_motion
// A vehicle (x forward, y left, z up) carries the sensor in an arbitrary orientation.
#include <assert.h>
#include <stdio.h>
#include "motion.h"

static float R[3][3]; // sensor = R * vehicle
static gx5_mot_t m;
static const gx5_mot_cfg_t cfg = {.accel_mg = 300, .brake_mg = 400, .turn_mg = 400};
static const float BIAS[3] = {-2.2f, 0.9f, -1.7f}; // gyro bias seen on the bench board
static const float OFFS[3] = {40, -25, 160};       // accelerometer offset, sensor frame

static void to_sensor(const float v[3], float s[3])
{
    for (int i = 0; i < 3; i++) s[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
}

// Drive for `secs`: linear acceleration (fwd, left) in mg, yaw rate in deg/s (+ = left),
// pitch rate in deg/s (nose up). Returns the last event seen, peak in *mg.
static gx5_mot_ev_t drive(float secs, float fwd, float left, float yaw, float pitch_rate, int *mg)
{
    static float pitch; // degrees, nose up
    gx5_mot_ev_t last = GX5_MOT_NONE;
    for (int i = 0; i < secs * GX5_MOT_HZ; i++) {
        pitch += pitch_rate / GX5_MOT_HZ;
        float p = pitch * (float)M_PI / 180;
        // Specific force in the vehicle frame: gravity reaction tilts back as the nose rises.
        float f[3] = {fwd + 1000 * sinf(p), left, 1000 * cosf(p)}, w[3] = {0, -pitch_rate, yaw}, a[3], g[3];
        to_sensor(f, a);
        to_sensor(w, g);
        for (int k = 0; k < 3; k++) {
            a[k] += OFFS[k] + (i % 2 ? 4 : -4); // offset + a little vibration
            g[k] += BIAS[k];
        }
        int peak = 0;
        gx5_mot_ev_t e = gx5_mot_step(&m, &cfg, a, g, &peak);
        if (e != GX5_MOT_NONE) {
            last = e;
            if (mg) *mg = peak;
        }
    }
    return last;
}

int main(void)
{
    // Sensor mounted tilted and rotated: rotate 40 deg about z, then 25 deg about x.
    float cz = cosf(0.7f), sz = sinf(0.7f), cx = cosf(0.44f), sx = sinf(0.44f);
    float Rz[3][3] = {{cz, -sz, 0}, {sz, cz, 0}, {0, 0, 1}}, Rx[3][3] = {{1, 0, 0}, {0, cx, -sx}, {0, sx, cx}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) R[i][j] = Rx[i][0] * Rz[0][j] + Rx[i][1] * Rz[1][j] + Rx[i][2] * Rz[2][j];

    int mg = 0;
    gx5_mot_reset(&m, NULL);
    assert(drive(0.5f, 200, 0, 0, 0, NULL) == GX5_MOT_NONE && !m.cal); // moving: no calibration yet
    drive(0.5f, 0, 0, 0, 0, NULL);                                     // rest of that window
    assert(drive(1.0f, 0, 0, 0, 0, NULL) == GX5_MOT_CALIBRATED);       // a quiet second
    assert(drive(5, 0, 0, 0, 0, NULL) == GX5_MOT_NONE);                // parked: nothing

    drive(1.0f, 450, 0, 0, 0, NULL);                                   // straight, forward unknown yet
    assert(drive(1, 0, 0, 0, 0, &mg) == GX5_MOT_HARSH && mg > 380 && mg < 480);

    drive(2, 0, 500, 25, 0, NULL);                                     // left turn
    assert(drive(1, 0, 0, 0, 0, &mg) == GX5_MOT_LEFT && mg > 430 && mg < 540);
    drive(2, 0, -500, -25, 0, NULL);                                   // right turn
    assert(drive(1, 0, 0, 0, 0, NULL) == GX5_MOT_RIGHT);
    assert(drive(1, 0, 150, 20, 0, NULL) == GX5_MOT_NONE);             // gentle turn

    drive(1.5f, 380, 0, 0, 0, NULL);                                   // forward is known now
    assert(drive(1, 0, 0, 0, 0, &mg) == GX5_MOT_ACCEL && mg > 320 && mg < 400);
    drive(1.5f, -550, 0, 0, 0, NULL);
    assert(drive(1, 0, 0, 0, 0, &mg) == GX5_MOT_BRAKE && mg > 470 && mg < 580);
    assert(drive(1.5f, -350, 0, 0, 0, NULL) == GX5_MOT_NONE);          // firm, under the brake threshold
    assert(drive(1, 0, 0, 0, 0, NULL) == GX5_MOT_NONE);
    assert(drive(1.5f, 0.1f * 1000, 0, 0, 0, NULL) == GX5_MOT_NONE);   // normal driving

    // Onto a 12 degree hill in 3 s: the gyro keeps "up" right, so no event and no
    // phantom horizontal acceleration (it would be ~200 mg from the accelerometer alone).
    assert(drive(3, 0, 0, 0, 4, NULL) == GX5_MOT_NONE);
    assert(sqrtf(gx5_dot(m.ah, m.ah)) < 40);
    drive(1.5f, -550, 0, 0, 0, NULL);                                  // braking on the hill still counts
    assert(drive(1, 0, 0, 0, 0, NULL) == GX5_MOT_BRAKE);

    // Next ignition on (still on the hill): what was learned carries over.
    float learned[3];
    memcpy(learned, m.left, sizeof learned);
    gx5_mot_reset(&m, learned);
    assert(drive(1.0f, 0, 0, 0, 0, NULL) == GX5_MOT_CALIBRATED);
    drive(1.5f, 380, 0, 0, 0, NULL);
    assert(drive(1, 0, 0, 0, 0, NULL) == GX5_MOT_ACCEL);

    puts("ok");
    return 0;
}
