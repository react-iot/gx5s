// gx5 motion — gyro + accelerometer fusion for driving events. Pure logic (no
// ESP-IDF), host-tested in fw/test/test_motion.c. The board may be mounted any way up.
//
//  1. Calibrate at ignition on, vehicle at rest: 1 s of quiet samples gives gravity
//     (so "up", and the accelerometer offset with it) and the gyro bias.
//  2. Track gravity: rotate it by the gyro every sample (hills, pitch under braking)
//     and pull it slowly back to the accelerometer (complementary filter).
//  3. Horizontal acceleration = accel - gravity, minus its vertical part.
//     Yaw rate = gyro . up; positive = anticlockwise from above = left turn.
//  4. Turns: yaw rate plus horizontal acceleration; side from the yaw sign.
//  5. Accelerate / brake need "forward". It is learned from turns: in a turn the
//     car accelerates toward the inside, so that direction (flipped for right turns)
//     is "left", and forward = left x up. Until then a straight-line event is
//     reported as GX5_MOT_HARSH (direction unknown).
#pragma once
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define GX5_MOT_HZ        50
#define GX5_MOT_MIN_TICKS 15   // an event must last 0.3 s
#define GX5_MOT_YAW_DPS   12   // above this the vehicle is turning
#define GX5_MOT_LEARN_MG  150  // turn acceleration worth learning from
#define GX5_MOT_LEARNED   (50 * 250.0f) // about 1 s of solid turning (samples x mg)

typedef enum { GX5_MOT_NONE, GX5_MOT_CALIBRATED, GX5_MOT_ACCEL, GX5_MOT_BRAKE, GX5_MOT_LEFT, GX5_MOT_RIGHT, GX5_MOT_HARSH } gx5_mot_ev_t;

typedef struct {
    int accel_mg, brake_mg, turn_mg; // thresholds
} gx5_mot_cfg_t;

typedef struct {
    int n;                 // calibration: samples in the current window
    float sum[6], lo[6], hi[6];
    bool cal;
    float g[3], bias[3];   // gravity (mg) and gyro bias (deg/s), sensor frame
    float left[3];         // learned: sum of "left" directions seen in turns (persist this)
    float ah[3], yaw;      // filtered horizontal acceleration (mg) and yaw rate (deg/s)
    int kind, ticks;       // event in progress
    float peak;
} gx5_mot_t;

static inline float gx5_dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static inline void gx5_cross(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

static inline const char *gx5_mot_name(gx5_mot_ev_t e)
{
    static const char *const n[] = {"none", "calibrated", "accel", "brake", "left", "right", "harsh"};
    return n[e];
}

// Start over (ignition on). `left` is what an earlier drive learned, or NULL.
static inline void gx5_mot_reset(gx5_mot_t *m, const float left[3])
{
    memset(m, 0, sizeof *m);
    if (left) memcpy(m->left, left, sizeof m->left);
}

// Feed one sample (GX5_MOT_HZ per second): acc in mg, gyro in deg/s. Returns
// GX5_MOT_CALIBRATED once, and a driving event when one ends (*peak_mg = its peak).
static inline gx5_mot_ev_t gx5_mot_step(gx5_mot_t *m, const gx5_mot_cfg_t *c, const float acc[3], const float gyro[3], int *peak_mg)
{
    const float dt = 1.0f / GX5_MOT_HZ;

    if (!m->cal) { // 1 s window; any movement in it starts a new one
        for (int i = 0; i < 6; i++) {
            float v = i < 3 ? acc[i] : gyro[i - 3];
            m->sum[i] += v;
            if (!m->n || v < m->lo[i]) m->lo[i] = v;
            if (!m->n || v > m->hi[i]) m->hi[i] = v;
        }
        if (++m->n < GX5_MOT_HZ) return GX5_MOT_NONE;
        bool quiet = true;
        for (int i = 0; i < 6; i++) quiet &= m->hi[i] - m->lo[i] < (i < 3 ? 60 : 3);
        for (int i = 0; i < 3; i++) {
            m->g[i] = m->sum[i] / m->n;
            m->bias[i] = m->sum[i + 3] / m->n;
        }
        m->n = 0;
        memset(m->sum, 0, sizeof m->sum);
        if (!quiet || gx5_dot(m->g, m->g) < 500 * 500) return GX5_MOT_NONE;
        m->cal = true;
        return GX5_MOT_CALIBRATED;
    }

    float w[3], wr[3], d[3], a[3], up[3];
    for (int i = 0; i < 3; i++) {
        w[i] = gyro[i] - m->bias[i];
        wr[i] = w[i] * (float)M_PI / 180;
    }
    // Gravity is fixed in the world, so in the sensor frame it turns against the body: dg/dt = g x w.
    gx5_cross(m->g, wr, d);
    for (int i = 0; i < 3; i++) m->g[i] += d[i] * dt;
    // Accelerometer correction: quick while nothing is happening, slow otherwise, so a
    // few seconds of braking barely leak into the gravity estimate.
    for (int i = 0; i < 3; i++) a[i] = acc[i] - m->g[i];
    float k = dt / (gx5_dot(a, a) < 100 * 100 ? 2.0f : 30.0f);
    for (int i = 0; i < 3; i++) {
        m->g[i] += k * a[i];
        a[i] = acc[i] - m->g[i];
    }

    float gn = sqrtf(gx5_dot(m->g, m->g));
    for (int i = 0; i < 3; i++) up[i] = m->g[i] / gn;
    float av = gx5_dot(a, up);
    for (int i = 0; i < 3; i++) m->ah[i] += 0.2f * (a[i] - av * up[i] - m->ah[i]); // ~0.1 s low-pass
    m->yaw += 0.2f * (gx5_dot(w, up) - m->yaw);

    float mag = sqrtf(gx5_dot(m->ah, m->ah));
    bool turning = fabsf(m->yaw) > GX5_MOT_YAW_DPS;
    if (turning && mag > GX5_MOT_LEARN_MG) // inside of the turn = left (yaw > 0) or right
        for (int i = 0; i < 3; i++) m->left[i] += m->yaw > 0 ? m->ah[i] : -m->ah[i];

    int kind = GX5_MOT_NONE;
    float level = mag;
    if (turning) {
        if (mag > c->turn_mg) kind = m->yaw > 0 ? GX5_MOT_LEFT : GX5_MOT_RIGHT;
    } else if (gx5_dot(m->left, m->left) > GX5_MOT_LEARNED * GX5_MOT_LEARNED) {
        float fwd[3];
        gx5_cross(m->left, up, fwd); // left x up = forward
        float lon = gx5_dot(m->ah, fwd) / sqrtf(gx5_dot(fwd, fwd));
        level = fabsf(lon);
        if (lon > c->accel_mg) kind = GX5_MOT_ACCEL;
        else if (lon < -c->brake_mg) kind = GX5_MOT_BRAKE;
    } else if (mag > (c->accel_mg < c->brake_mg ? c->accel_mg : c->brake_mg)) {
        kind = GX5_MOT_HARSH;
    }

    gx5_mot_ev_t ended = GX5_MOT_NONE;
    if (kind == m->kind) {
        m->ticks++;
        if (level > m->peak) m->peak = level;
    } else {
        if (m->kind != GX5_MOT_NONE && m->ticks >= GX5_MOT_MIN_TICKS) {
            ended = (gx5_mot_ev_t)m->kind;
            *peak_mg = (int)m->peak;
        }
        m->kind = kind;
        m->ticks = 1;
        m->peak = level;
    }
    return ended;
}
