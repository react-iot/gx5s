// CAN protocol logic (OBD-II and J1939) in plain C with fixed-size state.
// Pure and host-testable:
//   cc -Icomponents/gx5_proto/include test/test_can.c -o test/test_can && test/test_can   (from fw/)
#pragma once
#include <stddef.h>
#include "parse.h"

#define GX5_CAN_VAL_MAX  128  // bytes kept per value (longer responses are truncated)
#define GX5_CAN_VALUES   40   // distinct values tracked
#define GX5_CAN_SESSIONS 4    // multi-frame messages being assembled at once
#define GX5_J1939_ADDR   0xF8 // our J1939 source address

enum { GX5_CAN_OFF, GX5_CAN_LISTEN, GX5_CAN_OBD2, GX5_CAN_J1939 };

typedef struct {
    uint32_t id;
    bool ext; // 29-bit identifier
    uint8_t dlc;
    uint8_t data[8];
} gx5_frame_t;

// ── Vehicle profile ─────────────────────────────────────────────────────────
// A library of requests the vehicle answers and of named numbers in the answers,
// and one request list per state. Requests are raw frames:
//   OBD-II:  id 7E0, data 02010C           (engine speed)
//   UDS:     id 7C6, data 0322B002         (Kia cluster: fuel, odometer)
//   J1939:   id 18EA00F8, data EEFE00      (request PGN FEEE from address 00)
#define GX5_CAN_CMDS 32 // library: requests
#define GX5_CAN_SIGS 48 // library: decoded numbers
#define GX5_CAN_POLL 16 // requests per state
#define GX5_KEY_LEN  12

enum { GX5_ST_PRE, GX5_ST_ON, GX5_ST_OFF, GX5_ST_N }; // awake before ignition, ignition on, awake after ignition off
enum { GX5_IGN_ACC, GX5_IGN_VCC, GX5_IGN_SIG, GX5_IGN_FRAME }; // where "ignition on" comes from

typedef struct {
    uint8_t cmd; // library request
    uint16_t ds; // every this many 0.1 s; 0 = empty
} gx5_can_poll_t;

// A number cut out of an answer, described the way OBDb signal sets do
// (github.com/OBDb): `len` bits at bit `bix` of the data after the PID, bit 0 being
// the top bit of the first byte; value = raw * mul + add.
#define GX5_SIG_SIGNED 1
#define GX5_SIG_LE     2 // whole bytes, least significant first (OBDb "blsb")
typedef struct {
    char key[GX5_KEY_LEN]; // published name; "" = empty
    uint8_t cmd, len, flags;
    uint16_t bix;
    float mul, add;
} gx5_can_sig_t;

typedef struct {
    uint8_t src;            // GX5_IGN_*
    uint8_t at, mask, eq;   // FRAME: on while (data[at] & mask) == eq ...
    uint32_t id;            // ... in the broadcast frame with this id
    char key[GX5_KEY_LEN];  // SIG: on while this signal is answered with a value >= min
    float min;
    uint16_t on_mv, off_mv; // VCC: on at or above on_mv, off below off_mv for hold_s
    uint16_t hold_s;        // SIG/FRAME: off once the condition has not been seen for this long
} gx5_ign_cfg_t;

typedef struct {
    char name[24];
    uint32_t bitrate;
    uint8_t filter;  // J1939: keep only PGNs that are in the library
    uint16_t pre_s;  // woken by CAN traffic or motion: wait this long for ignition
    uint8_t vin_cmd; // library request that returns the VIN; >= GX5_CAN_CMDS = none
    gx5_ign_cfg_t ign;
    gx5_frame_t cmd[GX5_CAN_CMDS]; // dlc 0 = empty
    gx5_can_sig_t sig[GX5_CAN_SIGS];
    struct {
        uint8_t mode; // GX5_CAN_*
        gx5_can_poll_t poll[GX5_CAN_POLL];
    } st[GX5_ST_N];
} gx5_can_cfg_t;

// As shipped: the ACC wire is ignition, CAN listens while it is on.
static inline void gx5_can_defaults(gx5_can_cfg_t *c, uint32_t bitrate)
{
    memset(c, 0, sizeof *c);
    c->bitrate = bitrate;
    c->pre_s = 60;
    c->vin_cmd = 0xFF;
    c->ign.mask = 0xFF;
    c->ign.min = -1e30f; // any answer counts
    c->ign.hold_s = 10;
    c->st[GX5_ST_ON].mode = GX5_CAN_LISTEN;
}

typedef struct {
    int64_t seen_ms; // when the on condition last held; 0 = never
} gx5_ign_t;

// Latest value. OBD-II: id = ECU CAN id, mode = service, pid. J1939: id = PGN, mode = source address.
typedef struct {
    uint32_t id;
    uint16_t pid;
    uint8_t mode, len;
    bool used, updated; // updated: changed since it was last published
    uint8_t data[GX5_CAN_VAL_MAX];
} gx5_can_val_t;

typedef struct {
    bool active, rts;
    uint32_t key, pgn;
    uint16_t size, pos; // pos counts every byte, even those beyond buf
    uint8_t next, packets;
    uint8_t buf[256];
} gx5_can_sess_t;

typedef struct {
    gx5_can_val_t val[GX5_CAN_VALUES];
    gx5_can_sess_t sess[GX5_CAN_SESSIONS];
} gx5_can_state_t;

// ── Shared ──────────────────────────────────────────────────────────────────

// ponytail: fixed table; when it is full new values are dropped. Raise
// GX5_CAN_VALUES (or add eviction) if a vehicle needs more.
static inline void gx5_can_store(gx5_can_state_t *s, uint32_t id, uint8_t mode, uint16_t pid, const uint8_t *data, size_t len)
{
    gx5_can_val_t *v = NULL;
    if (len > GX5_CAN_VAL_MAX) len = GX5_CAN_VAL_MAX;
    for (int i = 0; i < GX5_CAN_VALUES; i++) {
        gx5_can_val_t *c = &s->val[i];
        if (c->used && c->id == id && c->mode == mode && c->pid == pid) {
            v = c;
            break;
        }
        if (!c->used && !v) v = c;
    }
    if (!v) return;
    if (v->used && v->len == len && !memcmp(v->data, data, len)) return; // unchanged
    v->used = v->updated = true;
    v->id = id;
    v->mode = mode;
    v->pid = pid;
    v->len = (uint8_t)len;
    memcpy(v->data, data, len);
}

static inline gx5_can_sess_t *gx5_can_sess(gx5_can_state_t *s, uint32_t key, bool create)
{
    gx5_can_sess_t *spare = &s->sess[0]; // all busy: the first one is recycled
    for (int i = 0; i < GX5_CAN_SESSIONS; i++) {
        if (s->sess[i].active && s->sess[i].key == key) return &s->sess[i];
        if (!s->sess[i].active) spare = &s->sess[i];
    }
    if (!create) return NULL;
    memset(spare, 0, offsetof(gx5_can_sess_t, buf));
    spare->active = true;
    spare->key = key;
    spare->next = 1;
    return spare;
}

static inline void gx5_can_sess_add(gx5_can_sess_t *x, const uint8_t *d, int n)
{
    for (int i = 0; i < n && x->pos < x->size; i++, x->pos++)
        if (x->pos < sizeof x->buf) x->buf[x->pos] = d[i];
}

// ── OBD-II / UDS over ISO-TP ────────────────────────────────────────────────

// Service response payload: [SID+0x40][PID][data..]; service 0x22 has a 16-bit PID.
static inline void gx5_obd_done(gx5_can_state_t *s, uint32_t id, const uint8_t *p, size_t len)
{
    if (len < 2 || p[0] == 0x7F || p[0] < 0x40) return; // negative response / not a response
    uint8_t mode = p[0] - 0x40;
    uint16_t pid = p[1];
    size_t off = 2;
    if (mode == 0x22) {
        if (len < 3) return;
        pid = (uint16_t)(pid << 8 | p[2]);
        off = 3;
    }
    gx5_can_store(s, id, mode, pid, p + off, len - off);
}

// Feed one received frame. Returns true if *reply (a flow-control frame) must be
// sent. *complete is set when a whole response has been stored.
// Gateway (extended) addressing, as used on 0x6xx ids, carries an address byte first.
static inline bool gx5_obd_feed(gx5_can_state_t *s, const gx5_frame_t *f, gx5_frame_t *reply, bool *complete)
{
    const uint8_t *d = f->data;
    int n = f->dlc;
    *complete = false;
    if (f->ext || f->id < 0x600 || n < 2) return false; // diagnostic responses only
    bool gw = (f->id & 0xF00) == 0x600 || d[0] >= 0x40;
    uint8_t gw_addr = d[0];
    if (gw) d++, n--;

    switch (d[0] >> 4) {
    case 0: { // single frame
        int len = d[0] & 0x0F;
        if (len < 1 || len > n - 1) return false;
        gx5_obd_done(s, f->id, d + 1, len);
        *complete = true;
        return false;
    }
    case 1: { // first frame: answer with flow control "send all"
        if (n < 2) return false;
        gx5_can_sess_t *x = gx5_can_sess(s, f->id, true);
        x->size = (uint16_t)((d[0] & 0x0F) << 8 | d[1]);
        gx5_can_sess_add(x, d + 2, n - 2);
        memset(reply, 0, sizeof *reply);
        reply->dlc = 8;
        if (gw) {
            reply->id = (f->id & 0xF00) | gw_addr;
            reply->data[0] = f->id & 0xFF;
            reply->data[1] = 0x30;
        } else {
            reply->id = f->id - 8; // 7E8 -> 7E0
            reply->data[0] = 0x30;
        }
        return true;
    }
    case 2: { // consecutive frame
        gx5_can_sess_t *x = gx5_can_sess(s, f->id, false);
        if (!x) return false;
        if ((d[0] & 0x0F) != (x->next++ & 0x0F)) { // lost a frame: abandon
            x->active = false;
            return false;
        }
        gx5_can_sess_add(x, d + 1, n - 1);
        if (x->pos >= x->size) {
            x->active = false;
            gx5_obd_done(s, f->id, x->buf, x->size < sizeof x->buf ? x->size : sizeof x->buf);
            *complete = true;
        }
        return false;
    }
    }
    return false; // flow control from others
}

// ── J1939 ───────────────────────────────────────────────────────────────────

static inline bool gx5_j1939_requested(const gx5_can_cfg_t *cfg, uint32_t pgn)
{
    for (int i = 0; i < GX5_CAN_CMDS; i++) {
        const gx5_frame_t *f = &cfg->cmd[i];
        if (f->ext && ((f->id >> 16) & 0xFF) == 0xEA && f->dlc >= 3 && (uint32_t)(f->data[0] | f->data[1] << 8 | f->data[2] << 16) == pgn)
            return true;
    }
    return false;
}

static inline void gx5_j1939_claim(gx5_frame_t *f)
{
    static const uint8_t name[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFE, 0x80};
    f->id = 0x18EEFF00 | GX5_J1939_ADDR;
    f->ext = true;
    f->dlc = 8;
    memcpy(f->data, name, 8);
}

static inline void gx5_j1939_done(gx5_can_state_t *s, const gx5_can_cfg_t *cfg, uint32_t pgn, uint8_t sa, const uint8_t *d, size_t len)
{
    if (cfg->filter && !gx5_j1939_requested(cfg, pgn)) return;
    gx5_can_store(s, pgn, sa, 0, d, len);
}

// Feed one received frame. Returns true if *reply must be sent (address claim,
// transport-protocol CTS or end-of-message acknowledgement).
static inline bool gx5_j1939_feed(gx5_can_state_t *s, const gx5_can_cfg_t *cfg, const gx5_frame_t *f, gx5_frame_t *reply)
{
    const uint8_t *d = f->data;
    if (!f->ext) return false;
    uint8_t pf = (f->id >> 16) & 0xFF, ps = (f->id >> 8) & 0xFF, sa = f->id & 0xFF;
    uint32_t pgn = pf < 240 ? (uint32_t)pf << 8 : (uint32_t)pf << 8 | ps; // PDU1 carries a destination in PS
    uint8_t da = pf < 240 ? ps : 0xFF;

    memset(reply, 0, sizeof *reply);
    reply->ext = true;
    reply->dlc = 8;

    switch (pgn) {
    case 0xEA00: // request: only "who has an address?" concerns us
        if (f->dlc < 3 || d[0] != 0x00 || d[1] != 0xEE || d[2] != 0x00 || (da != 0xFF && da != GX5_J1939_ADDR)) return false;
        gx5_j1939_claim(reply);
        return true;
    case 0xEC00: { // TP.CM: start of a multi-packet message
        if (f->dlc < 8 || !(d[0] == 0x20 || (d[0] == 0x10 && da == GX5_J1939_ADDR))) return false; // BAM, or RTS to us
        gx5_can_sess_t *x = gx5_can_sess(s, sa, true);
        x->size = (uint16_t)(d[1] | d[2] << 8);
        x->packets = d[3];
        x->pgn = (uint32_t)(d[5] | d[6] << 8 | d[7] << 16);
        x->rts = d[0] == 0x10;
        if (!x->rts) return false;
        reply->id = 0x1CEC0000 | (uint32_t)sa << 8 | GX5_J1939_ADDR; // CTS: send everything
        reply->data[0] = 0x11;
        reply->data[1] = d[3];
        reply->data[2] = 1;
        reply->data[3] = reply->data[4] = 0xFF;
        memcpy(reply->data + 5, d + 5, 3);
        return true;
    }
    case 0xEB00: { // TP.DT: 7 bytes per packet, numbered from 1
        gx5_can_sess_t *x = gx5_can_sess(s, sa, false);
        if (!x || f->dlc < 2) return false;
        if (d[0] != x->next++) {
            x->active = false;
            return false;
        }
        gx5_can_sess_add(x, d + 1, f->dlc - 1);
        if (x->pos < x->size) return false;
        x->active = false;
        gx5_j1939_done(s, cfg, x->pgn, sa, x->buf, x->size < sizeof x->buf ? x->size : sizeof x->buf);
        if (!x->rts) return false;
        reply->id = 0x1CEC0000 | (uint32_t)sa << 8 | GX5_J1939_ADDR; // end of message ACK
        reply->data[0] = 0x13;
        reply->data[1] = x->size & 0xFF;
        reply->data[2] = x->size >> 8;
        reply->data[3] = x->packets;
        reply->data[4] = 0xFF;
        reply->data[5] = x->pgn & 0xFF;
        reply->data[6] = (x->pgn >> 8) & 0xFF;
        reply->data[7] = (x->pgn >> 16) & 0xFF;
        return true;
    }
    case 0xEF00: // proprietary A: ignored
        return false;
    }
    gx5_j1939_done(s, cfg, pgn, sa, d, f->dlc);
    return false;
}

// ── Text forms ──────────────────────────────────────────────────────────────

// "7E8 01 0C 1AF8" (OBD-II), "7E8 22 F190 5756.." (service 22), "FEEE 00 8C7D.." (J1939).
// Returns the length written, or 0 if it does not fit in `size`.
static inline int gx5_can_fmt(const gx5_can_val_t *v, bool j1939, char *out, size_t size)
{
    int n = j1939 ? snprintf(out, size, "%04X %02X ", (unsigned)v->id, v->mode)
          : v->mode == 0x22 ? snprintf(out, size, "%03X %02X %04X ", (unsigned)v->id, v->mode, v->pid)
          : snprintf(out, size, "%03X %02X %02X ", (unsigned)v->id, v->mode, v->pid);
    if (n < 0 || (size_t)n + v->len * 2 + 1 > size) return 0;
    for (int i = 0; i < v->len; i++) n += sprintf(out + n, "%02X", v->data[i]);
    return n;
}

// Build one message of changed values: {"ts":N,"obd":["..",..]} (or "j1939"),
// clearing their updated flags. Returns the item count; *more is set when further
// changed values are left for another message.
static inline int gx5_can_chunk(gx5_can_state_t *s, bool j1939, long long ts, char *msg, size_t size, bool *more)
{
    int items = 0, n = snprintf(msg, size, "{\"ts\":%lld,\"%s\":[", ts, j1939 ? "j1939" : "obd");
    *more = false;
    for (int i = 0; i < GX5_CAN_VALUES; i++) {
        gx5_can_val_t *v = &s->val[i];
        if (!v->used || !v->updated) continue;
        // room for: comma, two quotes, the item, "]}" and the terminator
        int len = (size_t)n + 6 < size ? gx5_can_fmt(v, j1939, msg + n + 1 + (items > 0), size - n - 5) : 0;
        if (!len) {
            if (items) *more = true;  // goes in the next message
            else v->updated = false;  // can never fit: skip it
            continue;
        }
        if (items) msg[n++] = ',';
        msg[n++] = '"';
        n += len;
        msg[n++] = '"';
        v->updated = false;
        items++;
    }
    strcpy(msg + n, "]}");
    return items;
}

// Hex text (even length, at most `max` bytes) -> bytes. Returns the count or -1.
static inline int gx5_hex(const char *s, uint8_t *out, int max)
{
    size_t len = strlen(s);
    if (len % 2 || len / 2 > (size_t)max || strspn(s, "0123456789abcdefABCDEF") != len) return -1;
    for (size_t i = 0; i < len / 2; i++) {
        char b[3] = {s[2 * i], s[2 * i + 1], 0};
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    return (int)(len / 2);
}

// ── Signals and ignition ────────────────────────────────────────────────────

// The stored answer to library request `c`, or NULL. OBD-II/UDS: service and PID of
// the single-frame request [len][service][pid..], from the ECU addressed (7DF: any).
// J1939: the requested PGN.
// ponytail: gateway-addressed (0x6xx) requests are not matched; add when a vehicle needs it.
static inline gx5_can_val_t *gx5_cmd_val(gx5_can_state_t *s, const gx5_frame_t *c)
{
    if (c->dlc < 3) return NULL;
    uint16_t pid = c->data[1] == 0x22 && c->dlc >= 4 ? (uint16_t)(c->data[2] << 8 | c->data[3]) : c->data[2];
    for (int i = 0; i < GX5_CAN_VALUES; i++) {
        gx5_can_val_t *v = &s->val[i];
        if (!v->used) continue;
        if (c->ext ? v->id == (uint32_t)(c->data[0] | c->data[1] << 8 | c->data[2] << 16)
                   : v->mode == c->data[1] && v->pid == pid && (c->id == 0x7DF || v->id == c->id + 8))
            return v;
    }
    return NULL;
}

static inline bool gx5_sig_value(const gx5_can_sig_t *g, const uint8_t *d, size_t n, double *out)
{
    uint32_t raw = 0;
    if (!g->len || g->len > 32 || ((size_t)g->bix + g->len + 7) / 8 > n) return false;
    if (g->flags & GX5_SIG_LE) {
        if (g->bix % 8 || g->len % 8) return false;
        for (int i = g->len / 8 - 1; i >= 0; i--) raw = raw << 8 | d[g->bix / 8 + i];
    } else {
        for (int b = g->bix; b < g->bix + g->len; b++) raw = raw << 1 | (d[b / 8] >> (7 - b % 8) & 1);
    }
    double v = raw;
    if ((g->flags & GX5_SIG_SIGNED) && (raw >> (g->len - 1) & 1)) v -= (double)(1ULL << g->len);
    *out = v * g->mul + g->add;
    return true;
}

// The VIN in an answer (09 02: a count byte first; 22 F190: bare; J1939 FEEC: '*' after
// it): the last 17 letters and digits. False if there are not 17.
static inline bool gx5_vin_from(const uint8_t *d, size_t n, char out[18])
{
    while (n && !((d[n - 1] >= '0' && d[n - 1] <= '9') || (d[n - 1] >= 'A' && d[n - 1] <= 'Z'))) n--;
    if (n < 17) return false;
    for (int i = 0; i < 17; i++) {
        char ch = (char)d[n - 17 + i];
        if (!((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z'))) return false;
        out[i] = ch;
    }
    out[17] = '\0';
    return true;
}

static inline const gx5_can_sig_t *gx5_sig_find(const gx5_can_cfg_t *cfg, const char *key)
{
    for (int i = 0; i < GX5_CAN_SIGS; i++)
        if (cfg->sig[i].key[0] && !strcmp(cfg->sig[i].key, key)) return &cfg->sig[i];
    return NULL;
}

// Build {"ts":N,"sig":{"rpm":1726,..}} from signals whose answer changed, starting at
// signal *next (0 first; call again while it is below GX5_CAN_SIGS). Returns the item
// count. gx5_sig_sent() afterwards keeps those answers out of the raw list.
static inline int gx5_sig_chunk(const gx5_can_cfg_t *cfg, gx5_can_state_t *s, long long ts, char *msg, size_t size, int *next)
{
    int items = 0, n = snprintf(msg, size, "{\"ts\":%lld,\"sig\":{", ts);
    for (; *next < GX5_CAN_SIGS; ++*next) {
        const gx5_can_sig_t *g = &cfg->sig[*next];
        char item[GX5_KEY_LEN + 32];
        double x;
        if (!g->key[0] || g->cmd >= GX5_CAN_CMDS) continue;
        const gx5_can_val_t *v = gx5_cmd_val(s, &cfg->cmd[g->cmd]);
        if (!v || !v->updated || !gx5_sig_value(g, v->data, v->len, &x)) continue;
        int len = snprintf(item, sizeof item, "%s\"%s\":%.6g", items ? "," : "", g->key, x);
        if ((size_t)n + len + 3 > size) break; // the rest goes in the next message
        memcpy(msg + n, item, len);
        n += len;
        items++;
    }
    strcpy(msg + n, "}}");
    return items;
}

static inline void gx5_sig_sent(const gx5_can_cfg_t *cfg, gx5_can_state_t *s)
{
    for (int i = 0; i < GX5_CAN_SIGS; i++) {
        gx5_can_val_t *v = cfg->sig[i].key[0] && cfg->sig[i].cmd < GX5_CAN_CMDS ? gx5_cmd_val(s, &cfg->cmd[cfg->sig[i].cmd]) : NULL;
        if (v) v->updated = false;
    }
}

// Ignition from a broadcast frame (FRAME source). Call for every received frame.
static inline void gx5_ign_frame(const gx5_ign_cfg_t *c, gx5_ign_t *g, const gx5_frame_t *f, int64_t now_ms)
{
    if (c->src == GX5_IGN_FRAME && f->id == c->id && c->at < f->dlc && (f->data[c->at] & c->mask) == c->eq) g->seen_ms = now_ms;
}

// Ignition from an answered signal (SIG source). Call when an answer from `rx_id` completed.
// ponytail: any answer from that ECU re-checks the last stored value, so a stale value
// can hold ignition on for one poll interval longer.
static inline void gx5_ign_answer(const gx5_can_cfg_t *cfg, gx5_can_state_t *s, gx5_ign_t *g, uint32_t rx_id, int64_t now_ms)
{
    double x;
    const gx5_can_sig_t *sig = cfg->ign.src == GX5_IGN_SIG ? gx5_sig_find(cfg, cfg->ign.key) : NULL;
    const gx5_can_val_t *v = sig && sig->cmd < GX5_CAN_CMDS ? gx5_cmd_val(s, &cfg->cmd[sig->cmd]) : NULL;
    if (v && v->id == rx_id && gx5_sig_value(sig, v->data, v->len, &x) && x >= cfg->ign.min) g->seen_ms = now_ms;
}

static inline bool gx5_ign_on(const gx5_ign_cfg_t *c, const gx5_ign_t *g, int64_t now_ms)
{
    return g->seen_ms && now_ms - g->seen_ms < c->hold_s * 1000LL;
}

// ── Commands ────────────────────────────────────────────────────────────────

static inline double gx5_json_num(const char *o, const char *key, double def)
{
    const char *p = gx5_json_val(o, key);
    return p && (*p == '-' || (*p >= '0' && *p <= '9')) ? strtod(p, NULL) : def;
}

static inline bool gx5_json_flag(const char *o, const char *key, bool def)
{
    const char *p = gx5_json_val(o, key);
    return p ? *p == 't' || *p == '1' : def;
}

// Index of `name` in a NULL-terminated list, or -1.
static inline int gx5_pick(const char *name, const char *const *list)
{
    for (int i = 0; list[i]; i++)
        if (!strcmp(name, list[i])) return i;
    return -1;
}

// {"vehicle":{"name":"kia-cerato-2021","bitrate":500000,"filter":false,"pre_s":60,
//   "vin_pid":2,                           library request that returns the VIN; -1 = none
//   "ign":{"src":"acc"}                                        the ACC wire (default)
//   "ign":{"src":"vcc","on_mv":13200,"off_mv":12800,"hold_s":10}
//   "ign":{"src":"sig","key":"rpm","min":1,"hold_s":10}        no "min": any answer counts
//   "ign":{"src":"frame","id":"541","at":0,"mask":"FF","eq":"03","hold_s":3}}}
// Absent fields keep their value. False if there is no "vehicle" object or it is invalid.
static inline bool gx5_parse_vehicle(const char *js, gx5_can_cfg_t *cfg)
{
    static const char *const srcs[] = {"acc", "vcc", "sig", "frame", NULL};
    char name[sizeof cfg->name], src[8], hex[10];
    const char *o = gx5_json_val(js, "vehicle"), *p;
    gx5_ign_cfg_t *g = &cfg->ign;
    uint8_t b;
    if (!o || *o != '{' || !gx5_json_str(o, "name", name, sizeof name) || !gx5_json_str(o, "src", src, sizeof src)) return false;
    if (name[0]) strcpy(cfg->name, name);
    double rate = gx5_json_num(o, "bitrate", cfg->bitrate), pre = gx5_json_num(o, "pre_s", cfg->pre_s), hold = gx5_json_num(o, "hold_s", g->hold_s);
    double on = gx5_json_num(o, "on_mv", g->on_mv), off = gx5_json_num(o, "off_mv", g->off_mv), at = gx5_json_num(o, "at", g->at);
    if (rate < 10000 || rate > 1000000 || pre < 5 || pre > 3600 || hold < 1 || hold > 3600 || on < 0 || on > 40000 || off < 0 || off > on || at < 0 || at > 7) return false;
    if (src[0]) {
        int i = gx5_pick(src, srcs);
        if (i < 0) return false;
        g->src = (uint8_t)i;
    }
    if (gx5_json_val(o, "id")) {
        char *end;
        if (!gx5_json_str(o, "id", hex, sizeof hex) || !hex[0]) return false;
        unsigned long id = strtoul(hex, &end, 16);
        if (*end || id > 0x1FFFFFFF) return false;
        g->id = (uint32_t)id;
    }
    if (gx5_json_val(o, "mask")) {
        if (!gx5_json_str(o, "mask", hex, sizeof hex) || gx5_hex(hex, &b, 1) != 1) return false;
        g->mask = b;
    }
    if (gx5_json_val(o, "eq")) {
        if (!gx5_json_str(o, "eq", hex, sizeof hex) || gx5_hex(hex, &b, 1) != 1) return false;
        g->eq = b;
    }
    if (gx5_json_val(o, "key") && (!gx5_json_str(o, "key", g->key, sizeof g->key) || !g->key[0])) return false;
    if (g->src == GX5_IGN_VCC && (on < 1 || off < 1)) return false;
    if (g->src == GX5_IGN_SIG && !g->key[0]) return false;
    cfg->bitrate = (uint32_t)rate;
    cfg->pre_s = (uint16_t)pre;
    double vin = gx5_json_num(o, "vin_pid", cfg->vin_cmd);
    cfg->vin_cmd = vin >= 0 && vin < GX5_CAN_CMDS ? (uint8_t)vin : 0xFF;
    g->hold_s = (uint16_t)hold;
    g->on_mv = (uint16_t)on;
    g->off_mv = (uint16_t)off;
    g->at = (uint8_t)at;
    g->min = (float)gx5_json_num(o, "min", g->min);
    if ((p = gx5_json_val(o, "filter"))) cfg->filter = *p == 't' || *p == '1';
    return true;
}

// {"pid":{"n":0,"id":"7E0","data":"02010C"}}   library request n; no "id" clears it.
// Returns n, or -1 if there is no valid "pid" object.
static inline int gx5_parse_pid(const char *js, gx5_can_cfg_t *cfg)
{
    char id[10], data[17];
    gx5_frame_t f = {0};
    const char *o = gx5_json_val(js, "pid");
    if (!o || *o != '{') return -1;
    double n = gx5_json_num(o, "n", -1);
    if (n < 0 || n >= GX5_CAN_CMDS || !gx5_json_str(o, "id", id, sizeof id) || !gx5_json_str(o, "data", data, sizeof data)) return -1;
    if (id[0]) {
        char *end;
        unsigned long can_id = strtoul(id, &end, 16);
        int len = gx5_hex(data, f.data, 8);
        if (*end || can_id > 0x1FFFFFFF || len < 1) return -1;
        f.id = (uint32_t)can_id;
        f.ext = can_id > 0x7FF;
        f.dlc = (uint8_t)len;
    }
    cfg->cmd[(int)n] = f;
    return (int)n;
}

// {"sig":{"n":0,"key":"rpm","pid":0,"bix":0,"len":16,"mul":1,"div":4,"add":0,"sign":false,"blsb":false}}
// Signal n, decoded from the answer to library request "pid"; no "key" clears it.
// The field names are OBDb's. Returns n, or -1 if there is no valid "sig" object.
static inline int gx5_parse_sig(const char *js, gx5_can_cfg_t *cfg)
{
    gx5_can_sig_t g = {0};
    const char *o = gx5_json_val(js, "sig");
    if (!o || *o != '{') return -1;
    double n = gx5_json_num(o, "n", -1), div = gx5_json_num(o, "div", 1);
    if (n < 0 || n >= GX5_CAN_SIGS || !gx5_json_str(o, "key", g.key, sizeof g.key)) return -1;
    if (g.key[0]) {
        double cmd = gx5_json_num(o, "pid", -1), bix = gx5_json_num(o, "bix", 0), len = gx5_json_num(o, "len", 8);
        if (cmd < 0 || cmd >= GX5_CAN_CMDS || bix < 0 || len < 1 || len > 32 || bix + len > GX5_CAN_VAL_MAX * 8 || div == 0) return -1;
        if (strspn(g.key, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != strlen(g.key)) return -1; // goes into JSON as is
        g.cmd = (uint8_t)cmd;
        g.bix = (uint16_t)bix;
        g.len = (uint8_t)len;
        g.flags = (gx5_json_flag(o, "sign", false) ? GX5_SIG_SIGNED : 0) | (gx5_json_flag(o, "blsb", false) ? GX5_SIG_LE : 0);
        if ((g.flags & GX5_SIG_LE) && (g.bix % 8 || g.len % 8)) return -1;
        g.mul = (float)(gx5_json_num(o, "mul", 1) / div);
        g.add = (float)gx5_json_num(o, "add", 0);
    }
    cfg->sig[(int)n] = g;
    return (int)n;
}

// {"can":{"state":"pre|on|off","mode":"off|listen|obd2|j1939","poll":[[0,1],[3,0.5],[7,60]]}}
// One state: its mode and its request list as [library request, seconds] pairs. An
// absent "mode" or "poll" keeps its value; "poll":[] clears the list. Listen mode
// never transmits, so its list is not sent. Returns the state, or -1.
static inline int gx5_parse_can_state(const char *js, gx5_can_cfg_t *cfg)
{
    static const char *const states[] = {"pre", "on", "off", NULL}, *const modes[] = {"off", "listen", "obd2", "j1939", NULL};
    char state[8], mode[8];
    gx5_can_poll_t poll[GX5_CAN_POLL] = {0};
    const char *o = gx5_json_val(js, "can"), *p;
    if (!o || *o != '{' || !gx5_json_str(o, "state", state, sizeof state) || !gx5_json_str(o, "mode", mode, sizeof mode)) return -1;
    int st = gx5_pick(state, states), m = mode[0] ? gx5_pick(mode, modes) : cfg->st[st < 0 ? 0 : st].mode;
    if (st < 0 || m < 0) return -1;
    if ((p = gx5_json_val(o, "poll"))) {
        if (*p++ != '[') return -1;
        for (int i = 0;; i++) {
            char *end;
            while (*p == ' ' || *p == ',') p++;
            if (*p == ']') break;
            if (*p++ != '[' || i >= GX5_CAN_POLL) return -1;
            long cmd = strtol(p, &end, 10);
            if (end == p || *end != ',') return -1;
            double sec = strtod(p = end + 1, &end);
            if (end == p || *end != ']' || cmd < 0 || cmd >= GX5_CAN_CMDS || sec < 0.1 || sec > 6553) return -1;
            poll[i] = (gx5_can_poll_t){.cmd = (uint8_t)cmd, .ds = (uint16_t)(sec * 10 + 0.5)};
            p = end + 1;
        }
        memcpy(cfg->st[st].poll, poll, sizeof poll);
    }
    cfg->st[st].mode = (uint8_t)m;
    return st;
}
