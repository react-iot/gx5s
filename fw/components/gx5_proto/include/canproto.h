// CAN protocol logic (OBD-II and J1939) in plain C with fixed-size state.
// Pure and host-testable:
//   cc -Icomponents/gx5_proto/include test/test_can.c -o test/test_can && test/test_can   (from fw/)
#pragma once
#include "parse.h"

#define GX5_CAN_VAL_MAX  96   // bytes kept per value (longer responses are truncated)
#define GX5_CAN_VALUES   40   // distinct values tracked
#define GX5_CAN_REQS     16   // scheduled request slots
#define GX5_CAN_SESSIONS 4    // multi-frame messages being assembled at once
#define GX5_J1939_ADDR   0xF8 // our J1939 source address

enum { GX5_CAN_OFF, GX5_CAN_LISTEN, GX5_CAN_OBD2, GX5_CAN_J1939 };

typedef struct {
    uint32_t id;
    bool ext; // 29-bit identifier
    uint8_t dlc;
    uint8_t data[8];
} gx5_frame_t;

// A frame sent every interval_s while the bus is running. interval_s 0 = empty slot.
// OBD-II:  id 7DF, data 02 01 0C          (request engine speed)
// J1939:   id 18EA00F8, data EE FE 00     (request PGN FEEE from address 00)
typedef struct {
    gx5_frame_t frame;
    uint16_t interval_s;
} gx5_can_req_t;

typedef struct {
    uint8_t mode;   // GX5_CAN_*
    uint8_t filter; // J1939: keep only PGNs that are requested
    uint32_t bitrate;
    gx5_can_req_t req[GX5_CAN_REQS];
} gx5_can_cfg_t;

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
    for (int i = 0; i < GX5_CAN_REQS; i++) {
        const gx5_frame_t *f = &cfg->req[i].frame;
        if (cfg->req[i].interval_s && f->ext && ((f->id >> 16) & 0xFF) == 0xEA && f->dlc >= 3 &&
            (uint32_t)(f->data[0] | f->data[1] << 8 | f->data[2] << 16) == pgn)
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

// {"can":{"mode":"off|listen|obd2|j1939","bitrate":500000,"filter":false}}
// Absent fields keep their value. False if there is no "can" object or it is invalid.
static inline bool gx5_parse_can_cfg(const char *js, gx5_can_cfg_t *cfg)
{
    static const char *modes[] = {"off", "listen", "obd2", "j1939"};
    char mode[8];
    const char *o = gx5_json_val(js, "can"), *p;
    if (!o || *o != '{' || !gx5_json_str(o, "mode", mode, sizeof mode)) return false;
    if (mode[0]) {
        int m = 0;
        while (m < 4 && strcmp(mode, modes[m])) m++;
        if (m == 4) return false;
        cfg->mode = (uint8_t)m;
    }
    if ((p = gx5_json_val(o, "bitrate"))) {
        long b = atol(p);
        if (b < 10000 || b > 1000000) return false;
        cfg->bitrate = (uint32_t)b;
    }
    if ((p = gx5_json_val(o, "filter"))) cfg->filter = *p == 't' || *p == '1';
    return true;
}

// {"can_req":{"slot":0,"id":"7DF","data":"02010C","interval":10}}  (interval 0 clears the slot)
// Returns the slot, or -1 if there is no valid "can_req" object.
static inline int gx5_parse_can_req(const char *js, gx5_can_req_t *r)
{
    char id[10], data[17];
    const char *o = gx5_json_val(js, "can_req"), *p;
    memset(r, 0, sizeof *r);
    if (!o || *o != '{' || !(p = gx5_json_val(o, "slot"))) return -1;
    long slot = atol(p), interval = (p = gx5_json_val(o, "interval")) ? atol(p) : 0;
    if (slot < 0 || slot >= GX5_CAN_REQS || *gx5_json_val(o, "slot") < '0' || interval < 0 || interval > 65535) return -1;
    if (!interval) return (int)slot;
    if (!gx5_json_str(o, "id", id, sizeof id) || !gx5_json_str(o, "data", data, sizeof data)) return -1;
    char *end;
    unsigned long can_id = strtoul(id, &end, 16);
    int n = gx5_hex(data, r->frame.data, 8);
    if (!id[0] || *end || can_id > 0x1FFFFFFF || n < 1) return -1;
    r->frame.id = (uint32_t)can_id;
    r->frame.ext = can_id > 0x7FF;
    r->frame.dlc = (uint8_t)n;
    r->interval_s = (uint16_t)interval;
    return (int)slot;
}
