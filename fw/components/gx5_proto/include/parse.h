// Pure helpers, host-testable (from fw/):
//   cc -Icomponents/gx5_proto/include test/test_parse.c -o test/test_parse && test/test_parse
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Voltage on the ADC-VCC node (calibrated mV) -> harness supply mV.
// Schematic v210620 sheet 2: VCC-OUT - R25 100K - node - R34 10K - GND, and the node
// also drives Q4 (MMUN2233: 4.7K in series with the base, 47K base to emitter), so:
//   Q4 off (node < Vbe * 51.7/47):  node = Vo * 0.07731
//   Q4 on:                          node = Vo * 0.030982 + Vbe * 0.6592
// plus ~0.3 V for the input Schottky D10. Vbe is the per-board calibration
// (GX5_VCC_VBE_MV); 1 mV of Vbe moves the result 21 mV, and Vbe falls ~2 mV/degC.
static inline int gx5_vcc_node_to_mv(int node_mv, int vbe_mv)
{
    long vo = node_mv * 470L < vbe_mv * 517L ? node_mv * 100000L / 7731
                                             : (node_mv * 10000L - vbe_mv * 6592L) * 100 / 30982;
    return (int)vo + 300;
}

// "+CEREG: <n>,<stat>[,...]" / "+CREG: ..." -> registered (home or roaming)?
static inline bool gx5_reg_ok(const char *line)
{
    const char *c = strchr(line, ',');
    int stat = c ? atoi(c + 1) : -1;
    return stat == 1 || stat == 5;
}

// Last comma-separated integer of a URC, e.g. "+QMTOPEN: 0,0" -> 0. -1 if none.
static inline int gx5_last_int(const char *line)
{
    const char *c = strrchr(line, ',');
    return c ? atoi(c + 1) : -1;
}

// Split `+QMTRECV: <client>,<msgid>,"<topic>","<payload>"` in place.
static inline bool gx5_parse_qmtrecv(char *line, char **topic, char **payload)
{
    char *t = strchr(line, '"');
    if (!t) return false;
    char *te = strchr(t + 1, '"');
    if (!te || te[1] != ',' || te[2] != '"') return false;
    char *pe = strrchr(te + 3, '"');   // payload may itself contain quotes
    if (!pe) return false;
    *te = *pe = '\0';
    *topic = t + 1;
    *payload = te + 3;
    return true;
}

typedef struct {
    char host[64];
    uint16_t port;
    char user[32];
    char pass[64];
    bool tls;
} gx5_broker_t;

// Value position of `"key":` in a JSON text, or NULL.
// ponytail: flat textual lookup, no escapes or nesting awareness. Enough for the
// broker message; switch to cJSON if command payloads grow.
static inline const char *gx5_json_val(const char *js, const char *key)
{
    char pat[24];
    int n = snprintf(pat, sizeof pat, "\"%s\"", key);
    for (const char *p = js; (p = strstr(p, pat)); ) {
        p += n;
        while (*p == ' ') p++;
        if (*p != ':') continue; // that was a value, not a key
        p++;
        while (*p == ' ') p++;
        return p;
    }
    return NULL;
}

// Copy a JSON string value. Absent key -> "" and true. Not a string, or too long
// for `out`, or holding characters that would break an AT command -> false.
static inline bool gx5_json_str(const char *js, const char *key, char *out, size_t size)
{
    const char *p = gx5_json_val(js, key);
    out[0] = '\0';
    if (!p) return true;
    if (*p != '"') return false;
    const char *e = strchr(++p, '"');
    if (!e || (size_t)(e - p) >= size) return false;
    for (const char *c = p; c < e; c++)
        if (*c < 0x20 || *c == '\\') return false;
    memcpy(out, p, e - p);
    out[e - p] = '\0';
    return true;
}

// A location id goes into MQTT topics and AT commands: 1..9 of [A-Za-z0-9_-].
#define GX5_LOC_LEN 10
static inline bool gx5_id_ok(const char *s)
{
    size_t n = strspn(s, "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-");
    return n && n < GX5_LOC_LEN && !s[n];
}

// A firmware version goes into a URL: 1..15 of [A-Za-z0-9._-].
static inline bool gx5_ver_ok(const char *s)
{
    size_t n = strspn(s, "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-");
    return n && n < 16 && !s[n];
}

// Provisioning PIN (proof of possession): the last three bytes of the device id as
// one decimal number, 8 digits. C4:DD:57:E0:9F:AF -> 0xE09FAF -> "14720943".
static inline void gx5_pop(const uint8_t id[6], char out[9])
{
    snprintf(out, 9, "%08lu", (unsigned long)id[3] << 16 | (unsigned long)id[4] << 8 | id[5]);
}

// {"provision":{"pop":"14720943","broker":{...},"config":{...}}}
// Returns the provision object if the message is one and its PIN matches, else NULL
// (*is_provision tells a wrong PIN from an unrelated message).
static inline const char *gx5_provision(const char *js, const char *pop, bool *is_provision)
{
    char got[12];
    const char *o = gx5_json_val(js, "provision");
    *is_provision = o && *o == '{';
    if (!*is_provision) return NULL;
    return gx5_json_str(o, "pop", got, sizeof got) && !strcmp(got, pop) ? o : NULL;
}

// {"broker":{"host":"h","port":8883,"user":"u","pass":"p","tls":true}}
// host is required; port defaults to 8883, tls to true, user/pass to empty.
static inline bool gx5_parse_broker(const char *js, gx5_broker_t *b)
{
    const char *o = gx5_json_val(js, "broker");
    if (!o || *o != '{') return false;
    if (!gx5_json_str(o, "host", b->host, sizeof b->host) || !b->host[0]) return false;
    if (!gx5_json_str(o, "user", b->user, sizeof b->user)) return false;
    if (!gx5_json_str(o, "pass", b->pass, sizeof b->pass)) return false;
    const char *p = gx5_json_val(o, "port");
    long port = p ? atol(p) : 8883;
    if (port < 1 || port > 65535) return false;
    b->port = (uint16_t)port;
    p = gx5_json_val(o, "tls");
    b->tls = !p || *p == 't' || *p == '1';
    return true;
}

// UTC calendar time -> Unix epoch (newlib has no timegm). Valid for 1970..2099.
static inline long long gx5_epoch(int y, int mo, int d, int h, int mi, int s)
{
    static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    long long days = (y - 1970) * 365LL + (y - 1969) / 4 + cum[mo - 1] + d - 1;
    if (mo > 2 && y % 4 == 0) days++;
    return days * 86400 + h * 3600 + mi * 60 + s;
}

// `+CCLK: "yy/MM/dd,hh:mm:ss+zz"` -> UTC epoch. With AT+CTZU=1 (as set at bring-up)
// the modem clock is already UTC and zz is only the local zone, so zz is not applied
// (bench: "19:21:05+32" at 19:21:04 UTC, in a UTC+8 zone). False if malformed or the modem clock was never set (year < 2024).
static inline bool gx5_parse_cclk(const char *line, long long *utc)
{
    int y, mo, d, h, mi, s;
    const char *q = strchr(line, '"');
    if (!q || sscanf(q + 1, "%d/%d/%d,%d:%d:%d", &y, &mo, &d, &h, &mi, &s) < 6) return false;
    if (y < 24 || y > 69 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || s > 60) return false;
    *utc = gx5_epoch(2000 + y, mo, d, h, mi, s);
    return true;
}

// `+QGPSLOC: <utc>,<lat>,<lon>,<hdop>,<alt>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>`
// (mode 2, decimal degrees) -> JSON fragment
//   ,"lat":-33.86785,"lon":151.20732,"kmh":48.6,"cog":113.48,"sat":9
// cog = course over ground as the modem gives it (ddd.mm), sat = satellites used.
// Numbers are re-printed, so a leading zero from the modem cannot break the JSON.
// *utc (if not NULL) gets the fix time (hhmmss.s + ddmmyy), or 0 if implausible.
static inline bool gx5_fix_json(const char *line, char *out, size_t size, long long *utc)
{
    double v[11];
    const char *p = strchr(line, ':');
    if (!p) return false;
    for (int i = 0; i < 11; i++) {
        p += strspn(p + 1, " ") + 1; // past ':' or ',' and blanks
        size_t n = strcspn(p, ",");
        if (!n || strspn(p, "-.0123456789") != n) return false;
        v[i] = strtod(p, NULL);
        p += n;
        if (*p != ',' && i < 10) return false;
    }
    if (utc) {
        long t = (long)v[0], d = (long)v[9];
        int mo = d / 100 % 100, day = d / 10000;
        *utc = d % 100 >= 24 && mo >= 1 && mo <= 12 && day >= 1 && day <= 31 ? gx5_epoch(2000 + d % 100, mo, day, t / 10000, t / 100 % 100, t % 100) : 0;
    }
    return snprintf(out, size, ",\"lat\":%.5f,\"lon\":%.5f,\"kmh\":%.1f,\"cog\":%.2f,\"sat\":%d", v[1], v[2], v[7], v[6], (int)v[10]) < (int)size;
}
