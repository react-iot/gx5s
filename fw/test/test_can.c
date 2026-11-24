#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "canproto.h"

static gx5_can_state_t st;
static gx5_can_cfg_t cfg;

static gx5_frame_t fr(uint32_t id, const char *hex)
{
    gx5_frame_t f = {.id = id, .ext = id > 0x7FF};
    f.dlc = (uint8_t)gx5_hex(hex, f.data, 8);
    return f;
}

static gx5_can_val_t *val(uint32_t id, uint8_t mode, uint16_t pid)
{
    for (int i = 0; i < GX5_CAN_VALUES; i++)
        if (st.val[i].used && st.val[i].id == id && st.val[i].mode == mode && st.val[i].pid == pid) return &st.val[i];
    return NULL;
}

int main(void)
{
    gx5_frame_t f, reply;
    bool done;
    char txt[256];

    // OBD-II single frame: engine speed.
    f = fr(0x7E8, "04410C1AF8000000");
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && done);
    gx5_can_val_t *v = val(0x7E8, 0x01, 0x0C);
    assert(v && v->updated && v->len == 2 && v->data[0] == 0x1A && v->data[1] == 0xF8);
    assert(gx5_can_fmt(v, false, txt, sizeof txt) && !strcmp(txt, "7E8 01 0C 1AF8"));
    v->updated = false;
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && !v->updated); // same value: not flagged again
    f = fr(0x7E8, "04410C1B00000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    assert(v->updated && v->data[0] == 0x1B);

    // Ignored: broadcast traffic, negative response, extended ids.
    f = fr(0x130, "0441001122334455");
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && !done);
    f = fr(0x7E8, "037F011200000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    assert(!val(0x7E8, 0x3F, 0x01));

    // Multi-frame VIN (service 09 PID 02, 20 bytes): flow control, then assembly.
    f = fr(0x7E8, "1014490201574642");
    assert(gx5_obd_feed(&st, &f, &reply, &done) && !done);
    assert(reply.id == 0x7E0 && !reply.ext && reply.dlc == 8 && reply.data[0] == 0x30);
    f = fr(0x7E8, "2141413132333435");
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && !done);
    f = fr(0x7E8, "2236373839303132");
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && done);
    v = val(0x7E8, 0x09, 0x02);
    assert(v && v->len == 18 && v->data[0] == 0x01 && !memcmp(v->data + 1, "WFBAA123456789012", 17));

    // A lost consecutive frame abandons the message.
    f = fr(0x7EA, "1014490201574642");
    gx5_obd_feed(&st, &f, &reply, &done);
    f = fr(0x7EA, "2236373839303132"); // sequence 2 where 1 was expected
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && !done && !val(0x7EA, 0x09, 0x02));

    // Service 22 with a 16-bit PID, gateway addressing on a 0x6xx id.
    f = fr(0x612, "F10462455F040000");
    assert(!gx5_obd_feed(&st, &f, &reply, &done) && done);
    v = val(0x612, 0x22, 0x455F);
    assert(v && v->len == 1 && v->data[0] == 0x04);
    assert(gx5_can_fmt(v, false, txt, sizeof txt) && !strcmp(txt, "612 22 455F 04"));
    f = fr(0x612, "F1100A62F1900102"); // first frame through the gateway
    assert(gx5_obd_feed(&st, &f, &reply, &done));
    assert(reply.id == 0x6F1 && reply.data[0] == 0x12 && reply.data[1] == 0x30);

    // J1939 single frame, PDU2: PGN FEEE (engine temperature) from address 00.
    memset(&st, 0, sizeof st);
    f = fr(0x18FEEE00, "8C7DFFFFFFFFFFFF");
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));
    v = val(0xFEEE, 0x00, 0);
    assert(v && v->len == 8 && v->data[0] == 0x8C);
    assert(gx5_can_fmt(v, true, txt, sizeof txt) && !strcmp(txt, "FEEE 00 8C7DFFFFFFFFFFFF"));

    // Address-claim request -> we claim F8.
    f = fr(0x18EAFF00, "00EE00");
    assert(gx5_j1939_feed(&st, &cfg, &f, &reply));
    assert(reply.id == 0x18EEFFF8 && reply.ext && reply.data[7] == 0x80);
    f = fr(0x18EAFF00, "EEFE00"); // a request for some other PGN is not ours to answer
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));

    // BAM: 10 bytes of PGN FECA in two packets, no reply.
    f = fr(0x18ECFF21, "200A0002FFCAFE00");
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));
    f = fr(0x18EBFF21, "0101020304050607");
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply) && !val(0xFECA, 0x21, 0));
    f = fr(0x18EBFF21, "0208090AFFFFFFFF");
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));
    v = val(0xFECA, 0x21, 0);
    assert(v && v->len == 10 && v->data[0] == 0x01 && v->data[9] == 0x0A);

    // RTS addressed to us: CTS, then end-of-message ACK.
    f = fr(0x18ECF800, "10090002FFEBFE00");
    assert(gx5_j1939_feed(&st, &cfg, &f, &reply));
    assert(reply.id == 0x1CEC00F8 && reply.data[0] == 0x11 && reply.data[1] == 2 && reply.data[2] == 1 && reply.data[5] == 0xEB && reply.data[6] == 0xFE);
    f = fr(0x18EBF800, "0141424344454647");
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));
    f = fr(0x18EBF800, "024849FFFFFFFFFF");
    assert(gx5_j1939_feed(&st, &cfg, &f, &reply) && reply.data[0] == 0x13 && reply.data[1] == 9);
    v = val(0xFEEB, 0x00, 0);
    assert(v && v->len == 9 && !memcmp(v->data, "ABCDEFGHI", 9));
    f = fr(0x18EC3300, "10090002FFEBFE00"); // RTS to someone else: not ours
    assert(!gx5_j1939_feed(&st, &cfg, &f, &reply));

    // Filter: only requested PGNs are kept.
    memset(&st, 0, sizeof st);
    cfg.filter = 1;
    cfg.cmd[0] = fr(0x18EA00F8, "EEFE00");
    f = fr(0x18FEEE00, "8C7DFFFFFFFFFFFF");
    gx5_j1939_feed(&st, &cfg, &f, &reply);
    f = fr(0x18FEF100, "0102030405060708");
    gx5_j1939_feed(&st, &cfg, &f, &reply);
    assert(val(0xFEEE, 0, 0) && !val(0xFEF1, 0, 0));

    // Table full: further values are dropped, existing ones still update.
    memset(&st, 0, sizeof st);
    uint8_t b = 1;
    for (int i = 0; i < GX5_CAN_VALUES + 5; i++) gx5_can_store(&st, 0x700 + i, 1, 0, &b, 1);
    assert(val(0x700 + GX5_CAN_VALUES - 1, 1, 0) && !val(0x700 + GX5_CAN_VALUES, 1, 0));
    uint8_t big[200] = {7};
    gx5_can_store(&st, 0x700, 1, 0, big, sizeof big);
    assert(val(0x700, 1, 0)->len == GX5_CAN_VAL_MAX);
    assert(gx5_can_fmt(val(0x700, 1, 0), false, txt, 20) == 0); // does not fit

    // Publishing: changed values only, split across messages, flags cleared.
    memset(&st, 0, sizeof st);
    bool more;
    char msg[256];
    assert(gx5_can_chunk(&st, false, 5, msg, sizeof msg, &more) == 0 && !more);
    uint8_t rpm[2] = {0x1A, 0xF8}, spd = 0x32;
    gx5_can_store(&st, 0x7E8, 1, 0x0C, rpm, 2);
    gx5_can_store(&st, 0x7E8, 1, 0x0D, &spd, 1);
    assert(gx5_can_chunk(&st, false, 1794745845, msg, sizeof msg, &more) == 2 && !more);
    assert(!strcmp(msg, "{\"ts\":1794745845,\"obd\":[\"7E8 01 0C 1AF8\",\"7E8 01 0D 32\"]}"));
    assert(gx5_can_chunk(&st, false, 5, msg, sizeof msg, &more) == 0);   // nothing changed since
    gx5_can_store(&st, 0x7E8, 1, 0x0D, &spd, 1);
    assert(gx5_can_chunk(&st, false, 5, msg, sizeof msg, &more) == 0);   // same value again
    for (int i = 0; i < 6; i++) gx5_can_store(&st, 0xFE00 + i, 0, 0, big, 60); // 6 x 128 chars: needs several messages
    int total = 0, rounds = 0;
    do {
        int k = gx5_can_chunk(&st, true, 5, msg, sizeof msg, &more);
        assert(strlen(msg) < sizeof msg && msg[strlen(msg) - 1] == '}');
        total += k;
        rounds++;
    } while (more && rounds < 10);
    assert(total == 6 && rounds == 6);
    gx5_can_store(&st, 0xFE00, 0, 0, big, 60);
    big[0] = 9;
    gx5_can_store(&st, 0xFE00, 0, 0, big, 60);
    assert(gx5_can_chunk(&st, true, 5, msg, 40, &more) == 0 && !more);   // cannot fit at all: skipped, no loop
    assert(gx5_can_chunk(&st, true, 5, msg, sizeof msg, &more) == 0);

    // ── Vehicle profile ──
    static gx5_can_cfg_t c;
    gx5_can_defaults(&c, 500000);
    assert(c.ign.src == GX5_IGN_ACC && c.st[GX5_ST_ON].mode == GX5_CAN_LISTEN && c.st[GX5_ST_PRE].mode == GX5_CAN_OFF);

    // Library requests.
    assert(gx5_parse_pid("{\"pid\":{\"n\":0,\"id\":\"7E0\",\"data\":\"02010C\"}}", &c) == 0);
    assert(c.cmd[0].id == 0x7E0 && !c.cmd[0].ext && c.cmd[0].dlc == 8 && c.cmd[0].data[2] == 0x0C && !c.cmd[0].data[3]); // padded to 8
    assert(gx5_parse_pid("{\"pid\":{\"n\":1,\"id\":\"7C6\",\"data\":\"0322B002\"}}", &c) == 1);
    assert(gx5_parse_pid("{\"pid\":{\"n\":2,\"id\":\"18EA00F8\",\"data\":\"EEFE00\"}}", &c) == 2 && c.cmd[2].ext && c.cmd[2].dlc == 3); // J1939: as given
    assert(gx5_parse_pid("{\"pid\":{\"n\":2}}", &c) == 2 && !c.cmd[2].dlc); // clear
    assert(gx5_parse_pid("{\"pid\":{\"n\":32,\"id\":\"7DF\",\"data\":\"0201\"}}", &c) == -1);
    assert(gx5_parse_pid("{\"pid\":{\"n\":1,\"id\":\"7DG\",\"data\":\"0201\"}}", &c) == -1);
    assert(gx5_parse_pid("{\"pid\":{\"n\":1,\"id\":\"7DF\",\"data\":\"020\"}}", &c) == -1);
    assert(gx5_parse_pid("{\"pid\":{\"n\":1,\"id\":\"7DF\",\"data\":\"010203040506070809\"}}", &c) == -1);
    assert(gx5_parse_pid("{\"pid\":{\"n\":1,\"id\":\"FFFFFFFFF\",\"data\":\"01\"}}", &c) == -1);
    assert(gx5_parse_pid("{\"ping\":1}", &c) == -1 && c.cmd[1].id == 0x7C6); // failures change nothing

    // Signals, OBDb style.
    assert(gx5_parse_sig("{\"sig\":{\"n\":0,\"key\":\"rpm\",\"pid\":0,\"bix\":0,\"len\":16,\"div\":4}}", &c) == 0);
    assert(gx5_parse_sig("{\"sig\":{\"n\":1,\"key\":\"fuel_l\",\"pid\":1,\"bix\":32,\"len\":8,\"div\":2}}", &c) == 1);
    assert(gx5_parse_sig("{\"sig\":{\"n\":2,\"key\":\"bat_v\",\"pid\":1,\"bix\":40,\"len\":8,\"div\":12.8}}", &c) == 2);
    assert(gx5_parse_sig("{\"sig\":{\"n\":3,\"key\":\"odo_km\",\"pid\":1,\"bix\":48,\"len\":24}}", &c) == 3);
    assert(gx5_parse_sig("{\"sig\":{\"n\":4,\"key\":\"x\",\"pid\":1,\"bix\":4,\"len\":16,\"blsb\":true}}", &c) == -1); // not whole bytes
    assert(gx5_parse_sig("{\"sig\":{\"n\":4,\"key\":\"x\",\"pid\":40}}", &c) == -1);
    assert(gx5_parse_sig("{\"sig\":{\"n\":4,\"key\":\"a\\\"b\",\"pid\":1}}", &c) == -1);
    assert(gx5_parse_sig("{\"sig\":{\"n\":4,\"key\":\"x\",\"pid\":1,\"div\":0}}", &c) == -1);
    assert(gx5_parse_sig("{\"sig\":{\"n\":3}}", &c) == 3 && !c.sig[3].key[0]); // clear
    assert(gx5_parse_sig("{\"sig\":{\"n\":3,\"key\":\"odo_km\",\"pid\":1,\"bix\":48,\"len\":24}}", &c) == 3);

    // Decoding, checked against OBDb/Kia-Cerato tests/test_cases/2021/commands/7C6.7CE.22B002.yaml:
    // FLI_VOL 11.5, CLUSTER_VOLTAGE 13.04688, ODO_MI 56595.
    memset(&st, 0, sizeof st);
    f = fr(0x7CE, "100F62B002D00000");
    assert(gx5_obd_feed(&st, &f, &reply, &done) && reply.id == 0x7C6);
    f = fr(0x7CE, "210017A700000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    f = fr(0x7CE, "22DD130000000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    assert(done);
    v = gx5_cmd_val(&st, &c.cmd[1]);
    double x;
    assert(v && v->len == 12 && gx5_sig_value(&c.sig[1], v->data, v->len, &x) && x == 11.5);
    assert(gx5_sig_value(&c.sig[2], v->data, v->len, &x) && x > 13.046 && x < 13.047);
    gx5_can_sig_t mi = {.key = "odo_mi", .cmd = 1, .bix = 72, .len = 24, .mul = 1};
    assert(gx5_sig_value(&mi, v->data, v->len, &x) && x == 56595);
    mi.bix = 80; // runs past the answer
    assert(!gx5_sig_value(&mi, v->data, v->len, &x));
    // Little-endian and signed.
    uint8_t raw[] = {0x34, 0x12, 0xFE, 0xFF, 0x80};
    gx5_can_sig_t le = {.bix = 0, .len = 16, .flags = GX5_SIG_LE, .mul = 1};
    assert(gx5_sig_value(&le, raw, sizeof raw, &x) && x == 0x1234);
    le = (gx5_can_sig_t){.bix = 16, .len = 16, .flags = GX5_SIG_LE | GX5_SIG_SIGNED, .mul = 1};
    assert(gx5_sig_value(&le, raw, sizeof raw, &x) && x == -2);
    le = (gx5_can_sig_t){.bix = 32, .len = 8, .flags = GX5_SIG_SIGNED, .mul = 0.5f, .add = 1};
    assert(gx5_sig_value(&le, raw, sizeof raw, &x) && x == -63);
    le = (gx5_can_sig_t){.bix = 39, .len = 1, .mul = 1}; // one bit: the last of byte 4
    assert(gx5_sig_value(&le, raw, sizeof raw, &x) && x == 0);

    // Published: decoded signals by name; that answer then stays out of the raw list.
    f = fr(0x7E8, "04410C1AF8000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    f = fr(0x7E8, "03410D2800000000"); // speed: no signal defined
    gx5_obd_feed(&st, &f, &reply, &done);
    int next = 0;
    assert(gx5_sig_chunk(&c, &st, 7, msg, sizeof msg, &next) == 4 && next == GX5_CAN_SIGS);
    assert(!strcmp(msg, "{\"ts\":7,\"sig\":{\"rpm\":1726,\"fuel_l\":11.5,\"bat_v\":13.0469,\"odo_km\":0}}"));
    next = 0;
    assert(gx5_sig_chunk(&c, &st, 7, msg, 44, &next) == 2 && next == 2); // small buffer: continues at signal 2
    assert(gx5_sig_chunk(&c, &st, 7, msg, 44, &next) == 2 && next == GX5_CAN_SIGS);
    gx5_sig_sent(&c, &st);
    next = 0;
    assert(gx5_sig_chunk(&c, &st, 7, msg, sizeof msg, &next) == 0);
    assert(gx5_can_chunk(&st, false, 7, msg, sizeof msg, &more) == 1 && !strcmp(msg, "{\"ts\":7,\"obd\":[\"7E8 01 0D 28\"]}"));

    // State lists.
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"mode\":\"obd2\",\"poll\":[[0,1],[1,0.5], [3,60]]}}", &c) == GX5_ST_ON);
    assert(c.st[GX5_ST_ON].mode == GX5_CAN_OBD2 && c.st[GX5_ST_ON].poll[0].ds == 10 && c.st[GX5_ST_ON].poll[1].cmd == 1 &&
           c.st[GX5_ST_ON].poll[1].ds == 5 && c.st[GX5_ST_ON].poll[2].ds == 600 && !c.st[GX5_ST_ON].poll[3].ds);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"off\",\"mode\":\"listen\"}}", &c) == GX5_ST_OFF && c.st[GX5_ST_OFF].mode == GX5_CAN_LISTEN);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"poll\":[]}}", &c) == GX5_ST_ON && !c.st[GX5_ST_ON].poll[0].ds && c.st[GX5_ST_ON].mode == GX5_CAN_OBD2);
    assert(gx5_parse_can_state("{\"can\":{\"mode\":\"obd2\"}}", &c) == -1);                                  // which state?
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"mode\":\"canopen\"}}", &c) == -1);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"poll\":[[32,1]]}}", &c) == -1);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"poll\":[[0,0]]}}", &c) == -1);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"poll\":[[0,1]\"}}", &c) == -1);
    assert(gx5_parse_can_state("{\"can\":{\"state\":\"on\",\"poll\":[[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1],[0,1]]}}", &c) == -1);

    // Vehicle header and ignition rule.
    assert(gx5_parse_vehicle("{\"vehicle\":{\"name\":\"kia-cerato-2021\",\"bitrate\":500000,\"pre_s\":90,\"ign\":{\"src\":\"sig\",\"key\":\"rpm\",\"min\":1,\"hold_s\":5}}}", &c));
    assert(!strcmp(c.name, "kia-cerato-2021") && c.pre_s == 90 && c.ign.src == GX5_IGN_SIG && !strcmp(c.ign.key, "rpm") && c.ign.min == 1 && c.ign.hold_s == 5);
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"ign\":{\"src\":\"gps\"}}}", &c));
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"bitrate\":5}}", &c));
    assert(!gx5_parse_vehicle("{\"can\":{\"state\":\"on\"}}", &c));
    static gx5_can_cfg_t bad;
    gx5_can_defaults(&bad, 500000);
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"ign\":{\"src\":\"vcc\"}}}", &bad));  // thresholds missing
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"ign\":{\"src\":\"vcc\",\"on_mv\":12800,\"off_mv\":13200}}}", &bad));
    assert(gx5_parse_vehicle("{\"vehicle\":{\"ign\":{\"src\":\"vcc\",\"on_mv\":13200,\"off_mv\":12800}}}", &bad) && bad.ign.on_mv == 13200);

    // Ignition from an answered signal: engine speed >= 1.
    gx5_ign_t ign = {0};
    assert(!gx5_ign_on(&c.ign, &ign, 1000));
    memset(&st, 0, sizeof st);
    f = fr(0x7E8, "04410C0000000000"); // 0 rpm: key on, engine off
    gx5_obd_feed(&st, &f, &reply, &done);
    gx5_ign_answer(&c, &st, &ign, f.id, 1000);
    assert(!gx5_ign_on(&c.ign, &ign, 1000));
    f = fr(0x7E8, "04410C1AF8000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    gx5_ign_answer(&c, &st, &ign, f.id, 2000);
    assert(gx5_ign_on(&c.ign, &ign, 2000) && gx5_ign_on(&c.ign, &ign, 6999) && !gx5_ign_on(&c.ign, &ign, 7000)); // no answer for hold_s
    gx5_ign_answer(&c, &st, &ign, 0x7E9, 8000); // another ECU answering proves nothing
    assert(!gx5_ign_on(&c.ign, &ign, 8000));
    c.ign.min = -1e30f; // any answer counts
    f = fr(0x7E8, "04410C0000000000");
    gx5_obd_feed(&st, &f, &reply, &done);
    gx5_ign_answer(&c, &st, &ign, f.id, 9000);
    assert(gx5_ign_on(&c.ign, &ign, 9000));

    // Ignition from a broadcast frame (GX5sV2's Kia rule: 0x541 byte 0 == 0x03).
    assert(gx5_parse_vehicle("{\"vehicle\":{\"ign\":{\"src\":\"frame\",\"id\":\"541\",\"at\":0,\"eq\":\"03\",\"hold_s\":3}}}", &c));
    assert(c.ign.src == GX5_IGN_FRAME && c.ign.id == 0x541 && c.ign.eq == 3 && c.ign.mask == 0xFF);
    ign.seen_ms = 0;
    f = fr(0x541, "0100000000000000");
    gx5_ign_frame(&c.ign, &ign, &f, 1000);
    assert(!gx5_ign_on(&c.ign, &ign, 1000));
    f = fr(0x541, "0300000000000000");
    gx5_ign_frame(&c.ign, &ign, &f, 2000);
    assert(gx5_ign_on(&c.ign, &ign, 4999) && !gx5_ign_on(&c.ign, &ign, 5000));
    f = fr(0x542, "0300000000000000");
    gx5_ign_frame(&c.ign, &ign, &f, 6000);
    assert(!gx5_ign_on(&c.ign, &ign, 6000));

    // VIN from the answers that carry one.
    char vin[18];
    assert(gx5_vin_from((const uint8_t *)"\x01" "1GX5SBENCHTEST001", 18, vin) && !strcmp(vin, "1GX5SBENCHTEST001")); // 09 02
    assert(gx5_vin_from((const uint8_t *)"1GX5SBENCHTEST001", 17, vin) && !strcmp(vin, "1GX5SBENCHTEST001"));         // 22 F190
    assert(gx5_vin_from((const uint8_t *)"1GX5SBENCHTEST001*", 18, vin) && !strcmp(vin, "1GX5SBENCHTEST001"));        // J1939 FEEC
    assert(!gx5_vin_from((const uint8_t *)"1GX5SBENCH", 10, vin) && !gx5_vin_from((const uint8_t *)"1GX5SBENCH\xFFTEST001", 17, vin));
    assert(gx5_parse_vehicle("{\"vehicle\":{\"vin_pid\":2}}", &c) && c.vin_cmd == 2);
    assert(gx5_parse_vehicle("{\"vehicle\":{\"vin_pid\":-1}}", &c) && c.vin_cmd == 0xFF);
    assert(!c.wake_vcc && gx5_parse_vehicle("{\"vehicle\":{\"wake\":\"vcc\"}}", &c) && c.wake_vcc);
    assert(gx5_parse_vehicle("{\"vehicle\":{\"name\":\"x\"}}", &c) && c.wake_vcc); // kept
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"wake\":\"gps\"}}", &c));
    assert(gx5_parse_vehicle("{\"vehicle\":{\"wake\":\"can\"}}", &c) && !c.wake_vcc);

    // Trip signals: last good values, "no value" readings dropped.
    static gx5_can_cfg_t tc;
    static gx5_trip_t trip;
    char tj[160];
    gx5_can_defaults(&tc, 500000);
    assert(gx5_parse_pid("{\"pid\":{\"n\":2,\"id\":\"7C6\",\"data\":\"0322B002\"}}", &tc) == 2);
    assert(gx5_parse_sig("{\"sig\":{\"n\":0,\"key\":\"fuel_l\",\"pid\":2,\"bix\":32,\"len\":8,\"div\":2,\"null\":127.5}}", &tc) == 0);
    assert(gx5_parse_sig("{\"sig\":{\"n\":1,\"key\":\"odo_km\",\"pid\":2,\"bix\":48,\"len\":24}}", &tc) == 1);
    assert(gx5_parse_vehicle("{\"vehicle\":{\"trip\":[\"odo_km\", \"fuel_l\",\"run_s\"]}}", &tc) && !strcmp(tc.trip[1], "fuel_l") && !tc.trip[3][0]);
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"trip\":[\"a\",\"b\",\"c\",\"d\",\"e\"]}}", &tc) && !strcmp(tc.trip[0], "odo_km"));
    assert(!gx5_parse_vehicle("{\"vehicle\":{\"trip\":[\"a_name_too_long\"]}}", &tc));
    memset(&st, 0, sizeof st);
    gx5_trip_json(&tc, &trip, tj, sizeof tj);
    assert(!tj[0]); // nothing read yet
    uint8_t b002[] = {0xD0, 0, 0, 0, 0x15, 0xA7, 0x01, 0x34, 0x22, 0, 0, 0}; // fuel 10.5 L, odo 78882 km
    gx5_can_store(&st, 0x7CE, 0x22, 0xB002, b002, sizeof b002);
    gx5_trip_update(&tc, &st, &trip);
    gx5_trip_json(&tc, &trip, tj, sizeof tj);
    assert(!strcmp(tj, ",\"odo_km\":78882,\"fuel_l\":10.5"));
    b002[4] = 0xFF; // ignition off: the cluster says "no fuel value" (127.5)
    b002[8] = 0x23;
    gx5_can_store(&st, 0x7CE, 0x22, 0xB002, b002, sizeof b002);
    gx5_trip_update(&tc, &st, &trip);
    gx5_trip_json(&tc, &trip, tj, sizeof tj);
    assert(!strcmp(tj, ",\"odo_km\":78883,\"fuel_l\":10.5")); // fuel keeps its last good value
    gx5_trip_json(&tc, &trip, tj, 20);
    assert(!strcmp(tj, ",\"odo_km\":78883"));                    // what does not fit is left out whole

    puts("ok");
    return 0;
}
