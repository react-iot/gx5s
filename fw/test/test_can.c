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
    cfg.req[0] = (gx5_can_req_t){.frame = fr(0x18EA00F8, "EEFE00"), .interval_s = 30};
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

    // Commands.
    gx5_can_cfg_t c = {.mode = GX5_CAN_LISTEN, .bitrate = 500000};
    assert(gx5_parse_can_cfg("{\"can\":{\"mode\":\"j1939\",\"bitrate\":250000,\"filter\":true}}", &c));
    assert(c.mode == GX5_CAN_J1939 && c.bitrate == 250000 && c.filter);
    assert(gx5_parse_can_cfg("{\"can\":{\"mode\":\"off\"}}", &c) && c.mode == GX5_CAN_OFF && c.bitrate == 250000);
    assert(!gx5_parse_can_cfg("{\"can\":{\"mode\":\"canopen\"}}", &c));
    assert(!gx5_parse_can_cfg("{\"can\":{\"bitrate\":5}}", &c));
    assert(!gx5_parse_can_cfg("{\"can_req\":{\"slot\":1}}", &c));

    gx5_can_req_t r;
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":3,\"id\":\"7DF\",\"data\":\"02010C\",\"interval\":10}}", &r) == 3);
    assert(r.frame.id == 0x7DF && !r.frame.ext && r.frame.dlc == 3 && r.frame.data[2] == 0x0C && r.interval_s == 10);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":0,\"id\":\"18EA00F8\",\"data\":\"EEFE00\",\"interval\":30}}", &r) == 0 && r.frame.ext);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":5,\"interval\":0}}", &r) == 5 && !r.interval_s); // clear
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":16,\"id\":\"7DF\",\"data\":\"0201\",\"interval\":1}}", &r) == -1);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":1,\"id\":\"7DG\",\"data\":\"0201\",\"interval\":1}}", &r) == -1);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":1,\"id\":\"7DF\",\"data\":\"020\",\"interval\":1}}", &r) == -1);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":1,\"id\":\"7DF\",\"data\":\"010203040506070809\",\"interval\":1}}", &r) == -1);
    assert(gx5_parse_can_req("{\"can_req\":{\"slot\":1,\"id\":\"FFFFFFFFF\",\"data\":\"01\",\"interval\":1}}", &r) == -1);
    assert(gx5_parse_can_req("{\"ping\":1}", &r) == -1);

    puts("ok");
    return 0;
}
