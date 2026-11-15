#include <assert.h>
#include <stdio.h>
#include "parse.h"

int main(void)
{
    // Bench 2026-11-15: 767 mV on the node at 12.19 V (INA219), Vbe 605 mV.
    assert(abs(gx5_vcc_node_to_mv(767, 605) - 12190) < 100);
    assert(abs(gx5_vcc_node_to_mv(325, 605) - 4500) < 100);    // no harness: back-fed, Q4 off
    assert(gx5_vcc_node_to_mv(0, 605) == 300);
    for (int n = 1; n < 3300; n++) {                           // monotonic, no step at the knee
        int d = gx5_vcc_node_to_mv(n, 605) - gx5_vcc_node_to_mv(n - 1, 605);
        assert(d >= 0 && d <= 40);
    }

    assert(gx5_reg_ok("+CEREG: 0,1"));
    assert(gx5_reg_ok("+CEREG: 2,5,\"1A2B\",\"01234567\",8"));
    assert(!gx5_reg_ok("+CEREG: 0,2"));
    assert(!gx5_reg_ok("+CEREG:"));

    assert(gx5_last_int("+QMTOPEN: 0,0") == 0);
    assert(gx5_last_int("+QMTOPEN: 0,3") == 3);
    assert(gx5_last_int("+QMTSTAT") == -1);

    char a[] = "+QMTRECV: 0,7,\"gx5s/abc/cmd\",\"{\"x\":\"y\"}\"";
    char *t, *p;
    assert(gx5_parse_qmtrecv(a, &t, &p));
    assert(!strcmp(t, "gx5s/abc/cmd") && !strcmp(p, "{\"x\":\"y\"}"));
    char b[] = "+QMTRECV: 0,7,\"topic\"";
    assert(!gx5_parse_qmtrecv(b, &t, &p));
    char c[] = "+QMTRECV: 0,7";
    assert(!gx5_parse_qmtrecv(c, &t, &p));

    gx5_broker_t br;
    assert(gx5_parse_broker("{\"broker\":{\"host\":\"mq.example.com\",\"port\":1883,\"user\":\"u\",\"pass\":\"host\",\"tls\":false}}", &br));
    assert(!strcmp(br.host, "mq.example.com") && br.port == 1883 && !strcmp(br.user, "u") && !strcmp(br.pass, "host") && !br.tls);
    assert(gx5_parse_broker("{ \"broker\" : { \"host\" : \"10.0.0.1\" } }", &br));   // defaults
    assert(br.port == 8883 && br.tls && !br.user[0] && !br.pass[0]);
    assert(gx5_parse_broker("{\"broker\":{\"pass\":\"host\",\"host\":\"h\"}}", &br) && !strcmp(br.host, "h")); // value named like a key
    assert(!gx5_parse_broker("{\"ping\":1}", &br));                                   // not a broker message
    assert(!gx5_parse_broker("{\"broker\":{\"port\":1883}}", &br));                   // host required
    assert(!gx5_parse_broker("{\"broker\":{\"host\":\"\"}}", &br));
    assert(!gx5_parse_broker("{\"broker\":{\"host\":\"h\",\"port\":70000}}", &br));
    assert(!gx5_parse_broker("{\"broker\":{\"host\":\"h\",\"port\":0}}", &br));
    assert(!gx5_parse_broker("{\"broker\":{\"host\":123}}", &br));                    // wrong type
    assert(!gx5_parse_broker("{\"broker\":{\"host\":\"h\",\"user\":\"a\\\\\\\"b\"}}", &br)); // escapes rejected
    char longhost[200] = "{\"broker\":{\"host\":\"";
    memset(longhost + strlen(longhost), 'a', 100);
    strcat(longhost, "\"}}");
    assert(!gx5_parse_broker(longhost, &br));                                         // overlong rejected, not truncated

    assert(gx5_epoch(1970, 1, 1, 0, 0, 0) == 0);
    assert(gx5_epoch(2000, 3, 1, 0, 0, 0) == 951868800);       // after a leap day
    assert(gx5_epoch(2026, 11, 15, 12, 30, 45) == 1794745845);
    assert(gx5_epoch(2024, 2, 29, 23, 59, 59) == 1709251199);

    long long utc;
    assert(gx5_parse_cclk("+CCLK: \"26/11/15,22:30:45+40\"", &utc) && utc == 1794745845); // UTC+10
    assert(gx5_parse_cclk("+CCLK: \"26/11/15,07:30:45-20\"", &utc) && utc == 1794745845); // UTC-5
    assert(!gx5_parse_cclk("+CCLK: \"80/01/06,00:00:10+00\"", &utc));                     // never synced
    assert(!gx5_parse_cclk("+CCLK: garbage", &utc));
    assert(!gx5_parse_cclk("+CCLK: \"26/13/15,22:30:45+40\"", &utc));

    char pop[9];
    bool isp;
    gx5_pop((const uint8_t[]){0xC4, 0xDD, 0x57, 0xE0, 0x9F, 0xAF}, pop);
    assert(!strcmp(pop, "14720943"));
    gx5_pop((const uint8_t[]){0xC4, 0xDD, 0x57, 0x00, 0x00, 0x2A}, pop);
    assert(!strcmp(pop, "00000042")); // always 8 digits
    const char *pm = "{\"provision\":{\"pop\":\"00000042\",\"broker\":{\"host\":\"mq.example.com\"},\"config\":{\"can\":{\"mode\":\"obd2\"}}}}";
    const char *po = gx5_provision(pm, pop, &isp);
    assert(po && isp && gx5_parse_broker(po, &br) && !strcmp(br.host, "mq.example.com") && gx5_json_val(po, "config"));
    assert(!gx5_provision(pm, "14720943", &isp) && isp);                                   // wrong PIN
    assert(!gx5_provision("{\"provision\":{\"pop\":42,\"broker\":{\"host\":\"h\"}}}", pop, &isp) && isp);   // PIN must be a string
    assert(!gx5_provision("{\"provision\":{\"broker\":{\"host\":\"h\"}}}", pop, &isp) && isp);            // no PIN
    assert(!gx5_provision("{\"broker\":{\"host\":\"h\"}}", pop, &isp) && !isp);                              // not a provisioning message

    char fix[128];
    assert(gx5_fix_json("+QGPSLOC: 061951.0,-33.86785,151.20732,0.7,62.2,2,113.48,48.6,26.2,151126,09", fix, sizeof fix));
    assert(!strcmp(fix, ",\"lat\":-33.86785,\"lon\":151.20732,\"kmh\":48.6,\"cog\":113.48,\"sat\":9")); // "09" -> 9
    assert(!gx5_fix_json("+QGPSLOC: 061951.0,-33.86785,151.20732", fix, sizeof fix));                               // truncated
    assert(!gx5_fix_json("+QGPSLOC: 061951.0,3352.0710S,15112.4392E,0.7,62.2,2,113.48,48.6,26.2,151126,09", fix, sizeof fix)); // not decimal
    assert(!gx5_fix_json("+QGPSLOC: 061951.0,,151.20732,0.7,62.2,2,113.48,48.6,26.2,151126,09", fix, sizeof fix));
    assert(!gx5_fix_json("+QGPSLOC: 061951.0,-33.86785,151.20732,0.7,62.2,2,113.48,48.6,26.2,151126,09", fix, 20));   // no room
    assert(!gx5_fix_json("+QGPSLOC", fix, sizeof fix));

    puts("ok");
    return 0;
}
