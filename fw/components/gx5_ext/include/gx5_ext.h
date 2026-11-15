// gx5_ext — the external UART on the wiring harness (SAT-TX / SAT-RX), line based.
// Each received line is posted as GX5_EV_EXT_RX (data: the line, NUL-terminated,
// at most GX5_EXT_LINE - 1 characters). Needs the default event loop.
#pragma once
#define GX5_EXT_LINE 128

void gx5_ext_init(void);
void gx5_ext_send(const char *line); // adds CR LF
