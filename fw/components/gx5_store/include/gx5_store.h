// gx5_store — line queue on the SD NAND for data that could not be sent.
// The SD-3V3 rail GPIO must be configured as an output before use.
#pragma once
#include <stdbool.h>

bool gx5_store_push(const char *line); // append one line
void gx5_store_clear(void);             // drop everything queued (factory reset)
// Hand up to `max` queued lines to `send`, oldest first; returns how many it took.
int gx5_store_drain(int max, bool (*send)(const char *line));
