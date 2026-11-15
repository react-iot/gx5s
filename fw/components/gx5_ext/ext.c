// gx5_ext — external UART on the wiring harness.
#include <string.h>
#include "driver/uart.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gx5.h"
#include "gx5_ext.h"

#ifdef CONFIG_GX5_EXT_UART
#define UART UART_NUM_2

// ponytail: bytes that arrive while the MCU is in automatic light sleep are lost
// (the pins are not IO_MUX UART pins). With ignition on the CAN driver keeps the
// MCU awake; add a PM lock here if the port must also work with ignition off.
static void ext_task(void *arg)
{
    char line[GX5_EXT_LINE];
    size_t n = 0;
    uint8_t c;
    for (;;) {
        if (uart_read_bytes(UART, &c, 1, portMAX_DELAY) != 1) continue;
        if (c == '\r' || c == '\n') {
            if (!n) continue;
            line[n] = '\0';
            esp_event_post(GX5_EVENT, GX5_EV_EXT_RX, line, n + 1, 0);
            n = 0;
        } else if (n < sizeof line - 1) {
            line[n++] = (char)c;
        }
    }
}

void gx5_ext_init(void)
{
    uart_config_t cfg = {.baud_rate = CONFIG_GX5_EXT_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .source_clk = UART_SCLK_DEFAULT};
    if (uart_driver_install(UART, 1024, 0, 0, NULL, 0) != ESP_OK || uart_param_config(UART, &cfg) != ESP_OK ||
        uart_set_pin(UART, CONFIG_GX5_EXT_TX_GPIO, CONFIG_GX5_EXT_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        return;
    xTaskCreate(ext_task, "ext", 3072, NULL, 5, NULL);
}

void gx5_ext_send(const char *line)
{
    uart_write_bytes(UART, line, strlen(line));
    uart_write_bytes(UART, "\r\n", 2);
}
#else
void gx5_ext_init(void) {}
void gx5_ext_send(const char *line) {}
#endif
