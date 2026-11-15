// gx5_store — offline queue on the SD NAND (FAT over SPI).
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gx5.h"
#include "gx5_store.h"
#include "sdmmc_cmd.h"

// The card is powered and mounted only for the duration of each operation.

#define QUEUE_FILE "/sd/queue.txt"
#define POS_FILE   "/sd/queue.pos" // byte offset of the first unsent line

#ifdef CONFIG_GX5_STORE
static const char *TAG = "store";
static sdmmc_card_t *s_card;
static RTC_DATA_ATTR bool s_queued = true; // unknown after power-on: look once

static void sd_unmount(void)
{
    if (s_card) esp_vfs_fat_sdcard_unmount("/sd", s_card);
    s_card = NULL;
    spi_bus_free(SPI2_HOST);
    gpio_set_level(GX5_PIN_SD_3V3, 0);
}

static bool sd_mount(void)
{
    spi_bus_config_t bus = {.mosi_io_num = GX5_PIN_SD_MOSI, .miso_io_num = GX5_PIN_SD_MISO, .sclk_io_num = GX5_PIN_SD_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1};
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    esp_vfs_fat_mount_config_t mount = {.format_if_mount_failed = true, .max_files = 2};

    host.slot = SPI2_HOST;
    slot.host_id = SPI2_HOST;
    slot.gpio_cs = GX5_PIN_SD_CS;
    // GPIO2 is the NAND's supply (no switch). At the default drive strength the
    // card mounts but every write times out (bench 2026-11-15); writes need up to 30 mA.
    gpio_set_drive_capability(GX5_PIN_SD_3V3, GPIO_DRIVE_CAP_3);
    gpio_set_level(GX5_PIN_SD_3V3, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (spi_bus_initialize(SPI2_HOST, &bus, SDSPI_DEFAULT_DMA) == ESP_OK && esp_vfs_fat_sdspi_mount("/sd", &host, &slot, &mount, &s_card) == ESP_OK) return true;
    ESP_LOGW(TAG, "SD NAND not available");
    s_card = NULL;
    sd_unmount();
    return false;
}

bool gx5_store_push(const char *line)
{
    struct stat st;
    bool ok = false;
    if (!sd_mount()) return false;
    if (stat(QUEUE_FILE, &st) == 0 && st.st_size > CONFIG_GX5_STORE_MAX_KB * 1024) {
        ESP_LOGW(TAG, "offline queue full, dropping");
    } else {
        FILE *f = fopen(QUEUE_FILE, "a");
        if (f) {
            ok = fprintf(f, "%s\n", line) > 0;
            ok = fclose(f) == 0 && ok; // the write only reaches the card here
        }
    }
    s_queued = true;
    sd_unmount();
    return ok;
}

// Hand up to `max` queued lines to `send`, oldest first. A line counts as sent once
// `send` accepts it (at most once from there on). Returns the number handed over.
int gx5_store_drain(int max, bool (*send)(const char *line))
{
    char line[GX5_PUB_MAX];
    long pos = 0;
    int n = 0;
    FILE *f, *p;

    if (!s_queued || !sd_mount()) return 0;
    if ((p = fopen(POS_FILE, "r"))) {
        if (fscanf(p, "%ld", &pos) != 1) pos = 0;
        fclose(p);
    }
    bool done = true;
    if ((f = fopen(QUEUE_FILE, "r"))) {
        fseek(f, pos, SEEK_SET);
        while (n < max && fgets(line, sizeof line, f)) {
            line[strcspn(line, "\r\n")] = '\0';
            if (line[0] && !send(line)) break;
            pos = ftell(f);
            n++;
        }
        done = n < max && feof(f);
        fclose(f);
    }
    if (done) {
        remove(QUEUE_FILE);
        remove(POS_FILE);
        s_queued = false;
    } else if ((p = fopen(POS_FILE, "w"))) {
        fprintf(p, "%ld", pos);
        fclose(p);
    }
    sd_unmount();
    return n;
}

void gx5_store_clear(void)
{
    if (!sd_mount()) return;
    remove(QUEUE_FILE);
    remove(POS_FILE);
    s_queued = false;
    sd_unmount();
}
#else
void gx5_store_clear(void) {}
bool gx5_store_push(const char *line) { return false; }
int gx5_store_drain(int max, bool (*send)(const char *line)) { return 0; }
#endif
