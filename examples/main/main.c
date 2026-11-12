/*
 * Check ToF - VL53L8CX ranging on ESP32-C6 (I2C)
 *
 * Wiring:
 *   ESP32-C6 3V3   -> VIN
 *   ESP32-C6 3V3   -> LPn          (high = sensor enabled)
 *   ESP32-C6 GND   -> GND
 *   ESP32-C6 GND   -> SPI_I2C_N    (low = I2C mode)
 *   ESP32-C6 GPIO6 -> SDA/MOSI
 *   ESP32-C6 GPIO7 -> SCL/CLK
 *
 * Config: 8x8 zones (64), 1 target per zone, 15 Hz continuous ranging (max for 8x8).
 * The sensor runs with ST defaults except target order = CLOSEST. No filtering happens
 * here: every zone is sent as the sensor reports it, all filtering is done in the web GUI.
 *
 * Output (UART, 921600 baud), one line per frame, read by gui/server.py:
 *   F2,<frame>,<t_ms>,<d0..d63>,<s0..s63>,<g0..g63>
 *     frame : sensor stream counter (0..255, +1 per frame -> gaps = missed frames)
 *     t_ms  : ESP32 uptime in ms when the frame was read
 *     d     : distance in mm as reported by the sensor (meaningless if s says so)
 *     s     : target_status (5 = valid, 9 = valid large pulse, 255 = no target, ...)
 *     g     : range sigma in mm (estimated uncertainty)
 *   All values are row-major (row 0 first). All other lines are ESP-IDF log lines;
 *   at start-up a "CFG ..." log line reports the configuration actually applied.
 *
 * Robustness: the firmware never gives up. If the sensor cannot be initialised it
 * retries forever, and if frames stop arriving while ranging it re-initialises the
 * sensor (e.g. after an MCU reset left the sensor mid-transfer, or a loose wire).
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "vl53l8cx_api.h"

#define TAG "check_tof"

#define PIN_SDA            6
#define PIN_SCL            7
#define I2C_SPEED_HZ       400000   /* 400 kHz; raise only with proper external pull-ups */
#define RANGING_HZ         15       /* max ranging frequency for 8x8 */
#define GRID               8        /* 8x8 = 64 zones */
#define FRAME_TIMEOUT_US   1000000  /* no frame for 1 s (expected every ~67 ms) -> restart sensor */
#define MAX_I2C_ERRORS     10       /* consecutive I2C errors while ranging -> restart sensor */

static i2c_master_bus_handle_t s_bus;


/* Log what the I2C bus and the sensor look like right now. Used when something fails so the
 * cause is visible: probe NOT_FOUND = nobody ACKs (sensor off/reset), TIMEOUT or SDA/SCL low
 * = bus held by someone, OK = sensor answers (then the problem is above the bus). */
static void log_bus_state(const char *why)
{
    esp_err_t p = i2c_master_probe(s_bus, VL53L8CX_DEFAULT_I2C_ADDRESS >> 1, 200);
    ESP_LOGW(TAG, "%s: probe(0x29)=%s SDA=%d SCL=%d", why, esp_err_to_name(p),
             gpio_get_level(PIN_SDA), gpio_get_level(PIN_SCL));
}

/* Detect, load firmware, configure and start ranging. Returns true on success. */
static bool sensor_start(VL53L8CX_Configuration *dev)
{
    uint8_t alive = 0;
    uint8_t status = vl53l8cx_is_alive(dev, &alive);
    if (status || !alive) {
        ESP_LOGW(TAG, "VL53L8CX not responding (status=%u, alive=%u)", status, alive);
        log_bus_state("is_alive failed");
        return false;
    }

    ESP_LOGI(TAG, "Loading sensor firmware (takes a few seconds)...");
    status = vl53l8cx_init(dev);
    if (status) {
        ESP_LOGW(TAG, "vl53l8cx_init failed (status=%u)", status);
        return false;
    }

    status  = vl53l8cx_set_resolution(dev, VL53L8CX_RESOLUTION_8X8);
    status |= vl53l8cx_set_ranging_frequency_hz(dev, RANGING_HZ);
    /* Report the closest target in each zone. Sharpener stays at the sensor default. */
    status |= vl53l8cx_set_target_order(dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    status |= vl53l8cx_start_ranging(dev);
    if (status) {
        ESP_LOGW(TAG, "configure/start_ranging failed (status=%u)", status);
        return false;
    }

    /* Read the configuration back from the sensor so the GUI shows what is really applied. */
    uint8_t res = 0, hz = 0, sharp = 0, order = 0;
    vl53l8cx_get_resolution(dev, &res);
    vl53l8cx_get_ranging_frequency_hz(dev, &hz);
    vl53l8cx_get_sharpener_percent(dev, &sharp);
    vl53l8cx_get_target_order(dev, &order);
    ESP_LOGI(TAG, "VL53L8CX ready (ULD %s), ranging 8x8 @ %d Hz", VL53L8CX_API_REVISION, RANGING_HZ);
    ESP_LOGI(TAG, "CFG zones=%u hz=%u sharpener=%u order=%s i2c_hz=%d", res, hz, sharp,
             order == VL53L8CX_TARGET_ORDER_CLOSEST ? "closest" : "strongest", I2C_SPEED_HZ);
    return true;
}

/* Append an unsigned/signed integer as decimal text; returns the new end pointer. */
static char *put_int(char *p, int v)
{
    char tmp[12];
    int n = 0;
    unsigned u = v < 0 ? (unsigned)(-v) : (unsigned)v;
    do { tmp[n++] = (char)('0' + u % 10); u /= 10; } while (u);
    if (v < 0) *p++ = '-';
    while (n) *p++ = tmp[--n];
    return p;
}

/* Send one frame as a single line, written with ONE call so the line is never
 * interleaved with log output and the UART is busy for the shortest possible time. */
static void print_frame(const VL53L8CX_Configuration *dev, const VL53L8CX_ResultsData *res)
{
    static char line[1280];   /* worst case: 20 + 64 * (7 + 4 + 6) = 1108 bytes; typical ~650 */
    char *p = line, *end = line + sizeof(line) - 8;
    const int nz = GRID * GRID;

    memcpy(p, "F2,", 3); p += 3;
    p = put_int(p, dev->streamcount); *p++ = ',';
    p = put_int(p, (int)(esp_timer_get_time() / 1000));
    for (int z = 0; z < nz && p < end; z++) {                       /* distance (mm) */
        *p++ = ','; p = put_int(p, res->distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * z]);
    }
    for (int z = 0; z < nz && p < end; z++) {                       /* status */
        int st = res->target_status[VL53L8CX_NB_TARGET_PER_ZONE * z];
        if (res->nb_target_detected[z] == 0) st = 255;              /* ST convention: no target */
        *p++ = ','; p = put_int(p, st);
    }
    for (int z = 0; z < nz && p < end; z++) {                       /* sigma (mm) */
        *p++ = ','; p = put_int(p, res->range_sigma_mm[VL53L8CX_NB_TARGET_PER_ZONE * z]);
    }
    *p++ = '\n';
    fwrite(line, 1, (size_t)(p - line), stdout);
    fflush(stdout);
}

void app_main(void)
{
    /* ---- I2C bus ---- */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,              /* ESP32-C6 has a single HP I2C port */
        .scl_io_num = PIN_SCL,
        .sda_io_num = PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* weak; add 2.2-4.7k external if unstable */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L8CX_DEFAULT_I2C_ADDRESS >> 1,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    static VL53L8CX_Configuration dev;      /* large: keep off the stack */
    static VL53L8CX_ResultsData results;
    dev.platform.bus_config = bus_cfg;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &dev.platform.handle));
    /* LPn is tied to 3V3, so no hardware reset (VL53L8CX_Reset_Sensor) is possible. */

    int attempt = 0;
    while (1) {
        /* ---- (re)start the sensor, forever ---- */
        /* If the MCU was reset in the middle of an I2C transfer the sensor may still
         * be driving SDA; clearing the bus first releases it. */
        i2c_master_bus_reset(s_bus);
        if (!sensor_start(&dev)) {
            attempt++;
            ESP_LOGE(TAG, "Sensor start failed (attempt %d) - check wiring/pull-ups, retrying", attempt);
            VL53L8CX_WaitMs(&dev.platform, attempt < 5 ? 200 : 1000);
            continue;
        }
        attempt = 0;

        /* ---- ranging loop with watchdog ---- */
        int64_t last_frame = esp_timer_get_time();
        int i2c_errors = 0;
        while (1) {
            uint8_t ready = 0;
            uint8_t st = vl53l8cx_check_data_ready(&dev, &ready);

            if (!st && ready) {
                st = vl53l8cx_get_ranging_data(&dev, &results);
                if (!st) {
                    print_frame(&dev, &results);
                    last_frame = esp_timer_get_time();
                }
            }
            i2c_errors = st ? i2c_errors + 1 : 0;

            if (i2c_errors >= MAX_I2C_ERRORS) {
                ESP_LOGW(TAG, "Too many I2C errors (last status=%u) - restarting sensor", st);
                log_bus_state("before restart");
                break;
            }
            if (esp_timer_get_time() - last_frame > FRAME_TIMEOUT_US) {
                ESP_LOGW(TAG, "No frame for %d ms - restarting sensor", FRAME_TIMEOUT_US / 1000);
                log_bus_state("before restart");
                break;
            }
            VL53L8CX_WaitMs(&dev.platform, 5);
        }
        vl53l8cx_stop_ranging(&dev);         /* best effort */
    }
}
