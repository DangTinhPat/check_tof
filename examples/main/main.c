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
 *
 * Output (UART, 115200), one line per frame, read by gui/server.py:
 *   F,<frame>,<d0>,<d1>,...,<d63>     distance in mm, row-major, 0 = invalid zone
 * All other lines are ESP-IDF log lines.
 *
 * Robustness: the firmware never gives up. If the sensor cannot be initialised it
 * retries forever, and if frames stop arriving while ranging it re-initialises the
 * sensor (e.g. after an MCU reset left the sensor mid-transfer, or a loose wire).
 */

#include <stdio.h>
#include <stdbool.h>
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
#define SHARPENER_PERCENT  20       /* 0..99; higher = crisper edges between objects */
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
    /* Sharper object edges: less blur from a bright/close target into neighbour
     * zones (ST default is 5%), and report the closest target in each zone. */
    status |= vl53l8cx_set_sharpener_percent(dev, SHARPENER_PERCENT);
    status |= vl53l8cx_set_target_order(dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    status |= vl53l8cx_start_ranging(dev);
    if (status) {
        ESP_LOGW(TAG, "configure/start_ranging failed (status=%u)", status);
        return false;
    }

    ESP_LOGI(TAG, "VL53L8CX ready (ULD %s), ranging 8x8 @ %d Hz", VL53L8CX_API_REVISION, RANGING_HZ);
    return true;
}

static void print_frame(const VL53L8CX_Configuration *dev, const VL53L8CX_ResultsData *res)
{
    printf("F,%u", dev->streamcount);
    for (int z = 0; z < GRID * GRID; z++) {
        int idx = VL53L8CX_NB_TARGET_PER_ZONE * z;
        uint8_t st = res->target_status[idx];
        /* status 5 = valid, 9 = valid with large pulse */
        int valid = (st == 5 || st == 9);
        printf(",%d", valid ? res->distance_mm[idx] : 0);
    }
    printf("\n");
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
