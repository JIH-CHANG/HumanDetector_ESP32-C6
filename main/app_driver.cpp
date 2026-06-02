/*
 * app_driver.cpp
 *
 * Hardware Driver Layer: ESP32-C6-DevKitC-1
 *   - MAX4400  : I2C Light Sensor (SDA=GPIO7, SCL=GPIO6)
 *   - C4001    : 24 GHz Millimeter-Wave Human Presence Sensor (UART1, RX=GPIO5, TX=GPIO4)
 *   - WS2812B  : Onboard RGB LED (GPIO8, RMT Driver)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "led_strip.h"   // managed_components/espressif__led_strip (vtable-style API)
#include "app_priv.h"

static const char *TAG = "app_driver";

/* =====================================================================
 * Hardware Pins / Constants Definition
 * ===================================================================== */

// I2C -- MAX4400 Light Sensor
#define I2C_MASTER_SCL_IO       GPIO_NUM_6
#define I2C_MASTER_SDA_IO       GPIO_NUM_7
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000      // 100 kHz

// MAX4400 Register map (7-bit address = 0x4A when ADDR pin = GND)
#define MAX4400_I2C_ADDR        0x4A
#define MAX4400_REG_STATUS      0x00        // bit7 = PWRON, bit0 = AMBDATARDY
#define MAX4400_REG_CONFIG      0x01        // default 0x03 (ALS enabled, 100 ms)
#define MAX4400_REG_AMB_MSB     0x04        // Ambient (clear) high byte
#define MAX4400_REG_AMB_LSB     0x05        // Ambient (clear) low byte

// C4001 UART
#define RADAR_UART_NUM          UART_NUM_1
#define RADAR_TXD_PIN           GPIO_NUM_4
#define RADAR_RXD_PIN           GPIO_NUM_5
#define RADAR_UART_BAUDRATE     9600
#define RADAR_BUF_SIZE          512

// WS2812B LED (RMT backend, legacy vtable-style driver)
#define LED_GPIO                GPIO_NUM_8
#define LED_STRIP_PIXELS        1           // DevKitC-1 has 1 RGB LED

/* =====================================================================
 * C4001 Protocol Constants
 * Uses ASCII NMEA-style format at 9600 baud:
 *   $DFHPD,<P>,<dist>,<speed>,<energy>*<chk>\r\n
 *   P = 0: No person, P = 1: Person present
 * ===================================================================== */

/* =====================================================================
 * Static Private Variables
 * ===================================================================== */
static led_strip_t *s_led_strip = NULL;  // vtable-style handle
static bool s_radar_presence = false;  // Latest radar state (interrupt-safe read)

/* =====================================================================
 * I2C Initialization
 * ===================================================================== */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags        = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

/* =====================================================================
 * MAX4400 Helper: Read Single Register
 * ===================================================================== */
static esp_err_t max4400_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT, MAX4400_I2C_ADDR,
        &reg, 1,
        data, len,
        pdMS_TO_TICKS(50));
}

/* =====================================================================
 * MAX4400 Helper: Write Single Register
 * ===================================================================== */
static esp_err_t max4400_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(
        I2C_MASTER_PORT, MAX4400_I2C_ADDR,
        buf, sizeof(buf),
        pdMS_TO_TICKS(50));
}

/* =====================================================================
 * UART Initialization (C4001)
 * ===================================================================== */
static void radar_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = RADAR_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_NUM, RADAR_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_NUM,
                                 RADAR_TXD_PIN, RADAR_RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "C4001 UART initialized (UART%d, RX=GPIO%d, TX=GPIO%d)",
             RADAR_UART_NUM, RADAR_RXD_PIN, RADAR_TXD_PIN);
}

/* =====================================================================
 * WS2812B LED Initialization (RMT)
 * ===================================================================== */
static void led_strip_init(void)
{
    // Use RMT channel 0 to drive WS2812B
    rmt_config_t rmt_cfg = RMT_DEFAULT_CONFIG_TX(LED_GPIO, RMT_CHANNEL_0);
    rmt_cfg.clk_div = 2;  // 40 MHz / 2 = 20 MHz
    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_cfg.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_STRIP_PIXELS,
                                      (led_strip_dev_t)rmt_cfg.channel);
    s_led_strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!s_led_strip) {
        ESP_LOGE(TAG, "Failed to create led_strip");
        return;
    }
    s_led_strip->clear(s_led_strip, 50);  // Default: off
    ESP_LOGI(TAG, "WS2812B LED strip initialized (GPIO%d)", LED_GPIO);
}

/* =====================================================================
 * C4001 Packet Parsing
 *
 * C4001 outputs ASCII NMEA-style format (9600 baud):
 *   $DFHPD,<P>,<dist>,<speed>,<energy>*<chk>\r\n
 *   P = 0: No person, P = 1: Person present
 * ===================================================================== */
static bool parse_c4001_frame(const uint8_t *buf, int len)
{
    // Search for "$DFHPD," prefix in buffer
    const char *prefix = "$DFHPD,";
    const int prefix_len = 7;

    for (int i = 0; i <= len - prefix_len; i++) {
        if (memcmp(buf + i, prefix, prefix_len) == 0) {
            // First field is presence flag
            char p = (char)buf[i + prefix_len];
            ESP_LOGI(TAG, "C4001 DFHPD frame: presence='%c'", p);
            return (p == '1');
        }
    }
    // No valid packet found, maintain previous state
    return s_radar_presence;
}

/* =====================================================================
 * Public API
 * ===================================================================== */

/**
 * @brief Initialize all drivers
 *        Call sequence: LED -> I2C -> UART
 */
esp_err_t app_driver_init(void)
{
    led_strip_init();

    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // I2C bus scan -- find all responding device addresses
    ESP_LOGI(TAG, "I2C scan:");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        esp_err_t probe = i2c_master_write_read_device(
            I2C_MASTER_PORT, addr, &dummy, 0, &dummy, 0, pdMS_TO_TICKS(10));
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "  found 0x%02X", addr);
        }
    }
    ret = max4400_write_reg(MAX4400_REG_CONFIG, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MAX4400 config write failed (check I2C wiring): %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "MAX4400 initialized at 0x%02X", MAX4400_I2C_ADDR);
    }

    radar_uart_init();

    ESP_LOGI(TAG, "All hardware drivers initialized.");
    return ESP_OK;
}

/**
 * @brief Read MAX4400 ambient light channel (clear channel)
 *        Return value unit: Lux (approximate, based on 100 ms integration)
 *
 * MAX4400 clear channel raw count (16-bit) to Lux formula (typical):
 *   Lux ≈ raw_count × 0.045
 * Actual coefficient depends on optical package; adjust LIGHT_SENSOR_LUX_FACTOR if needed.
 */
float app_driver_read_light_sensor(void)
{
#define LIGHT_SENSOR_LUX_FACTOR  0.045f

    // Wait for AMBDATARDY (bit0 of STATUS) -- max 200 ms
    uint8_t status = 0;
    for (int retry = 0; retry < 4; retry++) {
        if (max4400_read_reg(MAX4400_REG_STATUS, &status, 1) == ESP_OK) {
            if (status & 0x01) break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uint8_t msb = 0, lsb = 0;
    esp_err_t err_msb = max4400_read_reg(MAX4400_REG_AMB_MSB, &msb, 1);
    esp_err_t err_lsb = max4400_read_reg(MAX4400_REG_AMB_LSB, &lsb, 1);

    if (err_msb != ESP_OK || err_lsb != ESP_OK) {
        ESP_LOGW(TAG, "MAX4400 read failed, returning 0 lux");
        return 0.0f;
    }

    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    float lux = (float)raw * LIGHT_SENSOR_LUX_FACTOR;
    ESP_LOGI(TAG, "MAX4400 raw=%u -> %.1f lux", raw, lux);
    return lux;
}

/**
 * @brief Read C4001 24 GHz Millimeter-Wave Radar (human presence)
 *        true = person present, false = no person
 *
 * Strategy: Read all available bytes from UART FIFO, parse the last valid packet.
 * If buffer is empty (sensor not reporting yet), return last known state.
 */
bool app_driver_read_radar_sensor(void)
{
    size_t available = 0;
    uart_get_buffered_data_len(RADAR_UART_NUM, &available);

    if (available == 0) {
        ESP_LOGI(TAG, "C4001 UART empty (0 bytes), keeping last state: %s",
                 s_radar_presence ? "PRESENT" : "ABSENT");
        return s_radar_presence;
    }

    int read_len = (available > RADAR_BUF_SIZE) ? RADAR_BUF_SIZE : (int)available;
    uint8_t buf[RADAR_BUF_SIZE];
    int actual = uart_read_bytes(RADAR_UART_NUM, buf, read_len, pdMS_TO_TICKS(20));
    if (actual > 0) {
        // DEBUG: Print raw hex bytes, confirm C4001 packet format
        char hex_str[RADAR_BUF_SIZE * 3 + 1];
        int pos = 0;
        for (int i = 0; i < actual && pos < (int)sizeof(hex_str) - 3; i++) {
            pos += snprintf(hex_str + pos, sizeof(hex_str) - pos, "%02X ", buf[i]);
        }
        ESP_LOGI(TAG, "C4001 raw (%d bytes): %s", actual, hex_str);

        s_radar_presence = parse_c4001_frame(buf, actual);
    }

    ESP_LOGI(TAG, "C4001 -> %s", s_radar_presence ? "PRESENT" : "ABSENT");
    return s_radar_presence;
}

/**
 * @brief Set onboard WS2812B LED color to reflect occupancy state
 *        Person present = Blue (indicates person detected; actual light control by Google Home automation)
 *        No person = Off
 */
void app_driver_set_led(bool state)
{
    if (!s_led_strip) return;

    if (state) {
        // Blue: R=0, G=0, B=30
        s_led_strip->set_pixel(s_led_strip, 0, 0, 0, 30);
        s_led_strip->refresh(s_led_strip, 50);
    } else {
        s_led_strip->clear(s_led_strip, 50);
    }
    ESP_LOGD(TAG, "LED -> %s", state ? "ON (blue)" : "OFF");
}