/*
 * app_main.cpp -- my_presence_sensor
 *
 * Logic:
 *   1. Poll MAX4400 (lux) and C4001 (presence) every SENSOR_POLL_INTERVAL_MS
 *   2. Decision:
 *        if (lux < LIGHT_THRESHOLD_LUX) && radar == PRESENT  ->  Occupancy = 1
 *        else                                                 ->  Occupancy = 0
 *   3. On state change:
 *        a) Update onboard WS2812 LED (blue = occupied)
 *        b) Report OccupancySensing attribute via Matter / Thread
 *   4. Google Home triggers light automation based on OccupancySensing state
 *
 * Matter device type: Occupancy Sensor (0x0107)
 * Transport: Thread (IEEE 802.15.4, native on ESP32-C6)
 * Commissioning: BLE -> Thread over Matter
 */

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <app_priv.h>
#include <app_reset.h>
#include <iot_button.h>
#include <button_gpio.h>

/* ESP-Matter SDK */
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_identify.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

/* OccupancySensing cluster generated headers */
#include <clusters/OccupancySensing/ClusterId.h>
#include <clusters/OccupancySensing/AttributeIds.h>
#include <clusters/OccupancySensing/Enums.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

static const char *TAG = "app_main";

/* ===================================================================
 * Tunable parameters
 * =================================================================== */
#define LIGHT_THRESHOLD_LUX     50.0f   /* Radar ignored when lux >= this */
#define SENSOR_POLL_INTERVAL_MS 5000    /* ms between sensor polls         */

/* ===================================================================
 * Global state
 * =================================================================== */
static uint16_t     g_occupancy_ep_id    = 0;
static bool         g_matter_state       = false;
static TaskHandle_t g_sensor_task_handle = NULL;

/* ===================================================================
 * Matter attribute update callback
 * Called for PRE_UPDATE, POST_UPDATE, READ, WRITE on any attribute.
 * We only need to handle READ to report current state to controllers.
 * =================================================================== */
static esp_err_t app_attribute_update_cb(callback_type_t type,
                                          uint16_t endpoint_id,
                                          uint32_t cluster_id,
                                          uint32_t attribute_id,
                                          esp_matter_attr_val_t *val,
                                          void *priv_data)
{
    if (type == READ &&
        endpoint_id  == g_occupancy_ep_id &&
        cluster_id   == OccupancySensing::Id &&
        attribute_id == OccupancySensing::Attributes::Occupancy::Id)
    {
        val->val.u8 = g_matter_state ? 1u : 0u;
        ESP_LOGD(TAG, "READ Occupancy -> %s", g_matter_state ? "OCCUPIED" : "UNOCCUPIED");
    }
    return ESP_OK;
}

/* ===================================================================
 * Identify callback (required, even if unused)
 * =================================================================== */
static esp_err_t app_identification_cb(identification::callback_type_t type,
                                        uint16_t endpoint_id,
                                        uint8_t effect_id,
                                        uint8_t effect_variant,
                                        void *priv_data)
{
    ESP_LOGI(TAG, "Identify: type=%u effect=%u variant=%u ep=%u",
             type, effect_id, effect_variant, endpoint_id);
    return ESP_OK;
}

/* ===================================================================
 * Matter system event callback
 * =================================================================== */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized (memory reclaimed)");
        break;
    default:
        break;
    }
}

/* ===================================================================
 * Core sensor task
 * Polls MAX4400 and C4001 every SENSOR_POLL_INTERVAL_MS.
 * Calls attribute::update() on state change -- no manual lock needed.
 * =================================================================== */
static void app_sensor_update_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started (poll every %d ms)", SENSOR_POLL_INTERVAL_MS);

    while (true) {
        /* 1. Read hardware */
        float lux           = app_driver_read_light_sensor();
        bool  radar_present = app_driver_read_radar_sensor();

        /* 2. Decision: trust radar only when dark */
        bool new_state = false;
        if (lux < LIGHT_THRESHOLD_LUX) {
            new_state = radar_present;
            ESP_LOGI(TAG, "DARK (%.1f lux < %.1f) -> radar=%s",
                     lux, LIGHT_THRESHOLD_LUX, radar_present ? "PRESENT" : "ABSENT");
        } else {
            ESP_LOGI(TAG, "BRIGHT (%.1f lux >= %.1f) -> occupancy forced OFF",
                     lux, LIGHT_THRESHOLD_LUX);
        }

        /* 3. Report only on state change to avoid flooding Thread network */
        if (new_state != g_matter_state) {
            g_matter_state = new_state;
            ESP_LOGW(TAG, "Occupancy changed -> %s",
                     g_matter_state ? "OCCUPIED" : "UNOCCUPIED");

            /* 3a. Update onboard LED (blue = occupied, off = unoccupied) */
            app_driver_set_led(g_matter_state);

            /* 3b. Push OccupancySensing attribute to Matter stack.
             *     attribute::update() handles PRE/POST callbacks internally. */
            uint8_t occ_val = g_matter_state ? 1u : 0u;
            esp_matter_attr_val_t val = esp_matter_bitmap8(occ_val);
            esp_err_t err = attribute::update(
                g_occupancy_ep_id,
                OccupancySensing::Id,
                OccupancySensing::Attributes::Occupancy::Id,
                &val);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "attribute::update failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
    }
}

/* ===================================================================
 * app_main
 * =================================================================== */
extern "C" void app_main(void)
{
    esp_err_t err = ESP_OK;

    /* 1. NVS init -- required by Matter for credential storage */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. Hardware init (I2C / UART / RMT LED) */
    err = app_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "app_driver_init failed: %s", esp_err_to_name(err));
        return;
    }

    /* 3. Create Matter node (also creates root node endpoint 0).
     *    node::create() takes:
     *      - node::config_t*            root endpoint config (defaults OK)
     *      - attribute::callback_t      attribute read/write callback
     *      - identification::callback_t Identify cluster callback        */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "node::create() failed");
        return;
    }

    /* 4. Create OccupancySensor endpoint (device type 0x0107).
     *    feature_flags = kRadar (0x20) signals 24 GHz radar to Google Home. */
    occupancy_sensor::config_t occ_cfg;
    occ_cfg.occupancy_sensing.occupancy_sensor_type        = 0;  /* kPir -- legacy field */
    occ_cfg.occupancy_sensing.occupancy_sensor_type_bitmap = 0;
    occ_cfg.occupancy_sensing.feature_flags =
        static_cast<uint32_t>(OccupancySensing::Feature::kRadar);

    endpoint_t *ep = occupancy_sensor::create(node, &occ_cfg, ENDPOINT_FLAG_NONE, NULL);
    if (!ep) {
        ESP_LOGE(TAG, "occupancy_sensor::create() failed");
        return;
    }
    g_occupancy_ep_id = endpoint::get_id(ep);
    ESP_LOGI(TAG, "OccupancySensor endpoint_id = %d", g_occupancy_ep_id);

    /* 5. BOOT button: long press > 5 s -> factory reset */
    {
        button_handle_t btn_handle = NULL;
        const button_config_t btn_cfg = {0};
        const button_gpio_config_t btn_gpio_cfg = {
            .gpio_num    = 9,
            .active_level = 0,      /* active-low (pulled high, pressed = low) */
            .enable_power_save = false,
            .disable_pull = false,
        };
        if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle) == ESP_OK) {
            app_reset_button_register(btn_handle);
        } else {
            ESP_LOGW(TAG, "Failed to register BOOT button for factory reset");
        }
    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* 6. OpenThread platform config (native IEEE 802.15.4 on ESP32-C6) */
    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    /* 7. Start Matter (Thread transport, BLE commissioning) */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Matter stack initialized over Thread");

    /* 8. Optional interactive shell */
#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::attribute_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

    /* 9. Launch sensor FreeRTOS task (4 KB stack, priority 5) */
    xTaskCreate(app_sensor_update_task, "sensor_task", 4096,
                NULL, 5, &g_sensor_task_handle);
}
