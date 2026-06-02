#pragma once
#include "esp_err.h"
#include "esp_openthread_types.h"

/* OpenThread platform config macros for ESP32-C6 native 802.15.4 radio.
 * These are guarded by CHIP_DEVICE_CONFIG_ENABLE_THREAD at the call site. */
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                               \
    {                                                                       \
        .radio_mode = RADIO_MODE_NATIVE,                                    \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                \
    {                                                                       \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                  \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                \
    {                                                                       \
        .storage_partition_name = "nvs", .netif_queue_size = 10,            \
        .task_queue_size = 10,                                               \
    }

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_driver_init(void);
float app_driver_read_light_sensor(void);
bool app_driver_read_radar_sensor(void);
void app_driver_set_led(bool state);

#ifdef __cplusplus
}
#endif