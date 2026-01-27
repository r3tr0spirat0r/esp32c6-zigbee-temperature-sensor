#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "ds18b20.h"
#include "onewire_bus.h"

// Ignorujemy ostrzeżenia o przestarzałych funkcjach
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define GPIO_DS18B20       4
#define SENSOR_ENDPOINT    1
#define PAYLOAD_REPORT_DELAY_MS 30000 

static const char *TAG = "CWU_SENSOR_ZB";
ds18b20_device_handle_t ds18b20;

static float read_temp(void) {
    float temp = 0;
    ds18b20_trigger_temperature_conversion(ds18b20);
    vTaskDelay(pdMS_TO_TICKS(800)); 
    ds18b20_get_temperature(ds18b20, &temp);
    return temp;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    esp_zb_app_signal_type_t sig_type = *signal_struct->p_app_signal;
    if (sig_type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
}

static void temp_report_task(void *pvParameters) {
	// Sprawdzenie czy uchwyt do czujnika został utworzony w app_main
    if (ds18b20 == NULL) {
        ESP_LOGE(TAG, "Task raportowania przerwany: Brak zainicjowanego czujnika!");
        vTaskDelete(NULL); // Usuwa ten konkretny task
        return; 
    }
    
    while (1) {
        float temp = read_temp();
        ESP_LOGI(TAG, "Temperatura CWU: %.2f C", temp);
        // Zigbee oczekuje temperatury w skali 0.01°C
        int16_t zb_temp = (int16_t)(temp * 100);

        esp_zb_zcl_set_attribute_val(
            SENSOR_ENDPOINT, 
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
            &zb_temp, 
            false);

        vTaskDelay(pdMS_TO_TICKS(PAYLOAD_REPORT_DELAY_MS));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // OneWire setup
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = { .bus_gpio_num = GPIO_DS18B20 };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

	vTaskDelay(pdMS_TO_TICKS(100));
	
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    
    if (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {}; 
        ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20));
        ESP_LOGI("MAIN", "Czujnik DS18B20 znaleziony!");
    } else {
        ESP_LOGE("MAIN", "NIE ZNALEZIONO CZUJNIKA! Sprawdź GPIO 4.");
        // Nie przerywamy programu, pozwalamy Zigbee wystartować
    }
    
    onewire_del_device_iter(iter);

    // Zigbee config
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = { .ed_timeout = 0x08, .keep_alive = 3000 }
    };
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    int16_t m_val = 0, min_v = -5000, max_v = 10000;
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &m_val);
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &min_v);
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &max_v);
    
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = SENSOR_ENDPOINT, 
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, 
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_cfg);

    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    xTaskCreate(temp_report_task, "temp_report_task", 4096, NULL, 5, NULL);
    
    while(1) {
        esp_zb_main_loop_iteration();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}