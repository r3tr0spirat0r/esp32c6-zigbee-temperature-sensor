#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "ds18b20.h"
#include "onewire_bus.h"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define GPIO_DS18B20 4
#define SENSOR_ENDPOINT 1
#define PAYLOAD_REPORT_DELAY_MS 30000

#define MANUFACTURER_NAME "DIY"
#define MODEL_IDENTIFIER "ESP32C6_TEMP"

static const char *TAG = "CWU_SENSOR_ZB";

ds18b20_device_handle_t ds18b20;
static bool device_joined = false;

static float read_temp(void) {
    float temp = 0;
    esp_err_t err = ds18b20_trigger_temperature_conversion(ds18b20);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Błąd trigger conversion: %d", err);
        return -999;
    }
    vTaskDelay(pdMS_TO_TICKS(800));
    err = ds18b20_get_temperature(ds18b20, &temp);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Błąd odczytu temperatury: %d", err);
        return -999;
    }
    return temp;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
        default:
            ESP_LOGD(TAG, "Otrzymano zapytanie action: %d", callback_id);
            break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    esp_zb_app_signal_type_t sig_type = *signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    
    ESP_LOGI(TAG, "📡 Sygnał Zigbee: %d, status: %d", sig_type, err_status);
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "⚙️  Brak sieci w pamięci, rozpoczynam steering...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            ESP_LOGI(TAG, "🆕 Pierwsze uruchomienie urządzenia");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;
            
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            ESP_LOGI(TAG, "🔄 Restart urządzenia");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;
            
        case ESP_ZB_BDB_SIGNAL_STEERING:
            ESP_LOGI(TAG, "🎯 STEERING RESULT: status=%d (%s)", err_status, 
                     err_status == ESP_OK ? "SUCCESS" : "FAILED");
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "✅ Połączono z siecią Zigbee!");
                ESP_LOGI(TAG, "   Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
                device_joined = true;
            } else {
                ESP_LOGW(TAG, "❌ Steering failed (status: %d), ponawiam za 10s...", err_status);
                vTaskDelay(pdMS_TO_TICKS(10000));
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
            
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            ESP_LOGI(TAG, "📢 Device announce wysłany");
            break;
            
        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "🔓 Permit join zmieniony");
            }
            break;
            
        default:
            ESP_LOGD(TAG, "Zignorowany sygnał: %d", sig_type);
            break;
    }
}

static void temp_report_task(void *pvParameters) {
    if (ds18b20 == NULL) {
        ESP_LOGE(TAG, "❌ Brak czujnika!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "🌡️  Task raportowania uruchomiony");
    ESP_LOGI(TAG, "✅ Rozpoczynam odczyty temperatury co %d sekund", PAYLOAD_REPORT_DELAY_MS / 1000);

    while (1) {
        float temp = read_temp();
        
        if (temp == -999) {
            ESP_LOGW(TAG, "⚠️  Odczyt nieudany");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        ESP_LOGI(TAG, "🌡️  Pomiar CWU: %.2f °C", temp);
        
        if (device_joined) {
            int16_t zb_temp = (int16_t)(temp * 100);
            
            esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
                SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                &zb_temp,
                false);
            
            if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "📤 Wysłano do Zigbee: %d (x100) = %.2f °C", zb_temp, temp);
            } else {
                ESP_LOGW(TAG, "⚠️  Błąd wysyłania: %d", status);
            }
        } else {
            ESP_LOGW(TAG, "⏳ Nie połączono z Zigbee");
        }
        
        vTaskDelay(pdMS_TO_TICKS(PAYLOAD_REPORT_DELAY_MS));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "🚀 Start czujnika DS18B20 Zigbee");
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "🔧 Czyszczenie NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS zainicjalizowane");

    ESP_LOGI(TAG, "🔍 Szukam czujnika DS18B20 na GPIO %d...", GPIO_DS18B20);
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = { .bus_gpio_num = GPIO_DS18B20 };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    vTaskDelay(pdMS_TO_TICKS(200));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    
    if (onewire_device_iter_get_next(iter, &next_onewire_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20));
        ESP_LOGI(TAG, "✅ DS18B20 wykryty i gotowy!");
        
        float test_temp = read_temp();
        if (test_temp != -999) {
            ESP_LOGI(TAG, "🌡️  Test odczytu: %.2f °C", test_temp);
        }
    } else {
        ESP_LOGE(TAG, "❌ Nie znaleziono DS18B20 na GPIO %d!", GPIO_DS18B20);
    }
    onewire_del_device_iter(iter);

    ESP_LOGI(TAG, "⚙️  Inicjalizacja stosu Zigbee...");
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = 0,
                .keep_alive = 0,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_set_rx_on_when_idle(true);
    ESP_LOGI(TAG, "✅ Zigbee router skonfigurowany (RX always ON)");

    ESP_LOGI(TAG, "📦 Tworzenie klastrów Zigbee...");
    
    // ============= BASIC CLUSTER (0x0000) =============
    esp_zb_attribute_list_t *basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    
    // Manufacturer Name - musi być w stałej pamięci lub skopiowane
    char manufacturer[33] = MANUFACTURER_NAME;  // +1 dla \0
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    
    // Model Identifier - musi być w stałej pamięci lub skopiowane
    char model[33] = MODEL_IDENTIFIER;  // +1 dla \0
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);
    
    // Power Source: 0x04 = DC Source (zasilacz)
    uint8_t power_source = 0x04;
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    
    ESP_LOGI(TAG, "   ✓ Basic cluster (Manufacturer: %s, Model: %s)", manufacturer, model);

    // ============= IDENTIFY CLUSTER (0x0003) =============
    esp_zb_attribute_list_t *identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    ESP_LOGI(TAG, "   ✓ Identify cluster");

    // ============= TEMPERATURE CLUSTER (0x0402) =============
    esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    int16_t m_val = 0, min_v = -5000, max_v = 10000;
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &m_val);
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &min_v);
    esp_zb_temperature_meas_cluster_add_attr(temp_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &max_v);
    ESP_LOGI(TAG, "   ✓ Temperature cluster (-50°C do 100°C)");

    // ============= DODAJ WSZYSTKIE CLUSTERY DO LISTY =============
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // ============= ENDPOINT CONFIGURATION =============
    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_cfg);
    esp_zb_device_register(ep_list);
    ESP_LOGI(TAG, "✅ Endpoint %d zarejestrowany", SENSOR_ENDPOINT);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    esp_zb_set_tx_power((int8_t)12);
    ESP_LOGI(TAG, "📡 Radio: wszystkie kanały, TX power: 12 dBm");

    // Rejestruj action handler
    esp_zb_core_action_handler_register(zb_action_handler);
    
    ESP_LOGI(TAG, "🚀 Uruchamiam stos Zigbee...");
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "✅ Stos Zigbee uruchomiony");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    xTaskCreate(temp_report_task, "temp_report_task", 4096, NULL, 5, NULL);

    while (1) {
        esp_zb_main_loop_iteration();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
