#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_hidd.h"
#include "esp_err.h"

#define BUTTON_NEXT     27
#define BUTTON_PREV     26   // Changed from 12 to 26 to avoid bootstrap issues

#define HID_R_CTRL      0x00
#define KEY_RIGHT_ARROW 0x4F
#define KEY_LEFT_ARROW  0x50

static const char *TAG = "ESP32_PRESENTER";

// Handle for HID connection. Set by ESP-IDF HID callbacks.
static esp_hidd_dev_t *hid_dev = NULL;
static bool connected = false;

// BLE advertising parameters
static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,                    // Flags: General Discoverable Mode, BR/EDR Not Supported
    0x03, 0x03, 0x12, 0x18,              // Complete 16-bit Service UUIDs (HID Service)
    0x03, 0x19, 0xC1, 0x03,              // Appearance: Keyboard (0x03C1)
    0x0F, 0x09, 'E', 'S', 'P', '3', '2', ' ', 'P', 'r', 'e', 's', 'e', 'n', 't', 'e', 'r'  // Complete local name
};

static uint8_t scan_rsp_data[] = {
    0x0F, 0x09, 'E', 'S', 'P', '3', '2', ' ', 'P', 'r', 'e', 's', 'e', 'n', 't', 'e', 'r'  // Complete local name
};

// HID Report Map for a basic keyboard
static uint8_t hid_report_map[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

// HID report descriptor
static esp_hid_raw_report_map_t hid_report_map_config = {
    .data = hid_report_map,
    .len = sizeof(hid_report_map),
};

// GAP callback for BLE advertising
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(TAG, "Advertising stopped");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Connection params updated: status = %d, min_int = %d, max_int = %d, conn_int = %d, latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(TAG, "Packet length set complete");
        break;
    case ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(TAG, "Bond device complete");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(TAG, "Passkey request");
        // For ESP_IO_CAP_NONE, we auto-accept with passkey 0
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(TAG, "Numeric comparison request");
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "Security request");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "Authentication completed successfully");
            ESP_LOGI(TAG, "Waiting for HID service connection...");
        } else {
            ESP_LOGE(TAG, "Authentication failed, reason: %d", param->ble_security.auth_cmpl.fail_reason);
            // Restart advertising on authentication failure
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    default:
        break;
    }
}

// HID callback: update handle on connection
void hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    ESP_LOGI(TAG, "HID event received: %ld", id);

    switch (event) {
        case ESP_HIDD_START_EVENT:
            ESP_LOGI(TAG, "HID device started");
            if (param->start.status == ESP_OK) {
                ESP_LOGI(TAG, "HID device started successfully");
                // Set battery level
                esp_hidd_dev_battery_set(hid_dev, 100);
            } else {
                ESP_LOGE(TAG, "HID device start failed");
            }
            break;
        case ESP_HIDD_CONNECT_EVENT:
            ESP_LOGI(TAG, "HID device connected");
            connected = true;
            esp_ble_gap_stop_advertising();
            break;
        case ESP_HIDD_DISCONNECT_EVENT:
            ESP_LOGI(TAG, "HID device disconnected");
            connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_HIDD_PROTOCOL_MODE_EVENT:
            ESP_LOGI(TAG, "HID protocol mode: %s",
                     param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
            break;
        case ESP_HIDD_OUTPUT_EVENT:
            ESP_LOGI(TAG, "HID output event");
            break;
        case ESP_HIDD_FEATURE_EVENT:
            ESP_LOGI(TAG, "HID feature event");
            break;
        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID device stopped");
            break;
        default:
            ESP_LOGW(TAG, "Unhandled HID event: %d", event);
            break;
    }
}void send_key(uint8_t keycode) {
    ESP_LOGI(TAG, "Sending keycode: 0x%02X", keycode);

    // Check ESP-IDF's internal connection status
    bool esp_connected = esp_hidd_dev_connected(hid_dev);
    ESP_LOGI(TAG, "HID connection status - our flag: %d, ESP-IDF status: %d", connected, esp_connected);

    if (hid_dev && esp_connected) {
        uint8_t report[] = {HID_R_CTRL, 0, keycode, 0, 0, 0, 0, 0};
        // Press
        esp_err_t ret = esp_hidd_dev_input_set(hid_dev, 0, 0x01, report, sizeof(report));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Key pressed successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send key press: %s", esp_err_to_name(ret));
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // Release
        memset(report, 0, sizeof(report));
        ret = esp_hidd_dev_input_set(hid_dev, 0, 0x01, report, sizeof(report));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Key released successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send key release: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "Cannot send key - device not connected (hid_dev=%p, our_connected=%d, esp_connected=%d)",
                 hid_dev, connected, esp_connected);
    }
}

void button_task(void *arg) {
    ESP_LOGI(TAG, "Button task starting...");

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_NEXT) | (1ULL << BUTTON_PREV),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIO %d (NEXT) and GPIO %d (PREV) configured as inputs with pull-up", BUTTON_NEXT, BUTTON_PREV);

    bool next_last = 1, prev_last = 1;
    int loop_count = 0;

    while (1) {
        bool next_now = gpio_get_level(BUTTON_NEXT);
        bool prev_now = gpio_get_level(BUTTON_PREV);

        // Log GPIO levels every 2.5 seconds for debugging
        if (loop_count % 100 == 0) {
            // GPIO level logging disabled to reduce log clutter
            // ESP_LOGI(TAG, "GPIO levels - NEXT(GPIO%d): %d, PREV(GPIO%d): %d",
            //          BUTTON_NEXT, next_now, BUTTON_PREV, prev_now);
        }

        // Button pressed (LOW due to pull-up)
        if (!next_now && next_last) {
            ESP_LOGI(TAG, "NEXT button pressed!");
            send_key(KEY_RIGHT_ARROW); // Next
        }
        if (!prev_now && prev_last) {
            ESP_LOGI(TAG, "PREV button pressed!");
            send_key(KEY_LEFT_ARROW); // Previous
        }

        next_last = next_now;
        prev_last = prev_now;
        loop_count++;
        vTaskDelay(25 / portTICK_PERIOD_MS); // 25ms debounce
    }
}

void app_main(void) {
    // Initialize NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Bluetooth/HID device initialization
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register GAP callback
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize HID before setting security params
    ESP_LOGI(TAG, "Initializing HID device...");
    esp_hid_device_config_t config = {
        .vendor_id = 0x16C0,
        .product_id = 0x05DF,
        .version = 0x0100,
        .device_name = "ESP32 Presenter",
        .manufacturer_name = "Espressif",
        .serial_number = "1234567890",
        .report_maps = &hid_report_map_config,
        .report_maps_len = 1,
    };

    ret = esp_hidd_dev_init(&config, ESP_HID_TRANSPORT_BLE, hidd_event_callback, &hid_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID device init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "HID device initialized successfully, hid_dev=%p", hid_dev);

    // Set initial battery level
    esp_hidd_dev_battery_set(hid_dev, 100);
    ESP_LOGI(TAG, "Battery level set to 100%%");

    // Set BLE security requirements for HID - use simpler bonding
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = 0;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));    // Set advertising data
    ESP_LOGI(TAG, "Setting advertising data...");
    ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        return;
    }
    adv_config_done |= ADV_CONFIG_FLAG;

    // Set scan response data
    ESP_LOGI(TAG, "Setting scan response data...");
    ret = esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_data, sizeof(scan_rsp_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config scan response data failed: %s", esp_err_to_name(ret));
        return;
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;

    // Force start advertising
    ESP_LOGI(TAG, "Starting advertising manually...");
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return;
    }

    // Start task for button polling
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "Presenter started. Pair this BLE device on Linux.");
}
