#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "aes/esp_aes.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "pgpemu.h"
#include "pgp-cert.h"
#include "secrets.h"
#include "esp_gatt_common_api.h"

#include "tftspi.h"
#include "tft.h"
//#include "spiffs_vfs.h" //not needed
#include <time.h>
#include <unistd.h>

#include "nvs_flash.h"
#include "nvs.h"

int8_t font_color = 0;

uint8_t color_size = 9;
color_t colors[] = {
    { 255 - 66, 255 - 135, 255 - 245}, //blue 
    { 255 - 245, 255 - 245, 255 - 66}, //yellow
    { 255 - 245, 255 - 20, 255 - 20}, //red
    { 255 - 81, 255 - 245, 255 - 66}, //green
    { 255 - 65, 255 - 242, 255 - 222}, //aqua
    { 255 - 250, 255 - 160, 255 - 229}, //pink
    { 255 - 177, 255 - 44, 255 - 230}, //purple
    { 0, 0, 0}, //white
    { 255 - 64, 255 - 64, 255 - 64} //gray
};

int16_t pokemon_seen = 0;
int16_t pokemon_caught = 0;
int16_t pokestops = 0;
int16_t pokemon_changed = 0;
int8_t device = 0;

uint8_t screensaver = 0;
uint8_t screensaver_time = 30;

uint8_t waiting_time = 60;
uint8_t current_time = 60;

uint8_t charging = 0;
uint8_t screen_on = 1;
uint8_t display_state = 0;
uint8_t display_state_old = 0;
uint8_t display_state_ready = 0;

uint8_t refresh_screen = 1;

int8_t minute = 60;
int8_t second = 0;
int8_t charging_prev = 0;
int16_t battery_number = 0;

uint8_t nvs_lock = 0;

static QueueHandle_t button_queue;
static QueueHandle_t uart0_queue;

static xQueueHandle gpio_evt_queue = NULL;

TaskHandle_t xHandleMainTask = NULL;

static const char *TAG = "uart_events";

static uint8_t adv_config_done = 0;

uint16_t battery_handle_table[BATTERY_LAST_IDX];
uint16_t led_button_handle_table[LED_BUTTON_LAST_IDX];
uint16_t certificate_handle_table[CERT_LAST_IDX];

uint16_t last_conn_id = 0;
esp_gatt_if_t last_if = 0;

//in ESP32, bt_mac is base_mac + 2
uint8_t mac[6];
uint8_t bt_mac[6];

//uint8_t mac[] = {0x98, 0xB6, 0xE9, 0x13, 0xB5, 0x91};
//uint8_t bt_mac[] = {0x98, 0xB6, 0xE9, 0x13, 0xB5, 0x93};


uint8_t session_key[16];
int cert_state = 0;

static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* device name */
    0x10, 0x09, 'P', 'o', 'k', 'e', 'm', 'o', 'n', ' ', 'G', 'O', ' ', 'P', 'l', 'u', 's',
    //0x0c, 0x09, 'P', 'o', 'k', 'e', 'm', 'o', 'n', ' ', 'P', 'B', 'P',
    0x06, 0x20, 0x62, 0x04, 0xC5, 0x21, 0x00
};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00
};


static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst pgp_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] =
    {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t dummy_value[2] = {0x00, 0x00};
static uint8_t cert_buffer[378];


static const uint16_t GATTS_SERVICE_UUID_BATTERY = 0x180f;
static const uint16_t GATTS_CHAR_UUID_BATTERY_LEVEL = 0x2a19;
static uint8_t battery_char_value[1] = {0x60};


static uint8_t reconnect_challenge[32];
static int has_reconnect_key = 0;

static const esp_gatts_attr_db_t gatt_db_battery[BATTERY_LAST_IDX] = {

    // Service Declaration
    [IDX_BATTERY_SVC] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof (uint16_t), sizeof (GATTS_SERVICE_UUID_BATTERY), (uint8_t *) & GATTS_SERVICE_UUID_BATTERY}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY_LEVEL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_read_notify}
    },
    /* Characteristic Value */
    [IDX_CHAR_BATTERY_LEVEL_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & GATTS_CHAR_UUID_BATTERY_LEVEL, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (battery_char_value), (uint8_t *) battery_char_value}
    },

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_BATTERY_LEVEL_CFG] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof (uint16_t), sizeof (dummy_value), (uint8_t *) dummy_value}
    },
};



//uuid in reversed order
uint8_t GATTS_SERVICE_UUID_LED_BUTTON[ESP_UUID_LEN_128] = {0xeb, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};
uint8_t GATTS_CHAR_UUID_LED[ESP_UUID_LEN_128] = {0xec, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};
uint8_t GATTS_CHAR_UUID_BUTTON[ESP_UUID_LEN_128] = {0xed, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};
uint8_t GATTS_CHAR_UUID_UNKNOWN[ESP_UUID_LEN_128] = {0xee, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};
uint8_t GATTS_CHAR_UUID_UPDATE_REQUEST[ESP_UUID_LEN_128] = {0xef, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};
uint8_t GATTS_CHAR_UUID_FW_VERSION[ESP_UUID_LEN_128] = {0xf0, 0x9a, 0x93, 0xb9, 0xb5, 0x82, 0x4c, 0x5c, 0xa3, 0x63, 0xcb, 0x67, 0x62, 0x4, 0xc5, 0x21};

static const uint8_t led_char_value[100];

static const uint8_t button_value[2];

uint8_t GATTS_SERVICE_UUID_CERTIFICATE[ESP_UUID_LEN_128] = {0x37, 0x8e, 0xd, 0xef, 0x8e, 0x8b, 0x7f, 0xab, 0x33, 0x44, 0x89, 0x5b, 0x9, 0x77, 0xe8, 0xbb};
uint8_t GATTS_CHAR_UUID_CENTRAL_TO_SFIDA[ESP_UUID_LEN_128] = {0x38, 0x8e, 0xd, 0xef, 0x8e, 0x8b, 0x7f, 0xab, 0x33, 0x44, 0x89, 0x5b, 0x9, 0x77, 0xe8, 0xbb};
uint8_t GATTS_CHAR_UUID_SFIDA_COMMANDS[ESP_UUID_LEN_128] = {0x39, 0x8e, 0xd, 0xef, 0x8e, 0x8b, 0x7f, 0xab, 0x33, 0x44, 0x89, 0x5b, 0x9, 0x77, 0xe8, 0xbb};
uint8_t GATTS_CHAR_UUID_SFIDA_TO_CENTRAL[ESP_UUID_LEN_128] = {0x3a, 0x8e, 0xd, 0xef, 0x8e, 0x8b, 0x7f, 0xab, 0x33, 0x44, 0x89, 0x5b, 0x9, 0x77, 0xe8, 0xbb};
/*
  {ESP_GATT_AUTO_RSP/ESP_GATT_RSP_BY_APP},
  {uuid_length, uuid_val, attribute permission, max length of element, current length of element, element value array}
 */

static const esp_gatts_attr_db_t gatt_db_led_button[LED_BUTTON_LAST_IDX] = {
    // Service Declaration
    [IDX_LED_BUTTON_SVC] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & primary_service_uuid, ESP_GATT_PERM_READ,
            ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *) & GATTS_SERVICE_UUID_LED_BUTTON}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_LED] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_write}
    },
    [IDX_CHAR_LED_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_LED, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },


    [IDX_CHAR_BUTTON] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_notify}
    },
    [IDX_CHAR_BUTTON_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_BUTTON, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (button_value), (uint8_t *) button_value}
    },

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_BUTTON_CFG] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof (uint16_t), sizeof (dummy_value), (uint8_t *) dummy_value}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_UNKNOWN] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_write}
    },
    [IDX_CHAR_UNKNOWN_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_UNKNOWN, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_UPDATE_REQUEST] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_write}
    },
    [IDX_CHAR_UPDATE_REQUEST_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_UPDATE_REQUEST, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_FW_VERSION] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_read}
    },
    [IDX_CHAR_FW_VERSION_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_FW_VERSION, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },

};


static const esp_gatts_attr_db_t gatt_db_certificate[CERT_LAST_IDX] = {
    // Service Declaration
    [IDX_CERT_SVC] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & primary_service_uuid, ESP_GATT_PERM_READ,
            ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *) & GATTS_SERVICE_UUID_CERTIFICATE}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_CENTRAL_TO_SFIDA] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_write}
    },
    [IDX_CHAR_CENTRAL_TO_SFIDA_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_CENTRAL_TO_SFIDA, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },


    [IDX_CHAR_SFIDA_COMMANDS] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_notify}
    },
    [IDX_CHAR_SFIDA_COMMANDS_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_SFIDA_COMMANDS, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (led_char_value), (uint8_t *) led_char_value}
    },

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_SFIDA_COMMANDS_CFG] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof (uint16_t), sizeof (dummy_value), (uint8_t *) dummy_value}
    },

    /* Characteristic Declaration */
    [IDX_CHAR_SFIDA_TO_CENTRAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *) & character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *) & char_prop_read}
    },
    [IDX_CHAR_SFIDA_TO_CENTRAL_VAL] =
    {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *) & GATTS_CHAR_UUID_SFIDA_TO_CENTRAL, ESP_GATT_PERM_READ,
            MAX_VALUE_LENGTH, sizeof (cert_buffer), (uint8_t *) cert_buffer}
    },
};

const char *battery_char_names[] = {
    "BATTERY_SVC",
    "CHAR_BATTERY_LEVEL",
    "CHAR_BATTERY_LEVEL_VAL",
    "CHAR_BATTERY_LEVEL_CFG",
};

const char *led_button_char_names[] = {
    "LED_BUTTON_SVC",
    "CHAR_LED",
    "CHAR_LED_VAL",
    "CHAR_BUTTON",
    "CHAR_BUTTON_VAL",
    "CHAR_BUTTON_CFG,   "
    "CHAR_UNKNOWN",
    "CHAR_UNKNOWN_VAL",
    "CHAR_UPDATE_REQUEST",
    "CHAR_UPDATE_REQUEST_VAL",
    "CHAR_FW_VERSION",
    "CHAR_FW_VERSION_VAL"
};

const char *cert_char_names[] = {
    "CERT_SVC",
    "CHAR_CENTRAL_TO_SFIDA",
    "CHAR_CENTRAL_TO_SFIDA_VAL",
    "CHAR_SFIDA_COMMANDS",
    "CHAR_SFIDA_COMMANDS_VAL",
    "CHAR_SFIDA_COMMANDS_CFG",
    "CHAR_SFIDA_TO_CENTRAL",
    "CHAR_SFIDA_TO_CENTRAL_VAL"
};

const char *UNKNOWN_HANDLE_NAME = "<UNKNOWN HANDLE NAME>";

static int find_handle_index(uint16_t handle, uint16_t *handle_table, int count) {
    for (int i = 0; i < count; i++) {
        if (handle_table[i] == handle)
            return i;
    }
    return -1;
}

//for debugging

static const char *char_name_from_handle(uint16_t handle) {
    int idx;
    idx = find_handle_index(handle, battery_handle_table, BATTERY_LAST_IDX);
    if (idx >= 0)
        return battery_char_names[idx];
    idx = find_handle_index(handle, led_button_handle_table, LED_BUTTON_LAST_IDX);
    if (idx >= 0)
        return led_button_char_names[idx];

    idx = find_handle_index(handle, certificate_handle_table, CERT_LAST_IDX);
    if (idx >= 0)
        return cert_char_names[idx];

    return UNKNOWN_HANDLE_NAME;
}

static void generate_first_challenge() {
    //fill in cert_buffer
    uint8_t the_challenge[16];
    memset(the_challenge, 0x41, 16);
    uint8_t main_nonce[16];
    memset(main_nonce, 0x42, 16);

    //use fixed key for easier debugging
    memset(session_key, 0x43, 16);
    uint8_t outer_nonce[16];
    memset(outer_nonce, 0x44, 16);
    generate_chal_0(bt_mac, the_challenge, main_nonce, session_key, outer_nonce,
            (struct challenge_data *) cert_buffer);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            } else {
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
                TFT_setFont(MAIN_FONT, NULL);
                // TODO: connected
                screensaver = screensaver_time;
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            } else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                    param->update_conn_params.status,
                    param->update_conn_params.min_int,
                    param->update_conn_params.max_int,
                    param->update_conn_params.conn_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void pgp_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->handle = param->write.handle;
        prepare_write_env->prepare_buf = (uint8_t *) malloc(PREPARE_BUF_MAX_SIZE * sizeof (uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp) {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *) malloc(sizeof (esp_gatt_rsp_t));
        if (gatt_rsp != NULL) {

            /*
        gatt_rsp->attr_value.len = param->write.len;
        gatt_rsp->attr_value.handle = param->write.handle;
        gatt_rsp->attr_value.offset = param->write.offset;
        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
        esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
        if (response_err != ESP_OK){
           ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
        }
        free(gatt_rsp);
             */


        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK) {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
            param->write.value,
            param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

//prepare_write_env->prepare_buf
//param->write.len
//param->write.conn_id

void handle_protocol(esp_gatt_if_t gatts_if, const uint8_t *prepare_buf, int datalen, int conn_id) {
    switch (cert_state) {

        case 0:
        {
            if (datalen == 20) { //just assume server responds correctly
                uint8_t notify_data[4];
                memset(notify_data, 0, 4);
                notify_data[0] = 0x01;

                uint8_t nonce[16];

                memset(nonce, 0x42, 16);

                uint8_t temp[52];

                struct next_challenge* chal = (struct next_challenge*) temp;
                memset(temp, 0, sizeof (temp));
                generate_next_chal(0, session_key, nonce, chal);
                temp[0] = 0x01;

                memcpy(cert_buffer, temp, 52);

                //esp_log_buffer_hex(GATTS_TABLE_TAG, temp, sizeof(temp));

                esp_ble_gatts_set_attr_value(certificate_handle_table[IDX_CHAR_SFIDA_TO_CENTRAL_VAL], 52, cert_buffer);

                //esp_log_buffer_hex(GATTS_TABLE_TAG, cert_buffer, 52);

                cert_state = 1;
                esp_ble_gatts_send_indicate(gatts_if, conn_id, certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL], sizeof (notify_data), notify_data, false);
            } else {
                ESP_LOGE(GATTS_TABLE_TAG, "App sends incorrect len=%d", datalen);
            }
            break;
        }
        case 1:
        {
            //we need to decrypt and send challenge data from APP
            ESP_LOGI(GATTS_TABLE_TAG, "Received: %d", datalen);
            uint8_t temp[20];
            memset(temp, 0, sizeof (temp));
            decrypt_next(prepare_buf, session_key, temp + 4);
            temp[0] = 0x02;
            uint8_t notify_data[4];
            memset(notify_data, 0, 4);
            notify_data[0] = 0x02;

            ESP_LOGI(GATTS_TABLE_TAG, "Sending response");
            esp_log_buffer_hex(GATTS_TABLE_TAG, temp, sizeof (temp));

            memcpy(cert_buffer, temp, 20);
            esp_ble_gatts_set_attr_value(certificate_handle_table[IDX_CHAR_SFIDA_TO_CENTRAL_VAL], 20, cert_buffer);
            cert_state = 2;
            esp_ble_gatts_send_indicate(gatts_if, conn_id, certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL], sizeof (notify_data), notify_data, false);
            break;
        }
        case 2:
        {
            //TODO: what do we use the data for?
            ESP_LOGI(GATTS_TABLE_TAG, "Cert state==2 Received: %d", datalen);
            uint8_t temp[20];
            memset(temp, 0, sizeof (temp));
            decrypt_next(prepare_buf, session_key, temp + 4);
            ESP_LOGI(GATTS_TABLE_TAG, "DEBUG:");
            esp_log_buffer_hex(GATTS_TABLE_TAG, temp, sizeof (temp));

            has_reconnect_key = 1;
            //should be random, but this is easier for debugging
            memset(reconnect_challenge, 0x46, 32);

            uint8_t notify_data[4] = {0x04, 0x00, 0x23, 0x00};
            cert_state = 6;
            esp_ble_gatts_send_indicate(gatts_if,
                    conn_id,
                    certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL],
                    sizeof (notify_data), notify_data, false);
            display_state = 3;
            display_clean_stats();
            pokemon_caught = pokemon_seen = pokestops = 0;
            pokemon_changed = 2000;
            minute = 60;
            second = 0;
            break;
        }
        case 3:
        {
            if (datalen == 20) { //just assume server responds correctly
                esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_buf,
                        datalen);
                uint8_t notify_data[4] = {0x04, 0x00, 0x01, 0x00};
                esp_ble_gatts_send_indicate(gatts_if,
                        conn_id,
                        certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL],
                        sizeof (notify_data), notify_data, false);
                cert_state = 4;
            }
            break;
        }
        case 4:
        {
            memset(cert_buffer, 0, 4);
            generate_reconnect_response(session_key, prepare_buf + 4, cert_buffer + 4);
            cert_buffer[0] = 5;

            esp_log_buffer_hex(GATTS_TABLE_TAG, cert_buffer, 20);

            uint8_t notify_data[4];
            memset(notify_data, 0, 4);
            notify_data[0] = 0x05;

            esp_ble_gatts_set_attr_value(certificate_handle_table[IDX_CHAR_SFIDA_TO_CENTRAL_VAL], 20, cert_buffer);
            esp_ble_gatts_send_indicate(gatts_if, conn_id, certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL], sizeof (notify_data), notify_data, false);
            cert_state = 5;
            break;
        }
        case 5:
        {
            if (datalen == 5) { //just assume server responsds correctly
                uint8_t notify_data[4] = {0x04, 0x00, 0x02, 0x00};
                esp_ble_gatts_send_indicate(gatts_if,
                        conn_id,
                        certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL],
                        sizeof (notify_data), notify_data, false);
                cert_state = 6;
                display_state = 3;
                display_clean_stats();
                pokemon_caught = pokemon_seen = pokestops = 0;
                pokemon_changed = 2000;
                minute = 60;
                second = 0;
            }
            break;
        }
        default:
            ESP_LOGE(GATTS_TABLE_TAG, "Unhandled state: %d", cert_state);
            break;
    }
}

void handle_led_notify_from_app(const uint8_t *buffer) {
    int number_of_patterns = buffer[3] & 0x1f;
    int priority = (buffer[3] >> 5) & 0x7;
    uint8_t pattern_id = 0;

    ESP_LOGI(GATTS_TABLE_TAG, "LED: Pattern Count=%d priority: %d", number_of_patterns, priority);
    //1 pattern = 3 bytes
    for (int i = 0; i < number_of_patterns; i++) {
        int p = 4 + 3 * i;
        const uint8_t *pat = &buffer[p];
        uint8_t duration = pat[0];
        uint8_t red = pat[1] & 0xf;
        uint8_t green = (pat[1] >> 4) & 0xf;
        uint8_t blue = pat[2] & 0xf;
        ESP_LOGI(GATTS_TABLE_TAG, "*(%d) #%02x%02x%02x", duration, red, green, blue);
        pattern_id += (red + green + blue);
    }

    ESP_LOGI(GATTS_TABLE_TAG, "Pattern ID: %u", pattern_id);
    switch (pattern_id) {
        case PATTERN_STOP_SUCCES:
        case PATTERN_STOP_SUCCES2:
        case PATTERN_GYM_SUCCES:
            pokestops++;
            ESP_LOGI(GATTS_TABLE_TAG, "Pokestops: %u", pokestops);
            break;
        case PATTERN_CATCH_START:
            pokemon_seen++;
            ESP_LOGI(GATTS_TABLE_TAG, "Seen: %u", pokestops);
            break;
        case PATTERN_CATCH_SUCCES:
            pokemon_caught++;
            ESP_LOGI(GATTS_TABLE_TAG, "Caught: %u", pokestops);
            break;
        default:
            ESP_LOGI(GATTS_TABLE_TAG, "Other pattern: %u", pattern_id);
    }
    ESP_LOGI(GATTS_TABLE_TAG, "Sending push button");
    xQueueSend(button_queue, &number_of_patterns, portMAX_DELAY);

}

void pgp_exec_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf) {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);

        //it seems that this will be called only by Android version of the Pokemon Go App
        ESP_LOGI(GATTS_TABLE_TAG, "WRITE EVT");

        if (certificate_handle_table[IDX_CHAR_CENTRAL_TO_SFIDA_VAL] == prepare_write_env->handle) {

            handle_protocol(gatts_if,
                    prepare_write_env->prepare_buf,
                    prepare_write_env->prepare_len,
                    param->write.conn_id);


        } else if (led_button_handle_table[IDX_CHAR_LED_VAL] == prepare_write_env->handle) {
            handle_led_notify_from_app(prepare_write_env->prepare_buf);

        }

    } else {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
        {
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(PGP_DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof (raw_adv_data));
            if (raw_adv_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;

            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof (raw_scan_rsp_data));
            if (raw_scan_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            /* esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID); */
            /* if (create_attr_ret){ */
            /*     ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret); */
            /* } */
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_battery, gatts_if, BATTERY_LAST_IDX, BATTERY_INST_ID);
            if (create_attr_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table for battery failed, error code = %x", create_attr_ret);
            }

            create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_led_button, gatts_if, LED_BUTTON_LAST_IDX, LED_BUTTON_INST_ID);
            if (create_attr_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table for led button failed, error code = %x", create_attr_ret);
            }

            create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_certificate, gatts_if, CERT_LAST_IDX, CERT_INST_ID);
            if (create_attr_ret) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table for cert  failed, error code = %x", create_attr_ret);
            }
        }
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT state=%d %s", cert_state,
                    char_name_from_handle(param->read.handle));
            if (cert_state == 1) {
                ESP_LOGI(GATTS_TABLE_TAG, "DATA SENT TO APP");
                esp_log_buffer_hex(GATTS_TABLE_TAG, cert_buffer, 52);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT: %s", char_name_from_handle(param->write.handle));
            if (!param->write.is_prep) {
                // the data length of gattc write  must be less than MAX_VALUE_LENGTH.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT (state=%d), handle = %d, value len = %d, value :", cert_state, param->write.handle, param->write.len);
                ESP_LOGI(GATTS_TABLE_TAG, "DATA FROM APP");
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

                if (certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_CFG] == param->write.handle) {

                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001) {

                        uint8_t notify_data[4];
                        memset(notify_data, 0, 4);

                        if (has_reconnect_key) {
                            memset(cert_buffer, 0, 36);
                            cert_buffer[0] = 3;
                            notify_data[0] = 3;

                            memcpy(cert_buffer + 4, reconnect_challenge, 32);
                            esp_ble_gatts_set_attr_value(certificate_handle_table[IDX_CHAR_SFIDA_TO_CENTRAL_VAL], 36, cert_buffer);
                            cert_state = 3;

                        } else {
                            generate_first_challenge();
                            esp_ble_gatts_set_attr_value(certificate_handle_table[IDX_CHAR_SFIDA_TO_CENTRAL_VAL], 378, cert_buffer);
                        }

                        ESP_LOGI(GATTS_TABLE_TAG, "start CERT PAIRING len = %d", param->write.len);
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, certificate_handle_table[IDX_CHAR_SFIDA_COMMANDS_VAL],
                                sizeof (notify_data), notify_data, false);


                    } else if (descr_value == 0x000) {
                        ESP_LOGI(GATTS_TABLE_TAG, "notify disable");
                    }
                } else if (certificate_handle_table[IDX_CHAR_CENTRAL_TO_SFIDA_VAL] == param->write.handle) {
                    ESP_LOGI(GATTS_TABLE_TAG, "Write CENTRAL TO SFIDA: state=%d len=%d", cert_state, param->write.len);

                    handle_protocol(gatts_if,
                            param->write.value,
                            param->write.len,
                            param->write.conn_id);
                } else if (led_button_handle_table[IDX_CHAR_LED_VAL] == param->write.handle) {
                    handle_led_notify_from_app(param->write.value);
                    return;
                } else {
                    ESP_LOGE(GATTS_TABLE_TAG, "unhandled data: handle: %d", param->write.handle);
                    for (int i = 0; i < CERT_LAST_IDX; i++) {
                        ESP_LOGE(GATTS_TABLE_TAG, "handle: %d=%s",
                                certificate_handle_table[i],
                                char_name_from_handle(certificate_handle_table[i]));
                    }
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }

            } else {
                /* handle prepare write */
                pgp_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prapare write data must be less than MAX_VALUE_LENGTH.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            pgp_exec_write_event_env(gatts_if, &prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d", param->conf.status);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof (esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400; // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            last_conn_id = param->write.conn_id;
            last_if = gatts_if;

            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            TFT_setFont(MAIN_FONT, NULL);
            display_state = 2;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            // TODO: disconnected
            display_state = 1;
            refresh_screen = 1;
            screensaver = screensaver_time;
            screen_on = 1;
            current_time = waiting_time;
            gpio_set_level(GPIO_OUTPUT_IO, screen_on);
            cert_state = 0;
            nvs_write();

            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            int found = 0;
            if (param->add_attr_tab.svc_uuid.len == ESP_UUID_LEN_16) {

                if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_BATTERY) {
                    memcpy(battery_handle_table, param->add_attr_tab.handles, sizeof (battery_handle_table));
                    esp_ble_gatts_start_service(battery_handle_table[IDX_BATTERY_SVC]);
                    ESP_LOGI(GATTS_TABLE_TAG, "create  battery attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                    found = 1;
                }
            }

            if (param->add_attr_tab.svc_uuid.len == ESP_UUID_LEN_128) {
                if (memcmp(param->add_attr_tab.svc_uuid.uuid.uuid128, GATTS_SERVICE_UUID_LED_BUTTON, ESP_UUID_LEN_128) == 0) {
                    memcpy(led_button_handle_table, param->add_attr_tab.handles, sizeof (led_button_handle_table));
                    esp_err_t response_err = esp_ble_gatts_start_service(led_button_handle_table[IDX_LED_BUTTON_SVC]);
                    if (response_err != ESP_OK) {
                        ESP_LOGE(GATTS_TABLE_TAG, "failed starting service: %d", response_err);
                    }
                    ESP_LOGI(GATTS_TABLE_TAG, "create  led button attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                    found = 1;
                }
                if (memcmp(param->add_attr_tab.svc_uuid.uuid.uuid128, GATTS_SERVICE_UUID_CERTIFICATE, ESP_UUID_LEN_128) == 0) {
                    memcpy(certificate_handle_table, param->add_attr_tab.handles, sizeof (certificate_handle_table));
                    esp_err_t response_err = esp_ble_gatts_start_service(certificate_handle_table[IDX_CERT_SVC]);
                    if (response_err != ESP_OK) {
                        ESP_LOGE(GATTS_TABLE_TAG, "failed starting service: %d", response_err);
                    }
                    ESP_LOGI(GATTS_TABLE_TAG, "create  cert attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                    found = 1;
                }
            }

            if (!found) {
                ESP_LOGE(GATTS_TABLE_TAG, "service not found");
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            ESP_LOGE(GATTS_TABLE_TAG, "unhandled event");
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            pgp_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == pgp_profile_tab[idx].gatts_if) {
                if (pgp_profile_tab[idx].gatts_cb) {
                    pgp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// =============================================================================
//                                TASKS
// =============================================================================

static void uart_event_task(void *pvParameters) {
    uart_event_t event;

    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *) &event, (portTickType) portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type) {
                    //Event of UART receving data
                    /*We'd better handler data event fast, there would be much more data events than
                    other types of events. If we take too much time on data event, the queue might
                    be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    if (dtmp[0] == 'q') {
                        ESP_LOGI(TAG, "[push button]");
                        uint8_t notify_data[2];
                        notify_data[0] = 0x03;
                        notify_data[1] = 0xff;

                        esp_ble_gatts_send_indicate(last_if, last_conn_id, led_button_handle_table[IDX_CHAR_BUTTON_VAL],
                                sizeof (notify_data), notify_data, false);
                    }

                    if (dtmp[0] == 'w') {
                        ESP_LOGI(TAG, "[unpush button]");
                        uint8_t notify_data[2];
                        notify_data[0] = 0x00;
                        notify_data[1] = 0x00;

                        esp_ble_gatts_send_indicate(last_if, last_conn_id, led_button_handle_table[IDX_CHAR_BUTTON_VAL],
                                sizeof (notify_data), notify_data, false);
                    }

                    break;
                    //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                    //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                    //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                    //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

static void auto_button_task(void *pvParameters) {
    ESP_LOGI("BUTTON", "[button task start]");
    while (1) {
        int element;
        if (xQueueReceive(button_queue, &element, portMAX_DELAY)) {
            ESP_LOGI("BUTTON", "[auto push button]");
            uint8_t notify_data[2];
            notify_data[0] = 0x03;
            notify_data[1] = 0xff;

            esp_ble_gatts_send_indicate(last_if,
                    last_conn_id,
                    led_button_handle_table[IDX_CHAR_BUTTON_VAL],
                    sizeof (notify_data), notify_data, false);
        }
    }
}

static void main_task(void *pvParameters) {
    int16_t task_time = 0;
    charging_prev = charging;
    current_time = waiting_time;
    while (1) {
        if (display_state == 3) {
            // Connected
            sleep(1); //wait for one second
            task_time++;
            second--;
            if (second == -1) {
                second = 59;
                minute--;
            }
            if (minute == -1) {
                minute = 0;
                second = 0;
                ESP_LOGI("CLOCK", "Time is up, restarting now..");
                nvs_write();
                fflush(stdout);
                esp_restart();
            }
            if (charging_prev != charging) {
                charging_prev = charging;
                if (charging == 0) {
                    screensaver = screensaver_time;
                }
            }
            if (screensaver > 0 && charging == 0) {
                screensaver--;
                if (screensaver == 0) {
                    gpio_set_level(GPIO_OUTPUT_IO, 0);
                    screen_on = 0;
                }
            }
            
            if (refresh_screen == 1) {
                draw_screen();
            }

            if (screen_on == 1) {
                draw_stats();
                if (task_time % 5 == 0 || refresh_screen == 1) {
                    draw_battery();
                }
                if (task_time % 10 == 0) {
                    nvs_write();
                }
            }
        } else {
            // Ready
            if (display_state == 1) current_time--;
            sleep(1);
            task_time++;
            if (screen_on == 1) {
                draw_status();
                draw_stats();
                if (task_time % 5 == 0 || refresh_screen == 1) {
                    draw_battery();
                }
            }
            
            if (refresh_screen == 1) {
                draw_screen();
            }
            
            if (charging == 1 || OPTION_USE_SLEEP == 0) {
                current_time = waiting_time + 2;
            } else {
                if (current_time == 0) {
                    // deep sleep
                    nvs_write();
                    fflush(stdout);
                    esp_bluedroid_disable();
                    esp_bt_controller_disable();
                    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
                    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
                    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
                    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
                    esp_deep_sleep_start();
                }
            }
        }
        refresh_screen = 0;
    }
}

static void gpio_task(void* arg) {
    uint32_t io_num;
    uint8_t pressed0 = 1;
    uint8_t pressed1 = 1;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            switch (io_num) {
                case GPIO_INPUT_IO_0:
                    // change color
                    if (screen_on == 1) {
                        if (gpio_get_level(io_num) == 0) {
                            if (pressed0 == 1) {
                                pressed0 = 0;
                                if (font_color < color_size - 1) {
                                    font_color++;
                                } else {
                                    font_color = 0;
                                }
                                _fg = colors[font_color];
                                pokemon_changed = 2000; //trigger update
                                if (screen_on == 1) {
                                    refresh_screen = 1;
                                }
                                if (display_state == 3) {
                                    pokemon_changed++;
                                }
                            }
                        } else {
                            pressed0 = 1;
                        }
                    }
                    break;
                case GPIO_INPUT_IO_1:
                    // toggle display
                    if (gpio_get_level(io_num) == 0) {
                        if (pressed1 == 1) {
                            pressed1 = 0;
                            if (screen_on == 1) {
                                screensaver = 0;
                                screen_on = 0;
                            } else {
                                screensaver = screensaver_time;
                                screen_on = 1;
                                pokemon_changed = 2000; //trigger update
                                refresh_screen = 1;
                            }
                            gpio_set_level(GPIO_OUTPUT_IO, screen_on);
                            if (screen_on == 0) {
                                display_clean();
                            }
                        }
                    } else {
                        pressed1 = 1;
                    }
                    break;
            }
        }
    }
}

// =============================================================================
//                                GPIO
// =============================================================================

void configure_gpio() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; //GPIO_PIN_INTR_POSEDGE; //GPIO_INTR_ANYEDGE 
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 1;
    esp_err_t error = gpio_config(&io_conf);
    if (error != ESP_OK) {
        printf("error configuring inputs \n");
    }

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    error = gpio_config(&io_conf);
    if (error != ESP_OK) {
        printf("error configuring outputs \n");
    }

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof (uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// =============================================================================
//                                  NVS
// =============================================================================

void nvs_init() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void nvs_read() {
    ESP_LOGI("NVS", "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "Done");

        // Read
        int16_t _pokemon_seen = 0; // value will default to 0, if not set yet in NVS
        int16_t _pokemon_caught = 0;
        int16_t _pokestops = 0;
        int8_t _font_color = 0;
        int8_t _minute = 60;
        int8_t _second = 0;
		int8_t _device = 0;
        err = nvs_get_i16(my_handle, "pokemon_seen", &_pokemon_seen);
        if (err == ESP_OK) {
            pokemon_seen = _pokemon_seen;
            ESP_LOGI("NVS", "Done");
        }
        err = nvs_get_i16(my_handle, "pokemon_caught", &_pokemon_caught);
        if (err == ESP_OK) {
            pokemon_caught = _pokemon_caught;
            ESP_LOGI("NVS", "Done");
        }
        err = nvs_get_i16(my_handle, "pokestops", &_pokestops);
        if (err == ESP_OK) {
            pokestops = _pokestops;
            ESP_LOGI("NVS", "Done");
        }
        err = nvs_get_i8(my_handle, "font_color", &_font_color);
        if (err == ESP_OK) {
            font_color = _font_color;
            ESP_LOGI("NVS", "Done");
        }
        err = nvs_get_i8(my_handle, "minute", &_minute);
        if (err == ESP_OK) {
            minute = _minute;
            ESP_LOGI("NVS", "Done");
        }
        err = nvs_get_i8(my_handle, "second", &_second);
        if (err == ESP_OK) {
            second = _second;
            ESP_LOGI("NVS", "Done");
        }
		err = nvs_get_i8(my_handle, "device", &_device);
        if (err == ESP_OK) {
            device = _device;
            ESP_LOGI("NVS", "Done");
        }

        // Close
        nvs_close(my_handle);
    }
}

void nvs_write() {
    if (nvs_lock == 0) {
        nvs_lock = 1;
        ESP_LOGI("NVS", "Opening Non-Volatile Storage (NVS) handle... ");
        nvs_handle my_handle;
        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK) {
            ESP_LOGI("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
        } else {
            ESP_LOGI("NVS", "Done");

            // Write
            err = nvs_set_i16(my_handle, "pokemon_seen", pokemon_seen);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }
            err = nvs_set_i16(my_handle, "pokemon_caught", pokemon_caught);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }
            err = nvs_set_i16(my_handle, "pokestops", pokestops);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }

            err = nvs_set_i8(my_handle, "font_color", font_color);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }
            
            err = nvs_set_i8(my_handle, "minute", minute);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }
            err = nvs_set_i8(my_handle, "second", second);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }
			err = nvs_set_i8(my_handle, "device", device);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }

            // Commit written value.
            // After setting any values, nvs_commit() must be called to ensure changes are written
            // to flash storage. Implementations may write to storage at other times,
            // but this is not guaranteed.
            ESP_LOGI("NVS", "Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            if (err == ESP_OK) {
                ESP_LOGI("NVS", "Done");
            }

            // Close
            nvs_close(my_handle);
        }
        nvs_lock = 0;
    }
}

int8_t nvs_check_stats_change() {
    if (display_state == 3) {
        if (pokemon_changed != (pokemon_seen + pokemon_caught + pokestops)) {
            pokemon_changed = (pokemon_seen + pokemon_caught + pokestops);
            return 1;
        }
    }
    return 0;
}

// =============================================================================
//                                DISPLAY
// =============================================================================

void init_display() {
    esp_err_t ret;

    // Display config
    tft_disp_type = DEFAULT_DISP_TYPE;
    _width = DEFAULT_TFT_DISPLAY_WIDTH; // smaller dimension
    _height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
    max_rdclock = 8000000;

    // Initialize pins
    TFT_PinsInit();

    // Configure SPI devices
    spi_lobo_device_handle_t spi;

    spi_lobo_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, // set SPI MISO pin
        .mosi_io_num = PIN_NUM_MOSI, // set SPI MOSI pin
        .sclk_io_num = PIN_NUM_CLK, // set SPI CLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 6 * 1024,
    };
    spi_lobo_device_interface_config_t devcfg = {
        .clock_speed_hz = 8000000, // Initial clock out at 8 MHz
        .mode = 0, // SPI mode 0
        .spics_io_num = -1, // we will use external CS pin
        .spics_ext_io_num = PIN_NUM_CS, // external CS pin
        .flags = LB_SPI_DEVICE_HALFDUPLEX, // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };

    ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret == ESP_OK);
    ESP_LOGI("SPI", "display device added to spi bus (%d)", SPI_BUS);
    disp_spi = spi;

    // ==== Test select/deselect ====
    ret = spi_lobo_device_select(spi, 1);
    assert(ret == ESP_OK);
    ret = spi_lobo_device_deselect(spi);
    assert(ret == ESP_OK);

    ESP_LOGI("SPI", "attached display device, speed=%u", spi_lobo_get_speed(spi));
    ESP_LOGI("SPI", "bus uses native pins: %s", (spi_lobo_uses_native_pins(spi) ? "true" : "false"));

    // Initialize display
    ESP_LOGI("SPI", "display init...");
    TFT_display_init();
    TFT_invertDisplay(INVERT_ON);

    TFT_resetclipwin();
    TFT_setRotation(LANDSCAPE);
    display_clean();
    TFT_setFont(MAIN_FONT, NULL);
    _fg = colors[font_color];
    _bg = TFT_WHITE; //TFT_WHITE;
    // Red = Cyan
    // Green = Pink
    // Yellow = Blue (dark)
    ESP_LOGI("SPI", "Done");
}

void display_clean() {
    TFT_fillRect(0, 0, _width, _height, TFT_WHITE);
}

void display_clean_status() {
    TFT_fillRect(0, _height-20, _width, 20, TFT_WHITE);
}

void display_clean_stats() {
    TFT_fillRect(27, 7, 32, 14, TFT_WHITE);
    TFT_fillRect(27+64, 7, 32, 14, TFT_WHITE);
    TFT_fillRect(27+128, 7, 32, 14, TFT_WHITE);
}

void draw_screen() {
    // Lines
    color_t col = colors[font_color];
    uint8_t y = 14;
    uint8_t x = 14;
    
    TFT_drawFastHLine(0, (y*2), _width, col);
    TFT_drawFastHLine(0, _height - (y*2), _width, col);
    
    // Pokeball icon
    TFT_drawCircle(x, y, 3, col);
    TFT_drawCircle(x, y, 8, col);
    
    TFT_drawFastHLine(x - 8, y, 6, col);
    TFT_drawFastHLine(x + 3, y, 6, col);
    x += 64;
    
    // Cloud icon
    TFT_drawArc(x - 1, y - 4, 4, 4, 270, 180, col, TFT_WHITE);
    TFT_drawArc(x + 8, y - 2, 4, 4, 270, 180, col, TFT_WHITE);
    TFT_drawArc(x - 5, y + 5, 4, 4, 0, 270, col, TFT_WHITE);
    
    TFT_drawFastHLine(x - 8, y - 1, 8, col);
    TFT_drawFastHLine(x + 1, y + 1, 8, col);
    TFT_drawFastHLine(x - 11, y + 2, 7, col);
    x += 64;
    
    // Pokeball icon
    TFT_drawCircle(x, y, 4, col);
    TFT_drawCircle(x, y, 8, col);
    
    TFT_drawFastHLine(x - 8, y - 2, 5, col);
    TFT_drawFastHLine(x - 8, y + 1, 5, col);
    TFT_drawFastHLine(x + 3, y - 2, 5, col);
    TFT_drawFastHLine(x + 3, y + 1, 5, col);
    
    TFT_fillRect(x - 8, y - 1, 18, 2, TFT_WHITE);
    TFT_fillCircle(x, y, 1, col);
}

void draw_status() {
    if (display_state_old != display_state) {
        display_state_old = display_state;
        display_clean_status();
    }
    char *str;
    if (display_state == 0) {
        // Draw splash
        TFT_setFont(POKEMON48_FONT, NULL);
        str = "Sulpog";
        TFT_print(str, ((dispWin.x2 - dispWin.x1) / 2) - (TFT_getStringWidth(str) / 2), (dispWin.y2 - dispWin.y1) / 2 - (TFT_getfontheight() / 2));
        return;
    } else if (display_state == 1) {
        // Ready state
        if (display_state_ready != charging) {
            display_clean_status();
            display_state_ready = charging;
        }
        char outbuf[16];
        if (charging == 1 || OPTION_USE_SLEEP == 0) {
            snprintf(outbuf, sizeof (outbuf), "Ready");
        } else {
            snprintf(outbuf, sizeof (outbuf), "Ready %02d", current_time);
        }
        TFT_setFont(DETAIL_FONT, NULL);
        TFT_print(outbuf, (_width / 2) - (TFT_getStringWidth(outbuf) / 2), (_height - 14) - (TFT_getfontheight() / 2));
        return;
    } else if (display_state == 2) {
        // Connecting
        TFT_fillRect((_width / 2) - 20, _height * 0.7, 40, 13, TFT_WHITE);
        TFT_setFont(DETAIL_FONT, NULL);
        str = "Connecting";
    } else {
        // Connected
        TFT_setFont(DETAIL_FONT, NULL);
        str = "";
        //TODO: draw bluetooth device name
    }
    TFT_print(str, (_width / 2) - (TFT_getStringWidth(str) / 2), (_height - 14) - (TFT_getfontheight() / 2));
}

void draw_stats() {
    //stats
    if (nvs_check_stats_change() == 1 || display_state < 3 || refresh_screen == 1) {
        TFT_setFont(DETAIL_FONT, NULL);

        //draw catches
        char outbuf[8];
        snprintf(outbuf, sizeof (outbuf), "%u", pokemon_caught);
        TFT_print(outbuf, 28, 9);
        
        //draw flees
        char outbuf2[8];
        snprintf(outbuf2, sizeof (outbuf2), "%u", pokemon_seen-pokemon_caught);
        TFT_print(outbuf2, 28+64, 9);
        
        //draw stops
        char outbuf3[8];
        snprintf(outbuf3, sizeof (outbuf3), "%u", pokestops);
        TFT_print(outbuf3, 28+128, 9);
    }

    //clock
    //TFT_setFont(MAIN_FONT, NULL);
    TFT_setFont(DIGITAL32_FONT, NULL);
    char str[16];
    snprintf(str, sizeof (str), "%02d:%02d", minute, second);
    uint8_t ww = TFT_getStringWidth(str);
    TFT_print(str, ((dispWin.x2 - dispWin.x1) / 2) - (ww / 2), (dispWin.y2 - dispWin.y1) / 2 - (TFT_getfontheight() / 2));
}

void draw_device() {
	char str[5];
    snprintf(str, sizeof (str), "%i", device);
	TFT_setFont(DETAIL_FONT, NULL);
	TFT_print(str, _width-10, (_height - 14) - (TFT_getfontheight() / 2));
}

uint32_t calculate_battery() {
    //battery percentage
    uint32_t adc_reading = 0;
    uint8_t NO_OF_SAMPLES = 64;
    adc1_config_width(ADC_WIDTH_BIT_10); //ADC_WIDTH_BIT_10
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); //ADC_ATTEN_DB_6

    //esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //Type of calibration value used to characterize ADC is eFuse Vref

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_reading /= NO_OF_SAMPLES;

    charging = (adc_reading >= BATTERY_MAX + BATTERY_OFFSET);
	
	battery_char_value[0] = (((double)(adc_reading - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100);
	if (battery_char_value[0] > 100) battery_char_value[0] = 100;
	
	//ESP_LOGI("BATTERY", "Perc %i", battery_char_value[0]);
	
	// Send battery percentage
	//uint8_t notify_percentage[1];
	//memset(notify_percentage, 0, 1);
	//notify_percentage[0] = battery_char_value[0];
	//esp_ble_gatts_send_indicate(last_if, last_conn_id, battery_handle_table[IDX_CHAR_BATTERY_LEVEL_VAL], sizeof (notify_percentage), notify_percentage, false);

    //ESP_LOGI("BATTERY", "Raw %i", adc_reading);

    //uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    //ESP_LOGI("BATTERY", "Voltage %imV", voltage);
    return adc_reading;
}

void draw_battery() {
    //draw battery number
    //TFT_setFont(DETAIL_FONT, NULL);
    uint32_t adc_reading = calculate_battery();
    
    /*uint16_t length = snprintf(NULL, 0, "%u", adc_reading);
    char* battery_str = malloc(length + 1);
    snprintf(battery_str, length + 1, "%u", adc_reading);
    uint8_t w = TFT_getStringWidth(battery_str);
    uint8_t h = TFT_getfontheight(battery_str);
    TFT_print(battery_str, _width - w - 3, _height - h - 5);
    free(battery_str);*/

    uint8_t x = _width - 30;
    uint8_t y = 14;

    uint8_t width = 24;
    uint8_t height = 14;
    adc_reading = (adc_reading > BATTERY_MIN) ? adc_reading : BATTERY_MIN;
    adc_reading = (adc_reading < BATTERY_MAX) ? adc_reading : BATTERY_MAX;
    uint8_t part = ((float) (adc_reading - BATTERY_MIN) / (float) (BATTERY_MAX - BATTERY_MIN)) * (width - 4);

    TFT_drawRect(x, y - (height / 2), width, height, colors[font_color]);
    TFT_fillRect(x + width, (uint8_t) y - (height / 4), (uint8_t) (width / 8), (uint8_t) (height / 2), colors[font_color]);

    TFT_fillRect(x + 2, y - (height / 2) + 2, part, height - 4, colors[font_color]);
    color_t black = {255, 255, 255};
    TFT_fillRect(x + 2 + part, y - (height / 2) + 2, (width - 4) - part, height - 4, black);
}

// =============================================================================
//                                MAIN
// =============================================================================

void app_main() {
	calculate_battery();

    // NVS
    nvs_init();
    nvs_read();
	
	// Set device
	set_device(device);

    // Draw
    init_display();
    ESP_LOGI("DISPLAY", "Trying to draw something..");
    display_clean();
    draw_status();
    ESP_LOGI("DISPLAY", "Done");

    ESP_LOGI("GPIO", "Initialising button..");
    configure_gpio();
    ESP_LOGI("GPIO", "Done");
	
	// Check for device switch
	if (gpio_get_level(GPIO_INPUT_IO_1) == 0) {
		if (device > 4) {
			device = 0;
		} else {
			device = device + 1;
		}
		set_device(device);
		nvs_write();
	}
	draw_device();
	
    /* Initialize NVS. */
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    button_queue = xQueueCreate(10, sizeof ( int));

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    xTaskCreate(auto_button_task, "auto_button_task", 2048, NULL, 12, NULL);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    memcpy(bt_mac, MAC, 6);
    memcpy(mac, MAC, 6);

    mac[5] -= 2; //TODO: check what happens if last byte is 0 or 1

    esp_base_mac_addr_set(mac);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_log_buffer_hex(GATTS_TABLE_TAG, cert_buffer, sizeof (cert_buffer));
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; //set the IO capability to No output No input
    uint8_t key_size = 16; //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof (uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof (uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof (uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you,
    and the init key means which key you can distribut to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof (uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof (uint8_t));


    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    
    if (xHandleMainTask == NULL) {
        xTaskCreate(main_task, "main_task", 2048, NULL, 12, &xHandleMainTask);
    }
    
    display_clean();
    draw_screen();
    display_state = 1;
}