#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_gap_ble_api.h"
#include "esp_bt.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"

#define SPI_BUS TFT_HSPI_HOST

#define GATTS_TABLE_TAG "PGPEMU"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define PGP_DEVICE_NAME          "Pokemon GO Plus"
//#define PGP_DEVICE_NAME          "Pokemon PBP"
/* #define SVC_INST_ID                 0 */
/* #define BATTERY_INST_ID                 1 */
/* #define LED_BUTTON_INST_ID                 2 */


#define BATTERY_INST_ID                 0
#define LED_BUTTON_INST_ID              1
#define CERT_INST_ID              2

#define NVS_STATS 0
#define NVS_COLOR 1


/* The max length of characteristic value. When the gatt client write or prepare write,
 *  the data length must be less than MAX_VALUE_LENGTH.
 */
#define MAX_VALUE_LENGTH 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define GPIO_INPUT_IO_0 0
#define GPIO_INPUT_IO_1 35
#define GPIO_OUTPUT_IO 4
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1)
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO)
#define ESP_INTR_FLAG_DEFAULT 0

#define PATTERN_STOP_START 204
#define PATTERN_STOP_SUCCES 135
#define PATTERN_STOP_SUCCES2 225
#define PATTERN_GYM_SUCCES 59
#define PATTERN_CATCH_START 114
#define PATTERN_CATCH_START_NEW 0 //new encounters
#define PATTERN_CATCH_SUCCES 6
#define PATTERN_CATCH_FAIL 117

// The battery decreased faster below 500 and really fast below 450
#define BATTERY_MAX 596 //600 should be safer
#define BATTERY_MIN 420
#define BATTERY_OFFSET 20
// The absolute minimum is 383. This should be increased, because the last 50 or so goes really fast

#define MAIN_FONT DEJAVU24_FONT
#define DETAIL_FONT DEFAULT_FONT

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_HSPI_HOST

#define EX_UART_NUM UART_NUM_0

//Battery service

enum {
    IDX_BATTERY_SVC,
    IDX_CHAR_BATTERY_LEVEL,
    IDX_CHAR_BATTERY_LEVEL_VAL,
    IDX_CHAR_BATTERY_LEVEL_CFG,
    BATTERY_LAST_IDX
};

//LED/BUTTON service

enum {
    IDX_LED_BUTTON_SVC,
    IDX_CHAR_LED,
    IDX_CHAR_LED_VAL,
    IDX_CHAR_BUTTON,
    IDX_CHAR_BUTTON_VAL,
    IDX_CHAR_BUTTON_CFG,
    IDX_CHAR_UNKNOWN,
    IDX_CHAR_UNKNOWN_VAL,
    IDX_CHAR_UPDATE_REQUEST,
    IDX_CHAR_UPDATE_REQUEST_VAL,
    IDX_CHAR_FW_VERSION,
    IDX_CHAR_FW_VERSION_VAL,
    LED_BUTTON_LAST_IDX
};

//Certificate service

enum {
    IDX_CERT_SVC,
    IDX_CHAR_CENTRAL_TO_SFIDA,
    IDX_CHAR_CENTRAL_TO_SFIDA_VAL,
    IDX_CHAR_SFIDA_COMMANDS,
    IDX_CHAR_SFIDA_COMMANDS_VAL,
    IDX_CHAR_SFIDA_COMMANDS_CFG,
    IDX_CHAR_SFIDA_TO_CENTRAL,
    IDX_CHAR_SFIDA_TO_CENTRAL_VAL,
    CERT_LAST_IDX
};

typedef struct {
    uint16_t handle;
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

// Functions
static int find_handle_index(uint16_t handle, uint16_t *handle_table, int count);
static const char *char_name_from_handle(uint16_t handle);
static void generate_first_challenge();
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void pgp_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void handle_protocol(esp_gatt_if_t gatts_if, const uint8_t *prepare_buf, int datalen, int conn_id);
void handle_led_notify_from_app(const uint8_t *buffer);
void pgp_exec_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Tasks
static void auto_button_task(void *pvParameters);
static void uart_event_task(void *pvParameters);
static void main_task(void *pvParameters);
static void gpio_task(void* arg);

// GPIO
void configure_gpio();
static void IRAM_ATTR gpio_isr_handler(void* arg);

// NVS
void nvs_init();
void nvs_write();
void nvs_read();
void nvs_check_color_change();
int8_t nvs_check_stats_change();

// Display
void init_display();
void display_clean();
void display_clean_status();
void display_clean_stats();
void draw_screen();
void draw_status();
void draw_stats();
uint32_t calculate_battery();
void draw_battery();

// Main
void app_main();