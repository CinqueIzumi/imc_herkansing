#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

// Used for the LCD
#include "smbus.h"
#include "i2c-lcd1602.h"

// Used for the temp sensor
#include "dht11.h"

// Used for the on-board buttons
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"

#define TAG "app"

// LCD1602
#define LCD_NUM_ROWS               3
#define LCD_NUM_COLUMNS            20
#define LCD_NUM_VISIBLE_COLUMNS    20

// Undefine USE_STDIN if no stdin is available (e.g. no USB UART) - a fixed delay will occur instead of a wait for a keypress.
#define USE_STDIN  1
//#undef USE_STDIN

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

static i2c_lcd1602_info_t* lcd_info;

int current_temp = 24;
int pref_temp = 20;
int current_hum = 30;

// Used to determine and control the LCD screen state
enum screen_state {
    SCREEN_TEMP = 0x00,
    SCREEN_HUM = 0x01,
};
static int16_t screen_current_state = 0x00; 

static void init(void)
{
    ESP_LOGI(TAG, "[ 1 ] Set up I2C");
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);

    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    ESP_LOGI(TAG, "[ 2 ] Set up SMBus");
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    ESP_LOGI(TAG, "[ 3 ] Set up LCD1602 device with backlight off");
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    ESP_LOGI(TAG, "[ 4 ] Setting up the DHT sensor");
    DHT11_init(GPIO_NUM_5);
}

void screen_temperature_task(void * pvParameter) 
{
    char snum_current_temp[2];
    sprintf(snum_current_temp, "%d", current_temp);

    char snum_pref_temp[2];
    sprintf(snum_pref_temp, "%d", pref_temp);

    ESP_LOGI(TAG, "Displaying temperature screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current   temp:   ");
    i2c_lcd1602_write_string    (lcd_info, snum_current_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 1);
    i2c_lcd1602_write_string    (lcd_info, "Preferred temp:   ");
    i2c_lcd1602_write_string    (lcd_info, snum_pref_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Humidity");

    i2c_lcd1602_move_cursor     (lcd_info, 0, 3);
    i2c_lcd1602_write_string    (lcd_info, "Vol+/-> Pref +/-");

    vTaskDelete(NULL);
}

void screen_humidity_task(void * pvParameter) 
{
    char snum_current_hum[2];
    sprintf(snum_current_hum, "%d", current_hum);

    ESP_LOGI(TAG, "Displaying humidity screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current Humidity: ");
    i2c_lcd1602_write_string    (lcd_info, snum_current_hum);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Temp");

    vTaskDelete(NULL);
}

void update_dht_data(void)
{
    current_temp = DHT11_read().temperature;
    current_hum = DHT11_read().humidity;

    printf("Temperature is %d \n", DHT11_read().temperature);
    printf("Humidity is %d\n", DHT11_read().humidity);
    ESP_LOGI(TAG, "Data has been updated");
}

static TaskHandle_t active_task_handle;
void switch_screen_states(int16_t newState)
{
    update_dht_data();
    switch(newState) 
    {
        case SCREEN_TEMP:
            xTaskCreate(screen_temperature_task, "screen_temp_task", 1024*2, (void*)0, 10, &active_task_handle);
            screen_current_state = newState;
            break;
        case SCREEN_HUM:
            xTaskCreate(screen_humidity_task, "screen_hum_task", 1024*2, (void*)0, 10, &active_task_handle);
            screen_current_state = newState;
            break;
    }

    screen_current_state = newState;
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    const int input_type = evt->type;

    if(input_type == 1)
    {
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_SET:
                if(screen_current_state == 0x00)
                {
                    switch_screen_states(0x01); 
                } else 
                {
                    switch_screen_states(0x00);
                }
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                if(screen_current_state == 0x00)
                {
                    if(pref_temp > 0)
                    {
                        pref_temp--;
                        ESP_LOGI(TAG, "Preferred temperature has been reduced");
                        switch_screen_states(0x00);
                    }
                }
                break;
            case INPUT_KEY_USER_ID_PLAY:
                update_dht_data();
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                if(screen_current_state == 0x00)
                {
                    if(pref_temp < 30)
                    {
                        pref_temp++;
                        ESP_LOGI(TAG, "Preferred temperature has been increased");
                        switch_screen_states(0x00);
                    }
                }
                break;
            default:
                break;
        }
    }
    return ESP_OK;
}

void app_main()
{
    init();

    ESP_LOGI(TAG, "[ 5 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[ 6 ] Initialize Button peripheral with board init");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 7 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, NULL);

    ESP_LOGI(TAG, "[ 8 ] Waiting for a button to be pressed ...");

    // Set the start screen to the temp screen
    switch_screen_states(0x00);
}