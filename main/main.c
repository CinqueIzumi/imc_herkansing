#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

#include "smbus.h"
#include "i2c-lcd1602.h"

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

char* current_temp = "24";
char* pref_temp = "20";
char* current_hum = "30";

static void init(void)
{
    // Set up I2C
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

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
}

void screen_temperature_task(void * pvParameter) 
{
    ESP_LOGI(TAG, "Displaying temperature screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current   temp:   ");
    i2c_lcd1602_write_string    (lcd_info, current_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 1);
    i2c_lcd1602_write_string    (lcd_info, "Preferred temp:   ");
    i2c_lcd1602_write_string    (lcd_info, pref_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Humidity");

    i2c_lcd1602_move_cursor     (lcd_info, 0, 3);
    i2c_lcd1602_write_string    (lcd_info, "Vol+/-> Pref +/-");

    vTaskDelete(NULL);
}

void screen_humidity_task(void * pvParameter) 
{
    ESP_LOGI(TAG, "Displaying humidity screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current    Hum:   ");
    i2c_lcd1602_write_string    (lcd_info, current_hum);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Temp");

    vTaskDelete(NULL);
}

void app_main()
{
    init();

    // xTaskCreate(&screen_temperature_task, "screen_temp_task", 4096, NULL, 5, NULL);
    xTaskCreate(&screen_humidity_task, "screen_hum_task", 4096, NULL, 5, NULL);
}