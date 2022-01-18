#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

/* Used for the LCD */
#include "smbus.h"
#include "i2c-lcd1602.h"

/* Used for the rotary encoder */
#include "qwiic_twist.h"

/* Used for the temp sensor */
#include "dht11.h"

/* Used for the on-board buttons */
#include "esp_peripherals.h"
#include "periph_adc_button.h"
#include "input_key_service.h"

/* Define the tag, which will be used for logging */
#define TAG "imc_herkansing"

#define RELAY_GPIO GPIO_NUM_26

/* LCD1602 */
#define LCD_NUM_ROWS               3
#define LCD_NUM_COLUMNS            20
#define LCD_NUM_VISIBLE_COLUMNS    20

/* I2C */
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     /* disabled */
#define I2C_MASTER_RX_BUF_LEN    0                     /* disabled */
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

/* Variables used to keep track of the global components */
static i2c_lcd1602_info_t* lcd_info;
static qwiic_twist_t* qwiic_handle;

/* Queue used to send and receive the data */
QueueHandle_t xStructQueue = NULL;

/* Struct which shall be used to hold and pass around the data for the LCD screen */
struct LcdData
{
    int current_temp;
    int current_humidity;
    int preferred_temp;
    int16_t current_screen;
} xLcdData;

/* Ensure that the struct can be passed by value */
typedef struct LcdData LcdData;

/* Variables used to keep track of the current states */
static TaskHandle_t active_task_handle;

/* Used to determine and control the LCD screen state */
enum screen_state {
    SCREEN_TEMP = 0x00,
    SCREEN_HUM = 0x01,
};

/* Function declarations due to there not being a header file */
void initialize_queues(void);
void lcd_screen_task(void *pvParameters);
void draw_temperature_screen(LcdData given_data);
void draw_humidity_screen(LcdData given_data);
void update_dht_data();
void rotary_encoder_on_clicked();
void rotary_encoder_on_moved(int16_t idx);
static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx);

void relay_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad RELAY_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(RELAY_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Relay off (output low) */
        gpio_set_level(RELAY_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Relay on (output high) */
        gpio_set_level(RELAY_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* The main function */
void app_main()
{
    ESP_LOGI(TAG, "[ 1 ] Setting up I2C");
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  /* GY-2561 provides 10kΩ pullups */
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  /* GY-2561 provides 10kΩ pullups */
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    ESP_LOGI(TAG, "[ 2 ] Setting up SMBus");
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    ESP_LOGI(TAG, "[ 3 ] Initializing the queues");
    initialize_queues();

    ESP_LOGI(TAG, "[ 4 ] Setting up LCD1602 device with backlight off");
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    ESP_LOGI(TAG, "[ 5 ] Setting up the DHT sensor");
    DHT11_init(GPIO_NUM_5);
    update_dht_data();

    ESP_LOGI(TAG, "[ 6 ] Setting up the rotary encoder");
    qwiic_twist_t* qwiic_config = qwiic_twist_malloc();
    qwiic_config->port = I2C_MASTER_NUM;
    qwiic_config->i2c_addr = 0x3F;
    qwiic_config->onButtonClicked = &rotary_encoder_on_clicked;
    qwiic_config->onMoved = &rotary_encoder_on_moved;
    ESP_ERROR_CHECK(qwiic_twist_init(qwiic_config));
    ESP_ERROR_CHECK(qwiic_twist_set_color(qwiic_config, 0,0,0));
    qwiic_handle = qwiic_config;

    ESP_LOGI(TAG, "[ 7 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[ 8 ] Initialize Button peripheral with board init");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 9 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_cfg.based_cfg.task_stack = 4 * 1024;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, NULL);

    /* Initialize the LCD screen task */
    xTaskCreate(lcd_screen_task, "lcd_screen_task", 1024*2, (void*)0, 10, &active_task_handle);
    xTaskCreate(relay_task, "relay_task", 1024*2, NULL, 5, NULL);

    /* Add the initialized struct to the queue, to show the current temperature on boot */
    xQueueSend(xStructQueue, (void *) &xLcdData, ( TickType_t ) 0);

    /* Initialize the rotary encoder */
    if (qwiic_twist_start_task(qwiic_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "TASK_START_QWIIC_FAILED");
        esp_restart();
    }
}

/* A function which initialized the queue, which can be used to communicatie between tasks */
void initialize_queues(void)
{
    /* Set the default data of the LCD data struct */
    xLcdData.current_humidity = 0;
    xLcdData.current_temp = 0;
    xLcdData.preferred_temp = 18;
    xLcdData.current_screen = 0x00;

    /* Initialize the queue */
    xStructQueue = xQueueCreate(
        /* The maximum number of items the queue can hold*/
        5,
        /* The size of each struct, which the queue should be able to hold */
        sizeof( xLcdData )
    );

    /* Check to determine whether the queue has been initialized successfully */
    if(xStructQueue == NULL)
    {
        ESP_LOGE(TAG, "Queue has not been initialized successfully");
    }
}

/* A function which draws and maintains the temperature screen */
void lcd_screen_task(void *pvParameters)
{
    /* Tasks should never return, to prevent starvation, event-based tasks are recommended */
    for(; ;)
    {
        /* Initialize the received structure struct */
        struct LcdData xReceivedStructure;

        /* Check whether there's any new events in the queue */
        BaseType_t result;
        result = xQueueReceive(xStructQueue, &xReceivedStructure, ( TickType_t ) 10);

        if(result == pdPASS)
        {
            /* Draw the correct screen, based on which screen is marked as the current screen */
            switch(xReceivedStructure.current_screen)
            {
                case SCREEN_TEMP:
                    draw_temperature_screen(xReceivedStructure);
                    break;
                case SCREEN_HUM:
                    draw_humidity_screen(xReceivedStructure);
                    break;
            }
        }
    }

    /* If the task does happen to exist, ensure that the exit is a clean one */
    vTaskDelete(NULL);
}

/* A function which draws and maintains the temperature screen */
void draw_temperature_screen(LcdData given_data) 
{
    /* Convert the global int variables to strings */
    char snum_current_temp[2];
    sprintf(snum_current_temp, "%d", given_data.current_temp);

    char snum_pref_temp[2];
    sprintf(snum_pref_temp, "%d", given_data.preferred_temp);

    /* Display the temperature screen on the lcd  */
    ESP_LOGI(TAG, "Displaying temperature screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current   temp:   ");
    i2c_lcd1602_write_string    (lcd_info, snum_current_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 1);
    i2c_lcd1602_write_string    (lcd_info, "Preferred temp:   ");
    i2c_lcd1602_write_string    (lcd_info, snum_pref_temp);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Hum  Play> Upda");

    i2c_lcd1602_move_cursor     (lcd_info, 0, 3);
    i2c_lcd1602_write_string    (lcd_info, "Vol+/-> Pref +/-");
}

/* A function which draws and maintains the humidity screen */
void draw_humidity_screen(LcdData given_data) 
{
    /* Convert the global int variables to strings */
    char snum_current_hum[2];
    sprintf(snum_current_hum, "%d", given_data.current_humidity);

    /* Display the humidity screen on the lcd  */
    ESP_LOGI(TAG, "Displaying humidity screen");
    i2c_lcd1602_clear           (lcd_info);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 0);
    i2c_lcd1602_write_string    (lcd_info, "Current Humidity: ");
    i2c_lcd1602_write_string    (lcd_info, snum_current_hum);

    i2c_lcd1602_move_cursor     (lcd_info, 0, 2);
    i2c_lcd1602_write_string    (lcd_info, "Set> Temp Play> Upda");
}

/* A function which updates the DHT data */
void update_dht_data()
{
    /* Update the struct, based on the DHT values */
    xLcdData.current_temp = DHT11_read().temperature;
    xLcdData.current_humidity = DHT11_read().humidity;

    /* Log the results in the console */
    printf("Temperature is %d \n", xLcdData.current_temp);
    printf("Humidity is %d\n", xLcdData.current_humidity);
    ESP_LOGI(TAG, "Data has been updated");
}

/* A function which handles the execution when the rotary encoder has been clicked */
void rotary_encoder_on_clicked()
{
    ESP_LOGI(TAG, "Rotary encoder has been clicked!");

    /* Update the struct which holds the data */
    update_dht_data();

    /* Send the entire struct to the queue */
    xQueueSend(
        /* The handle of the queue */
        xStructQueue,
        /* The adress of the struct which should be sent */
        (void *) &xLcdData,
        /* Block time of 0 says don't block if the queue is already full.
        Check the value returned by xQueueSend() to know if the message
        was sent to the queue successfully. */
        ( TickType_t ) 0
    );

    vTaskDelay(1000 / portTICK_RATE_MS);
}

/* A function which handles the execution when the rotary encoder has been moved */
void rotary_encoder_on_moved(int16_t idx)
{ 
    /* Switch between screens based on the direction the rotary encoder has been rotated in */
    if(idx == 1)
    {
        /* Rotary encoder has been moved clockwise */
        xLcdData.current_screen = 0x01;
    } 
    else if (idx == -1)
    {
        /* Rotary encoder has been moved counter-clockwise*/
        xLcdData.current_screen = 0x00;
    }

    /* Send the entire struct to the queue */
    xQueueSend(
        /* The handle of the queue */
        xStructQueue,
        /* The adress of the struct which should be sent */
        (void *) &xLcdData,
        /* Block time of 0 says don't block if the queue is already full.
        Check the value returned by xQueueSend() to know if the message
        was sent to the queue successfully. */
        ( TickType_t ) 0
    );

    vTaskDelay(1000 / portTICK_RATE_MS);
}

/* A function which handles the on-board key inputs */
static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    const int input_type = evt->type;

    if(input_type == 1)
    {
        /* Handle the pressed button accordingly by checking what button had been clicked */
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_SET:
                /* Navigate to the currently non-active screen */
                if(xLcdData.current_screen == 0x00)
                {
                    xLcdData.current_screen = 0x01;
                }
                else
                {
                    xLcdData.current_screen = 0x00;
                }
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                /* Only reduce the preffered temp if the user is in the temperature screen, and the preffered temperature is above 0 */
                if(xLcdData.current_screen == 0x00)
                {
                    if(xLcdData.preferred_temp > 0)
                    {
                        xLcdData.preferred_temp -= 1;
                        ESP_LOGI(TAG, "Preferred temperature has been decreased");
                    }
                } 
                break;
            case INPUT_KEY_USER_ID_PLAY:
                /* Update the sensor's data */
                update_dht_data();
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                /* Only increase the preferred temp if the user is in the temperature screen, and the preferred temperature is below 30 */
                if(xLcdData.current_screen == 0x00)
                {
                    if(xLcdData.preferred_temp < 30)
                    {
                        xLcdData.preferred_temp += 1;
                        ESP_LOGI(TAG, "Preferred temperature has been increased");
                    }
                }
                break;
            default:
                break;
        }
        /* Add the modified lcd data to the queue */
        xQueueSend(xStructQueue, (void *) &xLcdData, ( TickType_t ) 0);

        /* Add a delay, to prevent the buttons from being spammable */
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    return ESP_OK;
}