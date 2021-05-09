/* USB Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device printing output from
// the application.

#include <stdio.h>
#include <stdlib.h>
#include <sys/reent.h>
#include "esp_log.h"
#include "esp_vfs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cmd_system.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"

#define BAT_IC_ADDR 0x36

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

gpio_num_t i2c_gpio_sda = 8;
gpio_num_t i2c_gpio_scl = 9;
uint32_t i2c_frequency = 100000;
i2c_port_t i2c_port = I2C_NUM_0;

static const char *TAG = "example";

#define GPIO_OUTPUT_IO_0    6
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    return i2c_param_config(i2c_port, &conf);
}

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BAT_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BAT_IC_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void task_max17048_read_vcell(void *ignore)
{
	uint8_t vcell_h;
    uint8_t vcell_l;

    uint8_t soc_h;
    uint8_t soc_l;

    uint8_t crate_h;
    uint8_t crate_l;


	esp_err_t espErr;

    while(true)
    {

        vTaskDelay(800/portTICK_PERIOD_MS);

        espErr = i2c_master_sensor_test(i2c_port,0x02, &vcell_h,&vcell_l);
        espErr = i2c_master_sensor_test(i2c_port,0x4, &soc_h,&soc_l);
        espErr = i2c_master_sensor_test(i2c_port,0x16, &crate_h,&crate_l);
        
        float lux = (float)(vcell_h<<8 | vcell_l) * (float)0.078125;
        printf("%.6f V\n", lux);
        
        float chrg = (float)(soc_h<<8 | soc_l) * (float)0.00390625;
        printf("%.6f CHRG\n", chrg);

        float crate = (float)(crate_h<<8 | crate_l) * (float)0.208;
        printf("%.6f per hr\n", crate);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

	vTaskDelete(NULL);
}

void util_bm83_wake()
{
    // Set MFB to 1, wait 3 seconds, then zero.
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

void util_bm83_sleep()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
}

void task_bm83_wakeup(void *ignore)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);    
    
    util_bm83_wake();
    // vTaskDelay(3000/portTICK_PERIOD_MS);
    // util_bm83_sleep();

    
    util_bm83_wake();
    char PON_DATA[5] = {0xAA,0x00,0x03,0x02,0x00,0x51,0xAA};
    uart_write_bytes(UART_NUM_1, &PON_DATA, 7);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    char POFF_DATA[5] = {0xAA,0x00,0x03,0x02,0x00,0x52,0xA9};
    uart_write_bytes(UART_NUM_1, &POFF_DATA, 7);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Enter pairing mode
    char PAIR_DATA[5] = {0xAA,0x00,0x03,0x02,0x00,0x5D,0x9E};
    uart_write_bytes(UART_NUM_1, &PAIR_DATA, 7);

    // Setup the BM83
    vTaskDelete(NULL);
}

int sendData(const char* logName, const char* data)
{
    util_bm83_wake();
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    util_bm83_sleep();
    return txBytes;
}

void sendACK(const char* logName)
{
    // util_bm83_wake();
    // char* ACK_DATA[6] = {0xAA,0x00,2,0x14,0x33,0xB7}; //CHK = 1 + ~(0x00 + 0x02+ 0x14 +0x033)
    char FW_Q_DATA[5] = {0xAA,0x00,1,0x08,0xF7};
    const int txBytes = uart_write_bytes(UART_NUM_1, &FW_Q_DATA, 5);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    // util_bm83_sleep();
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        // sendData(TX_TASK_TAG, "Hello world");
        sendACK(TX_TASK_TAG);
        // sendData(TX_TASK_TAG, ACK_DATA, 6);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void app_main(void)
{
    /* Setting TinyUSB up */
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));

    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
    ESP_LOGI(TAG, "USB initialization DONE");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "UART Init");
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    // Attempt to ask for battery chip data
    // xTaskCreate(&task_max17048_read_vcell, "task_max17048_read_vcell",  2048, NULL, 6, NULL);

    xTaskCreate(&task_bm83_wakeup, "task_bm83_wakeup",  2048, NULL, 6, NULL);
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // i2c_driver_delete(i2c_port);

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}