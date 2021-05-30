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
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"

static const char *TAG = "example";

#define GPIO_OUTPUT_IO_0    6
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

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
    
    // Press+release the wakeup button/pin
    printf("Begin BM83 wakeup\n");
    util_bm83_wake();
    // util_bm83_sleep();
    vTaskDelay(1000 / portTICK_PERIOD_MS);   

    // RISE MFB for command
    // util_bm83_wake();

    /*
    Commands in decimal to send (including checksums etc)
    [170, 0, 3, 2, 0, 83, 168]
    [170, 0, 3, 2, 0, 84, 167]
    [170, 0, 3, 2, 0, 81, 170]
    [170, 0, 3, 2, 0, 82, 169]
    */
    
    // Press power button
    char PON_DATA[7] = {170,0,3,2,0,83,168};
    printf("CMD1\n");
    uart_write_bytes(UART_NUM_1, &PON_DATA, 7);
    vTaskDelay(1000 / portTICK_PERIOD_MS);    

    char POFF_DATA[7] = {170,0,3,2,0,84,167};
    printf("CMD2\n");
    uart_write_bytes(UART_NUM_1, &POFF_DATA, 7);
    
    vTaskDelay(3000 / portTICK_PERIOD_MS);   
    
    // Press pair button
    char P2ON_DATA[7] = {170,0,3,2,0,81,170};
    printf("CMD3\n");
    uart_write_bytes(UART_NUM_1, &P2ON_DATA, 7);
    vTaskDelay(1000 / portTICK_PERIOD_MS);    
    char P2OFF_DATA[7] = {170,0,3,2,0,82,169};
    printf("CMD4\n");
    uart_write_bytes(UART_NUM_1, &P2OFF_DATA, 7);
    
    // Release MFB
    printf("Done\n");
    util_bm83_sleep();

    // Setup the BM83 complete
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


static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes.", rxBytes);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
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

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&task_bm83_wakeup, "task_bm83_wakeup",  2048, NULL, 6, NULL);

    
    // vTaskDelay(30000 / portTICK_PERIOD_MS);    
    // // [170, 0, 3, 2, 0, 52, 199]
    // // Press pair button
    // char SKIP_SONG_DATA[7] = {170,0,3,2,0,52,199};
    // printf("Skipping song...\n");
    // uart_write_bytes(UART_NUM_1, &SKIP_SONG_DATA, 7);

    // vTaskDelay(3000 / portTICK_PERIOD_MS);    
    // // [170, 0, 3, 2, 0, 52, 199]
    // printf("Skipping song...\n");
    // uart_write_bytes(UART_NUM_1, &SKIP_SONG_DATA, 7);

    // vTaskDelay(3000 / portTICK_PERIOD_MS);    
    // // [170, 0, 3, 2, 0, 52, 199]
    // printf("Skipping song...\n");
    // uart_write_bytes(UART_NUM_1, &SKIP_SONG_DATA, 7);

    // vTaskDelay(3000 / portTICK_PERIOD_MS);    
    // printf("Toggle play pause...\n");
    // char PLAYPAUSE_SONG_DATA[7] = {170, 0, 3, 4, 0, 7, 242};
    // uart_write_bytes(UART_NUM_1, &PLAYPAUSE_SONG_DATA, 7);
}