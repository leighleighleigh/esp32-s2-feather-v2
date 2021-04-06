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

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"

// LEDC/PWM driver
#include "driver/ledc.h"

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO (12)
#define LEDC_LS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO (14)
#define LEDC_LS_CH1_CHANNEL LEDC_CHANNEL_1

#define LEDC_TEST_CH_NUM (2) // There are 16 channels, two groups of 8. One groups is High Speed.
#define LEDC_TEST_DUTY1 (7000) // 0.5V?
#define LEDC_TEST_DUTY2 (7100) // 1V?
#define LEDC_TEST_DUTY3 (7200) // 2V?
#define LEDC_TEST_DUTY4 (7300) // 3V?
#define LEDC_TEST_FADE_TIME (10)

static const char *TAG = "example";

static uint8_t lbuf[1024 + 1];
uint8_t lbufIndex = 0;
static uint8_t buf[1024 + 1];

uint16_t pwmDuty = 0;
uint16_t prevPwmDuty = 0;


void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_USB_CDC_RX_BUFSIZE, &rx_size);

    if (ret == ESP_OK) {
        // Append to long buffer
        for(int i = 0; i<rx_size; i++)
        {
            lbuf[lbufIndex] = buf[i];
            lbufIndex++;
        }
        
        // buf[rx_size] = '\0';
        if(buf[rx_size-1] == '\n' || buf[rx_size-1] == '\r' || buf[rx_size] == '\n' || buf[rx_size] == '\n')
        {
            lbuf[lbufIndex] = '\0';
            // Convert to value
            // uint16_t val = atoi((char*)lbuf);
            char *eptr;
            uint16_t val = strtoul((char*)lbuf, &eptr, 10);

            if(val > 1000 && val < 16000){
                printf("\nGot value: %s\n", lbuf);
                pwmDuty = val;
            }else{
                printf("\nGot data (%d bytes): %s\n", lbufIndex, lbuf);
            }
            lbufIndex = 0;
        }
        
    } else {
        printf("Read error");
    }

    /* write back */
    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    tinyusb_cdcacm_write_flush(itf, 0);
}

void app_main(void)
{
    /* Setting TinyUSB up */
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t tusb_cfg = {0}; // the configuration uses default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = {
    .usb_dev = TINYUSB_USBDEV_0,
    .cdc_port = TINYUSB_CDC_ACM_0,
    .rx_unread_buf_sz = 64,
    .callback_rx = &tinyusb_cdc_rx_callback,
    .callback_rx_wanted_char = NULL,
    .callback_line_state_changed = NULL,
    .callback_line_coding_changed = NULL
};
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));

    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
    ESP_LOGI(TAG, "USB initialization DONE");

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Setup the PWM stuff
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_14_BIT, // resolution of PWM duty 
        .freq_hz = 1000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {.channel = LEDC_LS_CH0_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH0_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
        {.channel = LEDC_LS_CH1_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH1_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    // uint8_t *data = (uint8_t *) malloc(1024);

    // while(1){
    //     printf("LEDC duty = %d\n", LEDC_TEST_DUTY1);
    //     for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
    //     {
    //         ledc_set_fade_with_time(ledc_channel[ch].speed_mode,ledc_channel[ch].channel, 7000, LEDC_TEST_FADE_TIME);
    //         ledc_fade_start(ledc_channel[ch].speed_mode,ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
    //     }
    //     vTaskDelay(5000/portTICK_PERIOD_MS);
    // }


    while(1){
    if(pwmDuty != prevPwmDuty)
    {

        printf("\nLEDC duty = %d\n", pwmDuty);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
        {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,ledc_channel[ch].channel, pwmDuty, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }   
        
        prevPwmDuty = pwmDuty;

    }
    }

}