/* I2S Example

    This example code will output 100Hz sine wave and triangle wave to 2-channel of I2S driver
    Every 5 seconds, it will change bits_per_sample [16, 24, 32] for i2s data

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>

#include "esp_log.h"
#include "esp_vfs.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"
// #include "io_mux_reg.h"
#include "driver/gpio.h"


#define SAMPLE_RATE     (36000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI              (3.14159265)
#define I2S_BCK_IO      (GPIO_NUM_34)
#define I2S_WS_IO       (GPIO_NUM_33)
#define I2S_DO_IO       (GPIO_NUM_21)
#define I2S_DI_IO       (-1)
#define I2S_MUTE        (GPIO_NUM_7)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

static const char *TAG = "example";

static void setup_triangle_sine_waves(int bits)
{
    int *samples_data = malloc(((bits+8)/16)*SAMPLE_PER_CYCLE*4);
    unsigned int i, sample_val;
    double sin_float, triangle_float, triangle_step = (double) pow(2, bits) / SAMPLE_PER_CYCLE;
    size_t i2s_bytes_write = 0;

    printf("\r\nTest bits=%d free mem=%d, written data=%d\n", bits, esp_get_free_heap_size(), ((bits+8)/16)*SAMPLE_PER_CYCLE*4);

    triangle_float = -(pow(2, bits)/2 - 1);

    for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
        sin_float = sin(i * 2 * PI / SAMPLE_PER_CYCLE);
        if(sin_float >= 0)
            triangle_float += triangle_step;
        else
            triangle_float -= triangle_step;

        sin_float *= (pow(2, bits)/2 - 1);

        if (bits == 16) {
            sample_val = 0;
            sample_val += (short)triangle_float;
            sample_val = sample_val << 16;
            sample_val += (short) sin_float;
            samples_data[i] = sample_val;
        } else if (bits == 24) { //1-bytes unused
            samples_data[i*2] = ((int) triangle_float) << 8;
            samples_data[i*2 + 1] = ((int) sin_float) << 8;
        } else {
            samples_data[i*2] = ((int) triangle_float);
            samples_data[i*2 + 1] = ((int) sin_float);
        }

    }

    i2s_set_clk(I2S_NUM, SAMPLE_RATE, bits, 2);
    //Using push
    // for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
    //     if (bits == 16)
    //         i2s_push_sample(0, &samples_data[i], 100);
    //     else
    //         i2s_push_sample(0, &samples_data[i*2], 100);
    // }
    // or write
    i2s_write(I2S_NUM, samples_data, ((bits+8)/16)*SAMPLE_PER_CYCLE*4, &i2s_bytes_write, 100);

    free(samples_data);
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

    // setup mute pin
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<I2S_MUTE);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(I2S_MUTE,1);

    // Setup the MCLK pin
    // IO18 / U1RXD

    // WRITE_REG(IO_MUX_PIN_CTRL, 0x0);
    REG_WRITE(PIN_CTRL,0xFFFFF0F0);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_DAC_2_U, FUNC_DAC_2_CLK_OUT3);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

    // Direct the clock out
    // PIN_CTRL[3:0] = 0x0;
    // PIN_CTRL[11:8] = 0x0;

    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    int test_bits = 16;
    while (1) {
        setup_triangle_sine_waves(test_bits);
        vTaskDelay(5000/portTICK_RATE_MS);
        test_bits += 8;
        if(test_bits > 32)
            test_bits = 16;

    }

}
