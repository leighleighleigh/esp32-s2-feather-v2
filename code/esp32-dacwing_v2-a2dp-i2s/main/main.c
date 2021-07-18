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
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cmd_system.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"

#include "sdkconfig.h"
#include "math.h"
#include "driver/i2s.h"


// DAC STUFF
#define DAC_IC_ADDR 0x1A
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
gpio_num_t i2c_gpio_sda = 21;
gpio_num_t i2c_gpio_scl = 22;
uint32_t i2c_frequency = 100000;
i2c_port_t i2c_port = I2C_NUM_0;

// I2S STUFF
#define SAMPLE_RATE     (96000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI              (3.14159265)

#define I2S_BCK_IO      (GPIO_NUM_26)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_27)
#define I2S_DI_IO       (-1)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)



static const char *TAG = "example";


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

void task_dac_start(void *ignore)
{
	esp_err_t espErr;

    // Setup the PWRDN registers
    uint8_t pwrReg = 0x6;
    uint8_t activeReg = 0x9;
    uint8_t clockReg = 0x8;
    uint8_t dacReg = 0x5;
    uint8_t interfaceReg = 0x7;

    // SETUP CORE CLOCK
    ESP_LOGI(TAG,"RESET!");
    // Set ACTIVE registers
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, 0xF<<1, ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); 
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);

    vTaskDelay(1000 / portTICK_PERIOD_MS);   


    // SETUP CORE CLOCK
    ESP_LOGI(TAG,"SETTING SAMPLING CONTROL!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address

    uint8_t firstByte = (clockReg<<1);
    // uint8_t secondByte = 0b00000000;
    uint8_t secondByte = 0b10011100;

    i2c_master_write_byte(cmd, firstByte, ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, secondByte, ACK_CHECK_EN); // Write data (8 bits)
    
    i2c_master_stop(cmd); 
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);
    
    ESP_LOGI(TAG,"SETTING INTERFACE FORMAT!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address

    i2c_master_write_byte(cmd, (interfaceReg<<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00001010, ACK_CHECK_EN); // Write data (8 bits)
    
    i2c_master_stop(cmd); 
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);

    ESP_LOGI(TAG,"SENDING POWER ON!");
    // Set PWR registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    
    i2c_master_write_byte(cmd, (pwrReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b01100111, ACK_CHECK_EN); // Write data (8 bits)

    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);
    
    ESP_LOGI(TAG,"SENDING ACTIVE ON!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (activeReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 1, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);
    
    ESP_LOGI(TAG,"UNMUTING!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (dacReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);

	vTaskDelete(NULL);
}


void task_i2cscanner(void *ignore) {
	ESP_LOGD(TAG, ">> i2cScanner");

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}

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
    vTaskDelay(3000 / portTICK_PERIOD_MS);

	ESP_LOGI(TAG, "USB initialization PRINT");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    
    ESP_LOGI(TAG, "MCLK START");
    // Setup the MCLK pin on IO0
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    REG_WRITE(PIN_CTRL, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);   


    ESP_LOGI(TAG, "I2C SCAN START");
    xTaskCreate(&task_i2cscanner, "task_i2cscanner",  2048, NULL, 6, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);   

    // IO18 / U1RXD
    // WRITE_REG(IO_MUX_PIN_CTRL, 0x0);
    // REG_WRITE(PIN_CTRL,0xFFFFF0F0);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    // Direct the clock out
    // PIN_CTRL[3:0] = 0x0;
    // PIN_CTRL[11:8] = 0x0;

    ESP_LOGI(TAG, "DAC START");
    xTaskCreate(&task_dac_start, "task_dac_start", 2048, NULL, 7, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);   


    ESP_LOGI(TAG, "I2S START");
    vTaskDelay(1000 / portTICK_PERIOD_MS);   


    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 24,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    int test_bits = 24;
    while (1) {
        setup_triangle_sine_waves(test_bits);
        vTaskDelay(5000/portTICK_RATE_MS);
    }
}