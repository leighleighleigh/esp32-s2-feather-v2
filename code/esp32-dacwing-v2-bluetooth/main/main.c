// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "soc/io_mux_reg.h"

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

#define SAMPLE_RATE 44100
#define SAMPLE_DEPTH 16

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);


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
    uint8_t sampleCtlReg = 0x8;
    uint8_t aapReg = 0x4;
    uint8_t dapReg = 0x5;
    uint8_t interfaceReg = 0x7;
    uint8_t lhpReg = 0x2;

    // SETUP CORE CLOCK
    ESP_LOGI(BT_AV_TAG,"RESET!");
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


    ESP_LOGI(BT_AV_TAG,"SENDING POWER ON!");
    // Set PWR registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (pwrReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00010000, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);



    // SETUP CORE CLOCK
    ESP_LOGI(BT_AV_TAG,"SETTING SAMPLING CONTROL!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    uint8_t firstByte = (sampleCtlReg<<1);
    uint8_t secondByte = 0b00100000;
    i2c_master_write_byte(cmd, firstByte, ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, secondByte, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); 
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);


    
    ESP_LOGI(BT_AV_TAG,"SETTING INTERFACE FORMAT!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (interfaceReg<<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00000010, ACK_CHECK_EN); // Write data (8 bits)   // 16bit I2C
    // TODO: TRY 01 instead of 10.
    i2c_master_stop(cmd); 
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);


    ESP_LOGI(BT_AV_TAG,"SET LHP VOL!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (lhpReg <<1) | 0b1, ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b10110000, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);


    ESP_LOGI(BT_AV_TAG,"SETUP DIGITAL AUDIO PATH!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (dapReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00000100, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);


    ESP_LOGI(BT_AV_TAG,"SETUP ANALOG AUDIO PATH!");
    // Set ACTIVE registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    i2c_master_write_byte(cmd, (aapReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00010010, ACK_CHECK_EN); // Write data (8 bits)
    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);


    ESP_LOGI(BT_AV_TAG,"SENDING ACTIVE ON!");
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
    

    ESP_LOGI(BT_AV_TAG,"SENDING POWER ON OUTPD!");
    // Set PWR registers
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // Start i2c
    i2c_master_write_byte(cmd, DAC_IC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // Write address
    
    i2c_master_write_byte(cmd, (pwrReg <<1), ACK_CHECK_EN); // Write register + B8
    i2c_master_write_byte(cmd, 0b00000000, ACK_CHECK_EN); // Write data (8 bits)

    i2c_master_stop(cmd); // End
    espErr = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS); // Perform operation with timeout
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(espErr);

	vTaskDelete(NULL);
}


void task_i2cscanner(void *ignore) {
	ESP_LOGD(BT_AV_TAG, ">> i2cScanner");

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

void app_main(void)
{
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    

    ESP_LOGI(BT_AV_TAG, "I2C SCAN START");
    xTaskCreate(&task_i2cscanner, "task_i2cscanner",  2048, NULL, 6, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);   

    ESP_LOGI(BT_AV_TAG, "DAC START");
    xTaskCreate(&task_dac_start, "task_dac_start", 2048, NULL, 7, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);   

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);


    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_DEPTH,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = true,
        .tx_desc_auto_clear = true
    };


    i2s_driver_install(0, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .bck_io_num = CONFIG_EXAMPLE_I2S_BCK_PIN,
        .ws_io_num = CONFIG_EXAMPLE_I2S_LRCK_PIN,
        .data_out_num = CONFIG_EXAMPLE_I2S_DATA_PIN,
        .data_in_num = -1                                                       //Not used
    };

    i2s_set_pin(0, &pin_config);
    i2s_set_clk(0, SAMPLE_RATE, SAMPLE_DEPTH, I2S_CHANNEL_STEREO);


    ESP_LOGI(BT_AV_TAG, "MCLK START");
    
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    // WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL) & 0xFFFFFFF0);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
    REG_WRITE(PIN_CTRL, 0xF<<8);
    int test = REG_READ(PIN_CTRL);
    printf("%#032x\n",test);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);   


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

    /*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);



}

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
    return;
}
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = "ESP_SPEAKER";
        esp_bt_dev_set_device_name(dev_name);

        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        /* initialize AVRCP target */
        assert (esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        /* initialize A2DP sink */
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_sink_init();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}
