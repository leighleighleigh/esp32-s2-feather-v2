// USB CDC Serial
// MCP2518 SPI library
// BATT_IC

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
// #include "cmd_i2ctools.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

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

#define SPI_BUS    SPI2_HOST
#define DMA_CHAN SPI_BUS
#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   12

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


static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((eeprom_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((eeprom_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((eeprom_context_t*)t->user)->cfg.cs_io, 0);
    
}

void get_mcp2518_id(spi_device_handle_t spi)
{
    printf("Asking MCP2518 for data...\n");
    
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.cmd=0x1 // READ cmd
    t.addr=0x1 // Address
    
    t.length=8;                     //Command is 8 bits
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    printf("OK?\n");
}

static esp_err_t spi_master_driver_initialize(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPI_BUS+1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI_BUS, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);

    // Setup the MCP2518 device
    spi_device_handle_t spi;
    
    spi_device_interface_config_t devcfg={
        .command_bits = 4, // 4 bit cmd, 12 bit address
        .clock_speed_hz=20*1000*1000,           //Clock out at 10 MHz (MCP2518 supports 20Mhz)
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    ret=spi_bus_add_device(SPI_BUS,&devcfg,&spi);
    ESP_ERROR_CHECK(ret);

    // Poll the MCP2518
    get_mcp2518_id(spi);
    return ret;
}

static esp_err_t i2c_master_sensor_read(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_h, uint8_t *data_l)
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

    // Can add loop here
    vTaskDelay(800/portTICK_PERIOD_MS);

    espErr = i2c_master_sensor_read(i2c_port,0x02, &vcell_h,&vcell_l);
    espErr = i2c_master_sensor_read(i2c_port,0x4, &soc_h,&soc_l);
    espErr = i2c_master_sensor_read(i2c_port,0x16, &crate_h,&crate_l);
    
    float lux = (float)(vcell_h<<8 | vcell_l) * (float)0.078125;
    printf("%.6f V\n", lux);
    
    float chrg = (float)(soc_h<<8 | soc_l) * (float)0.00390625;
    printf("%.6f CHRG\n", chrg);

    float crate = (float)(crate_h<<8 | crate_l) * (float)0.208;
    printf("%.6f per hr\n", crate);
        
	vTaskDelete(NULL);
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

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    // Attempt to ask for battery chip data
    xTaskCreate(&task_max17048_read_vcell, "task_max17048_read_vcell",  2048, NULL, 6, NULL);


    spi_master_driver_initialize();

    // i2c_driver_delete(i2c_port);
}