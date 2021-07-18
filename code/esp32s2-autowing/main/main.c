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

#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "sdkconfig.h"

#define HAS_BAT_IC 1

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
#define PIN_NUM_MISO 35
#define PIN_NUM_MOSI 37
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   12

// Commands are 4 bits. Address is 12 bits. Data is in bytes (0-N).
#define MCP_CMD_RESET 0x0 // - Address argument 0
#define MCP_CMD_READ 0x3 // + A. Read SFR/RAM FROM address A.
#define MCP_CMD_WRITE 0x2 // + A. Write SFR/RAM to address A.

static const char *TAG = "example";


#if HAS_BAT_IC
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
#endif

uint32_t mcp2518_cmd(spi_device_handle_t spi, const uint8_t cmd, const uint16_t addr, const uint8_t txLen, const uint8_t rxLen)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    
    // Setup flags for return value!
    if(rxLen != 0)
    {
        t.flags |= SPI_TRANS_USE_RXDATA;
    }
    
    if(txLen != 0)
    {
        t.flags |= SPI_TRANS_USE_TXDATA;
    }

    // FORM THE ADDR AND CMD STUFF!
    // I couldnt get variable bit lengths to work (4 for command, 12 for address)
    // So I just made them manually out of two 8-bit parts (cmd + addr).
    uint8_t b0 = ((cmd<<4) & 0b11110000); // Command is first 4 bytes
    b0 = b0 | ((addr >> 8) & 0b00001111);
    uint8_t b1 = addr & 0b11111111;

    t.cmd = b0;
    t.addr = b1;       
    
    t.length=txLen; // INCLUDES cmd + addr? INCLUDES data bytes.
    t.rxlength=rxLen;

    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    // Return any result (use if expected)
    return *(uint32_t*)t.rx_data;
}

uint32_t mcp2518_cmd_data(spi_device_handle_t spi, const uint8_t cmd, const uint16_t addr, const uint8_t txLen, const uint8_t rxLen, const uint8_t b3, const uint8_t b2, const uint8_t b1, const uint8_t b0)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    
    // Setup flags for return value!
    if(rxLen != 0)
    {
        t.flags |= SPI_TRANS_USE_RXDATA;
    }
    
    if(txLen != 0)
    {
        t.flags |= SPI_TRANS_USE_TXDATA;
        t.tx_data[0] = b0;
        t.tx_data[1] = b1;
        t.tx_data[2] = b2;
        t.tx_data[3] = b3;
    }

    // FORM THE ADDR AND CMD STUFF!
    // I couldnt get variable bit lengths to work (4 for command, 12 for address)
    // So I just made them manually out of two 8-bit parts (cmd + addr).
    uint8_t c0 = ((cmd<<4) & 0b11110000); // Command is first 4 bytes
    c0 = c0 | ((addr >> 8) & 0b00001111);
    uint8_t a1 = addr & 0b11111111;

    t.cmd = c0;
    t.addr = a1;       
    
    t.length=txLen; // INCLUDES cmd + addr? INCLUDES data bytes.
    t.rxlength=rxLen;

    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    // Return any result (use if expected)
    return *(uint32_t*)t.rx_data;
}

void mcp2518_reset(spi_device_handle_t spi)
{
    printf("Resetting CAN IC...\n");
    mcp2518_cmd(spi,MCP_CMD_RESET,0x0,0,0);
}

void get_mcp2518_id(spi_device_handle_t spi)
{
    printf("Asking MCP2518 for its ID...\n");
    // 1. SEND COMMAND FOR ID
    uint32_t result = mcp2518_cmd(spi, MCP_CMD_READ, 0xE14,32,32);
    printf("GOT: 0x%X\n",(uint32_t)result);
}

void mcp2518_test_ram(spi_device_handle_t spi)
{
    printf("Writing 0xFF to RAM @ 0x400\n");
    // 1. WRITE TO RAM
    uint32_t result1 = mcp2518_cmd_data(spi,MCP_CMD_WRITE,0x400,32,0,0XFA,0XAA,0XBB,0XCC);
    // 2. READ FROM RAM
    uint32_t result = mcp2518_cmd(spi, MCP_CMD_READ, 0x400,32,32);
    printf("GOT: 0x%X\n",(uint32_t)result);
}

void mcp2518_set_bitrate_regs(spi_device_handle_t spi)
{
    printf("Requesting CONFIGURATION mode.\n");
    // 1. Set Can Control register
    // Firsly read the existing register.
    uint32_t C1CON_READ = mcp2518_cmd(spi, MCP_CMD_READ, 0x0,32,32);
    printf("GOT: 0x%X\n",(uint32_t)C1CON_READ);

    uint8_t REQOP = 0b100; // See Page 27 for REQOP values.
    uint8_t BRSDIS = 0b1; // Disable bit rate switching (to faster ones).
    uint8_t WAKFIL = 0b1; // Use physical CAN filter for wakeup of IC.

    // Apply over C1CON_READ
    // First, split into four parts
    uint8_t C1CON_3 = (C1CON_READ & 0xFF000000) >> 24;
    uint8_t C1CON_2 = (C1CON_READ & 0x00FF0000) >> 16;
    uint8_t C1CON_1 = (C1CON_READ & 0x0000FF00) >> 8;
    uint8_t C1CON_0 = (C1CON_READ & 0x000000FF);

    printf("C1CON_3: 0x%X\n",(uint8_t)C1CON_3);
    printf("C1CON_2: 0x%X\n",(uint8_t)C1CON_2);
    printf("C1CON_1: 0x%X\n",(uint8_t)C1CON_1);
    printf("C1CON_0: 0x%X\n",(uint8_t)C1CON_0);

    // Clear REQOP, and apply.
    C1CON_3 &= 0b11111000;
    C1CON_3 |= REQOP;
    // Clear BRSDIS and WAKFIL
    C1CON_1 &= 0b11101110;
    C1CON_1 |= BRSDIS << 4;
    C1CON_1 |= WAKFIL;

    // Build new C1CON
    uint32_t C1CON = (C1CON_3 << 24) | (C1CON_2 << 16) | (C1CON_1 << 8) | (C1CON_0);
    printf("C1CON: 0x%X\n",(uint32_t)C1CON);
    uint32_t C1CON_WRITE = mcp2518_cmd_data(spi,MCP_CMD_WRITE,0x0,32,0,C1CON_3,C1CON_2,C1CON_1,C1CON_0);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    uint32_t C1NBTCFG_READ = mcp2518_cmd(spi,MCP_CMD_READ,0x4,32,32);
    printf("C1NBTCFG_READ: 0x%X\n",(uint32_t)C1NBTCFG_READ);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    printf("Settings bitrate regs\n");
    // 2. Set C1NBTCFG (Nominal Bit Time Config)
    uint8_t BRP = 4;
    uint8_t TSEG1 = 240;
    uint8_t TSEG2 = 60;
    uint8_t SJW = 60;
    uint32_t C1NBTCFG_WRITE = mcp2518_cmd_data(spi,MCP_CMD_WRITE,0x4,32,0,BRP,TSEG1,TSEG2,SJW);
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    C1NBTCFG_READ = mcp2518_cmd(spi,MCP_CMD_READ,0x4,32,32);
    printf("C1NBTCFG_READ: 0x%X\n",(uint32_t)C1NBTCFG_READ);
    
    // Request internal loopback mode
    REQOP = 0b101; // External loopback Mode. // See Page 27 for REQOP values.
    C1CON_3 &= 0b11111000;
    C1CON_3 |= REQOP;
    C1CON_WRITE = mcp2518_cmd_data(spi,MCP_CMD_WRITE,0x0,32,0,C1CON_3,C1CON_2,C1CON_1,C1CON_0);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Re-check C1CON
    C1CON_READ = mcp2518_cmd(spi, MCP_CMD_READ, 0x0,32,32);
    printf("GOT: 0x%X\n",(uint32_t)C1CON_READ);
    printf("OPMOD: 0x%X\n",(uint32_t)(C1CON_READ>>21)&0b111);
}


static spi_device_handle_t spi_master_driver_initialize(void)
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
        .command_bits = 8, // 4 bit cmd
        .address_bits = 8, // 12 bit address
        .clock_speed_hz=10000000,           //Clock out at 10 MHz (MCP2518 supports 20Mhz)
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    ret=spi_bus_add_device(SPI_BUS,&devcfg,&spi);
    ESP_ERROR_CHECK(ret);

    return spi;
}

#if HAS_BAT_IC
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
    while(1)
    {

        vTaskDelay(2000/portTICK_PERIOD_MS);

        espErr = i2c_master_sensor_read(i2c_port,0x02, &vcell_h,&vcell_l);
        espErr = i2c_master_sensor_read(i2c_port,0x4, &soc_h,&soc_l);
        espErr = i2c_master_sensor_read(i2c_port,0x16, &crate_h,&crate_l);
        
        float cellVoltage = (float)(vcell_h<<8 | vcell_l) * (float)0.078125;
        printf("%.4f V\n", cellVoltage/1000);
        
        float chargePercentage = (float)(soc_h<<8 | soc_l) * (float)0.00390625;
        printf("%.3f %%\n", chargePercentage);

        float crate = (float)(crate_h<<8 | crate_l) * (float)0.208;
        printf("%.3f %% per hr\n", crate);
            
    }
    vTaskDelete(NULL);
}
#endif

void app_main(void)
{   
    ESP_LOGI(TAG, "SPI initialization");
    spi_device_handle_t spi = spi_master_driver_initialize();
 
    #if HAS_BAT_IC
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
    #endif

    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Reset the IC
    mcp2518_reset(spi);

    // Poll the MCP2518
    get_mcp2518_id(spi);

    // Do RAM test
    mcp2518_test_ram(spi);

    // Set bitrate registers
    mcp2518_set_bitrate_regs(spi);

    // i2c_driver_delete(i2c_port);
}