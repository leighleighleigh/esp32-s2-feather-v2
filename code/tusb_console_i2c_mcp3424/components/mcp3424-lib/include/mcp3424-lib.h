#ifndef MCP3424_H_
#define MCP3424_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MCP3424_OK 0
#define MCP3424_ERR -1
#define MCP3424_WARN -2
#define MCP3424_ERR_LEN 256

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

enum mcp3424_channel {
	MCP3424_CHANNEL_1,
	MCP3424_CHANNEL_2,
	MCP3424_CHANNEL_3,
	MCP3424_CHANNEL_4
};

enum mcp3424_conversion_mode {
	MCP3424_CONVERSION_MODE_ONE_SHOT,
	MCP3424_CONVERSION_MODE_CONTINUOUS
};

enum mcp3424_pga {
	MCP3424_PGA_1X,
	MCP3424_PGA_2X,
	MCP3424_PGA_4X,
	MCP3424_PGA_8X
};

enum mcp3424_resolution {
	MCP3424_RESOLUTION_12,
	MCP3424_RESOLUTION_14,
	MCP3424_RESOLUTION_16,
	MCP3424_RESOLUTION_18
};

typedef struct {
	i2c_port_t i2c_port;
	uint8_t addr;
	uint8_t config;
	int err;
	char errstr[MCP3424_ERR_LEN];
} mcp3424;

void mcp3424_init(mcp3424 *m, i2c_port_t i2c_port, uint8_t addr, enum mcp3424_resolution res);

void mcp3424_set_conversion_mode(mcp3424 *m, enum mcp3424_conversion_mode mode);
void mcp3424_set_pga(mcp3424 *m, enum mcp3424_pga pga);
void mcp3424_set_resolution(mcp3424 *m, enum mcp3424_resolution res);

enum mcp3424_conversion_mode mcp3424_get_conversion_mode(mcp3424 *m);
enum mcp3424_pga mcp3424_get_pga(mcp3424 *m);
enum mcp3424_resolution mcp3424_get_resolution(mcp3424 *m);

unsigned int mcp3424_get_raw(mcp3424 *m, enum mcp3424_channel channel);

#endif /* MCP3424_H_ */