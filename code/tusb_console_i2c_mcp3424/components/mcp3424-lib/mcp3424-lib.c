#include <stdio.h>
#include "mcp3424-lib.h"

static void mcp3424_set_channel(mcp3424 *m, enum mcp3424_channel channel) {
	m->config &= ~0x60;
	m->config |= (channel << 5);
}

void mcp3424_init(mcp3424 *m, i2c_port_t i2c_port, uint8_t addr, enum mcp3424_resolution res) {
	m->i2c_port = i2c_port;
	m->addr = addr;
	m->config = 0x00;
	m->err = MCP3424_OK;
	mcp3424_set_channel(m, MCP3424_CHANNEL_1);
	mcp3424_set_conversion_mode(m, MCP3424_CONVERSION_MODE_ONE_SHOT);
	mcp3424_set_pga(m, MCP3424_PGA_1X);
	mcp3424_set_resolution(m, res);
}

void mcp3424_set_conversion_mode(mcp3424 *m, enum mcp3424_conversion_mode mode) {
	m->config &= ~0x10;
	m->config |= (mode << 4);
}

void mcp3424_set_pga(mcp3424 *m, enum mcp3424_pga pga) {
	m->config &= ~0x03;
	m->config |= pga;
}

void mcp3424_set_resolution(mcp3424 *m, enum mcp3424_resolution res) {
	m->config &= ~0x0c;
	m->config |= (res << 2);
}

enum mcp3424_conversion_mode mcp3424_get_conversion_mode(mcp3424 *m) {
	return (m->config >> 4) & 0x03;
}

enum mcp3424_pga mcp3424_get_pga(mcp3424 *m) {
	return m->config & 0x03;
}

enum mcp3424_resolution mcp3424_get_resolution(mcp3424 *m) {
	return (m->config >> 2) & 0x03;
}

unsigned int mcp3424_get_raw(mcp3424 *m, enum mcp3424_channel channel) {
	int rv;
	ssize_t n;
	uint8_t reading[4];
	unsigned int raw;

    mcp3424_set_channel(m, channel);

	// if one shot, write ready bit to start new conversion on mcp3424
	if (mcp3424_get_conversion_mode(m) == MCP3424_CONVERSION_MODE_ONE_SHOT) {
		m->config |= (1 << 7);
	}

    // Set config
    // Make general call
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m->addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,&m->config,ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(m->i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret != ESP_OK){
        printf("Failed to set CONFIG\n");
        m->err = MCP3424_ERR;
    }

	// n = write(m->i2c_port, &m->config, 1);

	// if (n < 1) {
	// 	if (n == 0) {
	// 		m->err = MCP3424_WARN;
	// 	} else if (n == -1) {
	// 		m->err = MCP3424_ERR;
	// 	}
	// 	return 0;
	// }

	if (mcp3424_get_conversion_mode(m) == MCP3424_CONVERSION_MODE_ONE_SHOT) {
		m->config &= ~(1 << 7);
	}

	while (1) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, m->addr << 1 | READ_BIT, ACK_CHECK_EN);
        
        // Get 4 bytes!
		i2c_master_read(cmd, reading, 4 - 1, ACK_VAL);
    	i2c_master_read_byte(cmd, reading + 4 - 1, NACK_VAL);
        
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(m->i2c_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        
        if(ret != ESP_OK)
        {
            printf("Failed to get reading.\n");
        }
        
		// n = read(m->i2c_port, reading, 4);
        
        // // ADD 4 BYTE READ CODE HERE

		// if (n < 4) {
		// 	if (n >= 0) {
		// 		m->err = MCP3424_WARN;
		// 	} else if (n == -1) {
		// 		m->err = MCP3424_ERR;
		// 	}
		// 	return 0;
		// }

		// loop until ready bit is 0 (new reading)
		if (mcp3424_get_resolution(m) == MCP3424_RESOLUTION_18) {
			if ((reading[3] >> 7) == 0) {
				break;
			}
		} else {
			if ((reading[2] >> 7) == 0) {
				break;
			}
		}
	}

	printf("READING: %d,%d,%d,%d\n",reading[3],reading[2],reading[1],reading[0]);

	switch (mcp3424_get_resolution(m)) {
		case MCP3424_RESOLUTION_12:
			raw = ((reading[0] & 0x0f) << 8) | reading[1];
			break;
		case MCP3424_RESOLUTION_14:
			raw = ((reading[0] & 0x3f) << 8) | reading[1];
			break;
		case MCP3424_RESOLUTION_16:
			raw = (reading[0] << 8) | reading[1];
			break;
		case MCP3424_RESOLUTION_18:
			raw = ((reading[0] & 0x03) << 16) | (reading[1] << 8) | reading[2];
			break;
		default:
			m->err = MCP3424_ERR;
			return 0;
	}

	return raw;
}