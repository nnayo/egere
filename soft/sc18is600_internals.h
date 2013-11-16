//---------------------
//  Copyright (C) 2000-2009  <Yann GOUY>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; see the file COPYING.  If not, write to
//  the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
//  Boston, MA 02111-1307, USA.
//
//  you can write to me at <yann_gouy@yahoo.fr>
//

// SC18IS600 chip register description
//
// this chip is from NXP
//
// using the SPI interface, the protocol uses a 32-bit stream
// first byte is an opcode (read or write)
// two next bytes are the address (MSB first)
// last byte is the data field (read or written)
//

#ifndef __SC18IS600_INTERNALS_H__
# define __SC18IS600_INTERNALS_H__

# include "type_def.h"

//------------------------------------------
// private defines
//


//------------------------------------------
// private types
//

// register mapping
typedef struct io_config_t {
	uint8_t io3_1:1;
	uint8_t io3_0:1;
	uint8_t io2_1:1;
	uint8_t io2_0:1;
	uint8_t io1_1:1;
	uint8_t io1_0:1;
	uint8_t io0_1:1;
	uint8_t io0_0:1;
} _io_config_t;

typedef struct io_state_t {
	uint8_t :2;
	uint8_t gpio5:1;
	uint8_t gpio4:1;
	uint8_t gpio3:1;
	uint8_t gpio2:1;
	uint8_t gpio1:1;
	uint8_t gpio0:1;
} _io_state_t;

typedef struct i2c_timeout_t {
	uint8_t to:7;		// timeout value
	uint8_t te:1;		// enable
} _i2c_timeout_t;

typedef enum {
	SC18_TR_SUCCESS = 0,
	SC18_ADDR_NACK = 1,
	SC18_DATA_NACK = 2,
	SC18_BUS_BUSY = 3,
	SC18_TIMEOUT = 8,
	SC18_INVALID_CNT = 9,
} i2c_stat_e_t;

typedef struct i2c_stat_t {
	uint8_t :4;
	i2c_stat_e_t stat:4;
} _i2c_stat_t;

typedef struct i2c_addr_t {
	uint8_t addr:7;
	uint8_t :1;
} _i2c_addr_t;

typedef struct sc18is600_regs_t {
	struct io_config_t io_config;
	struct io_state_t io_state;
	uint8_t i2c_clk;				// I2C clock = 7.3728e6 / 4 / i2c_clk [hz]
	struct i2c_timeout_t i2c_to;
	struct i2c_stat_t i2c_stat;
	struct i2c_addr_t i2c_addr;
} _sc18is600_regs_t;


// SPI transfer protocol
typedef enum {
	SC18_WR_N = 0x00,	// write n bytes to I2C address [0x00, n, addr, data0, data1, .. ]
	SC18_RD_N = 0x01,	// read n bytes from I2C address [0x01, n, addr ]
	SC18_RD_WR = 0x02,	// read after write [0x02, n write, n read, addr wr, [data wr 0], addr rd ]
	SC18_RD_BUF = 0x06,	// read buffer [0x06, ... ]
	SC18_WR_WR = 0x03,	// write after writea[0x03, n wr 0, n wr 1, addr 0, [data wr 0], addr 1, [data wr 1]]
	SC18_SPI = 0x18,	// SPI ocnfiguration [0x18, 0x81] LSB first, [0x18, 0x42] MSB first (default)
	SC18_WR_I = 0x20,	// write internal register [0x20, reg offset, value]
	SC18_RD_I = 0x21,	// read internal register [0x21, reg offset, xx]
	SC18_PWR = 0x30,	// power-down mode [0x30, 0x5a, 0xa5]

} sc18is600_spi_protocol_t;

#endif	// __SC18IS600_INTERNALS_H__
