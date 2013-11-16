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

// the driver is intended for SC18IS600 chip from NXP
//

// design
//
//

#include "sc18is600.h"

#include "sc18is600_internals.h"

#include "drivers/spi.h"

#include "utils/pt.h"

#include <string.h>		// memcpy()


//-----------------------------------------------------
// private defines
//

// use an unused I2C address for SC18IS600 self address
#define SC18_I2C_ADDR	0x77

// set I2C bus speed to 97 kHz
#define SC18_I2C_SPEED	19

#define SC18_OFFSET(reg_name)	\
	(u8)(u16)(&((struct sc18is600_regs_t*)0)->reg_name)

#define SC18_BUF_SIZE	16


//-----------------------------------------------------
// private types
//


//-----------------------------------------------------
// private variables
//

static struct {
	pt_t pt;

	u8 tx[SC18_BUF_SIZE];
	u8 rx[SC18_BUF_SIZE];
} SC18;


//-----------------------------------------------------
// private functions
//

// set a register with given value
static PT_THREAD( SC18IS600_reg_set(pt_t* pt, u8 offset, u8 value) )
{
	PT_BEGIN(pt);

	SC18.tx[0] = SC18_WR_I;
	SC18.tx[1] = offset;
	SC18.tx[2] = value;

	SPI_master(SC18.tx, 3, NULL, 0);

	PT_WAIT_UNTIL(pt, SPI_is_fini())

	PT_END(pt);
}


// get a register value
static PT_THREAD( SC18IS600_reg_get(pt_t* pt, u8 offset, u8* reg) )
{
	PT_BEGIN(pt);

	SC18.tx[0] = SC18_RD_I;
	SC18.tx[1] = offset;
	SC18.tx[2] = 0x00;

	SPI_master(SC18.tx, 3, SC18.rx, 3);

	PT_WAIT_UNTIL(pt, SPI_is_fini());

	*reg = SC18.rx[2];

	PT_END(pt);
}


//-----------------------------------------------------
// public functions
//

// initialization of the SC18IS600 component
PT_THREAD( SC18IS600_init(pt_t* pt) )
{
	PT_BEGIN(pt);

	// init SPI bus
	// SPI link configuration (SC18IS600 : spi mode 3, MSB first, 1 MHz)
	SPI_init(SPI_MASTER, SPI_THREE, SPI_MSB, SPI_DIV_16);

	// set own I2C address
	PT_SPAWN(pt, &SC18.pt, SC18IS600_reg_set(&SC18.pt, SC18_OFFSET(i2c_addr), SC18_I2C_ADDR));

	// set I2C bus speed
	PT_SPAWN(pt, &SC18.pt, SC18IS600_reg_set(&SC18.pt, SC18_OFFSET(i2c_clk), SC18_I2C_SPEED));

	// check status
	struct i2c_stat_t i2c_stat;
	do {
		PT_SPAWN(pt, &SC18.pt, SC18IS600_reg_get(&SC18.pt, SC18_OFFSET(i2c_stat), (u8*)&i2c_stat) );
	} while (i2c_stat.stat != SC18_TR_SUCCESS);

	PT_EXIT(pt);

	PT_END(pt);
}


// send n data to I2C addr
PT_THREAD( SC18IS600_tx(pt_t* pt, u8 addr, u8* data, u8* n) )
{
	PT_BEGIN(pt);

	// can't receive no more data than the buffer size
	if ( *n > SC18_BUF_SIZE - 3 ) {
		*n = SC18_BUF_SIZE - 3;
	}

	// send the write command with the data block
	SC18.tx[0] = SC18_WR_N;
	SC18.tx[1] = *n;
	SC18.tx[2] = addr << 1;
	memcpy(SC18.tx + 3, data, *n);

	SPI_master(SC18.tx, 3 + *n, NULL, 0);

	PT_WAIT_UNTIL(pt, SPI_is_fini());

	PT_END(pt);
}


// read n data from I2C addr
PT_THREAD( SC18IS600_rx(pt_t* pt, u8 addr, u8* data, u8* n) )
{
	PT_BEGIN(pt);

	// can't receive no more data than the buffer size
	if ( *n > SC18_BUF_SIZE - 1 ) {
		*n = SC18_BUF_SIZE - 1;
	}

	// send the read command
	SC18.tx[0] = SC18_RD_N;
	SC18.tx[1] = *n;
	SC18.tx[2] = addr << 1;

	SPI_master(SC18.tx, 3, NULL, 0);
	PT_WAIT_UNTIL(pt, SPI_is_fini());

	// check data was received
	struct i2c_stat_t i2c_stat;
	do {
		PT_SPAWN(pt, &SC18.pt, SC18IS600_reg_get(&SC18.pt, SC18_OFFSET(i2c_stat), (u8*)&i2c_stat) );
	} while (i2c_stat.stat != SC18_TR_SUCCESS);

	// retreive data
	SC18.tx[0] = SC18_RD_BUF;

	SPI_master(SC18.tx, 1, SC18.rx, *n + 1);
	PT_WAIT_UNTIL(pt, SPI_is_fini());

	// copy data to user
	memcpy(data, SC18.rx + 1, *n);

	PT_END(pt);
}


