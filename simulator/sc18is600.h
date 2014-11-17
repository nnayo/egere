/*
	sc18is600.c

	Copyright Yann GOUY <yann_gouy@yahoo.fr>

	partially taken by examples/board_i2c_eeprom
	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */


//
// quite basic simulation of the SC18IS600 SPI/I2C bridge
//


#ifndef __SC18IS600_H__
# define __SC18IS600_H__

# include "sim_avr.h"
# include "avr_ioport.h"		// AVR_IOCTL_IOPORT_GETSTATE()
# include "sim_io.h"			// avr_ioctl()

typedef enum {
	SC18_I2C_IRQ_OUT,
	SC18_I2C_IRQ_IN,
	SC18_I2C_IRQ_CNT
} sc18_i2c_irq_t;

// instantiate a SC18IS600 SPI/I2C bridge
struct sc18is600_t* sc18is600_alloc(struct avr_t * avr);

// release the instantiated SC18IS600 SPI/I2C bridge
void sc18is600_free(struct sc18is600_t* sc18);

// to allow other components to connect to the I2C bus
avr_irq_t * sc18is600_i2c_irq_get(struct sc18is600_t* sc18);

#endif	// __SC18IS600_H__
