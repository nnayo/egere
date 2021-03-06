/*
	mpu6050.h

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
// quite basic simulation of the MPU-6050 I2C component
//
// it can be configured to connect to the I2C bus of the simulated SC18IS600

#ifndef __MPU6050_H__
# define __MPU6050_H__

#include "sc18is600.h"


// create a MPU6050
struct mpu6050_t* mpu6050_alloc(struct avr_t * avr, uint8_t addr, struct sc18is600_t* sc18);


// release the MPU6050
void mpu6050_free(struct mpu6050_t* mpu);


#endif	// __MPU6050_H__
