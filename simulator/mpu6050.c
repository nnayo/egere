/*
	mpu-6050.c

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

#include "mpu6050.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "avr_twi.h"

#include "sc18is600.h"

//--------------------------------------------------------------------
// private defines
//

#define NB_REGS	0x100


//--------------------------------------------------------------------
// private structure definitions
//

typedef enum {
    MPU_IRQ_IN,
    MPU_IRQ_OUT,
    MPU_IRQ_COUNT,
} _mpu_irq_t;


typedef enum {
	MPU_FSM_IDLE,
	MPU_FSM_STARTED,
	MPU_FSM_MTX,
	MPU_FSM_MRX,
} _mpu6050_fsm_t;


typedef struct mpu6050_t {
	struct avr_t * avr;		// AVR access for time
	avr_irq_t *	irq;		// irq list

	_mpu6050_fsm_t	state;	// automata state
	uint8_t self_addr;		// slave address
	uint8_t bus:1;			// bus started

	uint8_t current_reg;	// current accessed register
	int write_step;			// to handle modification of the register index
	uint8_t regs[NB_REGS];	// internal register map
} mpu6050_t;


typedef struct simu_profil_t {
	float date;
	float acc_x;
	float acc_y;
	float acc_z;
	float temp;
	float gyr_x;
	float gyr_y;
	float gyr_z;
} simu_profil_struct;


//--------------------------------------------------------------------
// private constants
//

// measure profiles
static struct simu_profil_t profil[] = { 
	// date, acc x, acc y, acc z, temp, gyr x, gyr y, gyr z
	{    0.,   1.0,    0.,    0.,  20.,    0.,    0.,    0.    },
	{ 9.999,   1.0,    0.,    0.,  20.,    0.,    0.,    0.    },
	{   10.,    5.,    0.,    0.,  20.,    0.,    0.,    0.    },
	{   11.,    5.,    0.,    0.,  20.,    0.,    0.,    0.    },
	{   15.,    1.,    0.,    0.,  20.,    0.,    0.,    0.    },
};


static struct {
	uint16_t acc_x;
	uint16_t acc_y;
	uint16_t acc_z;
	uint16_t temp;
	uint16_t gyr_x;
	uint16_t gyr_y;
	uint16_t gyr_z;

	struct simu_profil_t * profil;
} simu;

# define BRIGHT_COLOR   "\x1b[95m"
# define NORMAL_COLOR   "\x1b[0m"

static char* msg2chr[] = {
	BRIGHT_COLOR"0"NORMAL_COLOR,
	BRIGHT_COLOR"["NORMAL_COLOR,
	BRIGHT_COLOR"@"NORMAL_COLOR,
	BRIGHT_COLOR"D"NORMAL_COLOR,
	BRIGHT_COLOR"+"NORMAL_COLOR,
	BRIGHT_COLOR"-"NORMAL_COLOR,
	BRIGHT_COLOR"w"NORMAL_COLOR,
	BRIGHT_COLOR"]"NORMAL_COLOR
};


//--------------------------------------------------------------------
// private variables
//


//--------------------------------------------------------------------
// private functions
//

// using 2 points of the profile, linearly interpolate the measures at given date
static void simu_interpolate(float date, const struct simu_profil_t * p1, const struct simu_profil_t * p2)
{
	float acc_x;
	float temp;

	if ( p2 != NULL) {
		acc_x = p1->acc_x + (date - p1->date) * (p2->acc_x - p1->acc_x) / (p2->date - p1->date);
		simu.acc_x = acc_x / 16. * (1 << 15);
		simu.acc_y = p1->acc_y + (date - p1->date) * (p2->acc_y - p1->acc_y) / (p2->date - p1->date);
		simu.acc_z = p1->acc_z + (date - p1->date) * (p2->acc_z - p1->acc_z) / (p2->date - p1->date);

		temp = p1->temp + (date - p1->date) * (p2->temp - p1->temp) / (p2->date - p1->date);
		simu.temp = (temp - 36.53) * 340;

		simu.gyr_x = p1->gyr_z + (date - p1->date) * (p2->gyr_x - p1->gyr_x) / (p2->date - p1->date);
		simu.gyr_y = p1->gyr_z + (date - p1->date) * (p2->gyr_y - p1->gyr_y) / (p2->date - p1->date);
		simu.gyr_z = p1->gyr_z + (date - p1->date) * (p2->gyr_z - p1->gyr_z) / (p2->date - p1->date);
	}
	else {
		simu.acc_x = p1->acc_x;
		simu.acc_y = p1->acc_y;
		simu.acc_z = p1->acc_z;

		temp = p1->temp;
		simu.temp = (temp - 36.53) * 340;

		simu.gyr_x = p1->gyr_x;
		simu.gyr_y = p1->gyr_y;
		simu.gyr_z = p1->gyr_z;
	}
}


// update MPU measure registers
static void simu_update(int cycle, int freq, uint8_t* regs)
{
	// compute curent date
	float date = 1. * cycle / freq;

	// linear interpolation of the values
	for (unsigned int i = 0; i < sizeof(profil) / sizeof(profil[0]); i++) {
		const struct simu_profil_t * prof1 = &simu.profil[i];
		const struct simu_profil_t * prof2 = NULL;

		// check if upper bound is available
		if ( i + 1 < sizeof(profil) / sizeof(profil[0]) ) {
			prof2 = &simu.profil[i + 1];
		}

		// find the profile with a correct date 
		if ( prof2 == NULL || (prof1->date <= date && date <= prof2->date) ) {
			// compute simulated values
			simu_interpolate(date, prof1, prof2);

			// done!
			break;
		}
	}

	// update registers
	regs[0] = (simu.acc_x & 0xff00) >> 8;	// acc X
	regs[1] = (simu.acc_x & 0x00ff) >> 0;
	regs[2] = (simu.acc_y & 0xff00) >> 8;	// acc Y
	regs[3] = (simu.acc_y & 0x00ff) >> 0;
	regs[4] = (simu.acc_z & 0xff00) >> 8;	// acc Z
	regs[5] = (simu.acc_z & 0x00ff) >> 0;

	regs[6] = (simu.temp & 0xff00) >> 8;	// temp
	regs[7] = (simu.temp & 0x00ff) >> 0;

	regs[8] = (simu.gyr_x & 0xff00) >> 8;	// gyr X
	regs[9] = (simu.gyr_x & 0x00ff) >> 0;
	regs[10] = (simu.gyr_y & 0xff00) >> 8;	// gyr Y
	regs[11] = (simu.gyr_y & 0x00ff) >> 0;
	regs[12] = (simu.gyr_z & 0xff00) >> 8;	// gyr Z
	regs[13] = (simu.gyr_z & 0x00ff) >> 0;
}


// called on every I2C transaction
static void mpu6050_i2c_in_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	(void)irq;

	mpu6050_t * mpu = (mpu6050_t*)param;
	avr_twi_msg_irq_t msg;

	msg.v = value;

	printf(BRIGHT_COLOR"MPU"NORMAL_COLOR": %s st:%d msg %s  addr 0x%02x+%c / data 0x%02x", __func__, mpu->state, msg2chr[msg.bus.msg], msg.bus.data >> 1, msg.bus.data & 0x01 ? 'R' : 'W', msg.bus.data);

	switch (mpu->state) {
	case MPU_FSM_IDLE:
		// start while really idle ?
		if (msg.bus.msg == TWI_MSG_START && !mpu->bus) {
			mpu->bus = 1;
			mpu->state = MPU_FSM_STARTED;
		}

		// end of transaction ?
		if (msg.bus.msg == TWI_MSG_STOP) {
			mpu->bus = 0;
		}
		printf("\n");
		break;

	case MPU_FSM_STARTED:
		// self address received ?
		if (msg.bus.msg == TWI_MSG_ADDR && (mpu->self_addr << 1) == (msg.bus.addr & 0xfe)) {
			printf("\n");
			// ack
			msg = avr_twi_irq_msg(TWI_MSG_ACK, mpu->self_addr);
			avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.v);

			// go to read or write access mode
			if (mpu->self_addr & 0x01)
				mpu->state = MPU_FSM_MRX;
			else
				mpu->state = MPU_FSM_MTX;

			// reset write sequence
			mpu->write_step = 0;

			// update simulation
			simu_update(mpu->avr->cycle, mpu->avr->frequency, mpu->regs + 0x3b);
		}
		// bad address !
		else {
			printf("\n");
			// nack
			msg = avr_twi_irq_msg(TWI_MSG_NACK, mpu->self_addr);
			avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.v);
			mpu->state = MPU_FSM_IDLE;
		}
		break;

	case MPU_FSM_MTX:
		// first write access since START ?
		if ( mpu->write_step == 0 ) {
			// set the register index
			mpu->current_reg = msg.bus.data;
			mpu->write_step = 1;
		}
		// next access(es)
		else {
			// write data at index
			mpu->regs[mpu->current_reg] = msg.bus.data;

			// and points to next register
			mpu->current_reg++;
		}
		printf("\n");

		// ack
		msg = avr_twi_irq_msg(TWI_MSG_ACK, mpu->self_addr);
		avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.v);
		break;

	case MPU_FSM_MRX:
		// nack received ?
		if (msg.bus.msg == TWI_MSG_NACK) {
			// abort transfert
			mpu->state = MPU_FSM_IDLE;
			break;
		}

		// else ack receie
		switch (mpu->current_reg) {
		case 0x75:	// MPU6050_WHO_AM_I
			msg = avr_twi_irq_msg(TWI_MSG_DATA, 0x68);
			break;

		default:
			msg = avr_twi_irq_msg(TWI_MSG_DATA, mpu->regs[mpu->current_reg]);
			break;
		}

		printf("\n");
        // send the data
		avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.v);

        // increment register pointer
		mpu->current_reg++;
		break;

	default:
		printf("\n");
		break;
	}
}

static const char * mpu_irq_names[MPU_IRQ_COUNT] = {
		[MPU_IRQ_IN] = "mpu6050.in",
		[MPU_IRQ_OUT] = "mpu6050.out",
};


struct mpu6050_t* mpu6050_alloc(struct avr_t * avr, uint8_t addr, struct sc18is600_t* sc18)
{
	struct mpu6050_t* mpu;

    // allocate and reset the object
	mpu = malloc(sizeof(mpu6050_t));
	memset(mpu, 0, sizeof(mpu6050_t));

	mpu->avr = avr;
	mpu->self_addr = addr;

    // create the irqs
	mpu->irq = avr_alloc_irq(&avr->irq_pool, MPU_IRQ_IN, MPU_IRQ_COUNT, mpu_irq_names);
	avr_irq_register_notify(mpu->irq + MPU_IRQ_IN, mpu6050_i2c_in_hook, mpu);

	if (sc18) {
		// connect to the TWI/i2c master of the SC18IS600
		avr_irq_t * sc18_irq = sc18is600_i2c_irq_get(sc18);
		avr_connect_irq(sc18_irq + SC18_I2C_IRQ_OUT, mpu->irq + MPU_IRQ_IN);
		avr_connect_irq(mpu->irq + MPU_IRQ_OUT, sc18_irq + SC18_I2C_IRQ_IN);
	}
	else {
		// "connect" the IRQs of the MPU to the TWI/i2c master of the AVR
		uint32_t i2c_irq_base = AVR_IOCTL_TWI_GETIRQ(0);
		avr_connect_irq(mpu->irq + MPU_IRQ_OUT, avr_io_getirq(avr, i2c_irq_base, TWI_IRQ_INPUT));
		avr_connect_irq(avr_io_getirq(avr, i2c_irq_base, TWI_IRQ_OUTPUT), mpu->irq + MPU_IRQ_IN);
	}

	return mpu;
}


void mpu6050_free(struct mpu6050_t* mpu)
{
	free(mpu);
}

