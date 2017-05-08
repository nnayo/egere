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

#include "mpu6050.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "avr_twi.h"


//--------------------------------------------------------------------
// private defines
//

#define MPU6050_DEFAULT_ADDR  0x34
#define MPU6050_REGS_NB  0x100


//--------------------------------------------------------------------
// private structure definitions
//

enum mpu6050_irq {
    MPU_IRQ_IN,
    MPU_IRQ_OUT,
    MPU_IRQ_COUNT,
};


enum mpu6050_fsm {
	MPU_FSM_IDLE,
	MPU_FSM_STARTED,
	MPU_FSM_MTX,
	MPU_FSM_MRX,
};


struct mpu6050 {
	struct avr_t* avr;	// AVR access for time
	avr_irq_t* irq; 	// irq list

	enum mpu6050_fsm state;	// automata state
	uint8_t self_addr;	// slave address

	uint8_t current_reg;	// current accessed register
	int write_step;		// to handle modification of the register index
	uint8_t regs[MPU6050_REGS_NB];	// internal register map
};


struct simu_profil {
	float date;
	float acc_x;
	float acc_y;
	float acc_z;
	float temp;
	float gyr_x;
	float gyr_y;
	float gyr_z;
};


//--------------------------------------------------------------------
// private constants
//

// measure profiles
//        _
//       / 
//      /
//     /
// ___%
//0  10  20 
static struct simu_profil profil[] = { 
	// date, acc x, acc y, acc z, temp, gyr x, gyr y, gyr z
	{    .0,    .0,    .0,   -1.,  20.,    .0,    .0,    .0    },
	{ 4.999,    .0,    .0,   -1.,  20.,    .0,    .0,    .0    },
	{    5.,    .0,    .0,   15.,  20.,    .0,    .0,    .0    },
	{ 5.999,    .4,    .3,    7.,  20.,    .0,    .0,    .0    },
	{    6.,    .2,    .5,   -2.,  20.,    .0,    .0,    .0    },
	{   15.,    .2,    .5,    .5,  20.,    .0,    .0,    .0    },
	{   16.,    .7,    .5,    0.,  20.,    .0,    .0,    .0    },
	{  17.6,    .5,    .9,    .5,  20.,    .0,    .0,    .0    },
	{   20.,    .5,    .2,    .7,  20.,    .0,    .0,    .0    },
	{   30.,    .2,    .5,    .2,  20.,    .0,    .0,    .0    },
};


static struct {
	uint16_t acc_x;
	uint16_t acc_y;
	uint16_t acc_z;
	uint16_t temp;
	uint16_t gyr_x;
	uint16_t gyr_y;
	uint16_t gyr_z;

	struct simu_profil* profil;
} simu;

# define BRIGHT_COLOR   "\x1b[95m"
# define NORMAL_COLOR   "\x1b[0m"

//static char* msg2chr[] = {
//	BRIGHT_COLOR"0"NORMAL_COLOR,
//	BRIGHT_COLOR"["NORMAL_COLOR,
//	BRIGHT_COLOR"@"NORMAL_COLOR,
//	BRIGHT_COLOR"D"NORMAL_COLOR,
//	BRIGHT_COLOR"+"NORMAL_COLOR,
//	BRIGHT_COLOR"-"NORMAL_COLOR,
//	BRIGHT_COLOR"w"NORMAL_COLOR,
//	BRIGHT_COLOR"]"NORMAL_COLOR
//};


//--------------------------------------------------------------------
// private variables
//


//--------------------------------------------------------------------
// private functions
//

// using 2 points of the profile, linearly interpolate the measures at given date
static void simu_interpolate(float date, const struct simu_profil* const p1, const struct simu_profil* const p2)
{
	float acc_x, acc_y, acc_z;
	float temp;
	float gyr_x, gyr_y, gyr_z;

	if ( p2 != NULL) {
		acc_x = p1->acc_x + (date - p1->date) * (p2->acc_x - p1->acc_x) / (p2->date - p1->date);
		acc_y = p1->acc_y + (date - p1->date) * (p2->acc_y - p1->acc_y) / (p2->date - p1->date);
		acc_z = p1->acc_z + (date - p1->date) * (p2->acc_z - p1->acc_z) / (p2->date - p1->date);

		temp = p1->temp + (date - p1->date) * (p2->temp - p1->temp) / (p2->date - p1->date);

		gyr_x = p1->gyr_z + (date - p1->date) * (p2->gyr_x - p1->gyr_x) / (p2->date - p1->date);
		gyr_y = p1->gyr_z + (date - p1->date) * (p2->gyr_y - p1->gyr_y) / (p2->date - p1->date);
		gyr_z = p1->gyr_z + (date - p1->date) * (p2->gyr_z - p1->gyr_z) / (p2->date - p1->date);
	}
	else {
		acc_x = p1->acc_x;
		acc_y = p1->acc_y;
		acc_z = p1->acc_z;

		temp = p1->temp;

		gyr_x = p1->gyr_x;
		gyr_y = p1->gyr_y;
		gyr_z = p1->gyr_z;
	}

        simu.acc_x = acc_x / 32. * (1 << 16);
        simu.acc_y = acc_y / 32. * (1 << 16);
        simu.acc_z = acc_z / 32. * (1 << 16);
        simu.temp = (temp - 36.53) * 340;
        simu.gyr_x = gyr_x / 4000. * (1 << 16);
        simu.gyr_y = gyr_y / 4000. * (1 << 16);
        simu.gyr_z = gyr_z / 4000. * (1 << 16);

	printf(BRIGHT_COLOR"MPU"NORMAL_COLOR": %s date = %5.2f : acc = {.x=%4.2f, .y=%4.2f, .z=%4.2f} ==> {0x%04x 0x%04x 0x%04x}\n", __func__, date, acc_x, acc_y, acc_z, simu.acc_x, simu.acc_y, simu.acc_z);
}


// update MPU measure registers
static void simu_update(int cycle, int freq, uint8_t* const regs)
{
	// compute curent date
	float date = 1. * cycle / freq;

	// linear interpolation of the values
	for (unsigned int i = 0; i < sizeof(profil) / sizeof(profil[0]); i++) {
		const struct simu_profil * prof1 = &simu.profil[i];
		const struct simu_profil * prof2 = NULL;

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

	struct mpu6050* mpu = (struct mpu6050*)param;
	struct avr_twi_msg_irq_t msg;

	msg.u.v = value;

	AVR_LOG(mpu->avr, LOG_TRACE, BRIGHT_COLOR"MPU"NORMAL_COLOR": %s st:%d msg 0x%02x addr 0x%02x+%c / data 0x%02x\n", __func__, mpu->state, msg.u.twi.msg, msg.u.twi.addr >> 1, msg.u.twi.addr & 0x01 ? 'R' : 'W', msg.u.twi.data);

        // if not self address,
        if ((mpu->self_addr << 1) != (msg.u.twi.addr & 0xfe)) {
                // ignore request and reset state
                //msg.u.v = avr_twi_irq_msg(0, mpu->self_addr, 0);
                //avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.u.v);
                mpu->state = MPU_FSM_IDLE;

                printf(BRIGHT_COLOR"MPU"NORMAL_COLOR": %s invalid address 0x%02x, ignoring request\n", __func__, msg.u.twi.addr >> 1);

                return;
        }

        // if STOP is received
        if (msg.u.twi.msg == TWI_COND_STOP) {
                // ignore request and reset state
                mpu->state = MPU_FSM_IDLE;

                return;
        }

	switch (mpu->state) {
	case MPU_FSM_IDLE:
                // only a START is expected, if not ignore request
		if (msg.u.twi.msg != TWI_COND_START)
                        break;

                // ack
                msg.u.v = avr_twi_irq_msg(TWI_COND_ACK, msg.u.twi.addr, 1);
                avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.u.v);

                // go to read or write access mode
                if (msg.u.twi.addr & 0x01)
                        mpu->state = MPU_FSM_MRX;
                else
                        mpu->state = MPU_FSM_MTX;

                // reset write sequence
                mpu->write_step = 0;

                // update simulation
                simu_update(mpu->avr->cycle, mpu->avr->frequency, mpu->regs + 0x3b);

		break;

	case MPU_FSM_MTX:
		// first write access since START ?
		if (mpu->write_step == 0) {
			// set the register index
			mpu->current_reg = msg.u.twi.data;
			mpu->write_step = 1;
		}
		// next access(es)
		else {
			// write data at index
			mpu->regs[mpu->current_reg] = msg.u.twi.data;

			// and points to next register
			mpu->current_reg++;
		}

		// ack
		msg.u.v = avr_twi_irq_msg(TWI_COND_ACK, msg.u.twi.addr, 1);
		avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.u.v);
		break;

	case MPU_FSM_MRX:
		// nack received ?
		if (msg.u.twi.msg == 0) {
			// abort transfert
			mpu->state = MPU_FSM_IDLE;
			break;
		}

		// else ack received
		switch (mpu->current_reg) {
		case 0x75:	// MPU6050_WHO_AM_I
			msg.u.v = avr_twi_irq_msg(TWI_COND_READ, msg.u.twi.addr, 0x68);
			break;

		default:
			msg.u.v = avr_twi_irq_msg(TWI_COND_READ, msg.u.twi.addr, mpu->regs[mpu->current_reg]);
			break;
		}

                // send the data
		avr_raise_irq(mpu->irq + MPU_IRQ_OUT, msg.u.v);

                // increment register pointer
		mpu->current_reg++;
		break;

	default:
		break;
	}
}

static const char* mpu_irq_names[MPU_IRQ_COUNT] = {
		[MPU_IRQ_IN] = "mpu6050.in",
		[MPU_IRQ_OUT] = "mpu6050.out",
};


void* simavr_ext_component_init(struct avr_t * avr)
{
	struct mpu6050* mpu;

        // allocate and reset the object
	mpu = calloc(1, sizeof(struct mpu6050));

	mpu->avr = avr;
	mpu->self_addr = MPU6050_DEFAULT_ADDR;

        // create the irqs
	mpu->irq = avr_alloc_irq(&avr->irq_pool, MPU_IRQ_IN, MPU_IRQ_COUNT, mpu_irq_names);
	avr_irq_register_notify(mpu->irq + MPU_IRQ_IN, mpu6050_i2c_in_hook, mpu);

        // "connect" the IRQs of the MPU to the TWI/i2c master of the AVR
        uint32_t i2c_irq_base = AVR_IOCTL_TWI_GETIRQ(0);
        avr_connect_irq(mpu->irq + MPU_IRQ_OUT, avr_io_getirq(avr, i2c_irq_base, TWI_IRQ_INPUT));
        avr_connect_irq(avr_io_getirq(avr, i2c_irq_base, TWI_IRQ_OUTPUT), mpu->irq + MPU_IRQ_IN);

        // set the simulation profil
        simu.profil = profil;

        return mpu;
}


void simavr_ext_component_fini(struct mpu6050* mpu)
{
	free(mpu);
}

