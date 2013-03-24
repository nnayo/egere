#include "mpu6050.h"

#include "dispatcher.h"

#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include <string.h>		// memcpy()


// MPU-6050 write protocol:
// START
// write register adress to MPU 
// then write as many bytes as possible
// write adress is incremented automatically
// STOP
//

// MPU-6050 read protocol:
// START
// write register adress to MPU 
// then read as many bytes as possible
// read adress is incremented automatically
// NACK
// STOP
//


// ------------------------------------------
// private definitions
//

#define IN_FIFO_SIZE	1

#define MPU_I2C_ADDR	(0x68 >> 1)

#define MPU6050_WHO_AM_I		0x75
#define MPU6050_SMPLRT_DIV		0x19
#define MPU6050_CONFIG			0x1a
#define MPU6050_GYRO_CONFIG		0x1b
#define MPU6050_ACCEL_CONFIG	0x1c
#define MPU6050_PWR_MGMT_1		0x6b

#define MPU6050_ACCEL_XOUT_H	0x3b
#define MPU6050_ACCEL_XOUT_L	0x3c
#define MPU6050_ACCEL_YOUT_H	0x3d
#define MPU6050_ACCEL_YOUT_L	0x3e
#define MPU6050_ACCEL_ZOUT_H	0x3f
#define MPU6050_ACCEL_ZOUT_L	0x40
#define MPU6050_TEMP_OUT_H		0x41
#define MPU6050_TEMP_OUT_L		0x42
#define MPU6050_GYRO_XOUT_H		0x43
#define MPU6050_GYRO_XOUT_L		0x44
#define MPU6050_GYRO_YOUT_H		0x45
#define MPU6050_GYRO_YOUT_L		0x46
#define MPU6050_GYRO_ZOUT_H		0x47
#define MPU6050_GYRO_ZOUT_L		0x48


// ------------------------------------------
// private variables
//

struct {
	pt_t pt;					// pt for sending thread
	dpt_interface_t interf;		// interface to the dispatcher
	u8 started;

	pt_t pt_spawn;				// pt for spawned threads

	frame_t in_buf[IN_FIFO_SIZE];
	fifo_t in_fifo;
	frame_t in_fr;				// incoming frame for acquisitions or commands

	u32 time_out;

	struct {
		u8 acc_x_hi;			// accel X axis MSB
		u8 acc_x_lo;			// accel X axis LSB
		u8 acc_y_hi;			// accel Y axis MSB
		u8 acc_y_lo;			// accel Y axis LSB
		u8 acc_z_hi;			// accel Z axis MSB
		u8 acc_z_lo;			// accel Z axis LSB
		u8 temp_hi;				// temperature MSB
		u8 temp_lo;				// temperature LSB
		u8 gyro_x_hi;			// gyro X axis MSB
		u8 gyro_x_lo;			// gyro X axis LSB
		u8 gyro_y_hi;			// gyro Y axis MSB
		u8 gyro_y_lo;			// gyro Y axis LSB
		u8 gyro_z_hi;			// gyro Z axis MSB
		u8 gyro_z_lo;			// gyro Z axis LSB
	} data;
} MPU;


// ------------------------------------------
// private functions
//


static PT_THREAD( MPU_init_pt_thread(pt_t* pt) )
{
	frame_t fr;

	PT_BEGIN(pt);

	// hard init
	DPT_lock(&MPU.interf);

	// set reg index to WHO_AM_I reg
	PT_WAIT_UNTIL(pt, frame_set_1(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_WRITE, 1, MPU6050_WHO_AM_I)
			&& DPT_tx(&MPU.interf, &fr));
	// wait response
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));

	// check it
	if ( fr.resp != 1 || fr.error != 0 || fr.orig != MPU_I2C_ADDR ) {
		// on error, retry
		PT_RESTART(pt);
	}

	// read WHO_AM_I reg
	PT_WAIT_UNTIL(pt, frame_set_0(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_READ, 1)
			&& DPT_tx(&MPU.interf, &fr));
	// wait response
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));

	// check WHO_AM_I : shall read 0x68
	if ( fr.resp != 1 || fr.error != 0 || fr.argv[0] != 0x68 ) {
		// on error, retry
		PT_RESTART(pt);
	}

	// set sampling rate to 100 Hz ( 1kHz / 10 ) : SMPLRT_DIV = 9
	// disable external sampling pin and enable DLPF at ~100 Hz : CONFIG = 2
	// set gyro full scale to 500 deg / s : GYRO_CONFIG = 0x08
	// set accel full scale to +-16G : ACCEL_CONFIG = 0x18

	// set reg index to SMPLRT_DIV reg
	// write SMPLRT_DIV, CONFIG, GYRO_CONFIG and ACCEL_CONFIG in a raw
	PT_WAIT_UNTIL(pt, frame_set_5(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_WRITE, 5, MPU6050_SMPLRT_DIV, 0x09, 0x02, 0x08, 0x18)
			&& DPT_tx(&MPU.interf, &fr));
	// wait response
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));

	// quit sleep mode : PWR_MGMT_1 = 0x00
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_WRITE, 2, MPU6050_PWR_MGMT_1, 0x00)
			&& DPT_tx(&MPU.interf, &fr));
	// wait response
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));

	DPT_unlock(&MPU.interf);
	PT_EXIT(pt);

	PT_END(pt);
}


static PT_THREAD( MPU_acquisition(pt_t* pt, u8 len, frame_t* fr) )
{
	PT_BEGIN(pt);

	// grant access for tx
	DPT_lock(&MPU.interf);

	// send a frame with the specified length
	PT_WAIT_UNTIL(pt, frame_set_0(fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_READ, len)
			&& DPT_tx(&MPU.interf, fr));
	// wait response
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, fr));

	// check it
	if ( fr->resp != 1 || fr->error != 0 || fr->orig != MPU_I2C_ADDR ) {
		// on error, retry
		PT_RESTART(pt);
	}

	DPT_unlock(&MPU.interf);

	PT_END(pt);
}


static PT_THREAD( MPU_thread(pt_t* pt) )
{
	frame_t fr;

	PT_BEGIN(pt);

	// wait application start signal
	if ( ! MPU.started ) {
		PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));
		if ( fr.cmde == FR_APPLI_START ) {
			MPU.started = 1;
		}
		else {
			PT_RESTART(pt);
		}
	}

	// check MPU hardware init
	PT_SPAWN(pt, &MPU.pt_spawn, MPU_init_pt_thread(&MPU.pt_spawn));

	MPU.time_out = 1 * TIME_1_SEC;
	while (1) {
		// data acquisition every 10 ms (100 Hz)
		PT_WAIT_UNTIL(pt, TIME_get() >= MPU.time_out);

		MPU.time_out += 10 * TIME_1_MSEC;

		DPT_lock(&MPU.interf);

		// set reg index to MPU6050_ACCEL_XOUT_H reg
		PT_WAIT_UNTIL(pt, frame_set_1(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, FR_I2C_WRITE, 1, MPU6050_ACCEL_XOUT_H)
				&& DPT_tx(&MPU.interf, &fr));
		// wait response
		PT_WAIT_UNTIL(pt, OK == FIFO_get(&MPU.in_fifo, &fr));

		// check it
		if ( fr.resp != 1 || fr.error != 0 || fr.orig != MPU_I2C_ADDR ) {
			// on error, retry
			DPT_unlock(&MPU.interf);
			continue;
		}

		// accel data: read burst from 0x3b to 0x40 (6 regs) 3 x 16-bit MSL first
		PT_SPAWN(pt, &MPU.pt_spawn, MPU_acquisition(&MPU.pt_spawn, 6, &fr));

		// save data
		memcpy(&MPU.data.acc_x_hi, &fr.argv[0], 6);

		// temp data: read burst from 0x41 to 0x42 (2 regs) 16-bit MSB first
		PT_SPAWN(pt, &MPU.pt_spawn, MPU_acquisition(&MPU.pt_spawn, 2, &fr));

		// save data
		memcpy(&MPU.data.temp_hi, &fr.argv[0], 2);

		// gyro data: read burst from 0x43 to 0x48 (6 regs) 3 x 16-bit MSL first
		PT_SPAWN(pt, &MPU.pt_spawn, MPU_acquisition(&MPU.pt_spawn, 6, &fr));

		// save data
		memcpy(&MPU.data.gyro_x_hi, &fr.argv[0], 6);

		DPT_lock(&MPU.interf);

		// build and send the acceleration data
		PT_WAIT_UNTIL(pt, frame_set_6(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_DATA_ACC, 6, MPU.data.acc_x_hi, MPU.data.acc_x_lo, MPU.data.acc_y_hi, MPU.data.acc_y_lo, MPU.data.acc_z_hi, MPU.data.acc_z_lo)
				&& DPT_tx(&MPU.interf, &fr));

		// build and send the gyroscopic data
		PT_WAIT_UNTIL(pt, frame_set_6(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_DATA_GYR, 6, MPU.data.gyro_x_hi, MPU.data.gyro_x_lo, MPU.data.gyro_y_hi, MPU.data.gyro_y_lo, MPU.data.gyro_z_hi, MPU.data.gyro_z_lo)
				&& DPT_tx(&MPU.interf, &fr));

		DPT_unlock(&MPU.interf);
	}

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void MPU_init(void)
{
	// init
	FIFO_init(&MPU.in_fifo, &MPU.in_buf, IN_FIFO_SIZE, sizeof(frame_t));

	MPU.interf.channel = 8;
	//MPU.interf.cmde_mask = _CM(FR_I2C_READ) | _CM(FR_I2C_WRITE);
	MPU.interf.cmde_mask = _CM(FR_I2C_READ) | _CM(FR_I2C_WRITE) | _CM(FR_APPLI_START);
	MPU.interf.queue = &MPU.in_fifo;
	DPT_register(&MPU.interf);

	MPU.started = 0;

	PT_INIT(&MPU.pt);
}


void MPU_run(void)
{
	(void)PT_SCHEDULE(MPU_thread(&MPU.pt));
}
