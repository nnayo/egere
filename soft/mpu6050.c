#include "mpu6050.h"

#include "dispatcher.h"

#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include <string.h>                // memcpy()


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

// uncomment the define below to use the SC18IS600 SPI-I2C bridge
//#define USE_SC18IS600

#ifdef USE_SC18IS600
# include "sc18is600.h"
#endif

#define IN_FIFO_SIZE        1

#define MPU_I2C_ADDR        (0x68 >> 1)

#define MPU6050_WHO_AM_I     0x75
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1   0x6b

#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48


// ------------------------------------------
// private variables
//

struct {
        pt_t pt;                                        // pt for sending thread
        struct scalp_dpt_interface interf;                // interface to the dispatcher
        u8 started;

        pt_t pt_spawn;                                // pt for spawned threads
#ifdef USE_SC18IS600
        pt_t pt_spawn_2;                        // pt for spawned threads
    u8 rx[10];
    u8 n;
#endif

        struct scalp in_buf[IN_FIFO_SIZE];
        struct nnk_fifo in_fifo;
        struct scalp in_fr;                                // incoming frame for acquisitions or commands

        u32 time_out;

        struct {
                u8 acc_x_hi;                        // accel X axis MSB
                u8 acc_x_lo;                        // accel X axis LSB
                u8 acc_y_hi;                        // accel Y axis MSB
                u8 acc_y_lo;                        // accel Y axis LSB
                u8 acc_z_hi;                        // accel Z axis MSB
                u8 acc_z_lo;                        // accel Z axis LSB
                u8 temp_hi;                                // temperature MSB
                u8 temp_lo;                                // temperature LSB
                u8 gyro_x_hi;                        // gyro X axis MSB
                u8 gyro_x_lo;                        // gyro X axis LSB
                u8 gyro_y_hi;                        // gyro Y axis MSB
                u8 gyro_y_lo;                        // gyro Y axis LSB
                u8 gyro_z_hi;                        // gyro Z axis MSB
                u8 gyro_z_lo;                        // gyro Z axis LSB
        } data;
} mpu;


// ------------------------------------------
// private functions
//


static PT_THREAD( mpu_init_pt_thread(pt_t* pt) )
{
        struct scalp fr;

        PT_BEGIN(pt);

#ifndef USE_SC18IS600
        // hard init
        scalp_dpt_lock(&mpu.interf);
#else
    u8 tx[5];
#endif

        // set reg index to WHO_AM_I reg
#ifndef USE_SC18IS600
        PT_WAIT_UNTIL(pt, scalp_set_1(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIWRITE, 1, MPU6050_WHO_AM_I)
                        && scalp_dpt_tx(&mpu.interf, &fr));
        // wait response
        PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, &fr));

        // check it
        if ( fr.resp != 1 || fr.error != 0 || fr.orig != MPU_I2C_ADDR ) {
                // on error, retry
                PT_RESTART(pt);
        }
#else
    tx[0] = MPU6050_WHO_AM_I;
    mpu.n = 1;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_tx(&mpu.pt_spawn_2, MPU_I2C_ADDR, tx, &mpu.n));
#endif

        // read WHO_AM_I reg
#ifndef USE_SC18IS600
        PT_WAIT_UNTIL(pt, scalp_set_0(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIREAD, 1)
                        && scalp_dpt_tx(&mpu.interf, &fr));
        // wait response
        PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, &fr));
#else
    mpu.n = 1;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_rx(&mpu.pt_spawn_2, MPU_I2C_ADDR, mpu.rx, &mpu.n));
#endif


        // check WHO_AM_I : shall read 0x68
#ifndef USE_SC18IS600
        if ( fr.resp != 1 || fr.error != 0 || fr.argv[0] != 0x68 ) {
                // on error, retry
                PT_RESTART(pt);
        }
#else
    if ( mpu.rx[0] != 0x68 ) {
                // on error, retry
                PT_RESTART(pt);
        }
#endif

        // set sampling rate to 100 Hz ( 1kHz / 10 ) : SMPLRT_DIV = 9
        // disable external sampling pin and enable DLPF at ~100 Hz : CONFIG = 2
        // set gyro full scale to 500 deg / s : GYRO_CONFIG = 0x08
        // set accel full scale to +-16G : ACCEL_CONFIG = 0x18

        // set reg index to SMPLRT_DIV reg
        // write SMPLRT_DIV, CONFIG, GYRO_CONFIG and ACCEL_CONFIG in a raw
#ifndef USE_SC18IS600
        PT_WAIT_UNTIL(pt, scalp_set_5(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIWRITE, 5, MPU6050_SMPLRT_DIV, 0x09, 0x02, 0x08, 0x18)
                        && scalp_dpt_tx(&mpu.interf, &fr));
        // wait response
        PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, &fr));
#else
    tx[0] = MPU6050_SMPLRT_DIV;
    tx[1] = 0x09;
    tx[2] = 0x02;
    tx[3] = 0x08;
    tx[4] = 0x18;
    mpu.n = 5;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_tx(&mpu.pt_spawn_2, MPU_I2C_ADDR, tx, &mpu.n));
#endif

        // quit sleep mode : PWR_MGMT_1 = 0x00
#ifndef USE_SC18IS600
        PT_WAIT_UNTIL(pt, scalp_set_2(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIWRITE, 2, MPU6050_PWR_MGMT_1, 0x00)
                        && scalp_dpt_tx(&mpu.interf, &fr));
        // wait response
        PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, &fr));

        scalp_dpt_unlock(&mpu.interf);
#else
    tx[0] = MPU6050_PWR_MGMT_1;
    tx[1] = 0x00;
    mpu.n = 2;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_tx(&mpu.pt_spawn_2, MPU_I2C_ADDR, tx, &mpu.n));
#endif

        PT_EXIT(pt);

        PT_END(pt);
}


static PT_THREAD( mpu_acquisition(pt_t* pt, u8 len, struct scalp* fr) )
{
        PT_BEGIN(pt);

#ifndef USE_SC18IS600
        // grant access for tx
        scalp_dpt_lock(&mpu.interf);
#endif

        // send a frame with the specified length
#ifndef USE_SC18IS600
        PT_WAIT_UNTIL(pt, scalp_set_0(fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIREAD, len)
                        && scalp_dpt_tx(&mpu.interf, fr));
        // wait response
        PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, fr));
#else
    mpu.n = len;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_rx(&mpu.pt_spawn_2, MPU_I2C_ADDR, mpu.rx, &mpu.n));
    memcpy(fr->argv, mpu.rx, mpu.n);
#endif

#ifndef USE_SC18IS600
        // check it
        if ( fr->resp != 1 || fr->error != 0 || fr->orig != MPU_I2C_ADDR ) {
                // on error, retry
                PT_RESTART(pt);
        }
#endif

#ifndef USE_SC18IS600
        scalp_dpt_unlock(&mpu.interf);
#endif

        PT_END(pt);
}


static PT_THREAD( mpu_thread(pt_t* pt) )
{
        struct scalp fr;
#ifdef USE_SC18IS600
        u8 tx[1];
        u8 n;
#endif

        PT_BEGIN(pt);

#ifdef USE_SC18IS600
        PT_SPAWN(pt, &mpu.pt_spawn, SC18IS600_init(&mpu.pt_spawn));
#endif
        // check MPU hardware init
        PT_SPAWN(pt, &mpu.pt_spawn, mpu_init_pt_thread(&mpu.pt_spawn));

        mpu.time_out = 1 * TIME_1_SEC;
        while (1) {
                // data acquisition every 10 ms (100 Hz)
                PT_WAIT_UNTIL(pt, nnk_time_get() >= mpu.time_out);

                mpu.time_out += 100 * TIME_1_MSEC;

#ifndef USE_SC18IS600
                scalp_dpt_lock(&mpu.interf);
#endif

                // set reg index to MPU6050_ACCEL_XOUT_H reg
#ifndef USE_SC18IS600
                PT_WAIT_UNTIL(pt, scalp_set_1(&fr, MPU_I2C_ADDR, DPT_SELF_ADDR, SCALP_TWIWRITE, 1, MPU6050_ACCEL_XOUT_H)
                                && scalp_dpt_tx(&mpu.interf, &fr));
                // wait response
                PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mpu.in_fifo, &fr));
#else
    tx[0] = MPU6050_ACCEL_XOUT_H;
    n = 1;
    PT_SPAWN(pt, &mpu.pt_spawn_2, SC18IS600_tx(&mpu.pt_spawn_2, MPU_I2C_ADDR, tx, &n));
#endif

#ifndef USE_SC18IS600
                // check it
                if ( fr.resp != 1 || fr.error != 0 || fr.orig != MPU_I2C_ADDR ) {
                        // on error, retry
                        scalp_dpt_unlock(&mpu.interf);
                        continue;
                }
#endif

                // accel data: read burst from 0x3b to 0x40 (6 regs) 3 x 16-bit MSL first
                PT_SPAWN(pt, &mpu.pt_spawn, mpu_acquisition(&mpu.pt_spawn, 6, &fr));

                // save data
                memcpy(&mpu.data.acc_x_hi, &fr.argv[0], 6);

                // temp data: read burst from 0x41 to 0x42 (2 regs) 16-bit MSB first
                PT_SPAWN(pt, &mpu.pt_spawn, mpu_acquisition(&mpu.pt_spawn, 2, &fr));

                // save data
                memcpy(&mpu.data.temp_hi, &fr.argv[0], 2);

                // gyro data: read burst from 0x43 to 0x48 (6 regs) 3 x 16-bit MSL first
                PT_SPAWN(pt, &mpu.pt_spawn, mpu_acquisition(&mpu.pt_spawn, 6, &fr));

                // save data
                memcpy(&mpu.data.gyro_x_hi, &fr.argv[0], 6);

                scalp_dpt_lock(&mpu.interf);

                // build and send the acceleration data
                PT_WAIT_UNTIL(pt, scalp_set_6(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_MPUACC, 6, mpu.data.acc_x_hi, mpu.data.acc_x_lo, mpu.data.acc_y_hi, mpu.data.acc_y_lo, mpu.data.acc_z_hi, mpu.data.acc_z_lo)
                                && scalp_dpt_tx(&mpu.interf, &fr));

                // build and send the gyroscopic data
                PT_WAIT_UNTIL(pt, scalp_set_6(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_MPUGYR, 6, mpu.data.gyro_x_hi, mpu.data.gyro_x_lo, mpu.data.gyro_y_hi, mpu.data.gyro_y_lo, mpu.data.gyro_z_hi, mpu.data.gyro_z_lo)
                                && scalp_dpt_tx(&mpu.interf, &fr));

                scalp_dpt_unlock(&mpu.interf);
        }

        PT_END(pt);
}


// ------------------------------------------
// public functions
//

void mpu_init(void)
{
        // init
        nnk_fifo_init(&mpu.in_fifo, &mpu.in_buf, IN_FIFO_SIZE, sizeof(struct scalp));

        mpu.interf.channel = 9;
        //mpu.interf.cmde_mask = _CM(SCALP_TWIREAD) | _CM(SCALP_TWIWRITE) | _CM(FR_APPLI_START);
        mpu.interf.cmde_mask = _CM(SCALP_TWIREAD) | _CM(SCALP_TWIWRITE);
        mpu.interf.queue = &mpu.in_fifo;
        scalp_dpt_register(&mpu.interf);

        mpu.started = 0;

        PT_INIT(&mpu.pt);
}


void mpu_run(void)
{
        (void)PT_SCHEDULE(mpu_thread(&mpu.pt));
}
