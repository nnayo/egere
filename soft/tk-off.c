#include "tk-off.h"

#include "dispatcher.h"

#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include "avr/io.h"	// ADC


// ------------------------------------------
// private definitions
//

#define IN_FIFO_SIZE	1

#define SAMPLEFREQ		100.0f	// Hz


// ------------------------------------------
// private variables
//

struct {
	pt_t pt;					// pt for sending thread
	dpt_interface_t interf;		// interface to the dispatcher

	frame_t in_buf[IN_FIFO_SIZE]; // incoming buffer and fifo for acquisitions or commands
	fifo_t in_fifo;

	frame_t fr;					// computation frame

	u32 thr_time_out;			// time-out for threshold duration
	u8 thr_duration;			// threshold duration in 0.1s
	s16 acc_thr;				// acceleration threshold in 0.1G

	u8 take_off_resp_rxed;		// flag to ensure only 1 take off command is send

	// accelerations are in [-16G; +16G]
	s16 acc_x;
	s16 acc_y;
	s16 acc_z;
	// rotation speeds are in [-500 deg/s; 500 deg/s]
	u16 gyr_x;
	u16 gyr_y;
	u16 gyr_z;

	// quaternion computation using Madgwick's IMU and AHRS algorithms.
	float beta;					// 2 * proportional gain (Kp)
	float q0, q1, q2, q3;		// quaternion of sensor frame relative to auxiliary frame
} TKF;


// ------------------------------------------
// private functions
//

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float inv_sqrt(float x)
{
	union {
		float f;
		long l;
	} conv_f2l;

	float halfx = 0.5f * x;
	float y = x;

	//long i = *(long*)&y;
	conv_f2l.f = y;
	long i = conv_f2l.l;

	i = 0x5f3759df - (i>>1);
	//y = *(float*)&i;
	conv_f2l.l = i;
	y = conv_f2l.f;

	y = y * (1.5f - (halfx * y * y));

	return y;
}


static void TKF_Madgwick(void)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	float q0 = TKF.q0;
	float q1 = TKF.q1;
	float q2 = TKF.q2;
	float q3 = TKF.q3;

	float ax = TKF.acc_x;
	float ay = TKF.acc_y;
	float az = TKF.acc_z;
	float gx = TKF.gyr_x;
	float gy = TKF.gyr_y;
	float gz = TKF.gyr_z;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= TKF.beta * s0;
		qDot2 -= TKF.beta * s1;
		qDot3 -= TKF.beta * s2;
		qDot4 -= TKF.beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / SAMPLEFREQ);
	q1 += qDot2 * (1.0f / SAMPLEFREQ);
	q2 += qDot3 * (1.0f / SAMPLEFREQ);
	q3 += qDot4 * (1.0f / SAMPLEFREQ);

	// Normalise quaternion
	recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	TKF.q0 = q0;
	TKF.q1 = q1;
	TKF.q2 = q2;
	TKF.q3 = q3;
}


// update threshold configuration from incoming command
static void TKF_config(void)
{
	TKF.thr_duration = TKF.fr.argv[0];	// threshold duration in 0.1s
	TKF.acc_thr = TKF.fr.argv[1];		// acceleration threshold in 0.1G converted to [-16G; +16G]
	TKF.acc_thr = TKF.acc_thr * 2048 / 10;
}


// compute if take-off threshold is triggered
static u8 TKF_compute(void)
{
	// if behond threshold
	if ( TKF.acc_x < TKF.acc_thr ) {
		// reset time_out
		TKF.thr_time_out = TIME_MAX;
		return KO;
	}
	// else if thresholf time-out is not running
	else if ( TKF.thr_time_out == TIME_MAX ) {
		// set it
		TKF.thr_time_out = TIME_get() + TKF.thr_duration * 100 * TIME_1_MSEC;
	}

	// if time-out elapses
	if ( TIME_get() > TKF.thr_time_out ) {
		// take-off is detected
		return OK;
	}

	return KO;
}


volatile	static s32 acc;
static PT_THREAD( TKF_thread(pt_t* pt) )
{
	frame_t fr;
	s16 gyr;

	PT_BEGIN(pt);

	// wait incoming acquisitions and commands
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&TKF.in_fifo, &TKF.fr));

	// responses are ignored except FR_TAKE_OFF
	if ( TKF.fr.resp && TKF.fr.cmde != FR_TAKE_OFF ) {
		// but don't forget to release the dispatcher
		DPT_unlock(&TKF.interf);

		// and restart the waiting
		PT_RESTART(pt);
	}

	switch(TKF.fr.cmde) {
	// take off response received
	case FR_TAKE_OFF:
		if ( TKF.fr.resp ) {
			TKF.take_off_resp_rxed = 1;
		}
		break;

	// configuration of the threshold
	case FR_TAKE_OFF_THRES:
		TKF_config();

		// send the response
		u8 swap = TKF.fr.dest;
		TKF.fr.dest = TKF.fr.orig;
		TKF.fr.orig = swap;
		TKF.fr.resp = 1;
		PT_WAIT_UNTIL(pt, DPT_tx(&TKF.interf, &TKF.fr));

		break;

	// acceleration data
	case FR_DATA_ACC:
		TKF.acc_x = TKF.fr.argv[0] << 8;
		TKF.acc_x |= TKF.fr.argv[1] << 0;

		TKF.acc_y = TKF.fr.argv[2] << 8;
		TKF.acc_y |= TKF.fr.argv[3] << 0;

		TKF.acc_z = TKF.fr.argv[4] << 8;
		TKF.acc_z |= TKF.fr.argv[5] << 0;

		if ( OK == TKF_compute() && ! TKF.take_off_resp_rxed ) {
			// send the take-off frame
			PT_WAIT_UNTIL(pt, frame_set_0(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_TAKE_OFF, 0)
					&& DPT_tx(&TKF.interf, &fr));
		}

		break;

	// rotation data
	case FR_DATA_GYR:
		TKF.gyr_x = TKF.fr.argv[0] << 8;
		TKF.gyr_x |= TKF.fr.argv[1] << 0;

		TKF.gyr_y = TKF.fr.argv[2] << 8;
		TKF.gyr_y |= TKF.fr.argv[3] << 0;

		TKF.gyr_z = TKF.fr.argv[4] << 8;
		TKF.gyr_z |= TKF.fr.argv[5] << 0;

		//TKF_Madgwick();
		break;

	default:
		break;
	}

	DPT_unlock(&TKF.interf);
	PT_RESTART(pt);

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void TKF_init(void)
{
	// init
	FIFO_init(&TKF.in_fifo, &TKF.in_buf, IN_FIFO_SIZE, sizeof(frame_t));

	TKF.interf.channel = 8;
	TKF.interf.cmde_mask = _CM(FR_TAKE_OFF) | _CM(FR_TAKE_OFF_THRES) | _CM(FR_DATA_ACC) | _CM(FR_DATA_GYR);
	TKF.interf.queue = &TKF.in_fifo;
	DPT_register(&TKF.interf);

	// default threshold duration and time-out and acceleration threshold
	TKF.thr_time_out = TIME_MAX;
	TKF.thr_duration = 255;	// 25.5s
	TKF.acc_thr = 20;	// 2.0G

	TKF.take_off_resp_rxed = 0;

	// quaternion init
	TKF.beta = 0.1f;
	TKF.q0 = 1.0;
	TKF.q1 = 0.0;
	TKF.q2 = 0.0;
	TKF.q3 = 0.0;

	PT_INIT(&TKF.pt);
}


void TKF_run(void)
{
	(void)PT_SCHEDULE(TKF_thread(&TKF.pt));
}
