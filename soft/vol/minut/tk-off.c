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


// ------------------------------------------
// private variables
//

struct {
	pt_t pt;					// pt for sending thread
	dpt_interface_t interf;		// interface to the dispatcher

	frame_t in_buf[IN_FIFO_SIZE];
	fifo_t in_fifo;
	frame_t in_fr;				// incoming frame for acquisitions or commands

	u32 thr_time_out;			// time-out for threshold duration
	u8 thr_duration;			// threshold duration in 0.1s
	u16 acc_thr;				// acceleration threshold in 0.1G
	u8 thr_flag;
} TKF;


// ------------------------------------------
// private functions
//

// update threshold configuration from incoming command
static void TKF_config(void)
{
	TKF.thr_duration = TKF.in_fr.argv[0];	// threshold duration in 0.1s
	TKF.acc_thr = TKF.in_fr.argv[1];		// acceleration threshold in 0.1G
}


// compute if take-off threshold is triggered
static u8 TKF_compute(void)
{
	s16 acc_x;

	// extract acceleration level
	acc_x = TKF.in_fr.argv[0] << 8;
	acc_x += TKF.in_fr.argv[1] << 0;

	// scale acceleration
	acc_x *= 11;
	acc_x /= 10;

	// if behond threshold
	if ( acc_x < TKF.acc_thr ) {
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


static PT_THREAD( TKF_thread(pt_t* pt) )
{
	frame_t fr;

	PT_BEGIN(pt);

	// wait incoming acquisitions and commands
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&TKF.in_fifo, &TKF.in_fr));

	// responses are ignored
	if ( TKF.in_fr.resp ) {
		PT_RESTART(pt);
	}

	switch(TKF.in_fr.cmde) {
	// configuration of the threshold
	case FR_TAKE_OFF_THRES:
		TKF_config();
		PT_RESTART(pt);
		break;

	// acceleration data
	case FR_DATA_ACC:
		if ( KO == TKF_compute()) {
			PT_RESTART(pt);
		}
		break;

	// rotation data
	case FR_DATA_GYR:
		if ( KO == TKF_compute()) {
			PT_RESTART(pt);
		}
		break;

	default:
		PT_RESTART(pt);
		break;
	}

	// send the take-off frame
	DPT_lock(&TKF.interf);
	PT_WAIT_UNTIL(pt, frame_set_0(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_TAKE_OFF, 0)
			&& DPT_tx(&TKF.interf, &fr));
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

	TKF.interf.channel = 5;
	TKF.interf.cmde_mask = _CM(FR_TAKE_OFF_THRES) | _CM(FR_DATA_ACC) | _CM(FR_DATA_GYR);
	TKF.interf.queue = &TKF.in_fifo;
	DPT_register(&TKF.interf);

	// default threshold duration and time-out and acceleration threshold
	TKF.thr_time_out = TIME_MAX;
	TKF.thr_duration = 255;	// 25.5s
	TKF.acc_thr = 20;	// 2.0G

	PT_INIT(&TKF.pt);
}


void TKF_run(void)
{
	(void)PT_SCHEDULE(TKF_thread(&TKF.pt));
}
