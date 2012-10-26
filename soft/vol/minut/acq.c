#include "acq.h"

#include "dispatcher.h"

#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include "externals/adxl345.h"

#include "avr/io.h"	// ADC


// ------------------------------------------
// private definitions
//


// ------------------------------------------
// private variables
//

struct {
	pt_t pt;			// pt for sending thread
	dpt_interface_t interf;		// interface to the dispatcher

	frame_t fr;			// frame for the sending thread

	u32 time;			// acquistion time

	u16 pressure;			// direct pressure from captor
	u16 diff_pressure;		// differential pressure between captor and offset
} ACQ;


// ------------------------------------------
// private functions
//

static PT_THREAD( ACQ_acq(pt_t* pt) )
{
	u16 acc_x;
	u16 acc_y;
	u16 acc_z;
	u16 gyr_x;
	u16 gyr_y;
	u16 gyr_z;

	PT_BEGIN(pt);

	// MPU-6050 acquisitions init
	PT_WAIT_UNTIL(pt, ADXL_init() && ADXL_range_set(ADXL_16G) );

	// common measurement frames header
	ACQ.fr.orig = DPT_SELF_ADDR;
	ACQ.fr.dest = DPT_SELF_ADDR;
	ACQ.fr.status = 0;

	while (1) {
		// if it is time for an acquisition
		PT_WAIT_UNTIL(pt, ACQ.time < TIME_get() );

		// compute next time acquisition
		ACQ.time = 100 * TIME_1_MSEC + TIME_get();

		// get the accelerations
		ADXL_get(&acc_x, &acc_y, &acc_z);

		// build the acceleration frame
		ACQ.fr.cmde = FR_DATA_ACC;
		ACQ.fr.argv[0] = (acc_x >> 8) & 0x00ff;
		ACQ.fr.argv[1] = (acc_x >> 0) & 0x00ff;
		ACQ.fr.argv[2] = (acc_y >> 8) & 0x00ff;
		ACQ.fr.argv[3] = (acc_y >> 0) & 0x00ff;
		ACQ.fr.argv[4] = (acc_z >> 8) & 0x00ff;
		ACQ.fr.argv[5] = (acc_z >> 0) & 0x00ff;

		// send the acceleration frame
		DPT_lock(&ACQ.interf);
		PT_WAIT_UNTIL(pt, DPT_tx(&ACQ.interf, &ACQ.fr));
		DPT_unlock(&ACQ.interf);

		// and loop for next acquisition
	}

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void ACQ_init(void)
{
	// init
	ACQ.interf.channel = 11;
	ACQ.interf.cmde_mask = 0;
	ACQ.interf.queue = NULL;
	DPT_register(&ACQ.interf);

	ACQ.time = 100 * TIME_1_MSEC;

	PT_INIT(&ACQ.pt);

	// disable analog inputs
	DIDR0 = _BV(ADC5D) | _BV(ADC4D) | _BV(ADC3D) | _BV(ADC2D);
}


void ACQ_run(void)
{
	// if incoming command available
	(void)PT_SCHEDULE(ACQ_acq(&ACQ.pt));
}
