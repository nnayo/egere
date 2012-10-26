#include "minut.h"
#include "servo.h"
#include "acq.h"

#include "drivers/timer2.h"
#include "utils/pt.h"
#include "utils/time.h"

#include "dispatcher.h"
#include "basic.h"
#include "reconf.h"
#include "dna.h"
#include "common.h"
#include "nat.h"
#include "log.h"
#include "time_sync.h"
#include "alive.h"
#include "cpu.h"

#include "avr/io.h"
#include "avr/interrupt.h"


// ------------------------------------------
// private definitions
//

#define TIMER2_TOP_VALUE	78


// ------------------------------------------
// private variables
//


// ------------------------------
// private functions
//

static void time(void* misc)
{
	(void)misc;

	// time update
	TIME_incr();
}


static u32 time_adjust(void)
{
	u8 val;
	u32 incr;

	val = TMR2_get_value();
	incr = TIME_get_incr();

	return incr * val / TIMER2_TOP_VALUE;
}


// ------------------------------
// public variables
//


// ------------------------------
// public functions
//

int main(void)
{
#if 0
	// if bad reset conditions
	if ( MCUSR & ( _BV(WDRF) | _BV(BORF) | _BV(EXTRF) ) ) {
		// loop for ever
		while (1)
			;
	}
	else {
		MCUSR = _BV(WDRF) | _BV(BORF) | _BV(EXTRF);
	}
#endif

	// init on-board time
	TIME_init(time_adjust);
	TIME_set_incr(10 * TIME_1_MSEC);

	// init every common module
	DPT_init();
	BSC_init();
//	RCF_init();
//	DNA_init(DNA_MINUT);
	CMN_init();
//	NAT_init();
//	LOG_init();
//	TSN_init();
//	ALV_init();
	CPU_init();

	// program and start timer2 for interrupt on compare every 10 ms
	TMR2_init(TMR2_WITH_COMPARE_INT, TMR2_PRESCALER_1024, TMR2_WGM_CTC, TIMER2_TOP_VALUE, time, NULL);
	TMR2_start();

	MNT_init();
	SERVO_init();
	ACQ_init();

	// enable interrupts
	sei();

	while (1) {
		// run every common module
		DPT_run();
		BSC_run();
//		RCF_run();
//		DNA_run();
		CMN_run();
//		NAT_run();
//		LOG_run();
//		TSN_run();
//		ALV_run();
		CPU_run();

		MNT_run();
		SERVO_run();
		ACQ_run();
	}

	// this point is never reached
	return 0;
}
