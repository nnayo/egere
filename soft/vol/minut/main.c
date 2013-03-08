#include "minut.h"
#include "servo.h"
#include "mpu6050.h"
#include "tk-off.h"

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

#if 0
arduino	atmega	function
-------+-------+--------

D9	PB1	servo cone
D10	PB2	servo aero

D13	PB5	led alive
D12	PB4	led open
D11	PB3	cone open switch

A4	PC4	sda
A5	PC5	scl

+9V	PWR	power in
+5V	+5V	power out
GND	GND	ground


MPU-6050	function
---------------+--------

sda		sda
scl		scl

+5V		power in
GND		ground


servo cône	function
---------------+--------

+5V		power in
ctrl		control signal
GND		ground


servo aero	function
---------------+--------

+5V		power in
ctrl		control signal
GND		ground


switch		function
---------------+--------

GND		ground
sig		signal



relay card	function
---------------+--------

+9V		power in (accu)
	GND		ground (accu)

	+6V		power out (servo cone)
	GND		ground (servo cone)
	+6V		power out (servo aero)
	GND		ground (servo aero)

	+9V		power out (arduino)
	+5V		power in (arduino)
	GND		ground (arduino)

	+5V		power out (MPU)
	GND		ground (MPU)

	sig		signal in (switch)
	sig		signal out (arduino)
	GND		ground (switch)

	sda		sda (arduino)
	sda		sda (MPU)
	scl		scl(arduino)
	scl		scl(MPU)

	ctrl		ctrl servo aero (arduino)
	ctrl		ctrl servo aero (servo)

	ctrl		ctrl servo cone (arduino)
	ctrl		ctrl servo cone (servo)

	led alive	signal in (arduino)
	led take-off	signal in (arduino)
#endif

// ------------------------------------------
// simavr options
//

#include "avr_mcu_section.h"

AVR_MCU(16000000, "atmega328p");

const struct avr_mmcu_vcd_trace_t simavr_conf[]  _MMCU_ = {
	{ AVR_MCU_VCD_SYMBOL("servo_cone"), .mask = _BV(PORTB1), .what = (void*)&PORTB, },
	{ AVR_MCU_VCD_SYMBOL("servo_aero"), .mask = _BV(PORTB2), .what = (void*)&PORTB, },
	{ AVR_MCU_VCD_SYMBOL("led_alive"), .mask = _BV(PORTB5), .what = (void*)&PORTB, },
	{ AVR_MCU_VCD_SYMBOL("led_open"), .mask = _BV(PORTB4), .what = (void*)&PORTB, },
	{ AVR_MCU_VCD_SYMBOL("cone_open_switch"), .mask = _BV(PORTB3), .what = (void*)&PINB, },

	{ AVR_MCU_VCD_SYMBOL("TWDR"), .what = (void*)&TWDR, },

	{ AVR_MCU_VCD_SYMBOL("TCCR1A"), .what = (void*)&TCCR1A, },
	{ AVR_MCU_VCD_SYMBOL("TCCR1B"), .what = (void*)&TCCR1B, },
	{ AVR_MCU_VCD_SYMBOL("TCCR1C"), .what = (void*)&TCCR1C, },
	{ AVR_MCU_VCD_SYMBOL("TCNT1H"), .what = (void*)&TCNT1H, },
	{ AVR_MCU_VCD_SYMBOL("TCNT1L"), .what = (void*)&TCNT1L, },
	{ AVR_MCU_VCD_SYMBOL("OCR1AH"), .what = (void*)&OCR1AH, },
	{ AVR_MCU_VCD_SYMBOL("OCR1AL"), .what = (void*)&OCR1AL, },
	{ AVR_MCU_VCD_SYMBOL("OCR1BH"), .what = (void*)&OCR1BH, },
	{ AVR_MCU_VCD_SYMBOL("OCR1BL"), .what = (void*)&OCR1BL, },

	{ AVR_MCU_VCD_SYMBOL("TIMSK1"), .what = (void*)&TIMSK1, },
	{ AVR_MCU_VCD_SYMBOL("TIFR1"), .what = (void*)&TIFR1, },

	{ AVR_MCU_VCD_SYMBOL("TCNT2"), .what = (void*)&TCNT2, },

	{ AVR_MCU_VCD_SYMBOL("UDR0"), .what = (void*)&UDR0, },
	{ AVR_MCU_VCD_SYMBOL("UDRIE0"), .mask = _BV(UDRIE0), .what = (void*)&UCSR0B, },
	{ AVR_MCU_VCD_SYMBOL("UCSR0A"), .what = (void*)&UCSR0A, },
};


// ------------------------------------------
// private definitions
//

//#define TIMER2_TOP_VALUE	78	// @ 8 MHz ==> 10 ms
#define TIMER2_TOP_VALUE	156	// @ 16 MHz ==> 10 ms


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
	NAT_init();
//	LOG_init();
//	TSN_init();
//	ALV_init();
	CPU_init();

	// program and start timer2 for interrupt on compare every 10 ms
	TMR2_init(TMR2_WITH_COMPARE_INT, TMR2_PRESCALER_1024, TMR2_WGM_CTC, TIMER2_TOP_VALUE, time, NULL);
	TMR2_start();

	MNT_init();
	SERVO_init();
	MPU_init();
	TKF_init();

	// enable interrupts
	sei();

	while (1) {
		// run every common module
		DPT_run();
		BSC_run();
//		RCF_run();
//		DNA_run();
		CMN_run();
		NAT_run();
//		LOG_run();
//		TSN_run();
//		ALV_run();
		CPU_run();

		MNT_run();
		SERVO_run();
		MPU_run();
		TKF_run();

		if ( TIME_get() > 15 * TIME_1_SEC ) {
			cli();
#include <avr/sleep.h>
			sleep_mode();
		}
	}

	// this point is never reached
	return 0;
}
