#include "servo.h"

#include "dispatcher.h"

#include "drivers/timer1.h"
#include "utils/pt.h"
#include "utils/fifo.h"
#include "utils/time.h"

#include "avr/io.h"


// ------------------------------------------
// private definitions
//

#define IN_FIFO_SIZE	3
#define OUT_FIFO_SIZE	3

#define SERVO_DDR	DDRB
#define SERVO_PORT	PORTB
#define SERVO_PIN	PINB
#define SERVO_CONE	_BV(PB1)
#define SERVO_AERO	_BV(PB2)


// ------------------------------------------
// private variables
//

struct {
	pt_t pt_out;	// pt for sending thread
	pt_t pt_in;		// pt for receiving thread

	dpt_interface_t interf;	// interface to the dispatcher

	struct {
		s8 open_pos;		// open position
		s8 close_pos;		// closed position
	} cone;

	struct {
		s8 open_pos;		// open position
		s8 close_pos;		// closed position
	} aero;

	// incoming frames fifo
	fifo_t in;
	frame_t in_buf[IN_FIFO_SIZE];

	// outgoing frames fifo
	fifo_t out;
	frame_t out_buf[OUT_FIFO_SIZE];

	frame_t out_fr;	// frame for the sending thread
	frame_t in_fr;	// frame for the cmde thread

} SRV;


// ------------------------------------------
// private functions
//

// activate the cone servo to drive it to the given position
static void SRV_cone_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	// compare = (position / 90) * 1000 + 3000
	// the computation shall be modified to fit in s16
	// the result is sure to fit in u16
	TMR1_compare_set(TMR1_A, ((s16)position * 100 / 9) + 3000);
}


// deactivate the cone servo to save power
static void SRV_cone_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	TMR1_compare_set(TMR1_A, 0);
}


// activate the aero servo to drive it to the given position
static void SRV_aero_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	// compare = (position / 90) * 1000 + 3000
	// the computation shall be modified to fit in s16
	// the result is sure to fit in u16
	TMR1_compare_set(TMR1_B, ((s16)position * 100 / 9) + 3000);
}


// deactivate the aero servo to save power
static void SRV_aero_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	TMR1_compare_set(TMR1_B, 0);
}


static void SRV_drive(u8 servo, u8 sense)
{
	switch (servo) {
	case FR_SERVO_CONE:
		switch (sense) {
		case FR_SERVO_OPEN:		// open
			SRV_cone_on(SRV.cone.open_pos);
			break;

		case FR_SERVO_CLOSE:	// close
			SRV_cone_on(SRV.cone.close_pos);
			break;

		case FR_SERVO_OFF:
			SRV_cone_off();
			break;

		default:
			break;
		}
		break;

	case FR_SERVO_AERO:
		switch (sense) {
		case FR_SERVO_OPEN:		// open
			SRV_aero_on(SRV.aero.open_pos);
			break;

		case FR_SERVO_CLOSE:	// close
			SRV_aero_on(SRV.aero.close_pos);
			break;

		case FR_SERVO_OFF:
			SRV_aero_off();
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}


static void SRV_cone_save(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:		// open position
		SRV.cone.open_pos = fr->argv[3];
		break;

	case FR_SERVO_CLOSE:	// closed position
		SRV.cone.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void SRV_cone_read(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:		// open position
		fr->argv[3] = SRV.cone.open_pos;
		break;

	case FR_SERVO_CLOSE:	// closed position
		fr->argv[3] = SRV.cone.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void SRV_aero_save(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:		// open position
		SRV.aero.open_pos = fr->argv[3];
		break;

	case FR_SERVO_CLOSE:	// closed position
		SRV.aero.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void SRV_aero_read(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:		// open position
		fr->argv[3] = SRV.aero.open_pos;
		break;

	case FR_SERVO_CLOSE:	// closed position
		fr->argv[3] = SRV.aero.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void SRV_position(frame_t* fr)
{
	switch ( fr->argv[0] ) {
	case FR_SERVO_CONE:
		switch ( fr->argv[1] ) {
		case FR_SERVO_SAVE:	// save
			SRV_cone_save(fr);
			break;

		case FR_SERVO_READ:	// read
			SRV_cone_read(fr);
			break;

		default:
			// shall never happen
			fr->error = 1;
			break;
		}
		break;

	case FR_SERVO_AERO:
		switch ( fr->argv[1] ) {
		case FR_SERVO_SAVE:	// save
			SRV_aero_save(fr);
			break;

		case FR_SERVO_READ:	// read
			SRV_aero_read(fr);
			break;

		default:
			// shall never happen
			fr->error = 1;
			break;
		}
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static PT_THREAD( SRV_in(pt_t* pt) )
{
	u8 swap;

	PT_BEGIN(pt);

	// if no incoming frame is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&SRV.in, &SRV.in_fr) );

	// if it is a response
	if (SRV.in_fr.resp) {
		// ignore it

		// release the dispatcher
		DPT_unlock(&SRV.interf);

		// and restart waiting
		PT_RESTART(pt);
	}

	SRV.in_fr.error = 0;

	switch (SRV.in_fr.cmde) {
		case FR_MINUT_SERVO_CMD:
			// drive the servo
			SRV_drive(SRV.in_fr.argv[0], SRV.in_fr.argv[1]);
			break;

		case FR_MINUT_SERVO_INFO:
			SRV_position(&SRV.in_fr);
			break;

		default:
			// shall never happen
			SRV.in_fr.error = 1;
			break;
	}

	// send the response
	swap = SRV.in_fr.orig;
	SRV.in_fr.orig = SRV.in_fr.dest;
	SRV.in_fr.dest = swap;
	SRV.in_fr.resp = 1;
	//SRV.in_fr.nat = 0;
	PT_WAIT_UNTIL(pt, OK == FIFO_put(&SRV.out, &SRV.in_fr));

	// and restart waiting for incoming command
	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( SRV_out(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until a frame to send is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&SRV.out, &SRV.out_fr));

	// send it throught the dispatcher
	DPT_lock(&SRV.interf);

	// some retry may be necessary
	PT_WAIT_UNTIL(pt, OK == DPT_tx(&SRV.interf, &SRV.out_fr));

	// release the dispatcher
	DPT_unlock(&SRV.interf);

	// loop back at start
	PT_RESTART(pt);

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void SRV_init(void)
{
	// init
	FIFO_init(&SRV.in, &SRV.in_buf, IN_FIFO_SIZE, sizeof(frame_t));
	FIFO_init(&SRV.out, &SRV.out_buf, OUT_FIFO_SIZE, sizeof(frame_t));

	SRV.interf.channel = 10;
	SRV.interf.cmde_mask = _CM(FR_MINUT_SERVO_CMD) | _CM(FR_MINUT_SERVO_INFO);
	SRV.interf.queue = &SRV.in;
	DPT_register(&SRV.interf);

	PT_INIT(&SRV.pt_in);
	PT_INIT(&SRV.pt_out);

	// configure port
	SERVO_DDR |= SERVO_CONE;
	SERVO_DDR |= SERVO_AERO;

	// init the driver, by default, the pwm is zero
	TMR1_init(TMR1_WITHOUT_INTERRUPT, TMR1_PRESCALER_8, TMR1_WGM_FAST_PWM_ICR1, COM1AB_1010, NULL, NULL);

	TMR1_compare_set(TMR1_CAPT, 40000);

	// launch the pwm generation
	TMR1_start();
}


void SRV_run(void)
{
	// if incoming command available
	(void)PT_SCHEDULE(SRV_in(&SRV.pt_in));

	// if outgoing frame to send
	(void)PT_SCHEDULE(SRV_out(&SRV.pt_out));
}
