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
	pt_t pt;		// pt for sending thread
	pt_t pt_cmde;		// pt for receiving thread

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
	frame_t cmd_fr;	// frame for the cmde thread

} SERVO;


// ------------------------------------------
// private functions
//

// activate the cone servo to drive it to the given position
static void SERVO_cone_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	TMR1_compare_set(TMR1_A, (position * 1000 / 90) + 3000);
}


// deactivate the cone servo to save power
static void SERVO_cone_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	TMR1_compare_set(TMR1_A, 0);
}


// activate the aero servo to drive it to the given position
static void SERVO_aero_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	TMR1_compare_set(TMR1_B, (position * 1000 / 90) + 3000);
}


// deactivate the aero servo to save power
static void SERVO_aero_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	TMR1_compare_set(TMR1_B, 0);
}


static void SERVO_drive(u8 servo, u8 sense)
{
	switch (servo) {
	case FR_SERVO_CONE:
		switch (sense) {
		case FR_SERVO_OPEN:	// open
			SERVO_cone_on(SERVO.cone.open_pos);
			break;

		case FR_SERVO_CLOSE:	// close
			SERVO_cone_on(SERVO.cone.close_pos);
			break;

		case FR_SERVO_OFF:
			SERVO_cone_off();
			break;

		default:
			break;
		}
		break;

	case FR_SERVO_AERO:
		switch (sense) {
		case FR_SERVO_OPEN:	// open
			SERVO_aero_on(SERVO.aero.open_pos);
			break;

		case FR_SERVO_CLOSE:	// close
			SERVO_aero_on(SERVO.aero.close_pos);
			break;

		case FR_SERVO_OFF:
			SERVO_aero_off();
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}


static void SERVO_cone_save(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:	// open position
		SERVO.cone.open_pos = fr->argv[3];
		break;

	case FR_SERVO_CLOSE:	// closed position
		SERVO.cone.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void SERVO_cone_read(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:	// open position
		fr->argv[3] = SERVO.cone.open_pos;
		break;

	case FR_SERVO_CLOSE:	// closed position
		fr->argv[3] = SERVO.cone.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void SERVO_aero_save(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:	// open position
		SERVO.aero.open_pos = fr->argv[3];
		break;

	case FR_SERVO_CLOSE:	// closed position
		SERVO.aero.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void SERVO_aero_read(frame_t* fr)
{
	switch ( fr->argv[2] ) {
	case FR_SERVO_OPEN:	// open position
		fr->argv[3] = SERVO.aero.open_pos;
		break;

	case FR_SERVO_CLOSE:	// closed position
		fr->argv[3] = SERVO.aero.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void SERVO_position(frame_t* fr)
{
	switch ( fr->argv[0] ) {
	case FR_SERVO_CONE:
		switch ( fr->argv[1] ) {
		case FR_SERVO_SAVE:	// save
			SERVO_cone_save(fr);
			break;

		case FR_SERVO_READ:	// read
			SERVO_cone_read(fr);
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
			SERVO_aero_save(fr);
			break;

		case FR_SERVO_READ:	// read
			SERVO_aero_read(fr);
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


static PT_THREAD( SERVO_cmde(pt_t* pt) )
{
	u8 swap;

	PT_BEGIN(pt);

	// if no incoming frame is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&SERVO.in, &SERVO.cmd_fr) );

	// if it is a response
	if (SERVO.cmd_fr.resp) {
		// ignore it

		// release the dispatcher
		DPT_unlock(&SERVO.interf);

		// and restart waiting
		PT_RESTART(pt);
	}

	SERVO.cmd_fr.error = 0;

	switch (SERVO.cmd_fr.cmde) {
		case FR_MINUT_SERVO_CMD:
			// drive the servo
			SERVO_drive(SERVO.cmd_fr.argv[0], SERVO.cmd_fr.argv[1]);
			break;

		case FR_MINUT_SERVO_INFO:
			SERVO_position(&SERVO.cmd_fr);
			break;

		default:
			// shall never happen
			SERVO.cmd_fr.error = 1;
			break;
	}

	// send the response
	swap = SERVO.cmd_fr.orig;
	SERVO.cmd_fr.orig = SERVO.cmd_fr.dest;
	SERVO.cmd_fr.dest = swap;
	SERVO.cmd_fr.resp = 1;
	//SERVO.cmd_fr.nat = 0;
	PT_WAIT_UNTIL(pt, OK == FIFO_put(&SERVO.out, &SERVO.cmd_fr));

	// and restart waiting for incoming command
	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( SERVO_resp(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until a frame to send is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&SERVO.out, &SERVO.out_fr));

	// send it throught the dispatcher
	DPT_lock(&SERVO.interf);

	// some retry may be necessary
	PT_WAIT_UNTIL(pt, OK == DPT_tx(&SERVO.interf, &SERVO.out_fr));

	// release the dispatcher
	DPT_unlock(&SERVO.interf);

	// loop back at start
	PT_RESTART(pt);

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void SERVO_init(void)
{
	// init
	FIFO_init(&SERVO.in, &SERVO.in_buf, IN_FIFO_SIZE, sizeof(frame_t));
	FIFO_init(&SERVO.out, &SERVO.out_buf, OUT_FIFO_SIZE, sizeof(frame_t));

	SERVO.interf.channel = 9;
	SERVO.interf.cmde_mask = _CM(FR_MINUT_SERVO_CMD) | _CM(FR_MINUT_SERVO_INFO);
	SERVO.interf.queue = &SERVO.in;
	DPT_register(&SERVO.interf);

	PT_INIT(&SERVO.pt);
	PT_INIT(&SERVO.pt_cmde);

	// configure port
	SERVO_DDR |= SERVO_CONE;
	SERVO_DDR |= SERVO_AERO;

	// init the driver, by default, the pwm is zero
	TMR1_init(TMR1_WITHOUT_INTERRUPT, TMR1_PRESCALER_8, TMR1_WGM_FAST_PWM_OCR1A, COM1AB_1010, NULL, NULL);

	// launch the pwm generation
	TMR1_start();
}


void SERVO_run(void)
{
	// if incoming command available
	(void)PT_SCHEDULE(SERVO_cmde(&SERVO.pt_cmde));

	// if outgoing frame to send
	(void)PT_SCHEDULE(SERVO_resp(&SERVO.pt));
}
