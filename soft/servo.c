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

static struct {
	pt_t pt_out;	// pt for sending thread
	pt_t pt_in;		// pt for receiving thread

	struct scalp_dpt_interface interf;	// interface to the dispatcher

	struct {
		s8 open_pos;		// open position
		s8 close_pos;		// closed position
	} cone;

	struct {
		s8 open_pos;		// open position
		s8 close_pos;		// closed position
	} aero;

	// incoming frames fifo
	struct nnk_fifo in;
	struct scalp in_buf[IN_FIFO_SIZE];

	// outgoing frames fifo
	struct nnk_fifo out;
	struct scalp out_buf[OUT_FIFO_SIZE];

	struct scalp out_fr;	// frame for the sending thread
	struct scalp in_fr;	// frame for the cmde thread
} srv;


// ------------------------------------------
// private functions
//

// activate the cone servo to drive it to the given position
static void srv_cone_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	// compare = (position / 90) * 1000 + 3000
	// the computation shall be modified to fit in s16
	// the result is sure to fit in u16
	nnk_tmr1_compare_set(NNK_TMR1_A, ((s16)position * 100 / 9) + 3000);
}


// deactivate the cone servo to save power
static void srv_cone_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	nnk_tmr1_compare_set(NNK_TMR1_A, 0);
}


// activate the aero servo to drive it to the given position
static void srv_aero_on(s8 position)
{
	// compute the compare value according to the required position and the prescaler
	// for position = -90 degrees, signal up time shall be 1 ms so compare = 2000
	// for position = 0 degrees, signal up time shall be 1.5 ms so compare = 3000
	// for position = +90 degrees, signal up time shall be 2 ms so compare = 4000
	// compare = (position / 90) * 1000 + 3000
	// the computation shall be modified to fit in s16
	// the result is sure to fit in u16
	nnk_tmr1_compare_set(NNK_TMR1_B, ((s16)position * 100 / 9) + 3000);
}


// deactivate the aero servo to save power
static void srv_aero_off(void)
{
	// setting the compare value to 0, ensure output pin is driven lo
	nnk_tmr1_compare_set(NNK_TMR1_B, 0);
}


static void srv_drive(u8 servo, u8 sense)
{
	switch (servo) {
	case SCALP_SERV_CONE:
		switch (sense) {
		case SCALP_SERV_OPEN:		// open
			srv_cone_on(srv.cone.open_pos);
			break;

		case SCALP_SERV_CLOSE:	// close
			srv_cone_on(srv.cone.close_pos);
			break;

		case SCALP_SERV_OFF:
			srv_cone_off();
			break;

		default:
			break;
		}
		break;

	case SCALP_SERV_AERO:
		switch (sense) {
		case SCALP_SERV_OPEN:		// open
			srv_aero_on(srv.aero.open_pos);
			break;

		case SCALP_SERV_CLOSE:	// close
			srv_aero_on(srv.aero.close_pos);
			break;

		case SCALP_SERV_OFF:
			srv_aero_off();
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}


static void srv_cone_save(struct scalp* fr)
{
	switch (fr->argv[2]) {
	case SCALP_SERV_OPEN:		// open position
		srv.cone.open_pos = fr->argv[3];
		break;

	case SCALP_SERV_CLOSE:	// closed position
		srv.cone.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void srv_cone_read(struct scalp* fr)
{
	switch (fr->argv[2]) {
	case SCALP_SERV_OPEN:		// open position
		fr->argv[3] = srv.cone.open_pos;
		break;

	case SCALP_SERV_CLOSE:	// closed position
		fr->argv[3] = srv.cone.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void srv_aero_save(struct scalp* fr)
{
	switch (fr->argv[2]) {
	case SCALP_SERV_OPEN:		// open position
		srv.aero.open_pos = fr->argv[3];
		break;

	case SCALP_SERV_CLOSE:	// closed position
		srv.aero.close_pos = fr->argv[3];
		break;

	default:
		// shall never happen
		fr->error = 1;
		break;
	}
}


static void srv_aero_read(struct scalp* fr)
{
	switch (fr->argv[2]) {
	case SCALP_SERV_OPEN:		// open position
		fr->argv[3] = srv.aero.open_pos;
		break;

	case SCALP_SERV_CLOSE:	// closed position
		fr->argv[3] = srv.aero.close_pos;
		break;

	default:
		// shall never happen
		fr->error = 1;
	}
}


static void srv_position(struct scalp* fr)
{
	switch (fr->argv[0]) {
	case SCALP_SERV_CONE:
		switch (fr->argv[1]) {
		case SCALP_SERV_SAVE:	// save
			srv_cone_save(fr);
			break;

		case SCALP_SERV_READ:	// read
			srv_cone_read(fr);
			break;

		default:
			// shall never happen
			fr->error = 1;
			break;
		}
		break;

	case SCALP_SERV_AERO:
		switch (fr->argv[1]) {
		case SCALP_SERV_SAVE:	// save
			srv_aero_save(fr);
			break;

		case SCALP_SERV_READ:	// read
			srv_aero_read(fr);
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


static PT_THREAD( srv_in(pt_t* pt) )
{
	u8 swap;

	PT_BEGIN(pt);

	// if no incoming frame is available
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&srv.in, &srv.in_fr) );

	// if it is a response
	if (srv.in_fr.resp) {
		// ignore it
		// and restart waiting
		PT_RESTART(pt);
	}

	srv.in_fr.error = 0;

	switch (srv.in_fr.cmde) {
		case SCALP_SERVOCMD:
			// drive the servo
			srv_drive(srv.in_fr.argv[0], srv.in_fr.argv[1]);
			break;

		case SCALP_SERVOINFO:
			srv_position(&srv.in_fr);
			break;

		default:
			// shall never happen
			srv.in_fr.error = 1;
			break;
	}

	// send the response
	swap = srv.in_fr.orig;
	srv.in_fr.orig = srv.in_fr.dest;
	srv.in_fr.dest = swap;
	srv.in_fr.resp = 1;
	//srv.in_fr.nat = 0;
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_put(&srv.out, &srv.in_fr));

	// and restart waiting for incoming command
	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( srv_out(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until a frame to send is available
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&srv.out, &srv.out_fr));

	// send it throught the dispatcher
	scalp_dpt_lock(&srv.interf);

	// some retry may be necessary
	PT_WAIT_UNTIL(pt, OK == scalp_dpt_tx(&srv.interf, &srv.out_fr));

	// release the dispatcher
	scalp_dpt_unlock(&srv.interf);

	// loop back at start
	PT_RESTART(pt);

	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void srv_init(void)
{
	// init
	nnk_fifo_init(&srv.in, &srv.in_buf, IN_FIFO_SIZE, sizeof(struct scalp));
	nnk_fifo_init(&srv.out, &srv.out_buf, OUT_FIFO_SIZE, sizeof(struct scalp));

	srv.interf.channel = 10;
	srv.interf.cmde_mask = SCALP_DPT_CM(SCALP_SERVOCMD)
                             | SCALP_DPT_CM(SCALP_SERVOINFO);
	srv.interf.queue = &srv.in;
	scalp_dpt_register(&srv.interf);

	PT_INIT(&srv.pt_in);
	PT_INIT(&srv.pt_out);

	// configure port
	SERVO_DDR |= SERVO_CONE;
	SERVO_DDR |= SERVO_AERO;

	// init the driver, by default, the pwm is zero
	nnk_tmr1_init(NNK_TMR1_WITHOUT_INTERRUPT, NNK_TMR1_PRESCALER_8, NNK_TMR1_WGM_FAST_PWM_ICR1, NNK_COM1AB_1010, NULL, NULL);

	nnk_tmr1_compare_set(NNK_TMR1_CAPT, 40000);

	// launch the pwm generation
	nnk_tmr1_start();
}


void srv_run(void)
{
	// if incoming command available
	(void)PT_SCHEDULE(srv_in(&srv.pt_in));

	// if outgoing frame to send
	(void)PT_SCHEDULE(srv_out(&srv.pt_out));
}
