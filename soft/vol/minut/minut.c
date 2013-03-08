#include "minut.h"

#include "type_def.h"
#include "dispatcher.h"
#include "dna.h"

#include "utils/pt.h"
#include "utils/time.h"
#include "utils/fifo.h"
#include "utils/state_machine.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

//for debug
#define static


// ------------------------------------------
// private definitions
//

#define NB_EVENTS	5
#define NB_CMDS		3
#define NB_OUT_FR	4

#define CONE_DDR			DDRB
#define CONE				PORTB
#define CONE_PIN			PB3
#define CONE_STATE_CLOSED	0
#define CONE_STATE_OPEN		_BV(CONE_PIN)

#define SAMPLING_START		(2 * TIME_1_SEC)
#define SAMPLING_PERIOD		(100 * TIME_1_MSEC)


// ------------------------------------------
// private types
//

typedef enum {
	MNT_EV_NONE,
	MNT_EV_CONE_OPEN,
	MNT_EV_CONE_CLOSED,
	MNT_EV_TIME_OUT,
	MNT_EV_TAKE_OFF,
} mnt_event_t;


// ------------------------------------------
// private variables
//

struct {
	dpt_interface_t interf;	// dispatcher interface

	pt_t pt_chk_cone;	// checking cone thread
	pt_t pt_chk_time_out;	// checking time-out thread
	pt_t pt_chk_cmds;	// checking commands thread
	pt_t pt_out;		// sending thread

	stm_t stm;

	u32 time_out;		// time-out target time
	u32 sampling_rate;	// sampling rate for door changings
	u32 take_off_time_out;	// take-off scan interval time
	u32 door_time_out;	// door scan interval time
	u32 check_time_out;	// state scan interval time

	u8 open_time;		// open time [0.0; 25.5] seconds from take-off detection

	u8 cone_state;

	// events fifo
	fifo_t ev_fifo;
	mnt_event_t ev_buf[NB_EVENTS];

	// incoming commands fifo
	fifo_t cmds_fifo;
	frame_t cmds_buf[NB_CMDS];
	frame_t cmd_fr;

	// outcoming frames fifo
	fifo_t out_fifo;
	frame_t out_buf[NB_OUT_FR];

	frame_t out_fr;		// frame for the sending thread

} MNT;


static const stm_transition_t init2cone_opening;
static const stm_transition_t init2aero_opening;
static const stm_transition_t cone_opening2aero_opening;
static const stm_transition_t cone_opening2cone_closed;
static const stm_transition_t aero_opening2aero_open;
static const stm_transition_t aero_open2cone_closing;
static const stm_transition_t cone_closing2cone_closed;
static const stm_transition_t cone_closed2waiting;
static const stm_transition_t waiting2flight;
static const stm_transition_t flight2cone_open;
static const stm_transition_t cone_open2braking;
static const stm_transition_t braking2cone_open;
static const stm_transition_t braking2parachute;


static const stm_state_t init;
static const stm_state_t cone_opening;
static const stm_state_t aero_opening;
static const stm_state_t aero_open;
static const stm_state_t cone_closing;
static const stm_state_t cone_closed;
static const stm_state_t waiting;
static const stm_state_t flight;
static const stm_state_t cone_open;
static const stm_state_t braking;
static const stm_state_t parachute;

static u8 init_action(pt_t* pt, void* args);
static u8 cone_opening_action(pt_t* pt, void* args);
static u8 aero_opening_action(pt_t* pt, void* args);
static u8 aero_open_action(pt_t* pt, void* args);
static u8 cone_closing_action(pt_t* pt, void* args);
static u8 cone_closed_action(pt_t* pt, void* args);
static u8 waiting_action(pt_t* pt, void* args);
static u8 flight_action(pt_t* pt, void* args);
static u8 cone_open_action(pt_t* pt, void* args);
static u8 braking_action(pt_t* pt, void* args);
static u8 parachute_action(pt_t* pt, void* args);


static const stm_transition_t init2cone_opening = {
	.ev = MNT_EV_CONE_CLOSED,
	.st = &cone_opening,
	.tr = &init2aero_opening,
};

static const stm_transition_t init2aero_opening = {
	.ev = MNT_EV_CONE_OPEN,
	.st = &aero_opening,
	.tr = NULL,
};

static const stm_transition_t cone_opening2aero_opening = {
	.ev = MNT_EV_CONE_OPEN,
	.st = &aero_opening,
	.tr = &cone_opening2cone_closed,
};

static const stm_transition_t cone_opening2cone_closed = {
	.ev = MNT_EV_TIME_OUT,
	.st = &cone_closed,
	.tr = NULL,
};

static const stm_transition_t aero_opening2aero_open = {
	.ev = MNT_EV_TIME_OUT,
	.st = &aero_open,
	.tr = NULL,
};

static const stm_transition_t aero_open2cone_closing = {
	.ev = MNT_EV_CONE_CLOSED,
	.st = &cone_closing,
	.tr = NULL,
};

static const stm_transition_t cone_closing2cone_closed = {
	.ev = MNT_EV_TIME_OUT,
	.st = &cone_closed,
	.tr = NULL,
};

static const stm_transition_t cone_closed2waiting = {
	.ev = MNT_EV_TIME_OUT,
	.st = &waiting,
	.tr = NULL,
};

static const stm_transition_t waiting2flight = {
	.ev = MNT_EV_TAKE_OFF,
	.st = &flight,
	.tr = NULL,
};

static const stm_transition_t flight2cone_open = {
	.ev = MNT_EV_TIME_OUT,
	.st = &cone_open,
	.tr = NULL,
};

static const stm_transition_t cone_open2braking = {
	.ev = MNT_EV_TIME_OUT,
	.st = &braking,
	.tr = NULL,
};

static const stm_transition_t braking2cone_open = {
	.ev = MNT_EV_TIME_OUT,
	.st = &cone_open,
	.tr = &braking2parachute,
};

static const stm_transition_t braking2parachute = {
	.ev = MNT_EV_CONE_OPEN,
	.st = &parachute,
	.tr = NULL,
};


static const stm_state_t init = {
	.action = init_action,
	.transition = &init2cone_opening,
};

static const stm_state_t cone_opening = {
	.action = cone_opening_action,
	.transition = &cone_opening2aero_opening,
};

static const stm_state_t aero_opening = {
	.action = aero_opening_action,
	.transition = &aero_opening2aero_open,
};

static const stm_state_t aero_open = {
	.action = aero_open_action,
	.transition = &aero_open2cone_closing,
};

static const stm_state_t cone_closing = {
	.action = cone_closing_action,
	.transition = &cone_closing2cone_closed,
};


static const stm_state_t cone_closed = {
	.action = cone_closed_action,
	.transition = &cone_closed2waiting,
};

static const stm_state_t waiting = {
	.action = waiting_action,
	.transition = &waiting2flight,
};

static const stm_state_t flight = {
	.action = flight_action,
	.transition = &flight2cone_open,
};

static const stm_state_t cone_open = {
	.action = cone_open_action,
	.transition = &cone_open2braking,
};

static const stm_state_t braking = {
	.action = braking_action,
	.transition = &braking2cone_open,
};

static const stm_state_t parachute = {
	.action = parachute_action,
	.transition = NULL,
};


// ------------------------------------------
// private functions
//

static u8 init_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// led alive 0.25s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0xa1, 0x00, 25)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_opening_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OPEN)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out 5s
	MNT.time_out = TIME_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 aero_opening_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde aero open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OPEN)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out 5s
	MNT.time_out = TIME_get() + 5 * TIME_1_SEC;

	// led alive 0.5s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0xa1, 0x00, 50)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 aero_open_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_closing_action(pt_t* pt, void* args)
{
	PT_BEGIN(pt);

	// time-out 5s
	MNT.time_out = TIME_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_closed_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out 1s
	MNT.time_out = TIME_get() + 1 * TIME_1_SEC;

	// led alive 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0xa1, 0x00, 10)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 waiting_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// led alive 1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0xa1, 0x00, 100)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 flight_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde aero close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out = flight time
	MNT.time_out = MNT.open_time * TIME_1_SEC / 10 + TIME_get();

	// led alive 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0xa1, 0x00, 10)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_open_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OPEN)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// led open 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0x09, 0x00, 10)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out 0.1s
	MNT.time_out = TIME_get() + 100 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 braking_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde aero open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OPEN)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// time-out 0.1s
	MNT.time_out = TIME_get() + 100 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 parachute_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	// led open 1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, 0x09, 0x00, 100)
			&& OK == FIFO_put(&MNT.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


// check cone changings
static PT_THREAD( MNT_check_cone(pt_t* pt) )
{
	mnt_event_t ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, TIME_get() > MNT.sampling_rate);

	// set next sampling period
	MNT.sampling_rate += SAMPLING_PERIOD;

	// read cone state
	u8 cone_state = CONE & _BV(CONE_PIN);

	// check if the cone state has not changed
	if ( cone_state == MNT.cone_state ) {
		PT_RESTART(pt);
	}

	// save new cone state
	MNT.cone_state = cone_state;

	// else generate the correspondig change event
	switch (MNT.cone_state) {
		case CONE_STATE_OPEN:
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_CONE_OPEN) && OK == FIFO_put(&MNT.ev_fifo, &ev) );
			break;

		case CONE_STATE_CLOSED:
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_CONE_CLOSED) && OK == FIFO_put(&MNT.ev_fifo, &ev) );
			break;

		default:
			break;
	}

	PT_RESTART(pt);

	PT_END(pt);
}


// check a time-out has elapsed
static PT_THREAD( MNT_check_time_out(pt_t* pt) )
{
	mnt_event_t ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, TIME_get() > MNT.time_out);

	// prevent any further time-out
	MNT.time_out = TIME_MAX;

	// generate the time-out event
	PT_WAIT_UNTIL(pt, (ev = MNT_EV_TIME_OUT) && OK == FIFO_put(&MNT.ev_fifo, &ev) );

	PT_RESTART(pt);

	PT_END(pt);
}


static void MNT_open_time(frame_t* fr)
{
	switch (fr->argv[0]) {
		case 0x00:
			// save new open time value
			MNT.open_time = fr->argv[1];
			break;

		case 0xff:
			// read open time value
			fr->argv[1] = MNT.open_time;
			break;

		default:
			// bad sub-command
			fr->error = 1;
			break;
	}
}


static PT_THREAD( MNT_check_commands(pt_t* pt) )
{
	mnt_event_t ev;
	u8 swap;

	PT_BEGIN(pt);

	// as long as there are no command
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MNT.cmds_fifo, &MNT.cmd_fr));

	switch (MNT.cmd_fr.cmde) {
		case FR_TAKE_OFF:
			// generate take-off event
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_TAKE_OFF) && OK == FIFO_put(&MNT.ev_fifo, &ev) );
			break;

		case FR_MINUT_TIME_OUT:
			MNT_open_time(&MNT.cmd_fr);
			break;

		case FR_STATE:
			if ( (MNT.cmd_fr.argv[0] == 0x7a) || (MNT.cmd_fr.argv[0] == 0x8b) ) {
				//MNT.state = MNT.cmd_fr.argv[1];
			}

			// don't respond, response will be done by CMN
			PT_RESTART(pt);
			break;

		default:
			// shall never happen
			break;
	}

	// build the response to the current command
	swap = MNT.cmd_fr.orig;
	MNT.cmd_fr.orig = MNT.cmd_fr.dest;
	MNT.cmd_fr.dest = swap;
	MNT.cmd_fr.resp = 1;

	// enqueue it
	PT_WAIT_UNTIL(pt, OK == FIFO_put(&MNT.out_fifo, &MNT.cmd_fr));

	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( MNT_send_frame(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until an outgoing frame is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&MNT.out_fifo, &MNT.out_fr));

	// send the frame throught the dispatcher
	DPT_lock(&MNT.interf);

	// some retry may be needed
	PT_WAIT_UNTIL(pt, OK == DPT_tx(&MNT.interf, &MNT.out_fr));

	// release the dispatcher
	DPT_unlock(&MNT.interf);

	// loop back for the next frame to send
	PT_RESTART(pt);
	
	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void MNT_init(void)
{
	// init state machine
	STM_init(&MNT.stm, &init);

	// set the door pins direction
	CONE_DDR &= ~_BV(CONE_PIN);

	// init the cone state with its opposite value to generate the first event
	MNT.cone_state = ~(CONE & _BV(CONE_PIN));

	// init fifoes
	FIFO_init(&MNT.ev_fifo, MNT.ev_buf, NB_EVENTS, sizeof(mnt_event_t));
	FIFO_init(&MNT.cmds_fifo, MNT.cmds_buf, NB_CMDS, sizeof(frame_t));
	FIFO_init(&MNT.out_fifo, MNT.out_buf, NB_OUT_FR, sizeof(frame_t));

	// register to dispatcher
	MNT.interf.channel = 4;
	MNT.interf.cmde_mask = _CM(FR_TAKE_OFF) | _CM(FR_MINUT_TIME_OUT) | _CM(FR_STATE);
	MNT.interf.queue = &MNT.cmds_fifo;
	DPT_register(&MNT.interf);

	// init threads
	PT_INIT(&MNT.pt_chk_time_out);
	PT_INIT(&MNT.pt_chk_cmds);
	PT_INIT(&MNT.pt_out);

	// prevent any time-out
	MNT.time_out = TIME_MAX;
	MNT.sampling_rate = SAMPLING_START;
}


void MNT_run(void)
{
	// event sources are :
	//  - take-off detector
	//  - time-out
	//  - door detectors
	//  - frame commands
	//
	//  each generated event is stored in a fifo

	// check if door has changed
	(void)PT_SCHEDULE(MNT_check_cone(&MNT.pt_chk_cone));

	// check if a time-out has elapsed
	(void)PT_SCHEDULE(MNT_check_time_out(&MNT.pt_chk_time_out));

	// treat each incoming commands
	(void)PT_SCHEDULE(MNT_check_commands(&MNT.pt_chk_cmds));

	// treat each new event
	mnt_event_t ev;

	// if there is an event
	if ( OK == FIFO_get(&MNT.ev_fifo, &ev) ) {
		// update the state machine
		STM_event(&MNT.stm, ev);
	}

	// update state machine
	STM_run(&MNT.stm);

	// send outgoing frame(s) if any
	(void)PT_SCHEDULE(MNT_send_frame(&MNT.pt_out));
}
