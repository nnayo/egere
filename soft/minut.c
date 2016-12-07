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
#define CONE				PINB
#define CONE_PIN			PB3
#define CONE_STATE_CLOSED	0
#define CONE_STATE_OPEN		_BV(CONE_PIN)

#define SAMPLING_START		(2 * TIME_1_SEC)
#define SAMPLING_PERIOD		(100 * TIME_1_MSEC)


// ------------------------------------------
// private types
//

enum mnt_event {
	MNT_EV_NONE,
	MNT_EV_SEPA_OPEN,
	MNT_EV_SEPA_CLOSED,
	MNT_EV_TIME_OUT,
	MNT_EV_TAKE_OFF,
	MNT_EV_BALISTIC_UP,
	MNT_EV_BALISTIC_DOWN,
	MNT_EV_LATERAL_ACC_TRIGGER,
};


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
	enum mnt_event ev_buf[NB_EVENTS];

	// incoming commands fifo
	fifo_t cmds_fifo;
	frame_t cmds_buf[NB_CMDS];
	frame_t cmd_fr;

	// outcoming frames fifo
	fifo_t out_fifo;
	frame_t out_buf[NB_OUT_FR];

	frame_t out_fr;		// frame for the sending thread

	u8 started:1;		// signal to application can be started
} mnt;


static const struct nnk_stm_transition init_to_lock0;
static const struct nnk_stm_transition init_to_waiting;
static const struct nnk_stm_transition lock0_to_lock1;
static const struct nnk_stm_transition lock1_to_lock2;
static const struct nnk_stm_transition lock2_to_waiting;
static const struct nnk_stm_transition waiting_to_flight;
static const struct nnk_stm_transition flight_to_balistic;
static const struct nnk_stm_transition flight_to_window_begin;
static const struct nnk_stm_transition balistic_to_open_seq;
static const struct nnk_stm_transition window_begin_to_open_seq_0;
static const struct nnk_stm_transition window_begin_to_open_seq_1;
static const struct nnk_stm_transition window_begin_to_open_seq_2;
static const struct nnk_stm_transition open_seq_to_brake;
static const struct nnk_stm_transition brake_to_unlock;
static const struct nnk_stm_transition unlock_to_brake;
static const struct nnk_stm_transition unlock_to_parachute;


static const struct nnk_stm_state init;
static const struct nnk_stm_state lock0;
static const struct nnk_stm_state lock1;
static const struct nnk_stm_state lock2;
static const struct nnk_stm_state waiting;
static const struct nnk_stm_state flight;
static const struct nnk_stm_state balistic;
static const struct nnk_stm_state window_begin;
static const struct nnk_stm_state open_seq;
static const struct nnk_stm_state brake;
static const struct nnk_stm_state unlock;
static const struct nnk_stm_state parachute;

static u8 init_action(pt_t* pt, void* args);
static u8 lock0_action(pt_t* pt, void* args);
static u8 lock1_action(pt_t* pt, void* args);
static u8 lock2_action(pt_t* pt, void* args);
static u8 waiting_action(pt_t* pt, void* args);
static u8 flight_action(pt_t* pt, void* args);
static u8 balistic_action(pt_t* pt, void* args);
static u8 window_begin_action(pt_t* pt, void* args);
static u8 open_seq_action(pt_t* pt, void* args);
static u8 brake_action(pt_t* pt, void* args);
static u8 unlock_action(pt_t* pt, void* args);
static u8 parachute_action(pt_t* pt, void* args);


//----------------------------------------------------------------------------
// transitions
//
static const struct nnk_stm_transition init_to_lock0 = {
    .ev = MNT_EV_SEPA_OPEN,
    .st = &lock0,
    .tr = &init_to_waiting,
};

static const struct nnk_stm_transition init_to_waiting = {
    .ev = MNT_EV_SEPA_CLOSED,
    .st = &waiting,
    .tr = NULL,
};

static const struct nnk_stm_transition lock0_to_lock1 = {
    .ev = MNT_EV_SEPA_CLOSED,
    .st = &lock1,
    .tr = NULL,
};

static const struct nnk_stm_transition lock1_to_lock2 = {
    .ev = MNT_EV_TIME_OUT,
    .st = &lock2,
    .tr = NULL,
};

static const struct nnk_stm_transition lock2_to_waiting = {
    .ev = MNT_EV_TIME_OUT,
    .st = &waiting,
    .tr = NULL,
};

static const struct nnk_stm_transition waiting_to_flight = {
    .ev = MNT_EV_TAKE_OFF,
    .st = &flight,
    .tr = NULL,
};

static const struct nnk_stm_transition flight_to_balistic = {
    .ev = MNT_EV_BALISTIC_UP,
    .st = &balistic,
    .tr = &flight_to_window_begin,
};

static const struct nnk_stm_transition flight_to_window_begin = {
    .ev = MNT_EV_TIME_OUT,
    .st = &window_begin,
    .tr = NULL,
};

static const struct nnk_stm_transition balistic_to_open_seq = {
    .ev = MNT_EV_TIME_OUT,
    .st = &open_seq,
    .tr = NULL,
};

static const struct nnk_stm_transition window_begin_to_open_seq_0 = {
    .ev = MNT_EV_TIME_OUT,
    .st = &open_seq,
    .tr = &window_begin_to_open_seq_1,
};

static const struct nnk_stm_transition window_begin_to_open_seq_1 = {
    .ev = MNT_EV_BALISTIC_DOWN,
    .st = &open_seq,
    .tr = &window_begin_to_open_seq_2,
};

static const struct nnk_stm_transition window_begin_to_open_seq_2 = {
    .ev = MNT_EV_LATERAL_ACC_TRIGGER,
    .st = &open_seq,
    .tr = NULL,
};

static const struct nnk_stm_transition open_seq_to_brake = {
    .ev = MNT_EV_TIME_OUT,
    .st = &brake,
    .tr = NULL,
};

static const struct nnk_stm_transition brake_to_unlock = {
    .ev = MNT_EV_TIME_OUT,
    .st = &unlock,
    .tr = NULL,
};

static const struct nnk_stm_transition unlock_to_brake = {
    .ev = MNT_EV_TIME_OUT,
    .st = &brake,
    .tr = &unlock_to_parachute,
};

static const struct nnk_stm_transition unlock_to_parachute = {
    .ev = MNT_EV_SEPA_OPEN,
    .st = &parachute,
    .tr = NULL,
};

//----------------------------------------------------------------------------
// states
//
static const struct nnk_stm_state init = {
	.action = init_action,
	.tr = &init_to_lock0,
};

static const struct nnk_stm_state lock0 = {
	.action = lock0_action,
	.transition = &lock0_to_lock1,
};

static const struct nnk_stm_state lock1 = {
	.action = lock1_action,
	.transition = &lock1_to_lock2,
};

static const struct nnk_stm_state lock2 = {
	.action = lock2_action,
	.transition = &lock2_to_waiting,
};

static const struct nnk_stm_state waiting = {
	.action = waiting_action,
	.transition = &waiting_to_flight,
};


static const struct nnk_stm_state flight = {
	.action = flight_action,
	.transition = &flight_to_balistic,
};

static const struct nnk_stm_state balistic = {
	.action = balistic_action,
	.transition = &balistic_to_open_seq,
};

static const struct nnk_stm_state window_begin = {
	.action = window_begin_action,
	.transition = &window_begin_to_open_seq_0,
};

static const struct nnk_stm_state open_seq = {
	.action = open_seq_action,
	.transition = &open_seq_to_brake,
};

static const struct nnk_stm_state brake = {
	.action = brake_action,
	.transition = &brake_to_unlock,
};

static const struct nnk_stm_state unlock = {
	.action = unlock_action,
	.transition = &unlock_to_brake,
};

static const struct nnk_stm_state parachute = {
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

	// signal init state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_INIT)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led alive 0.25s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_ALIVE, FR_LED_SET, 25)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_opening_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal cone opening state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_CONE_OPENING)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 5s
	mnt.time_out = TIME_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 aero_opening_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal aero opening state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_AERO_OPENING)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 5s
	mnt.time_out = TIME_get() + 5 * TIME_1_SEC;

	// led alive 0.5s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_ALIVE, FR_LED_SET, 50)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led open 0.5s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_OPEN, FR_LED_SET, 50)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 aero_open_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal aero open state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_AERO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_closing_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal cone closing state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_CONE_CLOSING)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 5s
	mnt.time_out = TIME_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_closed_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal cone closed state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_CONE_CLOSED)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 1s
	mnt.time_out = TIME_get() + 1 * TIME_1_SEC;

	// led alive 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_ALIVE, FR_LED_SET, 10)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led open off 0.0s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_OPEN, FR_LED_SET, 0)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 waiting_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal waiting state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_WAITING)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led alive 1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_ALIVE, FR_LED_SET, 100)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 flight_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal flight state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_FLIGHT)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero close
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_CLOSE)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out = flight time
	mnt.time_out = mnt.open_time * TIME_1_SEC / 10 + TIME_get();

	// led alive 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_ALIVE, FR_LED_SET, 10)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 cone_open_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal cone open state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_CONE_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led open 0.1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_OPEN, FR_LED_SET, 10)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 0.1s
	mnt.time_out = TIME_get() + 100 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 braking_action(pt_t* pt, void* args)
{
	frame_t fr;

	PT_BEGIN(pt);

	// signal braking state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_BRAKING)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero open
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OPEN)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// time-out 0.1s
	mnt.time_out = TIME_get() + 100 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 parachute_action(pt_t* pt, void* args)
{
    (void)args;

	frame_t fr;

	PT_BEGIN(pt);

	// signal parachute state
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_STATE, 0, FR_STATE_SET, FR_STATE_PARACHUTE)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_CONE, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, frame_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_MINUT_SERVO_CMD, 0, FR_SERVO_AERO, FR_SERVO_OFF)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	// led open 1s
	PT_WAIT_UNTIL(pt, frame_set_3(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, FR_LED_CMD, 0, FR_LED_OPEN, FR_LED_SET, 100)
			&& OK == FIFO_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


// check cone changings
static PT_THREAD( mnt_check_cone(pt_t* pt) )
{
	enum mnt_event ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, TIME_get() > mnt.sampling_rate);

	// set next sampling period
	mnt.sampling_rate += SAMPLING_PERIOD;

	// read cone state
	u8 cone_state = CONE & _BV(CONE_PIN);

	// check if the cone state has not changed
	if ( cone_state == mnt.cone_state ) {
		PT_RESTART(pt);
	}

	// save new cone state
	mnt.cone_state = cone_state;

	// else generate the correspondig change event
	switch (mnt.cone_state) {
		case CONE_STATE_OPEN:
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_CONE_OPEN) && OK == FIFO_put(&mnt.ev_fifo, &ev) );
			break;

		case CONE_STATE_CLOSED:
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_CONE_CLOSED) && OK == FIFO_put(&mnt.ev_fifo, &ev) );
			break;

		default:
			break;
	}

	PT_RESTART(pt);

	PT_END(pt);
}


// check a time-out has elapsed
static PT_THREAD( mnt_check_time_out(pt_t* pt) )
{
	enum mnt_event ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, TIME_get() > mnt.time_out);

	// prevent any further time-out
	mnt.time_out = TIME_MAX;

	// generate the time-out event
	PT_WAIT_UNTIL(pt, (ev = MNT_EV_TIME_OUT) && OK == FIFO_put(&mnt.ev_fifo, &ev) );

	PT_RESTART(pt);

	PT_END(pt);
}


static void mnt_open_time(frame_t* fr)
{
	switch (fr->argv[0]) {
		case 0x00:
			// save new open time value
			mnt.open_time = fr->argv[1];
			break;

		case 0xff:
			// read open time value
			fr->argv[1] = mnt.open_time;
			break;

		default:
			// bad sub-command
			fr->error = 1;
			break;
	}
}


static PT_THREAD( mnt_check_commands(pt_t* pt) )
{
	enum mnt_event ev;
	u8 swap;

	PT_BEGIN(pt);

	// as long as there are no command
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&mnt.cmds_fifo, &mnt.cmd_fr));

	// silently ignore incoming response
	if ( mnt.cmd_fr.resp == 1 ) {
		dpt_unlock(&mnt.interf);
		PT_RESTART(pt);
	}

	switch (mnt.cmd_fr.cmde) {
		case FR_TAKE_OFF:
			// generate take-off event
			PT_WAIT_UNTIL(pt, (ev = MNT_EV_TAKE_OFF) && OK == FIFO_put(&mnt.ev_fifo, &ev) );
			break;

		case FR_MINUT_TIME_OUT:
			mnt_open_time(&mnt.cmd_fr);
			break;

		case FR_STATE:
			if ( (mnt.cmd_fr.argv[0] == 0x7a) || (mnt.cmd_fr.argv[0] == 0x8b) ) {
				//mnt.state = mnt.cmd_fr.argv[1];
			}

			// don't respond, response will be done by CMN
			PT_RESTART(pt);
			break;

		case FR_APPLI_START:
			mnt.started = 1;

			// don't respond
			PT_RESTART(pt);
			break;

		default:
			// shall never happen
			break;
	}

	// build the response to the current command
	swap = mnt.cmd_fr.orig;
	mnt.cmd_fr.orig = mnt.cmd_fr.dest;
	mnt.cmd_fr.dest = swap;
	mnt.cmd_fr.resp = 1;

	// enqueue it
	PT_WAIT_UNTIL(pt, OK == FIFO_put(&mnt.out_fifo, &mnt.cmd_fr));

	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( mnt_send_frame(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until an outgoing frame is available
	PT_WAIT_UNTIL(pt, OK == FIFO_get(&mnt.out_fifo, &mnt.out_fr));

	// send the frame throught the dispatcher
	dpt_lock(&mnt.interf);

	// some retry may be needed
	PT_WAIT_UNTIL(pt, OK == dpt_tx(&mnt.interf, &mnt.out_fr));

	// release the dispatcher
	dpt_unlock(&mnt.interf);

	// loop back for the next frame to send
	PT_RESTART(pt);
	
	PT_END(pt);
}


// ------------------------------------------
// public functions
//

void mnt_init(void)
{
	// init state machine
	STM_init(&mnt.stm, &init);

	// set the door pins direction
	CONE_DDR &= ~_BV(CONE_PIN);

	// init the cone state with its opposite value to generate the first event
	mnt.cone_state = ~(CONE & _BV(CONE_PIN));

	// init fifoes
	FIFO_init(&mnt.ev_fifo, mnt.ev_buf, NB_EVENTS, sizeof(enum mnt_event));
	FIFO_init(&mnt.cmds_fifo, mnt.cmds_buf, NB_CMDS, sizeof(frame_t));
	FIFO_init(&mnt.out_fifo, mnt.out_buf, NB_OUT_FR, sizeof(frame_t));

	// register to dispatcher
	mnt.interf.channel = 7;
	mnt.interf.cmde_mask = _CM(FR_TAKE_OFF) | _CM(FR_MINUT_TIME_OUT) | _CM(FR_STATE) | _CM(FR_APPLI_START);
	mnt.interf.queue = &mnt.cmds_fifo;
	dpt_register(&mnt.interf);

	// init threads
	PT_INIT(&mnt.pt_chk_time_out);
	PT_INIT(&mnt.pt_chk_cmds);
	PT_INIT(&mnt.pt_out);

	// prevent any time-out
	mnt.time_out = TIME_MAX;
	mnt.sampling_rate = SAMPLING_START;

	// the application start signal shall be received
	mnt.started = 0;
}


void mnt_run(void)
{
	// event sources are :
	//  - take-off detector
	//  - time-out
	//  - door detectors
	//  - frame commands
	//
	//  each generated event is stored in a fifo

	if ( mnt.started ) {
		// check if door has changed
		(void)PT_SCHEDULE(mnt_check_cone(&mnt.pt_chk_cone));

		// check if a time-out has elapsed
		(void)PT_SCHEDULE(mnt_check_time_out(&mnt.pt_chk_time_out));
	}

	// treat each incoming commands
	(void)PT_SCHEDULE(mnt_check_commands(&mnt.pt_chk_cmds));

	if ( mnt.started ) {
		// treat each new event
		enum mnt_event ev;

		// if there is an event
		if ( OK == FIFO_get(&mnt.ev_fifo, &ev) ) {
			// send it to the state machine
			STM_event(&mnt.stm, ev);
		}

		// update state machine
		STM_run(&mnt.stm);
	}

	// send outgoing frame(s) if any
	(void)PT_SCHEDULE(mnt_send_frame(&mnt.pt_out));
}
