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

#define SEPA_DDR			DDRB
#define SEPA				PINB
#define SEPA_PIN			PB3
#define SEPA_STATE_CLOSED	0
#define SEPA_STATE_OPEN		_BV(SEPA_PIN)

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
	struct scalp_dpt_interface interf;	// dispatcher interface

	pt_t pt_chk_sepa;	// checking sepa thread
	pt_t pt_chk_time_out;	// checking time-out thread
	pt_t pt_chk_cmds;	// checking commands thread
	pt_t pt_out;		// sending thread

	struct nnk_stm stm;

	u32 time_out;		// time-out target time
	u32 sampling_rate;	// sampling rate for door changings
	u32 take_off_time_out;	// take-off scan interval time
	u32 door_time_out;	// door scan interval time
	u32 check_time_out;	// state scan interval time

	u8 window_begin_time;	// window begin time [0.0; 25.5] seconds from take-off detection
	u8 window_end_time;	// window end time [0.0; 25.5] seconds from take-off detection

	u8 sepa_state;

	// events fifo
	struct nnk_fifo ev_fifo;
	enum mnt_event ev_buf[NB_EVENTS];

	// incoming commands fifo
	struct nnk_fifo cmds_fifo;
	struct scalp cmds_buf[NB_CMDS];
	struct scalp cmd_fr;

	// outcoming frames fifo
	struct nnk_fifo out_fifo;
	struct scalp out_buf[NB_OUT_FR];

	struct scalp out_fr;		// frame for the sending thread
} mnt;


//----------------------------------------------------------------------------
// forward prototypes
//
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

static u8 action_init(pt_t* pt, void* args);
static u8 action_lock0(pt_t* pt, void* args);
static u8 action_lock1(pt_t* pt, void* args);
static u8 action_lock2(pt_t* pt, void* args);
static u8 action_waiting(pt_t* pt, void* args);
static u8 action_flight(pt_t* pt, void* args);
static u8 action_balistic(pt_t* pt, void* args);
static u8 action_window_begin(pt_t* pt, void* args);
static u8 action_open_seq(pt_t* pt, void* args);
static u8 action_brake(pt_t* pt, void* args);
static u8 action_unlock(pt_t* pt, void* args);
static u8 action_parachute(pt_t* pt, void* args);


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
	.action = action_init,
	.tr = &init_to_lock0,
};

static const struct nnk_stm_state lock0 = {
	.action = action_lock0,
	.tr = &lock0_to_lock1,
};

static const struct nnk_stm_state lock1 = {
	.action = action_lock1,
	.tr = &lock1_to_lock2,
};

static const struct nnk_stm_state lock2 = {
	.action = action_lock2,
	.tr = &lock2_to_waiting,
};

static const struct nnk_stm_state waiting = {
	.action = action_waiting,
	.tr = &waiting_to_flight,
};


static const struct nnk_stm_state flight = {
	.action = action_flight,
	.tr = &flight_to_balistic,
};

static const struct nnk_stm_state balistic = {
	.action = action_balistic,
	.tr = &balistic_to_open_seq,
};

static const struct nnk_stm_state window_begin = {
	.action = action_window_begin,
	.tr = &window_begin_to_open_seq_0,
};

static const struct nnk_stm_state open_seq = {
	.action = action_open_seq,
	.tr = &open_seq_to_brake,
};

static const struct nnk_stm_state brake = {
	.action = action_brake,
	.tr = &brake_to_unlock,
};

static const struct nnk_stm_state unlock = {
	.action = action_unlock,
	.tr = &unlock_to_brake,
};

static const struct nnk_stm_state parachute = {
	.action = action_parachute,
	.tr = NULL,
};


// ------------------------------------------
// private functions
//

static u8 action_init(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal init state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_INIT)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 0.5/2.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 50, 200)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_lock0(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal lock0 state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_LOCK0)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OPEN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero open
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OPEN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 0.0/1.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 0, 100)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led signal 0.1/0.1s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 10, 10)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_lock1(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal lock1 state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_LOCK1)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero close
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_CLOSE)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 1.0/0.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 100, 0)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led signal 0.0/1.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 0, 100)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out 5s
	mnt.time_out = nnk_time_get() + 5 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_lock2(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal lock2 state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_LOCK2)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone close
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_CLOSE)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led signal 1.0/0.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 100, 0)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out 1s
	mnt.time_out = nnk_time_get() + 1 * TIME_1_SEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_waiting(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal waiting state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_WAITING)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 0.4/2.0s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 40, 200)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_flight(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal flight state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_FLIGHT)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led signal 0.1/0.1s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_SIGNAL, SCALP_LED_SET, 10, 10)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out = window begin
	mnt.time_out = mnt.window_begin_time * TIME_1_SEC / 10 + nnk_time_get();

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_balistic(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal balistic state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_BALISTIC)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_window_begin(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal balistic state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_WINDOW_BEGIN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out = window end
	mnt.time_out = mnt.window_end_time * TIME_1_SEC / 10 + nnk_time_get();

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_open_seq(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal open seq state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_OPEN_SEQ)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OPEN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 0.1/0.1s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 10, 10)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out 0.2s
	mnt.time_out = nnk_time_get() + 200 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_brake(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal brake state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_BRAKE)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero open
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OPEN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out 0.2s
	mnt.time_out = nnk_time_get() + 200 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_unlock(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal unlock state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_UNLOCK)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone open
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OPEN)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// time-out 0.2s
	mnt.time_out = nnk_time_get() + 200 * TIME_1_MSEC;

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_parachute(pt_t* pt, void* args)
{
        (void)args;

	struct scalp fr;

	PT_BEGIN(pt);

	// signal parachute state
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_STATE, 0, SCALP_STAT_SET, SCALP_STAT_PARACHUTE)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde cone stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_CONE, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// cmde aero stop
	PT_WAIT_UNTIL(pt, scalp_set_2(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_SERVOCMD, 0, SCALP_SERV_AERO, SCALP_SERV_OFF)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led alive 1.0/2.5s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_ALIVE, SCALP_LED_SET, 100, 250)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	// led signal 1.0/2.5s
	PT_WAIT_UNTIL(pt, scalp_set_4(&fr, DPT_SELF_ADDR, DPT_SELF_ADDR, SCALP_LED, 0, SCALP_LED_SIGNAL, SCALP_LED_SET, 100, 250)
			&& OK == nnk_fifo_put(&mnt.out_fifo, &fr)
	);

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


// check separation changings
static PT_THREAD( mnt_check_sepa(pt_t* pt) )
{
	enum mnt_event ev;

	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, nnk_time_get() > mnt.sampling_rate);

	// set next sampling period
	mnt.sampling_rate += SAMPLING_PERIOD;

	// read sepa state
	u8 sepa_state = SEPA & _BV(SEPA_PIN);

	// check if the sepa state has not changed
	if ( sepa_state == mnt.sepa_state ) {
		PT_RESTART(pt);
	}

	// save new sepa state
	mnt.sepa_state = sepa_state;

	// else generate the correspondig change event
	switch (mnt.sepa_state) {
        case SEPA_STATE_OPEN:
                PT_WAIT_UNTIL(pt, (ev = MNT_EV_SEPA_OPEN) && OK == nnk_fifo_put(&mnt.ev_fifo, &ev) );
                break;

        case SEPA_STATE_CLOSED:
                PT_WAIT_UNTIL(pt, (ev = MNT_EV_SEPA_CLOSED) && OK == nnk_fifo_put(&mnt.ev_fifo, &ev) );
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
	PT_WAIT_UNTIL(pt, nnk_time_get() > mnt.time_out);

	// prevent any further time-out
	mnt.time_out = TIME_MAX;

	// generate the time-out event
	PT_WAIT_UNTIL(pt, (ev = MNT_EV_TIME_OUT) && OK == nnk_fifo_put(&mnt.ev_fifo, &ev) );

	PT_RESTART(pt);

	PT_END(pt);
}


static void mnt_time_out(struct scalp* fr)
{
	switch (fr->argv[0]) {
        case 0x00:
                // save new time values
                mnt.window_begin_time = fr->argv[1];
                mnt.window_end_time = fr->argv[2];
                break;

        case 0xff:
                // read open time value
                fr->argv[1] = mnt.window_begin_time;
                fr->argv[2] = mnt.window_end_time;
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
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mnt.cmds_fifo, &mnt.cmd_fr));

	// silently ignore incoming response
	if ( mnt.cmd_fr.resp == 1 ) {
		scalp_dpt_unlock(&mnt.interf);
		PT_RESTART(pt);
	}

	switch (mnt.cmd_fr.cmde) {
        case SCALP_MINUTTAKEOFF:
                // generate take-off event
                PT_WAIT_UNTIL(pt, (ev = MNT_EV_TAKE_OFF) && OK == nnk_fifo_put(&mnt.ev_fifo, &ev) );
                break;

        case SCALP_MINUTTIMEOUT:
                mnt_time_out(&mnt.cmd_fr);
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
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_put(&mnt.out_fifo, &mnt.cmd_fr));

	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( mnt_send_frame(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until an outgoing frame is available
	PT_WAIT_UNTIL(pt, OK == nnk_fifo_get(&mnt.out_fifo, &mnt.out_fr));

	// send the frame throught the dispatcher
	scalp_dpt_lock(&mnt.interf);

	// some retry may be needed
	PT_WAIT_UNTIL(pt, OK == scalp_dpt_tx(&mnt.interf, &mnt.out_fr));

	// release the dispatcher
	scalp_dpt_unlock(&mnt.interf);

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
	nnk_stm_init(&mnt.stm, &init);

	// set the door pins direction
	SEPA_DDR &= ~_BV(SEPA_PIN);

	// init the sepa state with its opposite value to generate the first event
	mnt.sepa_state = ~(SEPA & _BV(SEPA_PIN));

	// init fifoes
	nnk_fifo_init(&mnt.ev_fifo, mnt.ev_buf, NB_EVENTS, sizeof(enum mnt_event));
	nnk_fifo_init(&mnt.cmds_fifo, mnt.cmds_buf, NB_CMDS, sizeof(struct scalp));
	nnk_fifo_init(&mnt.out_fifo, mnt.out_buf, NB_OUT_FR, sizeof(struct scalp));

	// register to dispatcher
	mnt.interf.channel = 7;
	mnt.interf.cmde_mask = _CM(SCALP_MINUTTIMEOUT)
                                | _CM(SCALP_MPUACC);
	mnt.interf.queue = &mnt.cmds_fifo;
	scalp_dpt_register(&mnt.interf);

	// init threads
	PT_INIT(&mnt.pt_chk_time_out);
	PT_INIT(&mnt.pt_chk_cmds);
	PT_INIT(&mnt.pt_out);

	// prevent any time-out
	mnt.time_out = TIME_MAX;
	mnt.sampling_rate = SAMPLING_START;
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

        // check if separation has changed
        (void)PT_SCHEDULE(mnt_check_sepa(&mnt.pt_chk_sepa));

        // check if a time-out has elapsed
        (void)PT_SCHEDULE(mnt_check_time_out(&mnt.pt_chk_time_out));

	// treat each incoming commands
	(void)PT_SCHEDULE(mnt_check_commands(&mnt.pt_chk_cmds));

        // treat each new event
        enum mnt_event ev;

        // if there is an event
        if ( OK == nnk_fifo_get(&mnt.ev_fifo, &ev) ) {
                // send it to the state machine
                nnk_stm_event(&mnt.stm, ev);
        }

        // update state machine
        nnk_stm_run(&mnt.stm);

	// send outgoing frame(s) if any
	(void)PT_SCHEDULE(mnt_send_frame(&mnt.pt_out));
}
