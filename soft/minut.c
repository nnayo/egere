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

#include <string.h>  // memset()


//for debug
#define static


// ------------------------------------------
// private definitions
//

#define NB_EVENTS	3
#define NB_CMDS		3
#define NB_OUT_FR	4

#define SEPA_DDR			DDRB
#define SEPA				PINB
#define SEPA_PIN			PB3
#define SEPA_STATE_CLOSED	0
#define SEPA_STATE_OPEN		_BV(SEPA_PIN)

#define SAMPLING_START		(2 * TIME_1_SEC)
#define SAMPLING_PERIOD		(100 * TIME_1_MSEC)

#define AVERAGING_NB  4


// ------------------------------------------
// private types
//

enum mnt_event {
	MNT_EV_NONE,
	MNT_EV_MPU_READY,
	MNT_EV_SEPA_OPEN,
	MNT_EV_SEPA_CLOSED,
	MNT_EV_TIME_OUT,
	MNT_EV_TAKE_OFF,
	MNT_EV_BALISTIC_UP,
	MNT_EV_BALISTIC_DOWN,
	MNT_EV_LATERAL_ACC_TRIGGER,
};


union acc {
        s16 acc;
        struct {
                u8 lo;
                u8 hi;
        };
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

        struct {
                s16 take_off;   // [0.00G:25.5G]
                struct {
                        s16 lng;   // [0.00G:25.5G]
                        s32 lat;   // [0.00GG:2.55GG]
                } apogee;
        } thresholds;

	u8 sepa_state;          // separation flag

        struct {
                s16 lng;
                s32 lat;
                struct {
                        s16 lng[AVERAGING_NB];
                        s32 lat[AVERAGING_NB];
                        u8 idx;
                } average;
        } acc;

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
static const struct nnk_stm_transition init_to_ready;
static const struct nnk_stm_transition ready_to_lock0;
static const struct nnk_stm_transition ready_to_waiting;
static const struct nnk_stm_transition lock0_to_lock1;
static const struct nnk_stm_transition lock1_to_lock2;
static const struct nnk_stm_transition lock2_to_waiting;
static const struct nnk_stm_transition waiting_to_thrusting;
static const struct nnk_stm_transition thrusting_to_balistic;
static const struct nnk_stm_transition thrusting_to_detection;
static const struct nnk_stm_transition balistic_to_detection;
static const struct nnk_stm_transition detection_to_open_seq_0;
static const struct nnk_stm_transition detection_to_open_seq_1;
static const struct nnk_stm_transition detection_to_open_seq_2;
static const struct nnk_stm_transition open_seq_to_brake;
static const struct nnk_stm_transition brake_to_unlock;
static const struct nnk_stm_transition unlock_to_brake;
static const struct nnk_stm_transition unlock_to_parachute;


static const struct nnk_stm_state init;
static const struct nnk_stm_state ready;
static const struct nnk_stm_state lock0;
static const struct nnk_stm_state lock1;
static const struct nnk_stm_state lock2;
static const struct nnk_stm_state waiting;
static const struct nnk_stm_state thrusting;
static const struct nnk_stm_state balistic;
static const struct nnk_stm_state detection;
static const struct nnk_stm_state open_seq;
static const struct nnk_stm_state brake;
static const struct nnk_stm_state unlock;
static const struct nnk_stm_state parachute;

static u8 action_init(pt_t* pt, void* args);
static u8 action_ready(pt_t* pt, void* args);
static u8 action_lock0(pt_t* pt, void* args);
static u8 action_lock1(pt_t* pt, void* args);
static u8 action_lock2(pt_t* pt, void* args);
static u8 action_waiting(pt_t* pt, void* args);
static u8 action_thrusting(pt_t* pt, void* args);
static u8 action_balistic(pt_t* pt, void* args);
static u8 action_detection(pt_t* pt, void* args);
static u8 action_open_seq(pt_t* pt, void* args);
static u8 action_brake(pt_t* pt, void* args);
static u8 action_unlock(pt_t* pt, void* args);
static u8 action_parachute(pt_t* pt, void* args);


//----------------------------------------------------------------------------
// transitions
//
static const struct nnk_stm_transition init_to_ready = {
    .ev = MNT_EV_MPU_READY,
    .st = &ready,
    .tr = &init_to_ready,
};

static const struct nnk_stm_transition ready_to_lock0 = {
    .ev = MNT_EV_SEPA_OPEN,
    .st = &lock0,
    .tr = &ready_to_waiting,
};

static const struct nnk_stm_transition ready_to_waiting = {
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

static const struct nnk_stm_transition waiting_to_thrusting = {
    .ev = MNT_EV_TAKE_OFF,
    .st = &thrusting,
    .tr = NULL,
};

static const struct nnk_stm_transition thrusting_to_balistic = {
    .ev = MNT_EV_BALISTIC_UP,
    .st = &balistic,
    .tr = &thrusting_to_detection,
};

static const struct nnk_stm_transition thrusting_to_detection = {
    .ev = MNT_EV_TIME_OUT,
    .st = &detection,
    .tr = NULL,
};

static const struct nnk_stm_transition balistic_to_detection = {
    .ev = MNT_EV_TIME_OUT,
    .st = &detection,
    .tr = NULL,
};

static const struct nnk_stm_transition detection_to_open_seq_0 = {
    .ev = MNT_EV_TIME_OUT,
    .st = &open_seq,
    .tr = &detection_to_open_seq_1,
};

static const struct nnk_stm_transition detection_to_open_seq_1 = {
    .ev = MNT_EV_BALISTIC_DOWN,
    .st = &open_seq,
    .tr = &detection_to_open_seq_2,
};

static const struct nnk_stm_transition detection_to_open_seq_2 = {
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
	.tr = &init_to_ready,
};

static const struct nnk_stm_state ready = {
	.action = action_ready,
	.tr = &ready_to_lock0,
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
	.tr = &waiting_to_thrusting,
};


static const struct nnk_stm_state thrusting = {
	.action = action_thrusting,
	.tr = &thrusting_to_balistic,
};

static const struct nnk_stm_state balistic = {
	.action = action_balistic,
	.tr = &balistic_to_detection,
};

static const struct nnk_stm_state detection = {
	.action = action_detection,
	.tr = &detection_to_open_seq_0,
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

static u8 mnt_signal_event(enum mnt_event ev)
{
        struct scalp sclp;

        return scalp_set_1(&sclp, SCALP_DPT_BROADCAST_ADDR, SCALP_DPT_SELF_ADDR, SCALP_MINUTEVENT, ev)
                && nnk_fifo_put(&mnt.out_fifo, &sclp);
}


static u8 action_container_send(u8 cont_idx)
{
	struct scalp fr;

        // build the container
	scalp_set_4(&fr, SCALP_DPT_SELF_ADDR, SCALP_DPT_SELF_ADDR, SCALP_CONTAINER,
                        0, 0, cont_idx, SCALP_CONT_PRE_DEF_STORAGE);

        // try to queue it
        return nnk_fifo_put(&mnt.out_fifo, &fr);
}


static u8 action_init(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(1));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_ready(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(2));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_lock0(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(3));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_lock1(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(4));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_lock2(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(5));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_waiting(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(6));

        // wait take-off
	PT_YIELD_WHILE(pt, mnt.acc.lng < mnt.thresholds.take_off);

        // signal take-off to other components and to self
        // this will generate the event and thus the transition
        PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_TAKE_OFF));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_thrusting(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(7));

        // wait end of thrust
	PT_YIELD_WHILE(pt, mnt.acc.lng > 0);

        // signal balistic up
        PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_BALISTIC_UP));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_balistic(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(8));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}

static u8 action_detection(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(9));

        // wait falling down detection or apogee
	PT_YIELD_UNTIL(pt, mnt.acc.lng > mnt.thresholds.apogee.lng
                        || mnt.acc.lat > mnt.thresholds.apogee.lat);

        // signal balistic down
        if (mnt.acc.lng > mnt.thresholds.apogee.lng)
                PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_BALISTIC_DOWN));

        // signal lateral acceleration trigger
        if (mnt.acc.lat > mnt.thresholds.apogee.lat)
                PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_LAT_ACC_TRIG));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_open_seq(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(10));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_brake(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(11));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_unlock(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(12));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


static u8 action_parachute(pt_t* pt, void* args)
{
        (void)args;

	PT_BEGIN(pt);

	// execute PRE_DEF container scalp
	PT_WAIT_UNTIL(pt, action_container_send(13));

	PT_YIELD_WHILE(pt, OK);

	PT_END(pt);
}


// check separation changings
static PT_THREAD( mnt_check_sepa(pt_t* pt) )
{
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
                PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_SEPA_OPEN));
                break;

        case SEPA_STATE_CLOSED:
                PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_SEPA_CLOSED));
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
	PT_BEGIN(pt);

	// if current time is higher than the time-out target time
	PT_WAIT_UNTIL(pt, nnk_time_get() > mnt.time_out);

	// prevent any further time-out
	mnt.time_out = TIME_MAX;

	// generate the time-out event
	PT_WAIT_UNTIL(pt, mnt_signal_event(SCALP_MINU_EV_TIME_OUT));

	PT_RESTART(pt);

	PT_END(pt);
}


// convert scalp event to state machine event
static u8 mnt_event(struct scalp* sclp)
{
        enum mnt_event ev;

        switch (sclp->argv[0]) {
        case SCALP_MINU_EV_MPU_READY:
        case SCALP_MINU_EV_SEPA_OPEN:
        case SCALP_MINU_EV_SEPA_CLOSED:
        case SCALP_MINU_EV_TIME_OUT:
        case SCALP_MINU_EV_TAKE_OFF:
        case SCALP_MINU_EV_BALISTIC_UP:
        case SCALP_MINU_EV_BALISTIC_DOWN:
        case SCALP_MINU_EV_LAT_ACC_TRIG:
                ev = (enum mnt_event)sclp->argv[0];
                break;

        default:
                ev = MNT_EV_NONE;
                break;
        }

        return nnk_fifo_put(&mnt.ev_fifo, &ev);
}


// set timeout from scalp
static void mnt_time_out(struct scalp* fr)
{
	switch (fr->argv[0]) {
        case SCALP_MINU_SET:
                // set time-out relative to current time
                mnt.time_out = nnk_time_get() + fr->argv[1] * TIME_1_SEC / 10;
                break;

        case SCALP_MINU_INF:
                // set time-out to infinite
                mnt.time_out = TIME_MAX;
                break;

        default:
                // bad sub-command
                fr->error = 1;
                break;
	}
}


// save configuration thresholds
static void mnt_threshold(struct scalp* sclp)
{
        // TODO : thresholds scaled in respect with the MPU6050 range settings
        // acceleration are on 16-bit for a range [-16G:16G]
        // thus the scale factor is 65536 / 32
        // and the unit in the scalp is 0.1G for long and 0.01G for lat
        s32 scale;

        scale = (s32)sclp->argv[0];
        mnt.thresholds.take_off = scale * (65536 / 32) / 10;

        scale = (s32)sclp->argv[1];
        mnt.thresholds.apogee.lng = scale * (65536 / 32) / 10;

        scale = (s32)sclp->argv[2];
        mnt.thresholds.apogee.lat = scale * (65536 / 32) * (65536 / 32) / 100;
}


// average the accelerations
static void mnt_acc_compute(struct scalp* sclp)
{
        // extract accelerations then compute the averages
        union acc lng;

        lng.hi = sclp->argv[4];
        lng.lo = sclp->argv[5];

        mnt.acc.lng -= mnt.acc.average.lng[mnt.acc.average.idx];
        mnt.acc.average.lng[mnt.acc.average.idx] = lng.acc / AVERAGING_NB;
        mnt.acc.lng += mnt.acc.average.lng[mnt.acc.average.idx];

        union acc lat0;
        union acc lat1;

        lat0.hi = sclp->argv[0];
        lat0.lo = sclp->argv[1];
        lat1.hi = sclp->argv[2];
        lat1.lo = sclp->argv[3];

        s32 lat_acc = (s32)lat0.acc * lat0.acc + (s32)lat1.acc * lat1.acc;

        mnt.acc.lat -= mnt.acc.average.lat[mnt.acc.average.idx];
        mnt.acc.average.lat[mnt.acc.average.idx] = lat_acc / AVERAGING_NB;
        mnt.acc.lat += mnt.acc.average.lat[mnt.acc.average.idx];

        // update average insertion/extraction index
        mnt.acc.average.idx++;
        if (mnt.acc.average.idx >= AVERAGING_NB)
                mnt.acc.average.idx = 0;
}


static PT_THREAD( mnt_check_commands(pt_t* pt) )
{
	u8 swap;

	PT_BEGIN(pt);

	// as long as there are no command
	PT_WAIT_UNTIL(pt, nnk_fifo_get(&mnt.cmds_fifo, &mnt.cmd_fr));

	// silently ignore incoming response
	if ( mnt.cmd_fr.resp == 1 )
		PT_RESTART(pt);

	switch (mnt.cmd_fr.cmde) {
        case SCALP_MINUTEVENT:
                // translate external event to internal
                PT_WAIT_UNTIL(pt, mnt_event(&mnt.cmd_fr));
                break;

        case SCALP_MINUTTIMEOUT:
                mnt_time_out(&mnt.cmd_fr);
                break;

        case SCALP_MINUTTAKEOFFTHRES:
                mnt_threshold(&mnt.cmd_fr);
                break;

        case SCALP_MPUACC:
                mnt_acc_compute(&mnt.cmd_fr);
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
	PT_WAIT_UNTIL(pt, nnk_fifo_put(&mnt.out_fifo, &mnt.cmd_fr));

	PT_RESTART(pt);

	PT_END(pt);
}


static PT_THREAD( mnt_send_frame(pt_t* pt) )
{
	PT_BEGIN(pt);

	// wait until an outgoing frame is available
	PT_WAIT_UNTIL(pt, nnk_fifo_get(&mnt.out_fifo, &mnt.out_fr));

	// send the frame throught the dispatcher
	scalp_dpt_lock(&mnt.interf);

	// some retry may be needed
	PT_WAIT_UNTIL(pt, scalp_dpt_tx(&mnt.interf, &mnt.out_fr));

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
	mnt.interf.cmde_mask = SCALP_DPT_CM(SCALP_MINUTTIMEOUT)
                             | SCALP_DPT_CM(SCALP_MINUTEVENT)
                             | SCALP_DPT_CM(SCALP_MINUTTAKEOFFTHRES)
                             | SCALP_DPT_CM(SCALP_MPUACC);
	mnt.interf.queue = &mnt.cmds_fifo;
	scalp_dpt_register(&mnt.interf);

	// init threads
	PT_INIT(&mnt.pt_chk_time_out);
	PT_INIT(&mnt.pt_chk_cmds);
	PT_INIT(&mnt.pt_out);

	// prevent any time-out
	mnt.time_out = TIME_MAX;
	mnt.sampling_rate = SAMPLING_START;

        memset(&mnt.acc, 0, sizeof(mnt.acc));

        // prevent any false detection while the thresholds are not set
        mnt.thresholds.take_off = (s16)0x7fff;
        mnt.thresholds.apogee.lng = (s16)0x7fff;
        mnt.thresholds.apogee.lat = (s32)0x7fffffff;
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
        if (nnk_fifo_get(&mnt.ev_fifo, &ev)) {
                // send it to the state machine
                nnk_stm_event(&mnt.stm, ev);
        }

        // update state machine
        nnk_stm_run(&mnt.stm);

	// send outgoing frame(s) if any
	(void)PT_SCHEDULE(mnt_send_frame(&mnt.pt_out));
}
