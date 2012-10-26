#include "stubs.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>	// memcpy()

#include "test.h"

#include "type_def.h"


#include "dispatcher.h"
#include "dna.h"

#include "utils/fifo.h"
#include "utils/time.h"


// ------------------------------
// stubbing defines
//

#define MAX_ROUTES			2
#define NB_IN_FRAMES			(MAX_ROUTES / 2)
#define NB_OUT_FRAMES			(MAX_ROUTES / 2)
#define NB_APPLI_FRAMES			1

#define NB_FIFOES			3


// ------------------------------
// stubbing variables
//

static struct {
	u32 t;
	u32 incr;
} TIME;


static struct {
	dpt_interface_t* channels[DPT_CHAN_NB];	// available channels
	u64 lock;								// lock bitfield

	pt_t appli_pt;							// appli thread
	fifo_t appli_fifo;
	frame_t appli_buf[NB_APPLI_FRAMES];
	frame_t appli;

	pt_t in_pt;								// in thread
	fifo_t in_fifo;
	frame_t in_buf[NB_IN_FRAMES];
	frame_t in;	

	pt_t out_pt;							// out thread
	fifo_t out_fifo;
	frame_t out_buf[NB_OUT_FRAMES];
	frame_t out;
	frame_t hard;
	volatile u8 hard_fini;

	u8 sl_addr;								// own I2C slave address
	u32 time_out;							// tx time-out time
	u8 t_id;								// current transaction id value
} DPT;

static struct {
	int nb;
	fifo_t* list[NB_FIFOES];
} FIFO;

// ------------------------------
// stubbed variables
//

u8 PORTA;
u8 DDRA;

u8 PORTB;
u8 PINB;
u8 DDRB;

u8 PIND;
u8 DDRD;


// ------------------------------
// stubbing functions
//

void STUB_reset(void)
{
	for (int i = 0; i < FIFO.nb; i++) {
		FIFO.list[i] = NULL;
	}
	FIFO.nb = 0;
}


// ------------------------------
// stubbed functions
//


u32 TIME_get(void)
{
	TIME.t += 50 * TIME_1_MSEC;

	return TIME.t;
}


void TIME_incr(void)
{
	TIME.t += TIME.incr;
}


u32 TIME_get_incr(void)
{
	return TIME.incr;
}


void FIFO_init(fifo_t* f, void* buf, u16 nb_elem, u16 elem_size)
{
	for (int i = 0; i <= FIFO.nb; i++) {
		if ( FIFO.list[i] == NULL ) {
			FIFO.list[i] = f;

			TEST_log("FIFO_init #%d: nb elem = %d, elem size = %d", i, nb_elem, elem_size);
			FIFO.nb++;
			break;
		}
	}

	// set the internals thanks to the provided data
	f->lng = nb_elem;
	f->nb = 0;
	f->elem_size = elem_size;
	f->out = f->in = f->donnees = buf;
}


u8 FIFO_put(fifo_t* f, void* elem)
{
	for (int i = 0; i < FIFO.nb; i++) {
		if ( FIFO.list[i] == f ) {
			TEST_log("FIFO_put #%d", i);
			break;
		}
	}

	// if there's at least a free place
	if (f->nb < f->lng) {
		// add the new element
		memcpy(f->in, elem, f->elem_size);
		f->in += f->elem_size;

		// loop back at the end
		if (f->in >= f->donnees + f->lng * f->elem_size)
			f->in = f->donnees;
		f->nb++;

		return OK;
	} else {
		return KO;
	}
}


u8 FIFO_get(fifo_t* f, void* elem)
{
	for (int i = 0; i < FIFO.nb; i++) {
		if ( FIFO.list[i] == f ) {
			TEST_log("FIFO_get #%d", i);
			break;
		}
	}

	// if there's no element, quit
	if (f->nb == 0) {
		return KO;
	}

	// get the element
	memcpy(elem, f->out, f->elem_size);
	f->nb--;

	// set the extraction pointer to the next position
	f->out += f->elem_size;
	if (f->out >= f->donnees + f->lng * f->elem_size)
		f->out = f->donnees;

	return OK;
}


void DPT_register(dpt_interface_t* interf)
{
	TEST_log("DPT_register: chan #%d, mask 0x%08x", interf->channel, interf->cmde_mask);
}


void DPT_lock(dpt_interface_t* interf)
{
	TEST_log("DPT_lock: chan #%d, mask 0x%08x", interf->channel, interf->cmde_mask);

	// set the lock bit associated to the channel
	DPT.lock |= 1 << interf->channel;
}


void DPT_unlock(dpt_interface_t* interf)
{
	TEST_log("DPT_unlock: chan #%d, mask 0x%08x", interf->channel, interf->cmde_mask);

	// reset the lock bit associated to the channel
	DPT.lock &= ~(1 << interf->channel);
}


u8 DPT_tx(dpt_interface_t* interf, frame_t* fr)
{
	TEST_log("DPT_tx: chan #%d", interf->channel);

	u8 i;

	// if the tx is locked by a channel of higher priority
	for ( i = 0; (i < DPT_CHAN_NB) && (i < interf->channel); i++ ) {
		if ( DPT.lock & (1 << i) ) {
			// the sender shall retry
			// so return KO
			return KO;
		}
	}

	// if the sender didn't lock the channel
	if ( !(DPT.lock & (1 << interf->channel)) ) {
		// it can't send the frame
		return KO;
	}

	// if the frame is not a response
	if ( !fr->resp ) {
		// increment transaction id
		DPT.t_id++;

		// and set it in the current frame
		fr->t_id = DPT.t_id;
	}

	// try to enqueue the frame
	return FIFO_put(&DPT.appli_fifo, fr);
}


dna_list_t* DNA_list(u8* nb_is, u8* nb_bs)
{
	TEST_raise();

	return NULL;
}


#include "utils/state_machine.c"
