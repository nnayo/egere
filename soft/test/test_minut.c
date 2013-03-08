#include "../minut.h"


#include <string.h>

#include "test.h"
#include "stubs.h"

#include "avr/io.h"


// ------------------------------------------------
// tests suite
//

static void test_init(void)
{
	// check correct initialization
}


static void test_start_cone_closed(void)
{
	// run the test until in waiting state with the cone closed

	// cone is closed
	PORTB = _BV(PB2);

	MNT_run();
	TEST_check("FIFO_put #0");
	TEST_check("FIFO_get #1");
	TEST_check("FIFO_get #0");
	TEST_check("FIFO_get #2");

	// to this point, the state is 'cone_opening'
	
	// run 50 times to trigger the 5s time-out
	for (int i = 0; i < 50; i++) {
		MNT_run();
		TEST_check("FIFO_get #1");
		TEST_check("FIFO_get #0");
		TEST_check("FIFO_get #2");
	}

	// time-out is triggered thus an event is posted
	MNT_run();
	TEST_check("FIFO_put #0");
	TEST_check("FIFO_get #1");
	TEST_check("FIFO_get #0");
	TEST_check("FIFO_get #2");

	// to this point, the state is 'cone_closed'

	// run 10 times to trigger the 1s time-out
	for (int i = 0; i < 10; i++) {
		MNT_run();
		TEST_check("FIFO_get #1");
		TEST_check("FIFO_get #0");
		TEST_check("FIFO_get #2");
	}

	// time-out is triggered thus an event is posted
	MNT_run();
	TEST_check("FIFO_put #0");
	TEST_check("FIFO_get #1");
	TEST_check("FIFO_get #0");
	TEST_check("FIFO_get #2");

}

#if 0
static int test_state(void)
{
	char* res[] = {
		"DPT_init",
		"DPT_register : channel #3, cmde [0x0001c000]",
		"DPT_lock : channel #3",	// set/get state and bus
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x01, .cmde = 0x8e, .argv = 0x7a 77 33 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x02, .cmde = 0x8e, .argv = 0x00 77 33 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",	// set/get state
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x03, .cmde = 0x8e, .argv = 0x8b 88 44 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x04, .cmde = 0x8e, .argv = 0x00 88 33 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",	// set/get bus
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x05, .cmde = 0x8e, .argv = 0x9c 99 55 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x06, .cmde = 0x8e, .argv = 0x00 88 55 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",	// invalid set/get
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x07, .cmde = 0xce, .argv = 0xad aa 66 00 }",
		"DPT_unlock : channel #3",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x08, .cmde = 0x8e, .argv = 0x00 88 55 00 }",
		"DPT_unlock : channel #3",
		NULL
	};

	dpt_frame_t fr_get = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x00, 0x00, 0x00, 0x00 }
	};

	dpt_frame_t fr_set_state_n_bus = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x7a, 0x77, 0x33, 0x00 }
	};

	dpt_frame_t fr_set_state = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x8b, 0x88, 0x44, 0x00 }
	};

	dpt_frame_t fr_set_bus = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x9c, 0x99, 0x55, 0x00 }
	};

	dpt_frame_t fr_invalid_set = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0xad, 0xaa, 0x66, 0x00 }
	};


	DPT.rx(&fr_set_state_n_bus);

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_get);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_set_state);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_get);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_set_bus);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_get);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_invalid_set);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	DPT.rx(&fr_get);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	return TEST_check(res);
}


static int test_get_time(void)
{
	char* res[] = {
		"DPT_init",
		"DPT_register : channel #3, cmde [0x0001c000]",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x01, .cmde = 0x8f, .argv = 0x12 34 56 78 }",
		"DPT_unlock : channel #3",
		NULL
	};

	dpt_frame_t fr = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_TIME_GET,
		.argv = { 0x00, 0x00, 0x00, 0x00 }
	};


	TIME.t = 0x12345678 - TIME_1_MSEC;
	DPT.rx(&fr);
	CMN_run();

	DPT.tx_ret = OK;
	CMN_run();

	return TEST_check(res);
}


static int test_mux_cmde(void)
{
	char* res[] = {
		"DPT_init",
		"DPT_register : channel #3, cmde [0x0001c000]",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x01, .cmde = 0x90, .argv = 0xff 00 00 00 }",
		"DPT_unlock : channel #3",
		"PORTA = 0x00",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x02, .cmde = 0x90, .argv = 0x00 00 00 00 }",
		"DPT_unlock : channel #3",
		"PORTA = 0xf8",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x03, .cmde = 0xd0, .argv = 0x55 00 00 00 }",
		"DPT_unlock : channel #3",
		"PORTA = 0x00",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x04, .cmde = 0xd0, .argv = 0x55 00 00 00 }",
		"DPT_unlock : channel #3",
		"PORTA = 0xf8",
		NULL
	};

	dpt_frame_t fr_mux_on = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_MUX_POWER,
		.argv = { 0xff, 0x00, 0x00, 0x00 }
	};

	dpt_frame_t fr_mux_off = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_MUX_POWER,
		.argv = { 0x00, 0x00, 0x00, 0x00 }
	};

	dpt_frame_t fr_mux_error = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_MUX_POWER,
		.argv = { 0x55, 0x00, 0x00, 0x00 }
	};


	DPT.tx_ret = OK;

	PORTA = 0x00;
	DPT.rx(&fr_mux_on);
	CMN_run();
	CMN_run();
	TEST_log("PORTA = 0x%02x", PORTA);

	PORTA = 0xff;
	DPT.rx(&fr_mux_off);
	CMN_run();
	CMN_run();
	TEST_log("PORTA = 0x%02x", PORTA);

	PORTA = 0x00;
	DPT.rx(&fr_mux_error);
	CMN_run();
	CMN_run();
	TEST_log("PORTA = 0x%02x", PORTA);

	PORTA = 0xff;
	DPT.rx(&fr_mux_error);
	CMN_run();
	CMN_run();
	TEST_log("PORTA = 0x%02x", PORTA);

	return TEST_check(res);
}


static int test_leds(void)
{
	int i;

	char* res[] = {
		"DPT_init",
		"DPT_register : channel #3, cmde [0x0001c000]",
		"PORTA = 0x00",
		"DDRA = 0x87",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x01, .cmde = 0x8e, .argv = 0x8b 01 00 00 }",
		"DPT_unlock : channel #3",
		"TIME = 5000",
		"PORTA = 0x00",
		"TIME = 5010",
		"PORTA = 0x00",
		"TIME = 10010",
		"PORTA = 0x00",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x02, .cmde = 0x8e, .argv = 0x8b 02 00 00 }",
		"DPT_unlock : channel #3",
		"TIME = 12500",
		"PORTA = 0x00",
		"TIME = 12510",
		"PORTA = 0x02",
		"TIME = 14990",
		"PORTA = 0x02",
		"TIME = 15000",
		"PORTA = 0x00",
		"DPT_lock : channel #3",
		"DPT_tx : channel #3, fr = { .dest = 0x08, .orig = 0x09, .t_id = 0x03, .cmde = 0x8e, .argv = 0x8b 05 00 00 }",
		"DPT_unlock : channel #3",
		"TIME = 16250",
		"PORTA = 0x01",
		"TIME = 16260",
		"PORTA = 0x01",
		"TIME = 17490",
		"PORTA = 0x01",
		"TIME = 17500",
		"PORTA = 0x01",
		NULL
	};

	dpt_frame_t fr_wait_take_off = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x8b, 0x01, 0x00, 0x00 }
	};

	dpt_frame_t fr_wait_take_off_conf = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x8b, 0x02, 0x00, 0x00 }
	};

	dpt_frame_t fr_recovery = {
		.dest = 0x09,
		.orig = 0x08,
		.cmde = FR_STATE,
		.argv = { 0x8b, 0x05, 0x00, 0x00 }
	};


	DPT.tx_ret = OK;

	TEST_log("PORTA = 0x%02x", PORTA);
	TEST_log("DDRA = 0x%02x", DDRA);

	// WAIT_TAKE_OFF green period = 1 s
	DPT.rx(&fr_wait_take_off);
	for ( i = 0; i < 500; i++ )
		CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	// transition off -> ON
	CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	for ( i = 0; i < 500; i++ )
		CMN_run();
	// transition ON -> off
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);


	// WAIT_TAKE_OFF_CONF orange period = 500 ms
	DPT.rx(&fr_wait_take_off_conf);

	// transition ON -> off
	for ( i = 0; i < 250 - 1; i++ )
		CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	// transition off -> ON
	for ( i = 0; i < 250 - 2; i++ )
		CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);


	// RECOVERY red period = 250 ms
	DPT.rx(&fr_recovery);

	// transition ON -> off
	for ( i = 0; i < 125; i++ )
		CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	// transition off -> ON
	for ( i = 0; i < 125 - 2; i++ )
		CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	CMN_run();
	TEST_log("TIME = %d", TIME.t);
	TEST_log("PORTA = 0x%02x", PORTA);

	return TEST_check(res);
}
#endif


static void start(void)
{
#if 0
	frame_t fr_reset = {
		.dest = 0x01,
		.orig = 0x01,
		.cmde = FR_STATE,
		.argv = { 0x7a, 0x00, 0x00, 0xff }
	};

	memcpy(&EEP.container, &fr_reset, sizeof(fr_reset));
#endif

	STUB_reset();

	PORTB = 0x00;
	DDRB = 0x00;

	MNT_init();
	TEST_check("FIFO_init #0: nb elem = 5, elem size = 1");
	TEST_check("FIFO_init #1: nb elem = 3, elem size = 11");
	TEST_check("FIFO_init #2: nb elem = 4, elem size = 11");
	TEST_check("DPT_register: chan #4, mask 0x00710000");

	TEST_log("PORTB = 0x%02x", PORTB);
	TEST_log("DDRB = 0x%02x", DDRB);
	TEST_check("PORTB = 0x00");
	TEST_check("DDRB = 0x00");

}


static void stop(void)
{
}


int main(int argc, char* argv[])
{
	static TEST_list_t l = {
		.start = start,
		.stop = stop,
		.list = {
			{ test_init,			"init" },
			{ test_start_cone_closed,	"test starting with closed cone" },
#if 0
			{ test_state,			"test_state" },
			{ test_get_time,		"test_get_time" },
			{ test_mux_cmde,		"test_mux_cmde" },
			{ test_leds,			"test_leds" },
#endif
			{ NULL,				NULL }
		},
	};

	TEST_run(&l, argv[1]);

	return 0;
}
