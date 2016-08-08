#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sc18is600.h"
#include "mpu6050.h"

#include "sim_avr.h"
#include "avr_twi.h"
#include "sim_elf.h"
#include "sim_gdb.h"
#include "sim_vcd_file.h"


static avr_t* avr_setup(const char* fname, char* vcd_filename, int gdb, int gdb_port)
{
	printf("loading %s...\n", fname);

	elf_firmware_t f;
	elf_read_firmware(fname, &f);
	printf("firmware %s f=%d mmcu=%s\n", fname, (int)f.frequency, f.mmcu);
	strcpy(f.tracename, vcd_filename);

	avr_t * avr = NULL;
	avr = avr_make_mcu_by_name(f.mmcu);
	if (!avr) {
		fprintf(stderr, "AVR '%s' not known\n", f.mmcu);
		exit(1);
	}
	strcpy(avr->tag_name, fname);
	avr->log = 3;
	avr_init(avr);
	avr_load_firmware(avr, &f);

	if (gdb) {
		avr->gdb_port = gdb_port;
		avr->state = cpu_Stopped;
	}
	avr_gdb_init(avr);
	avr->log = 3;

	return avr;
}


int main(int argc, char* argv[])
{
	int debug = 0;

	if (argc == 2 && 0 == strncmp(argv[1], "-d", strlen("-d"))) {
		debug = 1;
	}

	// set every core
	avr_t* minut = avr_setup("minut.elf", "minut.vcd", debug, 7000);

	// all cores are to be connected on I2C bus
	avr_t* cores[] = {
		minut,
	};

	// connect minut to SC18IS600 bridge and bridge to MPU-6050
	struct sc18is600_t* sc18;

	sc18 = sc18is600_alloc(minut);
	mpu6050_alloc(minut, 0x68, sc18);

	printf( "\negere simulation launched\n");


	avr_cycle_count_t common_cycle = 0;
	avr_cycle_count_t min_cycle = 0;
#define DISPLAY_THRESHOLD	20*16e6
	avr_cycle_count_t common_cycle_display_trigger = DISPLAY_THRESHOLD;
	int state;

	while (1) {
		// avr->cycle fields of all cores shall be kept as close as possible
		for (unsigned int i = 0; i < sizeof(cores) / sizeof(cores[0]); i++) {
			// if the core is in advance, don't run it
			if (cores[i]->cycle > common_cycle)
				continue;

			// run the core for 1 cycle and check if in error
			state = avr_run(cores[i]);
			if ((state == cpu_Done) || (state == cpu_Crashed)) {
				printf("core #%d exits on error!\nquitting\n", i);
				goto exit;
			}
		}

		// update common cycle
		min_cycle = cores[0]->cycle;
		for (unsigned int i = 1; i < sizeof(cores) / sizeof(cores[0]); i++) {
			if (cores[i]->cycle < min_cycle) {
				min_cycle = cores[i]->cycle;
			}
		}
		if (min_cycle > common_cycle)
			common_cycle = min_cycle;

		// refresh common cycle display 
		if ( common_cycle >= common_cycle_display_trigger) {
			printf("cycle = %10ld (%9.1f s)\n", (long)common_cycle, common_cycle / 16e6);
			common_cycle_display_trigger += DISPLAY_THRESHOLD;
			goto exit;
		}
	}

exit:
	// stop cleanly
	for (unsigned int i = 0; i < sizeof(cores) / sizeof(cores[0]); i++) {
		avr_terminate(cores[i]);
	}
}
