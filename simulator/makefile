
CFLAGS = -g -I$(TROLL_PROJECTS)/simavr/simavr/sim -I. -std=c99 -Wall -Wextra
LDFLAGS = $(TROLL_PROJECTS)/simavr/libsimavr.a -lelf


all:	sim_egere minut.elf



sim_egere: sim_egere.o mpu6050.o sc18is600.o $(TROLL_PROJECTS)/simavr/libsimavr.a
	$(CC) sim_egere.o mpu6050.o sc18is600.o $(LDFLAGS) -o sim_egere

sim_egere.o: sim_egere.c
	$(CC) $(CFLAGS) -c sim_egere.c -o sim_egere.o

mpu6050.o: mpu6050.c mpu6050.h sc18is600.h
sc18is600.o: sc18is600.c sc18is600.h


minut.elf:
	ln -s ../soft/minut.elf minut.elf


clean:
	rm -f sim_egere *.o *.elf *.vcd *~ *.lix
