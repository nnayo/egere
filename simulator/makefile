CC = gcc

CFLAGS = \
	 -g \
	 -std=c99 \
	 -Wall \
	 -Wextra \
	 -Werror \
	 -fPIC \
	 -I. \
	 -I$(TROLL_PROJECTS)/simavr/simavr/sim

LDFLAGS = \
	  -shared \
	  $(TROLL_PROJECTS)/simavr/libsimavr.a \
	  -lelf \
	  -lm


all: mpu6050.so

debug: mpu6050.so
	gdb ../../simavr/simavr/run_avr

mpu6050.so: mpu6050.o
	$(CC) mpu6050.o $(LDFLAGS) -o mpu6050.so

mpu6050.o: mpu6050.c mpu6050.h sc18is600.h


clean:
	rm -f mpu6050.so *.o *.elf *.vcd *~ *.lix
