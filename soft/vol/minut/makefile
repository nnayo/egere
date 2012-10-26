NANOK_DIR	= /home/yog/TRoll/projects/nanoK
NANOK_CONF_PATH	= /home/yog/TRoll/projects/TROY/soft/vol/minut
COMMON_DIR	= ../common

PRG		= minut
OBJ		= minut.o
TASKS_OBJ	= TASK_back_gnd.o TASK_minut.o TASK_servo.o

#MCU_TARGET	= atmega128
#MCU_TARGET	= atmega16
MCU_TARGET	= atmega32
OPTIMIZE	= -Os -mcall-prologues -fshort-enums -Wall
#OPTIMIZE	= -Os -mcall-prologues -fnew-ra

#DEFS		= -I$(NANOK_DIR)/nanoK -I$(NANOK_DIR)/nanoK/utils -I$(NANOK_DIR)/nanoK/tasks -I.
#DEFS		= -I. -I$(NANOK_DIR) -I$(NANOK_DIR)/drivers -I$(NANOK_DIR)/utils -I..
DEFS		= -I. -I$(NANOK_DIR) -I$(COMMON_DIR) -I..
LIBS		= -lm

#DEBUG		= -gdwarf-2
DEBUG		= -g

# You should not have to change anything below here.

CC		= avr-gcc
AVRDUDE		= avrdude -P /dev/ttyACM0 -p atmega328p -c arduino

# Override is only needed by avr-lib build system.

override CFLAGS        = $(DEBUG) -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map,--cref,-T $(NANOK_DIR)/nanoK.x

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

#all: $(PRG).elf lst text eeprom
all: $(PRG).elf lst
	avr-size $(PRG).elf

libnanoK.a: force
	@cd $(NANOK_DIR); make -e NANOK_CONF_PATH=$(NANOK_CONF_PATH)
	cp $(NANOK_DIR)/libnanoK.a . 

libcommon.a: libnanoK.a force
	@cd $(COMMON_DIR); make -e NANOK_CONF_PATH=$(NANOK_CONF_PATH)
	cp $(COMMON_DIR)/libcommon.a . 

force:

$(PRG).elf: $(OBJ) $(TASKS_OBJ) libcommon.a libnanoK.a
	$(CC) $(CFLAGS) $(LDFLAGS) $(LIBS) -L. -lnanoK -lcommon $^ -o $@


eeprom: minut.bin
	$(AVRDUDE) -D -U eeprom:w:minut.bin:r


$(PRG).hex: $(PRG).elf
	$(OBJCOPY) -O ihex -R .eeprom $(PRG).elf $(PRG).hex

load: $(PRG).hex
	$(AVRDUDE) -U flash:w:$(PRG).hex


term:
	$(AVRDUDE) -t


clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map *.bin $(EXTRA_CLEAN_FILES)
	rm -fr *~
	rm -f $(OBJ)
	rm -f libcommon.a libnanoK.a
	@cd $(COMMON_DIR); make clean

very_clean: clean
	rm -fr *.*~
	rm -fr *.*bak
	rm -fr *.*swp
	@cd $(COMMON_DIR); make very_clean

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -S -x -p $< > $@
#	$(OBJDUMP) -h -S -x -D -s -g -t -a -f -p $< > $@
#	$(OBJDUMP) -h -S $< > $@

%.o: %.h

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@
	avr-size $@
