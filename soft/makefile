TROLL_PROJECTS = /home/yog/TRoll/projects
NANOK_DIR = $(TROLL_PROJECTS)/nanoK
SCALP_DIR = ./scalp

PRG = minut
SRCS = main.c minut.c mpu6050.c servo.c eeprom.c
OBJS = main.o minut.o mpu6050.o servo.o eeprom.o

CC = avr-gcc
CFLAGS = \
		 -g -std=c99 \
		 -Wall -Wextra -Werror \
		 -Os -mcall-prologues -fshort-enums \
		 -mmcu=atmega328p \
		 -I. \
		 -I$(NANOK_DIR) \
		 -I$(SCALP_DIR) \
		 -I$(TROLL_PROJECTS)/simavr/simavr/sim/avr

LDFLAGS = \
		  -g \
		  -Os -mcall-prologues -fshort-enums \
		  -mmcu=atmega328p \
		  -Wl,-Map,$(PRG).map,--cref \
		  -Wl,--undefined=_mmcu,--section-start=.mmcu=0x8000 \
		  -lm \
		  -L$(NANOK_DIR) -lnanoK \
		  -L$(SCALP_DIR) -lscalp

OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump

AVRDUDE = avrdude
AVRDUDEFLAGS = -P /dev/ttyACM0 -p atmega328p -c arduino

# for dependency autogeneration
# part #1
DEPDIR = .deps
$(shell mkdir -p $(DEPDIR) >/dev/null)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) -c
POSTCOMPILE = mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d

%.o : %.c
%.o : %.c $(DEPDIR)/%.d
	$(COMPILE.c) $(OUTPUT_OPTION) $<
	$(POSTCOMPILE)

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
# end part #1


.PHONY: all clean very_clean

all: $(PRG).elf lix
	avr-size $(PRG).elf

$(PRG).elf: $(OBJS) $(SCALP_DIR)/libscalp.a $(NANOK_DIR)/libnanoK.a
	$(CC) $(LDFLAGS) $^ -o $@

$(OBJS): $(SCALP_DIR)/libscalp.a $(NANOK_DIR)/libnanoK.a

$(NANOK_DIR)/libnanoK.a:
	make -C $(NANOK_DIR)

$(SCALP_DIR)/libscalp.a:
	make -C $(SCALP_DIR)

eeprom.hex: $(PRG).elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@
	
eeprom: eeprom.hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -D -U eeprom:w:eeprom_scalps.hex:r


$(PRG).hex: $(PRG).elf
	$(OBJCOPY) -O ihex -R .eeprom -R .mmcu $(PRG).elf $(PRG).hex

load: $(PRG).hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(PRG).hex


term:
	$(AVRDUDE) $(AVRDUDEFLAGS) -t


clean:
	rm -rf *.o $(PRG).elf *.bak eeprom.c
	rm -rf *.lix *.map *.bin
	make -C $(NANOK_DIR) clean
	make -C $(SCALP_DIR) clean

very_clean: clean
	rm -fr *.*~
	rm -fr *.*swp
	rm -fr $(DEPDIR) __pycache__
	@cd $(NANOK_DIR); make very_clean
	@cd $(SCALP_DIR); make very_clean

lix:  $(PRG).lix

%.lix: %.elf
	$(OBJDUMP) -Sdx $< > $@

eeprom.c: ./gen_eeprom.py
	./gen_eeprom.py eeprom.c

eeprom.o: eeprom.c

# for dependency autogeneration
# part #2
-include $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRCS)))
# end part #2

