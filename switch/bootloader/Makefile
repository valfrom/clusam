PRG            = clunet_bootloader
OBJ            = clunet_bootloader.o
LFUSE          = E4
HFUSE          = DA
MCU_PROGRAMMER = m8
BOOTSTART = 0x1C00

AVRDUDE        = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avrdude
CONF           = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf
OBJCOPY        = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-objcopy
OBJDUMP        = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-objdump
CC             = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-gcc

PORT           = `ls /dev/tty.usbmodem*`
PROGRAMMER     = -cstk500v1 -P $(PORT) -b19200

MCU_TARGET     = atmega8
#MCU_TARGET     = atmega328p
#MCU_TARGET     = atmega168
OPTIMIZE       = -Os

DEFS           =
LIBS           =

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -I. -I..
LDFLAGS       = -Wl,-Map,$(PRG).map

#override LDFLAGS       =  -Wl,--section-start=.text=$(BOOTSTART)
LDFLAGS += -Wl,--section-start=.text=$(BOOTSTART)

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
clunet_bootloader.o: clunet_bootloader.c defines.h ../../clunet.h ../clunet_config.h ../defines.h ../../bits.h
clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

program: hex
	$(AVRDUDE) -V -C $(CONF) -p $(MCU_PROGRAMMER) $(PROGRAMMER) -U flash:w:$(PRG).hex -U lfuse:w:0x$(LFUSE):m -U hfuse:w:0x$(HFUSE):m
