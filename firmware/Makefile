BOARD=board.map

OBJS=main.o


CPU=atmega328p
PORT=usb
PROG=stk500v2

#CPU=atmega32u4
#PORT=/dev/ttyACM0
#PROG=avr109

HEX=image.hex
ELF=image.elf
CC=avr-gcc
LD=$(CC)
SIZE=avr-size
CFLAGS=-mmcu=$(CPU) -g -Os -Wall -save-temps -fdata-sections
LDFLAGS=$(CFLAGS) -Wl,--gc-sections 
LIBS=-lc -lm
OBJCOPY=avr-objcopy
INCLUDES=-I../../forth

.PHONY: all

all: $(HEX)

$(OBJS): Makefile

upload: $(HEX)
	[ $(PORT) = usb ] || ard-reset-arduino $(PORT)
	avrdude -q -p $(CPU) -c $(PROG) -P $(PORT) \
	  -U flash:w:$(HEX):i

fuses:
	avrdude -q -p atmega328p -c stk500v2 -P usb \
	  -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0x04:m

size: $(ELF)
	$(SIZE) -C --mcu=$(CPU) $(ELF)

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) -Wl,-Map,image.map

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDES) -o $@ $<

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

clean:
	rm -f $(HEX) $(ELF) $(OBJS) board.h
