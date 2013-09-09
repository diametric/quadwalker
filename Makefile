# Device Info


DEVICE     = atmega328p
CLOCK      = 8000000
PROGRAMMER = -c avrispmkII -P usb
OBJECTS    = main.o
FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
#FUSES      = -U lfuse:w:0x62:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

AVRDUDE = /usr/local/CrossPack-AVR/bin/avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = /usr/local/CrossPack-AVR/bin/avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -lprintf_flt -lm -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

clean:
	rm -f main.hex main.elf $(OBJECTS)

main.elf: $(OBJECTS)
	$(COMPILE) -Wl,-u,vfprintf -lprintf_flt -lm -o main.elf  $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	/usr/local/CrossPack-AVR/bin/avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	/usr/local/CrossPack-AVR/bin/avr-size --format=avr --mcu=$(DEVICE) main.elf

disasm:	main.elf
	/usr/local/CrossPack-AVR/bin/avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
