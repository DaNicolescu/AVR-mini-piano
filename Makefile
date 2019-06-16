PORT = /dev/hidraw0

all: mini_piano.hex

mini_piano.hex: mini_piano.elf
	avr-objcopy  -j .text -j .data -O ihex $^ $@
	avr-size $@

mini_piano.elf: mini_piano.c lcd.c
	avr-g++ -mmcu=atmega324p -DF_CPU=16000000 -Os -Wall -o $@ $^

clean:
	rm -rf mini_piano.elf mini_piano.hex
