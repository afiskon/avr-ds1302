#!/bin/sh

set -e

CC='avr-gcc -std=c11 -Os -DF_CPU=16000000UL -mmcu=atmega328p -Wall -Werror -Wpedantic'

$CC main.c -o main
avr-objcopy -O ihex -R .eeprom main main.hex
avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:main.hex
