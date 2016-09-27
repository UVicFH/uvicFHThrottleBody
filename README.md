Firmware runs on custom PCB on custom UVic FH Throttle Body.

#VERY IMPORTANT:
You must change the wiring.c Arduino core file to make this code run correctly.
This file is found in:
C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\adruino

You must change the following line:

'''#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))'''

to:

'''#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(1 * 256))'''

If you do not make this change, millis(), micros(), delay(), and delayMicroseconds() will not work.
These are critical to the function of the program.

Once you make this change, compile and upload in the Arduino IDE as normal. No need to restart the IDE.
Make sure to change it back if you want to run normal code on any other arduino.
