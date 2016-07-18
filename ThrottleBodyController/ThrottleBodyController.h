/*
Supports firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/


#define DEBUG 1

#define BM0 (1<<0)
#define BM1 (1<<1)
#define BM2 (1<<2)
#define BM3 (1<<3)
#define BM4 (1<<4)
#define BM5 (1<<5)
#define BM6 (1<<6)
#define BM7 (1<<7)
#define BM8 (1<<8)
#define BM9 (1<<9)
#define BM10 (1<<10)
#define BM11 (1<<11)
#define BM12 (1<<12)
#define BM13 (1<<13)
#define BM14 (1<<14)
#define BM15 (1<<15)

#define SPI_CAN_CS 10
#define SPI_HALL_CS 9
#define CURRENT_SENS 5
#define VOLTAGE_SENS 6

#define ADC_FILTER_SIZE 4
#define HALL_FILTER_SIZE 5

#define HALL_GET_ANGLE 0x3FFF
#define HALL_ZERO_ANGLE_HIGH 0x0016
#define HALL_ZERO_ANGLE_LOW 0x0017
#define HALL_NOP_COMMAND 0

#define SPI_HALL_SELECT() PORTB &= ~(1<<SPI_HALL_CS)				//digitalWrite(SPI_HALL_CS, LOW)
#define SPI_HALL_UNSELECT() PORTB |= (1<<SPI_HALL_CS) 			//digitalWrite(SPI_HALL_CS, HIGH)

#define SPI_SETTINGS_HALL SPISettings(1000000, MSBFIRST, SPI_MODE_1);
#define SPI_SETTINGS_CAN SPISettings(1000000, MSBFIRST, SPI_MODE_0);
