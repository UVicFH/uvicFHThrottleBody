/*
Supports firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#ifndef THROTTLEBODYCONTROLLER_H
#define THROTTLEBODYCONTROLLER_H

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

#define SPI_CAN_CS 9
#define SPI_HALL_CS 10
#define CURRENT_SENS 5
#define VOLTAGE_SENS 6
#define TEMP_SENS 7
#define MOTOR_OPEN_PIN 5

#define ADC_FILTER_SIZE 4
#define HALL_AVERAGE_SIZE 1			//number of readings to take into account over 1ms. Makes reading more accurate.
#define HALL_FILTER_SIZE 4			//number of these averaged elements to take into account for reporting position over CAN. Might not be necessary.

#define PID_EXECUTION_INTERVAL 1	//number of ms between changes in PID controller
#define CAN_SEND_INTERVAL 100

#define HALL_GET_ANGLE 0x3FFF
#define HALL_ZERO_ANGLE_HIGH 0x0016
#define HALL_ZERO_ANGLE_LOW 0x0017
#define HALL_NOP_COMMAND 0

#define CAN_THROTTLE_MSG_ADDRESS 0x102
#define CAN_FEEDBACK_MSG_ADDRESS 0x101

#define SPI_HALL_SELECT() PORTB &= ~(0b00000100)				//digitalWrite(SPI_HALL_CS, LOW)
#define SPI_HALL_DESELECT() PORTB |= (0b00000100) 			//digitalWrite(SPI_HALL_CS, HIGH)

SPISettings SPI_SETTINGS_HALL(1000000, MSBFIRST, SPI_MODE1);
SPISettings SPI_SETTINGS_CAN(16000000, MSBFIRST, SPI_MODE0);

#endif // THROTTLEBODYCONTROLLER_H
