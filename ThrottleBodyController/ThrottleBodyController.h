/*
Supports firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/


#define DEBUG 1

#define SPI_CAN_CS 10
#define SPI_HALL_CS 9
#define CURRENT_SENS 5
#define VOLTAGE_SENS 6

#define ADC_FILTER_SIZE 4

#define GET_ANGLE 0xFFFF											// 0x3FFF command + 0b11000000,00000000 read and parity
#define SEND_ANGLE
#define ZERO_ANGLE

#define SPI_HALL_SELECT()   digitalWrite(SPI_HALL_CS, LOW)
#define SPI_HALL_UNSELECT() digitalWrite(SPI_HALL_CS, HIGH)

SPISettings spiSettingsHall(1000000, MSBFIRST, SPI_MODE_1);
SPISettings spiSettingsCan(1000000, MSBFIRST, SPI_MODE_0);