/*
This is the firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#include <SPI.h>
#include "mcp_can.h"
#include "ThrottleBodyController.h"
#include "Pin_Defines.h"
#include <TimerOne.h>

uint8_t canIntRecv = 0;
uint8_t instThrottleRequest = 0;
uint8_t thermistorTemp_degC = 0;

uint16_t motorPosQuad_deg = 0;
uint16_t filteredCurrentDraw_mA = 0;
uint16_t voltageInput_mV = 0;

float throttlePosHall_deg = 0;
float filteredThrottleRequest = 0;

void setup()
{
	MCP_CAN CAN(SPI_CAN_CS);                                          // set CS CAN pin
	SPI.setDataMode (SPI_MODE_1);
	Serial.begin(115200);
	Timer1.initialize(2000);  // 200 us = 5000 Hz

	//output = 1, input = 0
	DDRB |= 0b00000001; //PB0 is output (PB1 is PWM set up seperately) 
	DDRC |= 0b00111011; //PC0 PC1 PC3 PC4 & PC5 are outputs 
	DDRD |= 0b11110100; //PD2 PD4 PD5 PD6 & PD7 are outputs
  
	while(1)
	{
		if(CAN_OK == CAN.begin(CAN_500KBPS))
		{
			Serial.println("CAN BUS INIT GOOD");
			break;
		}
		else
		{
			Serial.println("CAN BUS INIT FAIL, RETRY");
			delay(100);
		}
	}

	attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
}

//gets the CAN message in the buffer, and reads the required values
void getCanMsg(void)
{
	uint8_t canMsgAddress = 0;
	uint8_t canMsgLength = 0;
	uint8_t canMsgData[8];

	while (CAN_MSGAVAIL == CAN.checkReceive()) 
    {
        // read data,  len: data length, buf: data buf
        canMsgAddress = CAN.getCanId();
        CAN.readMsgBuf(&canMsgLength, buf);
        if(canMsgAddress == 0x255)
        {
        	instThrottleRequest = canMsgData[0];
        }
        #if(DEBUG)
        {
        	Serial.print("CAN Throttle Request = ");
        	Serial.println(canMsgData[0]);
        }
        #endif
    }	
}

//sends the outgoing CAN message with updated variables
void sendCanMsg(void)
{

}

//calculates the current consumption of the device. oversample and average.
void calculateCurrent(void)
{
	uint16_t currentDraw_mA = analogRead(CURRENT_SENS)* math required to make work;

	filteredCurrentDraw_mA *= CURRENT_FILTER_SIZE;
	filteredCurrentDraw_mA = filteredCurrentDraw_mA - (filteredCurrentDraw_mA >> CURRENT_FILTER_SHIFT) + currentDraw_mA;
	filteredCurrentDraw_mA = filteredCurrentDraw_mA >> CURRENT_FILTER_SHIFT;
}

//calculates the voltage input to the device, accounting for voltage drop due to current through the FET and shunt
void calculateVoltage(void)
{

}

//calculates the temperature recorded by the thermistor
void calculateTemperature(void)
{

}

uint8_t filterShiftSize(uint8_t filterSize)
{
	uint8_t shiftSize = 0;
	switch(filterSize)
	{
		case 1:
			shiftSize = 0;
			break;
		case 2:
			shiftSize = 1;
			break;
		case 4:
			shiftSize = 2;
			break;
		case 8:
			shiftSize = 3;
			break;
		case 16:
			shiftSize = 4;
			break;
		case 32:
			shiftSize = 5;
			break;
		case 64:
			shiftSize = 6;
			break;
		case 128:
			shiftSize = 7;
			break;
		case 256:
			shiftSize = 8;
			break;
		case 512:
			shiftSize = 9;
			break;
		default:
			shiftSize = 0;
			break;
	}
	return shiftSize;	
}

void adcMovingSum(uint8_t adcPin, uint16_t *pAdcSum, uint8_t filterSize)
{
	uint16_t newAdcValue = analogRead(adcPin);
	*pAdcAverage = *pAdcAverage - (*pAdcAverage >> filterShiftSize(filterSize)) + newAdcValue;
}

void floatMovingSum(float *pFloatSum, uint8_t filterSize)
{

}

//gets the position of the low speed shaft from the hall sensor IC
void getHallPosition(void)
{

	receivedval16 = SPI.transfer(val16);
}

uint16_t as5048aNOP(uint8_t inTransaction)
{
	uint16_t returnVal
	uint16_t nopCommand = 0;

	if(inTransaction == 0)
	{
		SPI.beginTransaction(SPI_SETTINGS_HALL);
	}

	SPI_HALL_SELECT();
	returnVal = SPI.transfer16(HALL_NOP_COMMAND);
	SPI_HALL_UNSELECT();

	if(inTransaction == 0)
	{
		SPI.endTransaction();		
	}

	return returnVal;	
}

uint16_t as5048aSetParity(uint16_t command)
{
	uint16_t command = command;
	uint8_t parityBit = 0;

	for(int i = 0; i<=14; i++)
	{
		parityBit = (command>>(14-i)) ^= parityBit;
	}
	return (command | (parityBit << 15));					//OR operator sets the single bit
}

uint16_t as5048aRemoveParity(uint16_t command)
{
	return (command & 0b0011111111111111);					//AND operator clears first two bits
}

uint16_t as5048aReadCommand(uint16_t spiSendCommand)
{
	uint16_t returnVal = 0;

	spiSendCommand |= 0b0100000000000000; 					//OR operator sets the read/write bit to READ
	spiSendCommand = as5048aSetParity(spiSendCommand);

	SPI.beginTransaction(SPI_SETTINGS_HALL);
	SPI_HALL_SELECT();
	returnVal = SPI.transfer16(spiSendCommand);
	SPI_HALL_UNSELECT();
	SPI.endTransaction();
	return returnVal;
}

uint8_t as5048aWriteCommand(uint16_t spiSendAddress, uint16_t spiSendCommand)
{
	uint16_t returnVal = 0;

	spiSendAddress &= 0b1011111111111111;					//AND operator sets the read/write bit to WRITE
	spiSendAddress = as5048aSetParity(address);

	spiSendCommand &= 0b1011111111111111;					//AND operator sets the read/write bit to WRITE
	spiSendCommand = as5048aSetParity(spiSendCommand);

	SPI.beginTransaction(SPI_SETTINGS_HALL);

	SPI_HALL_SELECT();
	returnVal = SPI.transfer16(spiSendAddress);				//send the address to the sensor
	SPI_HALL_UNSELECT();

	SPI_HALL_SELECT();
	returnVal = SPI.transfer16(spiSendCommand);				//send the commanded value to the sensor
	SPI_HALL_UNSELECT();

	returnVal = as5048aNOP(1);								//send NOP to get the value in the register

	SPI.endTransaction();

	if(as5048aRemoveParity(returnVal) == as5048aRemoveParity(spiSendCommand))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void zeroHallPosition(void)
{
	uint16_t hallDataReceived = 0;
	uint16_t hallSendBuffer = 0;

	//get position first
	as5048aReadCommand(HALL_GET_ANGLE);
	hallDataReceived = as5048aRemoveParity(as5048aNOP(0));
	as5048aWriteCommand


	hallSendBuffer = hallDataReceived;
	
	//force R bit to its correct value, and set parity to 0 for later
	hallSendBuffer = hallSendBuffer & 0b0011111111111111;
	


	//set parity bit
	hallSendBuffer = hallSendBuffer & ((parityBit << 15) + 0b0011111111111111);

	SPI_HALL_SELECT();
	hallDataReceived = SPI.transfer16()

	SPI.endTransaction();

}

//calculates the position of the motor shaft from the quad encoder
void calculateQuadPosition(void)
{

}

//error check the two positions
void validatePositions(void)
{

}

//executes the PID controller and outputs to motor controller
void executePid(void)
{

}

//calculates the filtered (moving avg) throttle input value
void filterThrottle(void)
{

}

void loop()
{

}

void MCP2515_ISR()
{
    canIntRecv = 1;
}
