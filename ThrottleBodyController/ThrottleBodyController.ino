/*
This is the firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#include <SPI.h>
#include <mcp_can.h>
#include "ThrottleBodyController.h"s
//#include <TimerOne.h>

uint8_t canIntRecv = 0;
uint32_t previousPidMillis = 0;
uint32_t previousCanMillis = 0;

//received values from CAN
uint16_t instThrottleRequest_degx10 = 0;
uint8_t instThrottleBlip_deg = 0;
uint8_t instThrottleBlip_ms = 0;

//values to be sent over CAN
uint16_t filteredCurrentDraw_mA = 0;									
uint16_t filteredVoltage_mV = 0;										
uint8_t filteredTemp_degCx2 = 0;							
uint16_t zeroedHallPosition_degx10 = 0;							

//sensor feedback not sent over CAN
uint16_t absoluteHallPosition_nominal = 0;	
int32_t zeroedHallPosition_nominal = 0;         //note this is signed
uint16_t hallZeroPosition_nominal = 0;
float zeroedHallPosition_deg = 0;											


MCP_CAN CAN(SPI_CAN_CS);                      //set CS CAN pin

uint16_t as5048aRemoveParity(uint16_t command)
{
	return (command &= 0b0011111111111111);                 //clears parity and error bit
}


uint8_t as5048aSetParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}


uint16_t as5048aReadCommand(uint16_t spiSendCommand)
{
	uint16_t hallPosition_nominal = 0;

	spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
	spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

	SPI.beginTransaction(SPI_SETTINGS_HALL);

	digitalWrite(SPI_HALL_CS, LOW);
	hallPosition_nominal = SPI.transfer16(spiSendCommand);
	digitalWrite(SPI_HALL_CS,HIGH);

	SPI.endTransaction();

	hallPosition_nominal = as5048aRemoveParity(returnVal);

	return hallPosition_nominal;
}


uint16_t getAbsoluteHallPosition(void)
{
	uint16_t hallPositionSum_nominal = 0;
	for(int i=0; i<HALL_AVERAGE_SIZE; i++)
	{
		hallPositionSum_nominal += as5048aReadCommand(HALL_GET_ANGLE);
	}
	absoluteHallPosition_nominal = hallPositionSum_nominal >> filterShiftSize(HALL_AVERAGE_SIZE);
	return absoluteHallPosition_nominal;
}

void getZeroedHallPosition(void)
{
	zeroedHallPosition_nominal = absoluteHallPosition_nominal - hallZeroPosition_nominal;
	if(zeroedHallPosition_nominal < 0)
	{
		zeroedHallPosition_nominal = 0;
	}
	
	zeroedHallPosition_deg = (float) zeroedHallPosition_nominal/(16384.0/360.0);
	zeroedHallPosition_degx10 = (uint16_t) zeroedHallPosition_deg*10.0;
}


void findHallZeroPosition(void)
{
	uint32_t hallZeroPositionSum_nominal = 0;
	getAbsoluteHallPosition();

	for(int i = 0; i<HALL_ZERO_READING_COUNT; i++)
	{
		hallZeroPositionSum_nominal += getAbsoluteHallPosition();
		delay(50);
	}
	hallZeroPosition_nominal = hallZeroPositionSum_nominal >> filterShiftSize(HALL_ZERO_READING_COUNT);
}



/*
//gets the CAN message in the buffer, and reads the required values
void getCanMsg(void)
{
	uint8_t canMsgLength = 0;
	uint8_t canMsgData[8];

	while (CAN_MSGAVAIL == CAN.checkReceive()) 
	{
		// read data,  len: data length, buf: data buf
		CAN.readMsgBuf(&canMsgLength, canMsgData);
		instThrottleRequest_degx10 = (canMsgData[1]<<8) + canMsgData[0];
		instThrottleBlip_ms = canMsgData[2];
		instThrottleBlip_deg = canMsgData[3];
		if(DEBUG)
		{
			Serial.print("CAN Throttle Request = ");
			Serial.println(instThrottleRequest_degx10);
		}
	}
}
*/
//sends the outgoing CAN message with updated variables
void sendCanMsg(void)
{
	uint8_t canSendBuffer[8];

	canSendBuffer[0] = zeroedHallPosition_degx10 & 0b11111111;
	canSendBuffer[1] = zeroedHallPosition_degx10 >> 8;
	canSendBuffer[2] = filteredCurrentDraw_mA & 0b11111111;
	canSendBuffer[3] = filteredCurrentDraw_mA >> 8;
	canSendBuffer[4] = filteredVoltage_mV & 0b11111111;
	canSendBuffer[5] = filteredVoltage_mV >> 8;
	canSendBuffer[6] = 0;//filteredTemp_degCx2;
	
	SPI.beginTransaction(SPI_SETTINGS_CAN);
	CAN.sendMsgBuf(CAN_FEEDBACK_MSG_ADDRESS,0,8,canSendBuffer);
	SPI.endTransaction();
}

void filterHallPosition(void)
{
	uint16_t hallPosition_degx10 = (uint16_t) zeroedHallPosition_degx10 * 10; 

	filteredHallPosition_degx10 *= HALL_FILTER_SIZE;
	filteredHallPosition_degx10 = filteredHallPosition_degx10 - (filteredHallPosition_degx10 >> filterShiftSize(HALL_FILTER_SIZE)) + hallPosition_degx10;
	filteredHallPosition_degx10 = filteredHallPosition_degx10 >> filterShiftSize(HALL_FILTER_SIZE);
}


//calculates the current consumption of the device. oversample and average.
void calculateCurrent(void)
{
	float currentDraw_mA = analogRead(CURRENT_SENS)/1023.0*5/(0.5)*1000; //0.5 in denom = 0.01*0.05*1000

	filteredCurrentDraw_mA *= ADC_FILTER_SIZE;
	filteredCurrentDraw_mA = (uint16_t) ((float) ((filteredCurrentDraw_mA - (filteredCurrentDraw_mA/ADC_FILTER_SIZE) + currentDraw_mA)/ADC_FILTER_SIZE));
}

//calculates the voltage input to the device, accounting for voltage drop due to current through the FET and shunt
void calculateVoltage(void)
{
	uint16_t voltage_mV = (uint16_t) (analogRead(VOLTAGE_SENS)/1023.0*5000*13.3/3.3);

	filteredVoltage_mV *= ADC_FILTER_SIZE;
	filteredVoltage_mV = filteredVoltage_mV - (filteredVoltage_mV >> filterShiftSize(ADC_FILTER_SIZE)) + voltage_mV;
	filteredVoltage_mV = filteredVoltage_mV >> filterShiftSize(ADC_FILTER_SIZE);
}
/*
//calculates the temperature recorded by the thermistor
void calculateTemperature(void)
{
	uint16_t temp_degC = analogRead(TEMP_SENS)* math required to make work;

	filteredTemp_degCx2 *= ADC_FILTER_SIZE;
	filteredTemp_degCx2 = filteredTemp_degCx2 - (filteredTemp_degCx2 >> filterShiftSize(ADC_FILTER_SIZE)) + temp_degC;
	filteredTemp_degCx2 = filteredTemp_degCx2 >> filterShiftSize(ADC_FILTER_SIZE);
}

*/
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
	}
	return shiftSize;	
}
/*
void adcMovingSum(uint8_t adcPin, uint16_t *pAdcSum, uint8_t filterSize)
{
	uint16_t newAdcValue = analogRead(adcPin);
	*pAdcAverage = *pAdcAverage - (*pAdcAverage >> filterShiftSize(filterSize)) + newAdcValue;
}
*/



//calculates the position of the motor shaft from the quad encoder
void calculateQuadPosition(void)
{
	// use polling, and direct port manipulation. poll every millis.
	// if we're only polling every 1ms, and we're sitting on the edge of a signal, we might miss some of the transitions
	// if we require a state to be constant for 6-8ms before accepting it, that could help deal with bounces. 
	// if the state changes before 6-8ms we reset that timer and keep monitoring.
	// classic debounce of the signal.



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


void setup()
{

	Serial.begin(115200);
  SPI.begin();
//	Timer1.initialize(200);  // 200 us = 5000 Hz

	//output = 1, input = 0
	DDRB |= 0b00000110; //PB1 and PB2 are outputs
	DDRC |= 0b00000000; //no outputs on PC
	DDRD |= 0b00110000; //PD4 PD5 are outputs    output = 1  
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


	CAN.init_Mask(0, 0, 0xFFF);												//must set both masks; use standard CAN frame
	CAN.init_Mask(1, 0, 0xFFF);												//must set both masks; use standard CAN frame
	CAN.init_Filt(0, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 0 for receive buffer 0
	CAN.init_Filt(2, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 1 for receive buffer 1

	attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

	delay(500);
	zeroHallPosition();
}



void loop()
{
	/*
	if(canIntRecv ==1)
	{
		canIntRecv = 0;
		getCanMsg();
	}

 */
	if(millis() - previousPidMillis >= PID_EXECUTION_INTERVAL)
	{
		previousPidMillis = millis();
	getHallPosition();
	filteredHallPosition_degx10 = (uint16_t) averagedHallPosition_degx10;
		//executePid(); 
	}

	if(millis() - previousCanMillis >= CAN_SEND_INTERVAL)
	{
		previousCanMillis = millis();
		//getHallPosition();
	//filteredHallPosition_degx10 = (uint16_t) averagedHallPosition_degx10 * 10.0;
		calculateCurrent();
		calculateVoltage();
		//calculateTemperature();
		//filterHallPosition();
		sendCanMsg();
	}
	//getHallPosition();
	//delay(100);
}

void MCP2515_ISR()
{
	canIntRecv = 1;
}
