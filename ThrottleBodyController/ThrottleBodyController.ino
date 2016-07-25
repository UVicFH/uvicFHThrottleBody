/*
This is the firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#include <SPI.h>
#include "mcp_can.h"
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
uint16_t filteredCurrentDraw_mA = 0;									//done
uint16_t filteredVoltage_mV = 0;										//done
uint8_t filteredTemp_degCx2 = 0;										//done
uint16_t filteredHallPosition_degx10 = 0;								//done

//sensor feedback not sent over CAN
float averagedHallPosition_deg = 0;	
uint16_t motorPosQuad_deg = 0;											//done
uint32_t hallZeroPosition = 0;

void setup()
{
	MCP_CAN CAN(SPI_CAN_CS);											//set CS CAN pin
	Serial.begin(115200);
  SPI.begin();
//	Timer1.initialize(200);  // 200 us = 5000 Hz

	//output = 1, input = 0
	DDRB |= 0b00000110; //PB1 and PB2 are outputs
	DDRC |= 0b00000000; //no outputs on PC
	DDRD |= 0b00110000; //PD4 PD5 are outputs    output = 1
  /* commented for bringing up hall sensor  
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
    CAN.init_Filt(0, 0, CAN_THROTTLE_MSG_ADRESS);							//filter 0 for receive buffer 0
    CAN.init_Filt(2, 0, CAN_THROTTLE_MSG_ADRESS);							//filter 1 for receive buffer 1

	attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

	delay(200);
	zeroHallPosition();
	*/
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

//sends the outgoing CAN message with updated variables
void sendCanMsg(void)
{
	uint8_t canSendBuffer[8];
	canSendBuffer[0] = filteredHallPosition_degx10 >> 8;
	canSendBuffer[1] = filteredHallPosition_degx10;
	canSendBuffer[2] = filteredCurrentDraw_mA >> 8;
	canSendBuffer[3] = filteredCurrentDraw_mA;
	canSendBuffer[4] = filteredVoltage_mV >> 8;
	canSendBuffer[5] = filteredVoltage_mV;
	canSendBuffer[6] = filteredTemp_degCx2;

	sendMsgBuf(CAN_FEEDBACK_MSG_ADDRESS,0,8, canSendBuffer);
}

void filterHallPosition(void)
{
	uint8_t hallPosition_degx10 = (int) averagedHallPosition_deg * 10; 

	filteredHallPosition_degx10 *= HALL_FILTER_SIZE;
	filteredHallPosition_degx10 = filteredHallPosition_degx10 - (filteredHallPosition_degx10 >> filterShiftSize(HALL_FILTER_SIZE)) + hallPosition_degx10;
	filteredHallPosition_degx10 = filteredHallPosition_degx10 >> filterShiftSize(HALL_FILTER_SIZE);
}


//calculates the current consumption of the device. oversample and average.
void calculateCurrent(void)
{
	uint16_t currentDraw_mA = analogRead(CURRENT_SENS)* math required to make work;

	filteredCurrentDraw_mA *= ADC_FILTER_SIZE;
	filteredCurrentDraw_mA = filteredCurrentDraw_mA - (filteredCurrentDraw_mA >> filterShiftSize(ADC_FILTER_SIZE)) + currentDraw_mA;
	filteredCurrentDraw_mA = filteredCurrentDraw_mA >> filterShiftSize(ADC_FILTER_SIZE);
}

//calculates the voltage input to the device, accounting for voltage drop due to current through the FET and shunt
void calculateVoltage(void)
{
	uint16_t voltage_mV = analogRead(VOLTAGE_SENS)* math required to make work;

	filteredVoltage_mV *= ADC_FILTER_SIZE;
	filteredVoltage_mV = filteredVoltage_mV - (filteredVoltage_mV >> filterShiftSize(ADC_FILTER_SIZE)) + voltage_mV;
	filteredCVoltage_mV = filteredVoltage_mV >> filterShiftSize(ADC_FILTER_SIZE);
}

//calculates the temperature recorded by the thermistor
void calculateTemperature(void)
{
	uint16_t temp_degC = analogRead(TEMP_SENS)* math required to make work;

	filteredTemp_degCx2 *= ADC_FILTER_SIZE;
	filteredTemp_degCx2 = filteredTemp_degCx2 - (filteredTemp_degCx2 >> filterShiftSize(ADC_FILTER_SIZE)) + temp_degC;
	filteredTemp_degCx2 = filteredTemp_degCx2 >> filterShiftSize(ADC_FILTER_SIZE);
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
*/
uint16_t as5048aNOP(uint8_t inTransaction)
{
	uint16_t returnVal;
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
	uint8_t parityBit = 0;

	for(int i = 0; i<=14; i++)
	{
		parityBit = (command>>(14-i)) ^ parityBit;
	}
	return (command | (parityBit << 15));					//OR operator sets the single bit
}

uint16_t as5048aRemoveParity(uint16_t command)
{
	command &= (~1<<15);									//AND operator clears parity bit
	command &= (~1<<14);									//AND operator clears error bit
	return (command);
}

uint16_t as5048aReadCommand(uint16_t spiSendCommand)
{
	uint16_t returnVal = 0;

	//spiSendCommand |= (1<<14); 					//OR operator sets the read/write bit to READ
	//spiSendCommand = as5048aSetParity(spiSendCommand);
	Serial.print("spiSendCommand = ");
	Serial.println(spiSendCommand);

	SPI.beginTransaction(SPI_SETTINGS_HALL);
	SPI_HALL_SELECT();
	returnVal = SPI.transfer16(spiSendCommand);
	SPI_HALL_UNSELECT();
	SPI.endTransaction();
	return returnVal;
}

/*
uint8_t as5048aWriteCommand(uint16_t spiSendAddress, uint16_t spiSendCommand)
{
	uint16_t returnVal = 0;

	spiSendAddress &= ~(1<<14);								//AND operator sets the read/write bit to WRITE
	spiSendAddress = as5048aSetParity(spiSendAddress);		//sets parity required by as5048a

	spiSendCommand &= ~(1<<14);								//AND operator sets the read/write bit to WRITE
	spiSendCommand = as5048aSetParity(spiSendCommand);		//sets parity required by as5048a

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
	uint16_t upperZeroValue = 0;
	uint16_t lowerZeroValue = 0;
	uint8_t hallDataStatus = 0;

	//get position first
	as5048aReadCommand(HALL_GET_ANGLE);

	for(int i = 0; i<10; i++)
	{
		hallZeroPosition += as5048aReadCommand(HALL_GET_ANGLE);
	}

	hallZeroPosition = (uint32_t) (hallZeroPosition/10.0);
	if(DEBUG)
	{
		Serial.println("Hall zero measurements complete.");
		Serial.print("Hall sensor zero position is: ");
		Serial.println(hallZeroPosition);
	}

	//separate the upper and lower bits of the zero data to be sent
	//need to get into form of:
	// 0b00000000xxxxxxxx   highest 8 bits of the 14 bit value
	// 0b0000000000xxxxxx   lowest 6 bits of the 14 bit value

	highZeroValue = (hallZeroPosition >> 8) & (BM7 | BM6 | BM5 | BM4 | BM3 | BM2 | BM1 | BM0);
	lowZeroValue = hallZeroPosition & (BM5 | BM4 | BM3 | BM2 | BM1 | BM0);

	hallDataStatus = as5048aWriteCommand(HALL_ZERO_ANGLE_HIGH, highZeroValue)
	if(DEBUG)
	{
		if(hallDataSuccess==0)
		{
			Serial.println("Hall Zero Angle High Write Unsuccessful")	
		}
		else 
		{
			Serial.println("Hall Zero Angle High Write Successful")
		}
	}
	hallDataStatus = as5048aWriteCommand(HALL_ZERO_ANGLE_LOW, lowZeroValue)
	if(DEBUG)
	{
		if(hallDataSuccess==0)
		{
			Serial.println("Hall Zero Angle Low Write Unsuccessful")	
		}
		else 
		{
			Serial.println("Hall Zero Angle Low Write Successful")
		}
	}
	as5048aReadCommand(HALL_GET_ANGLE);			//read angle one last time so the next request will give angle back.
}
*/
uint8_t spiCalcEvenParity(uint16_t value){
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

uint16_t readHallSensorAngle(void){
	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | HALL_GET_ANGLE;

	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	uint8_t right_byte = command & 0xFF;
	uint8_t left_byte = ( command >> 8 ) & 0xFF;


	Serial.print("Read (0x");
	Serial.print(HALL_GET_ANGLE, HEX);
	Serial.print(") with command: 0b");
	Serial.println(command, BIN);


	//SPI - begin transaction
	SPI.beginTransaction(SPI_SETTINGS_HALL);

	//Send the command
	digitalWrite(SPI_HALL_CS, LOW);
	SPI.transfer(left_byte);
	SPI.transfer(right_byte);
	digitalWrite(SPI_HALL_CS,HIGH);

	//Now read the response
	digitalWrite(SPI_HALL_CS, LOW);
	left_byte = SPI.transfer(0x00);
	right_byte = SPI.transfer(0x00);
	digitalWrite(SPI_HALL_CS, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

  uint16_t combinedValue = (left_byte<<8)|(right_byte);
  

	Serial.print("Read returned: ");
	Serial.print(combinedValue);


	//Check if the error bit is set
	if (left_byte & 0x40) {
		Serial.println("Setting Error bit");
		//errorFlag = true;
	}
	else {
		//errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return (( ( left_byte & 0xFF ) << 8 ) | ( right_byte & 0xFF )) & ~0xC000;
}
//gets the position of the low speed shaft from the hall sensor IC
void getHallPosition(void)
{
	uint16_t hallPositionSum_nominal = 0;

	for(int i=0; i<HALL_AVERAGE_SIZE; i++)
	{
		hallPositionSum_nominal += as5048aReadCommand(HALL_GET_ANGLE);
	}

	Serial.print("Hall position = ");
	Serial.println(hallPositionSum_nominal);

	averagedHallPosition_deg = (hallPositionSum_nominal/HALL_AVERAGE_SIZE) / (16384/360); 			//average readings to find 0-16383 range from sensor, 0-360 degrees
}

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

void loop()
{
	/*
	if(canIntRecv ==1)
	{
		canIntRecv = 0;
		getCanMsg();
	}
	if(millis() - previousPidMillis >= PID_EXECUTION_INTERVAL)
	{
		previousPidMillis = millis();
		getHallPosition();
		executePid();
	}
	if(millis() - previousCanMillis >= CAN_SEND_INTERVAL)
	{
		calculateCurrent();
		calculateVoltage();
		calculateTemperature();
		filterHallPosition();
		sendCanMsg();
	}
	*/
	readHallSensorAngle();
	delay(500);
}

void MCP2515_ISR()
{
    canIntRecv = 1;
}
