/*
This is the firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
July 2016
*/

#include <SPI.h>
#include <mcp_can.h>
#include "ThrottleBodyController.h"
#include <TimerOne.h>
#include <math.h>

uint8_t canIntRecv = 0;
uint32_t previousPidMillis = 0;
uint32_t previousCanMillis = 0;
uint32_t previousValidityMillis = 0;
uint32_t pidTuningMillis1 = 0;
uint8_t pidTuningState = 0;

//received values from CAN
uint16_t instThrottleRequest_percentx10 = 0;
float instThrottleRequest_percent = 0;
uint8_t instThrottleBlip_deg = 0;
uint8_t instThrottleBlip_ms = 0;

//values to be sent over CAN
uint16_t filteredCurrentDraw_mA = 0;									
uint16_t filteredVoltage_mV = 0;										
uint8_t filteredTemp_degCx2 = 0;							
uint16_t zeroedHallPosition_percentx10 = 0;							

//sensor feedback not sent over CAN
uint16_t absoluteHallPosition_nominal = 0;	
int32_t zeroedHallPosition_nominal = 0;         //note this is signed
uint16_t hallZeroPosition_nominal = 0;
float zeroedHallPosition_percent = 0;

float controllerPTerm = 0;
float controllerITerm = 0;
float controllerDTerm = 0;
float controllerPositionErrorPrev_percent = 0;
float controllerResult = 0;											


MCP_CAN CAN(SPI_CAN_CS);                      //set CS CAN pin

uint16_t as5048aReadAndClearError(void)
{
	uint16_t spiSendCommand = HALL_GET_ERROR;
	uint16_t hallErrorMsg = 0b1111111111111111;

	spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
	spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

	SPI.beginTransaction(SPI_SETTINGS_HALL);

	digitalWrite(SPI_HALL_CS, LOW);
	SPI.transfer16(spiSendCommand);					// returned value is garbage angle from last request
	digitalWrite(SPI_HALL_CS, HIGH);

	spiSendCommand = HALL_GET_ANGLE;				// reset spi command to read angle (in preparation for next request)
	spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
	spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

	digitalWrite(SPI_HALL_CS, LOW);
	hallErrorMsg = SPI.transfer16(spiSendCommand);	// returned value is error message from last request
	digitalWrite(SPI_HALL_CS, HIGH);

	SPI.endTransaction();
	return hallErrorMsg;
}

void as5048aCheckError(uint16_t command) {
	if ((command & 0b0100000000000000) == 0b0100000000000000) 
	{
		Serial.print("Hall Sensor Error: ");
		Serial.println(as5048aReadAndClearError(), BIN);
	}
	return;	
}

uint16_t as5048aRemoveParity(uint16_t command)
{
	as5048aCheckError(command);
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


void as5048aReadAndAverage(uint16_t spiSendCommand)
{
	uint32_t hallPositionSum_nominal = 0;

	spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
	spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

	
	SPI.beginTransaction(SPI_SETTINGS_HALL);
	//SPI.transfer(spiSendCommand);
	//SPI.transfer(0);
	//delayMicroseconds(250);

	for(int i=0; i<HALL_AVERAGE_SIZE; i++)
	{
		// //SPI.transfer16(spiSendCommand);
		// digitalWrite(SPI_HALL_CS, HIGH);
		// SPI.transfer16(0x00);

		digitalWrite(SPI_HALL_CS, LOW);
		hallPositionSum_nominal += (uint32_t) (16384 - as5048aRemoveParity(SPI.transfer16(spiSendCommand)));
		digitalWrite(SPI_HALL_CS, HIGH);
		delayMicroseconds(250);
		//SPI.transfer(0);
	}	

	SPI.endTransaction();

	absoluteHallPosition_nominal = (uint16_t) (hallPositionSum_nominal >> filterShiftSize(HALL_AVERAGE_SIZE));
    //absoluteHallPosition_nominal = 16383 - absoluteHallPosition_nominal;							//reverse direction sensor works on
}

void getZeroedHallPosition(void)
{
	as5048aReadAndAverage(HALL_GET_ANGLE);
	zeroedHallPosition_nominal =  (int32_t) absoluteHallPosition_nominal - (int32_t) hallZeroPosition_nominal;
	if(zeroedHallPosition_nominal < 0)
	{
		zeroedHallPosition_nominal = 0;
	}

	zeroedHallPosition_percentx10 = (((uint32_t) zeroedHallPosition_nominal)*10) >> 5; //same as dividing by 32, or doing the complex float math we had here.
	Serial.println(zeroedHallPosition_percentx10);
	// zeroedHallPosition_percent = (float) zeroedHallPosition_nominal*360.0/16384.0*100.0/69.2;
	// zeroedHallPosition_percentx10 = (uint16_t) ((float)zeroedHallPosition_percent*10.0);
	// Serial.println(zeroedHallPosition_percentx10);
}


void findHallZeroPosition(void)
{
	uint32_t hallZeroPositionSum_nominal = 0;

	as5048aReadAndAverage(HALL_GET_ANGLE);
	hallZeroPosition_nominal = absoluteHallPosition_nominal;
	
	Serial.print("hallZeroPosition_nominal = ");
	Serial.println(hallZeroPosition_nominal);
}


//gets the CAN message in the buffer, and reads the required values
void getCanMsg(void)
{
	uint8_t canMsgLength = 0;
	uint8_t canMsgData[8];

	while (CAN_MSGAVAIL == CAN.checkReceive()) 
	{
		// read data,  len: data length, buf: data buf
		CAN.readMsgBuf(&canMsgLength, canMsgData);
		instThrottleRequest_percentx10 = ((canMsgData[1]<<8) | canMsgData[0]);
		//instThrottleBlip_ms = canMsgData[2];
		//instThrottleBlip_deg = canMsgData[3];
		instThrottleRequest_percent = (float) instThrottleRequest_percentx10 / 10.0;
	}
}


//sends the outgoing CAN message with updated variables
void sendCanMsg(void)
{
	uint8_t canSendBuffer1[8];
	uint8_t canSendBuffer2[8];

	canSendBuffer1[0] = zeroedHallPosition_percentx10 & 0b11111111;
	canSendBuffer1[1] = zeroedHallPosition_percentx10 >> 8;
	canSendBuffer1[2] = filteredCurrentDraw_mA & 0b11111111;
	canSendBuffer1[3] = filteredCurrentDraw_mA >> 8;
	canSendBuffer1[4] = filteredVoltage_mV & 0b11111111;
	canSendBuffer1[5] = filteredVoltage_mV >> 8;
	canSendBuffer1[6] = filteredTemp_degCx2;
	canSendBuffer1[7] = 0;

	canSendBuffer2[0] = (int8_t) instThrottleRequest_percent;
	canSendBuffer2[1] = ((int8_t) (100.0 * controllerResult)) & 0b01111111;
	canSendBuffer2[2] = 0;
	canSendBuffer2[3] = 0;
	canSendBuffer2[4] = 0;
	canSendBuffer2[5] = 0;
	canSendBuffer2[6] = 0; 
	canSendBuffer2[7] = 0; 
	
	SPI.beginTransaction(SPI_SETTINGS_CAN);
	CAN.sendMsgBuf(CAN_FEEDBACK_MSG_ADDRESS,0,8,canSendBuffer1);
	CAN.sendMsgBuf(0x104, 0, 8, canSendBuffer2);

	// while(CAN_MSGAVAIL == CAN.checkReceive())
	// {
 //    	uint8_t canMsgLength = 0;
 //    	uint8_t canMsgData[8];

 //    	CAN.readMsgBuf(&canMsgLength, canMsgData);
 //    	instThrottleRequest_percentx10 = ((canMsgData[1]<<8) | canMsgData[0]);
 //    	instThrottleRequest_percent = (float) instThrottleRequest_percentx10 / 10.0;
	// }

	SPI.endTransaction();
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


void calculateTemperature(void)
{
	float rThermistor_ohms = (float) 5.0*5000.0/(analogRead(TEMP_SENS)*5.0/1023.0);
  float lnOperand = rThermistor_ohms/(10000*pow(2.71828, (-THERMISTOR_B_CONSTANT/298.0)));
  float temp_degCx2 = 2.0 * (THERMISTOR_B_CONSTANT / log(lnOperand) - 273 + THERMISTOR_CALIB_DEGC);
  //Serial.println((uint32_t) temp_degCx2);

	filteredTemp_degCx2 *= ADC_FILTER_SIZE;
	filteredTemp_degCx2 = (uint16_t) ((float) ((filteredTemp_degCx2 - (filteredTemp_degCx2/ADC_FILTER_SIZE) + temp_degCx2)/ADC_FILTER_SIZE));
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
		default:
			shiftSize=0;
			break;

	}
	return shiftSize;	
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
	//calculate error

	float controllerPositionError_percent = instThrottleRequest_percent - zeroedHallPosition_percent;
/*
	if(instThrottleRequest_percent < 1.0)
	{
		controllerPositionError_percent = 0.0;
	}
*/
	controllerPTerm = (float) controllerPositionError_percent * CONTROLLER_KP;
	//controllerITerm = (float) controllerPositionError_percent * PID_EXECUTION_INTERVAL/1000.0 * CONTROLLER_KI;                                                //integral and derivative need to be fixed!!!
	//controllerDTerm = (float) (controllerPositionError_percent - controllerPositionErrorPrev_percent) / (PID_EXECUTION_INTERVAL/1000.0) * CONTROLLER_KD;

	//done calculating D term, can set previous to current now
	//controllerPositionErrorPrev_percent = controllerPositionError_percent;
/*
	// Conditions applied to integral term
	if(controllerPositionError_percent < CONTROLLER_I_TERM_RESET_THRESH_PERCENT)
	{
		controllerITerm = 0;
	}
	else if(controllerITerm > CONTROLLER_I_TERM_MAX)
	{
		controllerITerm = CONTROLLER_I_TERM_MAX;
	}
	else if(controllerITerm < CONTROLLER_I_TERM_MIN)
	{
		controllerITerm = CONTROLLER_I_TERM_MIN;
	}

	// Conditions applied to derivative term
	if(controllerDTerm > CONTROLLER_D_TERM_MAX)
	{
		controllerDTerm = CONTROLLER_D_TERM_MAX;
	}
	else if(controllerDTerm < CONTROLLER_D_TERM_MIN)
	{
		controllerDTerm = CONTROLLER_D_TERM_MIN;
	}
*/
	// Calculate controller result
	controllerResult = controllerPTerm + controllerITerm + controllerDTerm;

	// Limit controller effort
	if(controllerResult > CONTROLLER_EFFORT_MAX)
	{
		controllerResult = CONTROLLER_EFFORT_MAX;
	}
	else if(controllerResult < CONTROLLER_EFFORT_MIN)
	{
		controllerResult = CONTROLLER_EFFORT_MIN;
	}

	// Take action on the pin ports
	if(controllerResult > 0)
	{
		Timer1.pwm(MOTOR_CLOSE_PIN, 0);
		controllerResult = controllerResult * 255;
		analogWrite(MOTOR_OPEN_PIN, controllerResult);
	}
	else if(controllerResult <= 0)
	{
		analogWrite(MOTOR_OPEN_PIN, 0);
		// use spring to close
	}
}


void setup()
{

	Serial.begin(115200);
    SPI.begin();
	Timer1.initialize(1000);  // 1000 us = 1000 Hz

	//output = 1, input = 0
	DDRB |= 0b00001110; //PB1 and PB2 are outputs
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

  Timer1.pwm(MOTOR_CLOSE_PIN, 0);
  analogWrite(MOTOR_OPEN_PIN, 0);

	//CAN.init_Mask(0, 0, 0xFFF);												//must set both masks; use standard CAN frame
	//CAN.init_Mask(1, 0, 0xFFF);												//must set both masks; use standard CAN frame
	//CAN.init_Filt(0, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 0 for receive buffer 0
	//CAN.init_Filt(2, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 1 for receive buffer 1

	//attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

	delay(500);
	findHallZeroPosition();
}



void loop()
{
	if(millis() - previousPidMillis >= PID_EXECUTION_INTERVAL)
	{
		previousPidMillis = millis();
		getZeroedHallPosition();
		//executePid();

	}

	// if(millis() - previousValidityMillis >= VALIDITY_CHECK_INTERVAL)	
	// {
	// 	previousValidityMillis = millis();
	// 	getZeroedHallPosition();
	// }


	if(millis() - previousCanMillis >= CAN_SEND_INTERVAL)
	{
		previousCanMillis = millis();
		// calculateCurrent();
		// calculateVoltage();
		// calculateTemperature();
		//delayMicroseconds(250);
		sendCanMsg();
		delayMicroseconds(250);
	}
}

void MCP2515_ISR()
{
	canIntRecv = 1;
}
