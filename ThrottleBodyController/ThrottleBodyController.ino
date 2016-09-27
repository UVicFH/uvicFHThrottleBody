/*
This is the firmware for the UVic Formula Hybrid custom throttle body.
Ted Alley
September 2016

VERY IMPORTANT!!!!!!!!!!!!
wiring.c modified according to http://playground.arduino.cc/Main/TimerPWMCheatsheet. PRESCALE_FACTOR = 1
wiring.c file within folder C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino

*/

#include <SPI.h>
#include <mcp_can.h>
#include "ThrottleBodyController.h"
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
uint16_t currentDraw_mA = 0;									
uint16_t voltage_mV = 0;										
uint8_t temp_degCx2 = 0;							
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


MCP_CAN CAN(SPI_CAN_CS);                      //set CS CAN pin. used for CAN library operation.

//requests the error register and returns the value. second request is angle request so we can go back into normal operation.
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

//prints the value of the error register
void as5048aCheckError(uint16_t command) {
	if ((command & 0b0100000000000000) == 0b0100000000000000) 
	{
		Serial.print("Hall Sensor Error: ");
		Serial.println(as5048aReadAndClearError(), BIN);
	}
	return;	
}

//removes the first two bits of any received message, checks for error bit
uint16_t as5048aRemoveParity(uint16_t command)
{
	as5048aCheckError(command);
	return (command &= 0b0011111111111111);                 //clears parity and error bit
}


//returns the parity bit to create even parity in a 16 bit message
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

//sends the request to the hall sensor to read the position. calculates the read value from the sensor.
void as5048aReadAndAverage(uint16_t spiSendCommand)
{
	uint32_t hallPositionSum_nominal = 0;

	spiSendCommand |= 0b0100000000000000;           //OR operator sets the read/write bit to READ
	spiSendCommand |= ((uint16_t)as5048aSetParity(spiSendCommand)<<15);

	SPI.beginTransaction(SPI_SETTINGS_HALL);
	for(int i=0; i<HALL_AVERAGE_SIZE; i++)
	{
		digitalWrite(SPI_HALL_CS, LOW);
		hallPositionSum_nominal += (uint32_t) (16384 - as5048aRemoveParity(SPI.transfer16(spiSendCommand)));
		digitalWrite(SPI_HALL_CS, HIGH);
		delayMicroseconds(250);
	}	
	SPI.endTransaction();

	absoluteHallPosition_nominal = (uint16_t) (hallPositionSum_nominal >> filterShiftSize(HALL_AVERAGE_SIZE));
}

//used to subtract the read value from the pre-established zero point. Note integer types used for safe execution
void getZeroedHallPosition(void)
{
	as5048aReadAndAverage(HALL_GET_ANGLE);
	zeroedHallPosition_nominal =  (int32_t) absoluteHallPosition_nominal - (int32_t) hallZeroPosition_nominal;
	if(zeroedHallPosition_nominal < 0)
	{
		zeroedHallPosition_nominal = 0;
	}
	zeroedHallPosition_percentx10 = (((uint32_t) zeroedHallPosition_nominal)*10) >> 5; //same as dividing by 32, or doing zeroedHallPosition_nominal*360.0/16384.0*100.0/69.2.
}


//used at the beginning of program execution to find the zero point of the sensor.
void findHallZeroPosition(void)
{
	uint32_t hallZeroPositionSum_nominal = 0;

	as5048aReadAndAverage(HALL_GET_ANGLE);
	hallZeroPosition_nominal = absoluteHallPosition_nominal;
	
	Serial.print("hallZeroPosition_nominal = ");
	Serial.println(hallZeroPosition_nominal);
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
	SPI.endTransaction();
}

//gets the CAN message in the buffer, and reads the required values
void receiveCanMsg(void)
{
	while(CAN_MSGAVAIL == CAN.checkReceive())		//this used to be an 'if', try changing back if code does not work.
	{
  		uint8_t buf[8];
  		uint8_t len =0;
  		CAN.readMsgBuf(&len, buf);
  		instThrottleRequest_percentx10 = (buf[0] | (buf[1])<<8);
  		instThrottleRequest_percent = ((float) instThrottleRequest_percentx10) / 10.0;
  	}
}


//calculates the current consumption of the device. only reports current value; no averaging.
void calculateCurrent(void)
{
	currentDraw_mA = (uint16_t) (analogRead(CURRENT_SENS) * 10)  // analogRead(CURRENT_SENS)/1023.0*5/(0.01*0.05*1000)*1000
}

//calculates the voltage input to the device, accounts for voltage drop through shunt
void calculateVoltage(void)
{
	uint16_t voltage_mV = (uint16_t) (analogRead(VOLTAGE_SENS) * 20 + (float)(0.05 * currentDraw_mA)); //analogRead(VOLTAGE_SENS)/1023.0*5000*13.3/3.3
}

//calculates temperature according to thermistor calibrated curve
void calculateTemperature(void)
{
	float rThermistor_ohms = (float) 5.0*5000.0/(analogRead(TEMP_SENS)*5.0/1023.0);
    float lnOperand = rThermistor_ohms/(10000*pow(2.71828, (-THERMISTOR_B_CONSTANT/298.0)));
    temp_degCx2 = (uint8_t) (2.0 * (THERMISTOR_B_CONSTANT / log(lnOperand) - 273.0 + THERMISTOR_CALIB_DEGC));
}

//calculates the number of bit shifts required to perform a given division
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

//error check the two positions. still to be done.
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
		controllerResult = controllerResult * 255;
		analogWrite(MOTOR_OPEN_PIN, controllerResult);
	}
	else if(controllerResult <= 0)
	{
		analogWrite(MOTOR_OPEN_PIN, 0);
		// use spring to close
	}
}

//run once at the beginning of execution
void setup()
{
	TCCR0B = TCCR0B & 0b11111000 | 0x01;		//modify the prescale factor of timer0 so pins 5 and 6 have faster pwm (62.5kHz)
	TCCR1B = TCCR1B & 0b11111000 | 0x01;		//modify the prescale factor of timer1 so pins 9 and 10 have faster pwm (31.4kHz)

	Serial.begin(115200);
    SPI.begin();

	//Setting input and output pins. This was suitable for ETB Controller PCB R1, but will need to be adjusted for ETB Controller PCB R2.
	//output = 1, input = 0
	DDRB |= 0b00001110; //PB1 and PB2 are outputs
	DDRC |= 0b00000000; //no outputs on PC
	DDRD |= 0b00110000; //PD4 PD5 are outputs

	//Enable CAN communications
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

	//Set can masks and filters to only accept the throttle request message (this is critical so that the SPI bus doesn't get bogged down
	// with traffic due to the MCP2515 receiving other messages on the bus)
	CAN.init_Mask(0, 0, 0xFFF);												//must set both masks; use standard CAN frame
	CAN.init_Mask(1, 0, 0xFFF);												//must set both masks; use standard CAN frame
	CAN.init_Filt(0, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 0 for receive buffer 0
	CAN.init_Filt(2, 0, CAN_THROTTLE_MSG_ADDRESS);							//filter 1 for receive buffer 1								

	
	delay(250);		//delay after setting filters and masks before getting the zero position of the hall sensor.

	findHallZeroPosition();
}


//run continuously during execution
void loop()
{
	if(millis() - previousPidMillis >= PID_EXECUTION_INTERVAL)
	{
		previousPidMillis = millis();
		getZeroedHallPosition();
		//executePid();
	}

	if(millis() - previousValidityMillis >= VALIDITY_CHECK_INTERVAL)	//doesn't actually get value from second sensor. simply polls first sensor again to simulate more spi traffic.
	{
		previousValidityMillis = millis();
		getZeroedHallPosition();
	}

	if(millis() - previousCanMillis >= CAN_SEND_INTERVAL)
	{
		previousCanMillis = millis();
		// calculateCurrent();
		// calculateVoltage();
		// calculateTemperature();
		sendCanMsg();
		receiveCanMsg();
		delayMicroseconds(250);
	}
}
