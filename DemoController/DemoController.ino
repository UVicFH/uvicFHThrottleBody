#include <mcp_can.h>
#include <SPI.h>

/*
 * INPUTS:
 * Position POT to A0, 0 is low, 90 is high
 * WOT Button to D5, 0 is low, 1 is high
 * BLIP Button to D6, 0 is low, 1 is high
 * 
 * Drive buttons high (5V).
 * 
 * OUTPUTS:
 * CAN
 * POS over 0x00, standard frame, 4 bytes, takes float 
 * WOT over 0x01, standard frame, 1 byte, takes bool t/f
 * BLIP over 0x02, standard frame, 1 byte, takes bool t/f
 */
 
#define SPI_CS_PIN  9 // set SPI to D9 pin
MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin

uint32_t previousMillis_ms = 0;
uint32_t blipStart_ms = 0;
uint32_t blipEnd_ms = 0;

void setup() {
   Serial.begin(115200);
   while (CAN_OK != CAN.begin(CAN_500KBPS)) { // init can bus : baudrate = 500k
      Serial.println("CAN BUS Shield init. fail"); // fail code if
      Serial.println("Init. CAN BUS Shield again");
      delay(100);
   }
   Serial.println("CAN BUS Shield init. ok");
   pinMode(5,INPUT);
   pinMode(6,INPUT);
}

void loop() {
   if(millis() - previousMillis_ms > 10)
   {
     previousMillis_ms = millis();
     
     uint8_t buttonWot = digitalRead(5);
     uint8_t buttonBlip = digitalRead(6);
     uint16_t throttleRequest_percentx10 = (uint16_t) ((float) 1000.0 * analogRead(A0) / 1023.0);

     if(buttonBlip == 1 && ((millis() - blipStart_ms)>1000))
     {
       blipStart_ms = millis();
       blipEnd_ms = millis()+250;
     }
     if(millis() < blipEnd_ms)
     {
       throttleRequest_percentx10 += 100; //add a 10 percent blip
     }

     if(buttonWot == 1)
     {
       throttleRequest_percentx10 = 1000;
     }
     
     

     Serial.println(throttleRequest_percentx10);

     uint8_t canSendBuffer[3];
     canSendBuffer[0] = throttleRequest_percentx10 & 0b11111111;
     canSendBuffer[1] = throttleRequest_percentx10 >> 8;
     canSendBuffer[2] = buttonBlip;
   
     CAN.sendMsgBuf(0x102, 0, 3, canSendBuffer);   // send position over CAN

     canSendBuffer[0] = 0;
     canSendBuffer[1] = 0;
     canSendBuffer[2] = 0;

     CAN.sendMsgBuf(0x103, 0, 3, canSendBuffer);
     CAN.sendMsgBuf(0x106, 0, 3, canSendBuffer);
   }
}
