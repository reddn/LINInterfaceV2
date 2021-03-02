#include "defines.h"
#include <Arduino.h>
// #include <FlexCAN_T4.h>
#include "globalVars.h"
#include <eXoCAN.h>
#include  "canMessages.h"
#include "checksums.h"
#include "EPStoLKAS.h"
#include "LKAStoEPS.h"
#include "struct.h"
#include "sendSerial.h"

uint8_t blueLedOn = 0;

void handleLedFlashing();

//called from the main loop.  reads only when needed by the TIME_BETWEEN_DIGIAL_READS define, in milliseconds
//this helps from too many digitalRead calls slowing down reading from the other buses.
//reads all Pushbuttons and DIP switches to set them into a variable.  
//also reads the POT
void handleInputReads(){
	if( ( millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS){
		handleLedFlashing();
		
		if(LkasFromCanChecksumErrorCount > 2){
			OPSteeringControlMessageStatusPending = false;
			LkasFromCanFatalError = true;
			LkasFromCanStatus = 2;
			LkasFromCanChecksumError = 1;
		} else if(!LkasFromCanFatalError) LkasFromCanChecksumErrorCount = 0;

		if(OPTimeLastCANRecieved != 0){
			if(OPLkasActive) mainLedBlinkTimer = 500;
			else if( (millis() - OPTimeLastCANRecieved) < 1000 ) mainLedBlinkTimer = 1000;
			else mainLedBlinkTimer = 2000; 

			if(	(millis() - OPTimeLastCANRecieved) > 50){
				OPSteeringControlMessageStatusPending = false;
				LkasFromCanFatalError = true;
				LkasFromCanStatus = 1;
			}
		}

		lastDigitalReadTime = millis();
	} // end if(millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS)
} // end handleInputReads

void handleLedFlashing(){
	//OPSteeringControlMessageActive;
	
	digitalWrite(STATUS_LED,( EPStoLKASBuffer[2] >> 2 ) & B00000001);

	if( (millis() - lastRedLedToggle ) > mainLedBlinkTimer) {
		if(blueLedOn) {
			digitalWrite(BLUE_LED,0);
			blueLedOn = 0;
		} else{
			analogWrite(BLUE_LED,128);
			blueLedOn = 1;
		}
	}
} // end handleLedFlashing()

                    /*************** S E T U P ***************/
void setup() {
	EPStoLKAS_Serial.begin(9600,SERIAL_8E1);
	LKAStoEPS_Serial.begin(9600,SERIAL_8E1);
	// Serial.begin(9600);

	pinMode(BLUE_LED,OUTPUT);
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
	digitalWrite(BLUE_LED, LOW);
	// canSetup();
	// FCAN.begin();
	// FCAN.setBaudRate(500000);
	can.begin(STD_ID_LEN, BR500K, PORTB_8_9_XCVR);
    can.filterList16Init(0,0xe4,0,0,0);
	canMsg.busConfig = PORTB_8_9_XCVR;

}
int zeroVal = 0;

                    /*************** L O O P ***************/
void loop() {
  handleLKAStoEPS();
  handleEPStoLKAS();
  handleInputReads();

  if(can.receive(canMsg.txMsgID,zeroVal,canMsg.txMsg.bytes) > -1){
	handleLkasFromCanV3();
  }
}