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



//called from the main loop.  reads only when needed by the TIME_BETWEEN_DIGIAL_READS define, in milliseconds
//this helps from too many digitalRead calls slowing down reading from the other buses.
//reads all Pushbuttons and DIP switches to set them into a variable.  
//also reads the POT
void handleInputReads(){
	if( ( millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS){

		if( (millis() - OPTimeLastCANRecieved) < 1000 ) mainLedBlinkTimer = 200;
		else mainLedBlinkTimer = 2000; 
		
		//digitalWrite(BLUE_LED,( EPStoLKASBuffer[2] >> 2 ) & B00000001);
		
		if(LkasFromCanChecksumErrorCount > 2){
			OPSteeringControlMessageStatusPending = false;
			LkasFromCanFatalError = true;
			LkasFromCanStatus = 2;
		} else if(!LkasFromCanFatalError) LkasFromCanChecksumErrorCount = 0;


		if(	(millis() - OPTimeLastCANRecieved) > 50 ){
			OPSteeringControlMessageStatusPending = false;
			LkasFromCanFatalError = true;
			LkasFromCanStatus = 1;
		}

		lastDigitalReadTime = millis();
	} // end if true
}


                    /*************** S E T U P ***************/
void setup() {
	EPStoLKAS_Serial.begin(9600,SERIAL_8E1);
	LKAStoEPS_Serial.begin(9600,SERIAL_8E1);
	// outputSerial.begin(OUTPUTSERIAL_BAUD);
	// pinMode(analogRotaryInputPin,INPUT);
	// pinMode(PB1_spoofLKASLeft, INPUT_PULLUP);
	// pinMode(PB2_spoofLKASRight, INPUT_PULLUP);
	// pinMode(PB3_spoofLKASStop, INPUT_PULLUP);
	// pinMode(PB4_spoofLKASSteerWithPOTEnablePin, INPUT_PULLUP);
	// pinMode(A1_applySteeringPotPin, INPUT_PULLUP);
	// pinMode(DIP1_spoofFullMCU, INPUT_PULLUP);
	// pinMode(DIP2,INPUT_PULLUP);
	// pinMode(DIP5,INPUT_PULLUP); // **disabled, removed from code** DIP 5 is used as a test to disable LIN output...
	// pinMode(DIP7_SpoofSteeringWheelTorqueData_PIN, INPUT_PULLUP);
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

  if(can.receive(zeroVal,canMsg.txMsgID,canMsg.txMsg.bytes) > -1){
		handleLkasFromCanV3();
	}
  
	if((millis() - readLEDblinkLastChange) > mainLedBlinkTimer){
		digitalWrite(STATUS_LED,!digitalRead(STATUS_LED));
		readLEDblinkLastChange = millis();
		handleLkasFromCanV3();
	} 
}