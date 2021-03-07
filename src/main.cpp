#include "defines.h"
#include <Arduino.h>
#include "globalVars.h"
#include <eXoCAN.h>
#include  "canMessages.h"
#include "checksums.h"
#include "EPStoLKAS.h"
#include "LKAStoEPS.h"
#include "struct.h"
#include "sendSerial.h"

#define VERSION_YEAR 21
#define VERSION_MONTH 03
#define VERSION_DAY 07
#define VERSION_BUILD 4
#define VERSION_HW 2


uint8_t blueLedOn = 0;

void handleLedFlashing();
void handleSendFirmwareVersion();

//called from the main loop.  reads only when needed by the TIME_BETWEEN_DIGIAL_READS define, in milliseconds
//mainly so it doesnt do these checks in series of the main loop    
void handleInputReads(){
	if( ( millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS){
		handleLedFlashing();
		handleSendFirmwareVersion();
		if(LkasFromCanChecksumErrorCount > 2){
			LkasFromCanFatalError = 1;
			canSteerChecksumFatalError = 1;
		} else  LkasFromCanChecksumErrorCount = 0;;

		if(LkasFromCanCounterErrorCount > 2) {
			canSteerCounterFatalError = 1;
			LkasFromCanFatalError = 1;
		} else LkasFromCanCounterErrorCount = 0;


		if(OPTimeLastCANRecieved != 0){
			if(OPLkasActive) mainLedBlinkTimer = 500;
			else if( (millis() - OPTimeLastCANRecieved) < 1000 ) mainLedBlinkTimer = 1000;
			else mainLedBlinkTimer = 2000; 

			if(	(millis() - OPTimeLastCANRecieved) > 55){
				OPSteeringControlMessageStatusPendingData = false;
				OPSteeringControlMessageStatusPending = true;
				LkasFromCanFatalError = 1;
				OPSteeringMsgFatalLate = 1;
			}
		}

		lastDigitalReadTime = millis();
	} // end if(millis() - lastDigitalReadTime ) > TIME_BETWEEN_DIGITIAL_READS)
} // end handleInputReads

void handleLedFlashing(){
	
	digitalWrite(STATUS_LED,LkasFromCanFatalError);

	if( (millis() - lastRedLedToggle ) > mainLedBlinkTimer) {
		lastRedLedToggle = millis();
		analogWrite(BLUE_LED,105);
	}
	else digitalWrite(BLUE_LED,0);
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


uint8_t sendFirmwareToCanCounter = 230;
void handleSendFirmwareVersion(){
	if(sendFirmwareToCanCounter++ == 0){
		CAN_msg_t thisCanMsg;
		thisCanMsg.id = 0x208;
		thisCanMsg.len = 5;
		thisCanMsg.buf[0] = VERSION_YEAR;
		thisCanMsg.buf[1] = VERSION_MONTH;
		thisCanMsg.buf[2] = VERSION_DAY;
		thisCanMsg.buf[3] = VERSION_BUILD;
		thisCanMsg.buf[4] = VERSION_HW;
		// FCAN.write(thisCanMsg);
		// can.transmit(thisCanMsg.id,thisCanMsg.buf, thisCanMsg.len);
		sendCanMsg(&thisCanMsg);
	}
}

                    /*************** L O O P ***************/
void loop() {
  handleLKAStoEPS();
  handleEPStoLKAS();
  handleInputReads();

  if(can.receive(canMsg.txMsgID,zeroVal,canMsg.txMsg.bytes) > -1){
	handleLkasFromCanV3();
  }
}