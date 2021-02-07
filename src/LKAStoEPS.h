#ifndef LKASTOEPSH
#define LKASTOEPSH


// #include <FlexCAN_T4.h>
#include "globalExtern.h"
#include "checksums.h"
#include "canMessages.h"
#include "sendSerial.h"
#include "createLINMessages.h"


void  deconstructLKASMessage(uint8_t msg);


void handleLKAStoEPS(){
    
    if(!LKAStoEPS_Serial.available()) return;

	uint8_t rcvdByte = LKAStoEPS_Serial.read();
	deconstructLKASMessage(rcvdByte);
	
	// OLD version
    // if(OPSteeringControlMessageActive && !LkasFromCanFatalError){
    //     
    //     if(incomingMsg.counterBit == 0){
    //         if(OPLkasActive) createKLinMessageWBigSteerAndLittleSteer(OPBigSteer,OPLittleSteer);
    //         else sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]); 
    //     }
    //     if(	(millis() - OPTimeLastCANRecieved) > 50 ){
    //         LkasFromCanFatalError = true;
    //         LkasFromCanStatus = 1;
    //     }
    // } if(OPSteeringControlMessageActive) {
    //     if(incomingMsg.counterBit == 0) sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]); 
    // } else{
    //     LKAStoEPS_Serial.write(rcvdByte);    
    // }

	//new version
	if(incomingMsg.counterBit == 0){
		OPSteeringControlMessageActive = OPSteeringControlMessageStatusPending;
		if(LkasFromCanFatalError) OPLkasActive = false;
		if(OPSteeringControlMessageActive ){
			if(OPLkasActive) createKLinMessageWBigSteerAndLittleSteer(OPBigSteer,OPLittleSteer);
            else sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]);
		} else {
			sendArrayToLKAStoEPSSerial(&lkas_off_array[incomingMsg.counterBit][0]);
		}
	}else { // not the first byte in the frame
		if(!OPSteeringControlMessageActive) LKAStoEPS_Serial.write(rcvdByte);
	}


#ifdef DEBUG_PRINT_LKAStoEPS_LIN_INPUT
		outputSerial.print("\nL-I:");
		outputSerial.print(incomingMsg.totalCounter,DEC);
		outputSerial.print("-");
		printuint_t(rcvdByte);
#endif
}




void  deconstructLKASMessage(uint8_t msg){ //
	//figure out which message byte your in.. check if first 2 bits are 0's

	uint8_t offset4 = msg >> 4;
	if( offset4 < 4 ){
		incomingMsg.totalCounter = 0;
	} else {
		// outputSerial.write('\n');
		// printuint_t(msg);
		incomingMsg.totalCounter++;
	}
	if(incomingMsg.totalCounter > 3){ // ERROR!! this shouldnt happen
#ifdef DEBUG_PRINT_LKAStoEPS_ERRORS_SERIAL
		outputSerial.print("\nERROR:  incomingMsg.totalCounter is > 3 -- ");
		outputSerial.print(incomingMsg.totalCounter,DEC);
		outputSerial.println(" --   impossible  . resetting to 0 *** ");
#endif
		incomingMsg.totalCounter = 0;
	}
	incomingMsg.data[incomingMsg.totalCounter] = msg;
	switch (incomingMsg.totalCounter ){
		case 0:
			incomingMsg.counterBit = msg >> 5;
			incomingMsg.bigSteer = msg & B00001111;
			break;
		case 1:
			incomingMsg.littleSteer = msg & B00011111;
			incomingMsg.lkasOn = (msg >> 5) & B00000001;
			break;
		case 2:

			break;

		case 3:
			incomingMsg.checksum = msg;
			break;
	}
} // deconstructLKASMessage(uint8_t msg)



#endif