#ifndef SENDSERIALH
#define SENDSERIALH

#include "defines.h"
#include "Arduino.h"
#include "globalExtern.h"
#include "canMessages.h"

void sendArrayToEPStoLKASSerial(uint8_t *array);
void sendArrayToLKAStoEPSSerial(uint8_t *array);


//the name says it all. requites 5 byte array pointer
void sendArrayToEPStoLKASSerial(uint8_t *array){
	
	EPStoLKAS_Serial.write(*array);
	EPStoLKAS_Serial.write(*(array+1));
	EPStoLKAS_Serial.write(*(array+2));
	EPStoLKAS_Serial.write(*(array+3));
	EPStoLKAS_Serial.write(*(array+4));
}


void sendArrayToLKAStoEPSSerial(uint8_t *array){

	LKAStoEPS_Serial.write(*array);
	LKAStoEPS_Serial.write(*(array+1));
	LKAStoEPS_Serial.write(*(array+2));
	LKAStoEPS_Serial.write(*(array+3));
	
	CAN_msg_t thisCanMsg;
	thisCanMsg.id = 0x200;
	thisCanMsg.len = 4;
	thisCanMsg.buf[0] = *array;
	thisCanMsg.buf[1] = *(array+1);
	thisCanMsg.buf[2] = *(array+2);
	thisCanMsg.buf[3] = *(array+3);
	// FCAN.write(thisCanMsg);
	// can.transmit(thisCanMsg.id,thisCanMsg.buf, thisCanMsg.len);
	sendCanMsg(&thisCanMsg);




	#ifdef DEBUG_PRINT_LKAStoEPS_LIN_OUTPUT
	outputSerial.print("\nL-O:");
	printuint_t(*array);
	outputSerial.print("  ");
	printuint_t(*(array+1));
	outputSerial.print("  ");
	printuint_t(*(array+2));
	outputSerial.print("  ");
	printuint_t(*(array+3));
	outputSerial.print("  ");
	#endif
    
    //TODO: send this data to canbus

}


#endif