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