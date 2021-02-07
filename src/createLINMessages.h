#ifndef CREATELINMESSAGESH
#define CREATELINMESSAGESH

#include "Arduino.h"
#include "globalExtern.h"
#include "canMessages.h"
#include "sendSerial.h"

void createKLinMessageWBigSteerAndLittleSteer(uint8_t bigSteer, uint8_t littleSteer);
void createKLinMessage(int16_t applySteer);


//This function takes the signed int to make it into a valid LKAStoEPS message for steering.  
//Since everyting is triggered by the stock LKAS(MCU), the counter bits are synced on those messages.
// so if there is no LKAStoEPS message from the stock MCU, nothing will be sent by this device.  i could change this later, but probably not
void createKLinMessage(int16_t applySteer){
	uint8_t msg[4];
	msg[0] = 0;
	msg[1] = 0;
	msg[2] = 0;
	msg[3] = 0;

	//max is 255, min is -256 in the apply steer PID, if its outside of that (it shouldnt be), then put it 
	//back at 255 or -256
	if (applySteer > 255) applySteer = 255; // 0xFF or B0 1111 1111
	else if(applySteer < -256) applySteer = -256; // B? 1111 1111
	
	if(LkasOnIntroCountDown > 0){ // 
		applySteer = 0;
		LkasOnIntroCountDown--;
	}
	// push the 16 bit applySteer over 5 so you can get the 3MSB not inlucding the sign put it at the beginning of the bigSteer
	uint8_t bigSteer = 0x00;
	// extract the sign of apply steer (int16) and push it over to the 3rd offset, where it would be in the 2nd byte of the frame
	uint8_t bigSteerSign = 0; // >> old version.. doesn't work( (uint8_t) (applySteer >> 12 ) ) & 0x8 ; //0x8 = 0000 1000
	if(applySteer < 0 ) bigSteerSign = B00001000; 
	//get the bigsteer (3 MSB, exlcuding the sign bit) and add(OR) the sign
	bigSteer =  ( (applySteer >> 5) & 0x7 ) | bigSteerSign;  // 0x7 = 0000 0111  
	msg[0] = (incomingMsg.counterBit << 5) |  bigSteer;
	
	
	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	uint8_t littleSteer = (uint8_t) applySteer & 0x1F; // 0x1F = 0001 1111 ... Change only the last 5 bits
	msg[1] = littleSteer | 0xA0;  // 1010 0000
	//								   ^ lkas on
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	
	msg[3] = chksm(msg[0], msg[1], msg[2]);

	
		LKAStoEPS_Serial.write(msg[0]);
		LKAStoEPS_Serial.write(msg[1]);
		LKAStoEPS_Serial.write(msg[2]);
		LKAStoEPS_Serial.write(msg[3]);
	
#ifdef DEBUG_PRINT_LKAStoEPS_OUTPUT
	outputSerial.print("\nC-");
	printArrayInBinary(&msg[0],4);
#endif
}

// same as above 'createKLinMessage', but this accepts only the bigSteer and littleSteer as variables
void createKLinMessageWBigSteerAndLittleSteer(uint8_t bigSteer, uint8_t littleSteer){
	uint8_t msg[4];
	msg[0] = (incomingMsg.counterBit << 5) |  bigSteer;
	
	
	if(littleSteer > 0 || bigSteer > 0){
		littleSteer = littleSteer & B00011110;
		littleSteer = littleSteer | ( (lastLittleSteer1bit ^ 1 ) & B00000001);
	}

	//get the little steer in the 2nd byte. its the last 5 bits of applysteer. also add in the 0xA0 = 1010 0000
	msg[1] = littleSteer | 0xA0;  // 1010 0000
	//								   ^ lkas on
	msg[2] = 0x80; // this is static 0x80 = 1000 0000
	//										 ^ lkas on (1 is off)
	msg[3] = chksm(msg[0], msg[1], msg[2]);


	LKAStoEPS_Serial.write(msg[0]);
	LKAStoEPS_Serial.write(msg[1]);
	LKAStoEPS_Serial.write(msg[2]);
	LKAStoEPS_Serial.write(msg[3]);

	lastLittleSteer1bit = littleSteer & B00000001;

#ifdef DEBUG_PRINT_LKAStoEPS_LIN_OUTPUT
	outputSerial.print("\nL-O:");
	printArrayInBinary(&msg[0],4);
#endif
	CAN_message_t thisCanMsg;
	thisCanMsg.id = 0x200;
	thisCanMsg.len = 6;
	thisCanMsg.buf[0] = msg[0];
	thisCanMsg.buf[1] = msg[1];
	thisCanMsg.buf[2] = msg[2];
	thisCanMsg.buf[3] = msg[3];
	thisCanMsg.buf[4] = littleSteer;
	thisCanMsg.buf[4] |=  bigSteer << 5;
	thisCanMsg.buf[5] = bigSteer >> 3;
	// FCAN.write(thisCanMsg);
	can.transmit(thisCanMsg.id,thisCanMsg.buf, thisCanMsg.len);
}




#endif