#ifndef BUILDCANMESSAGESH
#define BUILDCANMESSAGESH


void buildSteerMotorTorqueCanMsg();
void buildSteerStatusCanMsg();
void buildSendAllLinDataCanMsg();
void buildSendEps2LkasValuesWhole();

void handleLkasFromCanV3();
uint8_t getNextOpenTxMailbox();
void sendCanMsg(CAN_msg_t *CAN_tx_msg);

CAN_msg_t msg;

#include <eXoCAN.h>
#include "struct.h"
#include "globalExtern.h"
#include "checksums.h"
#include "LKAStoEPS.h"
#include "createLINMessages.h"


#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU



// BO_ 427 STEER_MOTOR_TORQUE: 3 EPS
//  SG_ MOTOR_TORQUE : 0|10@0+ (1,0) [-256|256] "" EON
//  SG_ OUTPUT_DISABLED_INVERT : 15|1@0+ (1,0) [0|1] "not sure if its invert or some LDW" EON
//  SG_ UNK_3BIT_1 : 10|3@0+ (1,0) [0|7] "" EON
//  SG_ OUTPUT_DISABLED : 14|1@0+ (1,0) [0|1] "" EON
//  SG_ COUNTER : 21|2@0+ (1,0) [0|3] "" EON
//  SG_ CHECKSUM : 19|4@0+ (1,0) [0|15] "" EON

//removed
// SG_ CONFIG_VALID : 7|1@0+ (1,0) [0|1] "" EON   << not used

void buildSteerMotorTorqueCanMsg(){ //TODO: add to decclaration
	//outputSerial.print("\nSendingSteer TOrque Can MSg");
	// msgFrm msg; // move this to a global to save the assignment of id and len
	// CAN_msg_t msg;
	msg.id = 427;
	msg.len = 3;
	msg.buf[0] =  ( EPStoLKASBuffer[2] << 2 ) & B11100000;  //1 LSB bit of bigSteerTorque /// redone> this is the big torque steer 
	msg.buf[0] |= ( EPStoLKASBuffer[3] >> 2)  & B00011111 ; // all of SmallSteerTorque   ///redone this is the 5MSB of little torque steer
	
	msg.buf[1] =  ( EPStoLKASBuffer[2] >> 4 ) & B00000011; // 2 MSB of bigSteerTorque 	//redone this is the 2LSB of little torque steer on the MSB of the next byte
	msg.buf[1] |= ( incomingMsg.data[0] >> 2 ) & B00000100; //LKAS B0 O4  into CAN B1 O2
	msg.buf[1] |= ( EPStoLKASBuffer[1] ) // EPS B1 O5 (EPS_LKAS_ON aka LKAS_ON_FROM_EPS)

	msg.buf[2] =  (OPCanCounter << 4 ); // put in the counter
	msg.buf[2] |= ( EPStoLKASBuffer[0]   ) & B01000000; //EPS  B0 O6  into CAN B2 O6
	msg.buf[2] |= ( EPStoLKASBuffer[1]  << 1 ) & B10000000; //EPS  B1 O6  into CAN B2 O7  //dont know if this does anything,keeping

	msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3,(unsigned int)msg.id);
	
	sendCanMsg(&msg);

}


///// the only thing in this DBC that should be used is steeor_troque_sensor
// BO_ 399 STEER_STATUS: 3 EPS
//  SG_ COUNTER : 21|2@0+ (1,0) [0|3] "" EON
//  SG_ CHECKSUM : 19|4@0+ (1,0) [0|3] "" EON
//  SG_ STEER_TORQUE_SENSOR : 0|9@1- (1,0) [-256|256] "" XXX


// T T T T  T T T T
// S S S S  E L 0 T
// 0 0 C C  H H H H
//removed
//  SG_ STEER_STATUS : 39|4@0+ (1,0) [0|15] "" EON
//  SG_ STEER_CONTROL_ACTIVE : 35|1@0+ (1,0) [0|1] "" EON
//  SG_ STEER_CONFIG_INDEX : 43|4@0+ (1,0) [0|15] "" EON
//  SG_ STEER_ANGLE_RATE : 23|16@0- (-0.1,0) [-31000|31000] "deg/s" EON << TODO: check if OP uses this or the other STEER_ANGLE_RATE .. this one will not work

void buildSteerStatusCanMsg(){ //TODO: add to decclaration
	
	// outputSerial.print("\nsending Steer Status Cna MSg");
	// CAN_message_t msg; // move this to a global so you dont have to re assign the id and len
	// CAN_msg_t msg;
	msg.id = 399;
	msg.len = 4U;
	msg.buf[0] = EPStoLKASBuffer[0] << 5;   // 3 LSB of BigSteerTorque (4bit)
	msg.buf[0] |= EPStoLKASBuffer[1] & B00011111; // all of smallSteerTorque
	msg.buf[0] = ~msg.buf[0]; // invert the whole message to make negative positive, positive negative.  OP wants left positive (why??)
	msg.buf[1] =  ( ~(  EPStoLKASBuffer[0] >>3 ) )   & B00000001; // 1st MSB of bigSteerTorque (4bit) ... added NOT (~) to invert the sign


	//add other data from Teensy so OP can record it
	
	msg.buf[1] |= OPLkasActive << 1;  				// CAN B1 O1
	msg.buf[1] |= LkasFromCanFatalError << 2; 		// CAN B1 O2
	msg.buf[1] |= canSteerChecksumError << 3; 		// CAN B1 O3
	msg.buf[1] |= canSteerChecksumFatalError << 4; 	// CAN B1 O4
	msg.buf[1] |= EPStoLKASChecksumError << 5;  	// CAN B1 O5
	msg.buf[1] |= EPStoLKASChecksumFatalError << 6;	// CAN B1 O6
	msg.buf[1] |= OPSteeringControlMsgActive << 7;	// CAN B1 O7
	
	msg.buf[2] = LKAStoEPSForwarding << 5; 		// CAN B2 O5
	msg.buf[2] |= OPSteeringMsgLate << 6; 			// CAN B2 O6
	msg.buf[2] |= OPSteeringMsgFatalLate << 7;		// CAN B2 O7

	msg.buf[3]  = (OPCanCounter << 4 );				// put in the counter
	msg.buf[3]  = (canSteerCounterError << 6);		// CAN B3 O6
	msg.buf[3] |= (canSteerCounterFatalError << 7);	// CAN B3 O7 
	msg.buf[3] |= honda_compute_checksum(&msg.buf[0], msg.len, (unsigned int) msg.id);
	// FCAN.write(msg);
	sendCanMsg(&msg);
}
// struct msgFrm
// {
//   int txMsgID = 0x68; //volatile
//   idtype idLen = STD_ID_LEN;
//   uint8_t txMsgLen = 0x08;
//   MSG txMsg;
//   BusType busConfig = PORTA_11_12_XCVR;
//   int txDly = 5000;
// };

void buildSendAllLinDataCanMsg(){
	// CAN_msg_t msg;
	msg.id = 521;
	msg.len = 8U;
	msg.buf[0] =  incomingMsg.data[0];
	msg.buf[1] =  incomingMsg.data[1];
	msg.buf[2] =  incomingMsg.data[2];
	msg.buf[3] =  EPStoLKASBuffer[0];
	msg.buf[4] =  EPStoLKASBuffer[1];
	msg.buf[5] =  EPStoLKASBuffer[2];
	msg.buf[6] =  EPStoLKASBuffer[3]; 

	msg.buf[7] = (OPCanCounter << 4 ); // put in the counter
	msg.buf[7] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);

	sendCanMsg(&msg);
}


void buildSendEps2LkasValuesWhole(){
	// CAN_msg_t msg;
	msg.id = 522;
	msg.len = 8U;
	msg.buf[0] = incomingMsg.data[1] & B00011111;
	uint8_t bigSteer = incomingMsg.data[0] & B00001111;
	msg.buf[0] |= bigSteer << 5;
	msg.buf[1]  = bigSteer >> 3;   // this byte is only used at the LSB  B00000001
	msg.buf[1] |= (EPStoLKASBuffer[1] << 1) & B00111110; //little torque steer
	msg.buf[1] |= (EPStoLKASBuffer[0] << 6) & B11000000; // 2 LSB of the Big torque steer (4 bits)
	msg.buf[2]  = (EPStoLKASBuffer[0] >> 2) & B00000011;

	msg.buf[1] |=  EPStoLKASBuffer[3] << 1; //little motor torque @ B11111110
	msg.buf[2] =  (EPStoLKASBuffer[2] >> 3 ) & B00000111; //big motor torque at @ B00000111

	msg.buf[7] = (OPCanCounter << 4 ); // put in the counter
	msg.buf[7] |= honda_compute_checksum(&msg.buf[0],msg.len,(unsigned int) msg.id);
	// FCAN.write(msg);
	sendCanMsg(&msg);

}


void handleLkasFromCanV3(){


// 	BO_ 228 STEERING_CONTROL: 5 ADAS
//  SG_ STEER_TORQUE : 7|16@0- (1,0) [-3840|3840] "" EPS
//  SG_ STEER_TORQUE_REQUEST : 23|1@0+ (1,0) [0|1] "" EPS
//  SG_ SET_ME_X00 : 31|8@0+ (1,0) [0|0] "" EPS
//  SG_ COUNTER : 37|2@0+ (1,0) [0|3] "" EPS
//  SG_ CHECKSUM : 35|4@0+ (1,0) [0|3] "" EPS

	if(canMsg.txMsgID != 228) return;
	uint8_t lclBigSteer = 0;
	uint8_t lclLittleSteer = 0;
	
	lclBigSteer = ( canMsg.txMsg.bytes[0] >> 4 ) & B00001000;
	lclBigSteer |= ( canMsg.txMsg.bytes[1] >> 5 ) & B00000111;
	
	lclLittleSteer = canMsg.txMsg.bytes[1] & B00011111 ;
	
		// TODO: verify counter is working
	uint8_t lclCounter = (canMsg.txMsg.bytes[4] >> 4) & B00000011;
	bool counterVerified = true;  // need global counter   and counter error
	if(LkasFromCanCounter != lclCounter){
		LkasFromCanCounterErrorCount++;
		canSteerCounterError = 1;
		counterVerified = false;
		if(LkasFromCanCounterErrorCount > 2) {
			canSteerCounterFatalError = 1;
			LkasFromCanFatalError = 1;
		}
	} else {
		canSteerCounterError = 0;
	}
	LkasFromCanCounter = (lclCounter + 1U) & B00000011;  //if  lclCounter is 3,  adding 1 is 4 B00000100, which is zero if & B00000100
	
	bool checksumVerified = false;

	if(honda_compute_checksum((uint8_t*) &canMsg.txMsg.bytes[0],5, 228U) == (canMsg.txMsg.bytes[4] & B00001111 )) {
		checksumVerified = true;
		canSteerChecksumError = 0;
	}
	else {
		LkasFromCanChecksumErrorCount++;
		canSteerChecksumError = 1;
		if(LkasFromCanChecksumErrorCount > 2){
			LkasFromCanFatalError = 1;
			canSteerChecksumFatalError = 1;
		}
	}
	

	//canbus data time is checked in the handleLkastoEPS function, if no data has been received within 50ms . LKAS is not allowed to be active

	// set big/small steer in varible and that LKAS is on
	// so when its time to send a LKAS message, it just reads the data, make the checksum and send it
	if(counterVerified && checksumVerified){
		if((canMsg.txMsg.bytes[2] >> 7) == 1 ){ // if STEER REQUEST (aka LKAS enabled)
			OPLkasActive = true;
		} else {
			OPLkasActive = false;
		}
		
		OPBigSteer = lclBigSteer;
		OPLittleSteer = lclLittleSteer;
		OPApply_steer = (lclBigSteer & B00000111) << 5;
		OPApply_steer |= lclLittleSteer;
		if((lclBigSteer >> 3) == 1) OPApply_steer |= 0xFF00; 
		canSteerChecksumError = 0;
		if(!OPSteeringControlMessageActive){
			OPSteeringControlMessageStatusPending = true;  //im not sure this should be there TODO: check if its right
			OPSteeringControlMessageStatusPendingData = true;
		}
		sendSteerMotorTorqueFrameToCan = canMsg.txMsg.bytes[2] & B00000001; //used in a 'if' statement so any non zero number is true
		sendSteerStatusFrameToCan = canMsg.txMsg.bytes[2] & B00000010; 
		sendAllLinDataFrameToCan = canMsg.txMsg.bytes[2] & B00000100;
		sendLinWholeDataFrameToCan = canMsg.txMsg.bytes[2] & B00001000;
	} else{
		
		// TODO: send/set/notify something to show there was an error...
		// The intent is to do nothing in the event of a counter/checksum error of the CAN message.  if the last state was LKAS ACTIVE(on), and no messages are received in 50ms,
		// the handleLKAStoEPS code will fatal error the CAN and require a restart 
	}
	OPTimeLastCANRecieved = millis();
}

uint8_t getNextOpenTxMailbox(){
	// uint8_t openMailbox =255;
	//can1 base 0x4000 6400
	//offset 0x08
	//newbase 0x40006400b
	// uint8_t num = (CAN1->TSR >> 26) & 0x03;
	// for(uint8_t a =0 ; a < 3; a++){
	// 	if( (num >> a) == 1){
	// 		return a;
	// 	} 
	// }
	if ((CAN1->TSR&CAN_TSR_TME0) == CAN_TSR_TME0_Msk) return 0;
	if ((CAN1->TSR&CAN_TSR_TME1) == CAN_TSR_TME1_Msk) return 1;
	if ((CAN1->TSR&CAN_TSR_TME2) == CAN_TSR_TME2_Msk) return 2;
	return 255;
} 

void sendCanMsg(CAN_msg_t *CAN_tx_msg){
// 	if (CAN_tx_msg->format == EXTENDED_FORMAT) { // Extended frame format
//       out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
//   }
//   else {                                       // Standard frame format
//       out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
//   }
uint32_t out = (CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U;
//   // Remote frame
//   if (CAN_tx_msg->type == REMOTE_FRAME) {
//       out |= STM32_CAN_TIR_RTR;
//   }


	uint8_t mailbox = getNextOpenTxMailbox();
	if(mailbox > 3) return;
	CAN1->sTxMailBox[mailbox].TDTR &= ~(0xF);
	CAN1->sTxMailBox[mailbox].TDTR |= CAN_tx_msg->len & 0xFUL;

	CAN1->sTxMailBox[mailbox].TDLR  = 	(((uint32_t) CAN_tx_msg->buf[3] << 24) |
										((uint32_t) CAN_tx_msg->buf[2] << 16) |
										((uint32_t) CAN_tx_msg->buf[1] <<  8) |
										((uint32_t) CAN_tx_msg->buf[0]      ));
	CAN1->sTxMailBox[mailbox].TDHR  = 	(((uint32_t) CAN_tx_msg->buf[7] << 24) |
										((uint32_t) CAN_tx_msg->buf[6] << 16) |
										((uint32_t) CAN_tx_msg->buf[5] <<  8) |
										((uint32_t) CAN_tx_msg->buf[4]      ));

	// Send Go
	CAN1->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;
	return;
}


#endif