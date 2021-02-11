#ifndef BUILDCANMESSAGESH
#define BUILDCANMESSAGESH


void buildSteerMotorTorqueCanMsg();
void buildSteerStatusCanMsg();
void handleLkasFromCanV3();
uint8_t getNextOpenTxMailbox();
void sendCanMsg(CAN_msg_t *CAN_tx_msg);

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
	CAN_msg_t msg;
	msg.id = 427;
	msg.len = 3;
	msg.buf[0] = (EPStoLKASBuffer[2] << 4 ) & B1000000;  //1 LSB bit of bigSteerTorque 
	msg.buf[0] |= EPStoLKASBuffer[3] & B01111111 ; // all of SmallSteerTorque
	msg.buf[1] = ( EPStoLKASBuffer[2] >> 4 ) & B00000011; // 2 MSB of bigSteerTorque
	msg.buf[1] |= ( EPStoLKASBuffer[1] << 2 ) & B10000000;  // this is output_disabled_inverted
	msg.buf[1] |= ( EPStoLKASBuffer[2] << 2 )& B00011100; //UNK_3bit_1
	msg.buf[1] |=  EPStoLKASBuffer[2] & B01000000; //output_disabled
	
	msg.buf[2] = (OPCanCounter << 4 ); // put in the counter
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3,(unsigned int)msg.id);
	// FCAN.write(msg);
	// can.transmit(msg.id, msg.buf, msg.len);sendCanMsg(&msg);
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
	CAN_msg_t msg;
	msg.id = 399;
	msg.len = 3U;
	msg.buf[0] = EPStoLKASBuffer[0] << 5;   // 3 LSB of BigSteerTorque (4bit)
	msg.buf[0] |= EPStoLKASBuffer[1] & B00011111; // all of smallSteerTorque
	msg.buf[0] = ~msg.buf[0]; // invert the whole message to make negative positive, positive negative.  OP wants left positive (why??)
	msg.buf[1] =  ( ~(  EPStoLKASBuffer[0] >>3 ) )   & B00000001; // 1st MSB of bigSteerTorque (4bit) ... added NOT (~) to invert the sign


	//add other data from Teensy so OP can record it
	msg.buf[1] |= LkasFromCanStatus << 4; 
	msg.buf[1] |= OPLkasActive << 2;
	msg.buf[1] |= LkasFromCanFatalError << 3;
	
	msg.buf[2] = (OPCanCounter << 4 ); // put in the counter
	msg.buf[2] |= honda_compute_checksum(&msg.buf[0],3,(unsigned int) msg.id);
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




void handleLkasFromCanV3(){

	//TESTING
	if(canMsg.txMsgID != 228) return;
	
	buildSteerStatusCanMsg();
	buildSteerMotorTorqueCanMsg();
	createKLinMessageWBigSteerAndLittleSteer(3,3);



// 	BO_ 228 STEERING_CONTROL: 5 ADAS
//  SG_ STEER_TORQUE : 7|16@0- (1,0) [-3840|3840] "" EPS
//  SG_ STEER_TORQUE_REQUEST : 23|1@0+ (1,0) [0|1] "" EPS
//  SG_ SET_ME_X00 : 31|8@0+ (1,0) [0|0] "" EPS
//  SG_ COUNTER : 37|2@0+ (1,0) [0|3] "" EPS
//  SG_ CHECKSUM : 35|4@0+ (1,0) [0|3] "" EPS
	// if(canMsg.txMsgID != 228) return;

	// digitalToggle(BLUE_LED);

// #ifdef DEBUG_PRINT_OPtoCAN_INPUT
// 	outputSerial.print("\nCANmsg rcvd id: ");
// 	outputSerial.print(canmsg.txMsgID,DEC);
// 	outputSerial.print(":");
// 	for(uint8_t bb = 0; bb < canmsg.txMsgLen; bb++){
// 		printuint_t(canMsg.txMsg.bytes[bb]);
// 	}
// #endif 

	if((canMsg.txMsg.bytes[2] >> 7) == 1 ){ // if STEER REQUEST (aka LKAS enabled)
		OPLkasActive = true;
	} else {
		OPLkasActive = false;
	}

	uint8_t lclBigSteer = 0;
	uint8_t lclLittleSteer = 0;
	
	lclBigSteer = ( canMsg.txMsg.bytes[0] >> 4 ) & B00001000;
	lclBigSteer |= ( canMsg.txMsg.bytes[1] >> 5 ) & B00000111;
	
	lclLittleSteer = canMsg.txMsg.bytes[1] & B00011111 ;
	
		// TODO: verify counter is working
	uint8_t lclCounter = canMsg.txMsg.bytes[4] >> 4;
	bool counterVerified = false;  // need global counter   and counter error

	if(LkasFromCanCounter != lclCounter) LkasFromCanCounterErrorCount++;
	else LkasFromCanCounterErrorCount = 0;
	
	if(LkasFromCanCounter < 3) counterVerified = true;


	// TODO: verify checksum
	bool checksumVerified = false;

	if(honda_compute_checksum((uint8_t*) &canMsg.txMsg.bytes[0],5, 228U) == (canMsg.txMsg.bytes[5] & B00001111 )) LkasFromCanChecksumErrorCount = 0;
	else LkasFromCanChecksumErrorCount++;
	
	if(LkasFromCanCounterErrorCount < 3 ) checksumVerified = true;
	else checksumVerified = false;


	// TODO: Fix this, hard coded to true for testing
	counterVerified = true;
	checksumVerified = true;

	//canbus data time is checked in the handleLkastoEPS function, if no data has been received within 50ms . LKAS is not allowed to be active

	// set big/small steer in varible and that LKAS is on
	// so when its time to send a LKAS message, it just reads the data, make the checksum and send it
	if(counterVerified && checksumVerified){

		
		OPBigSteer = lclBigSteer;
		OPLittleSteer = lclLittleSteer;
		OPApply_steer = (lclBigSteer & B00000111) << 5;
		OPApply_steer |= lclLittleSteer;
		if((lclBigSteer >> 3) == 1) OPApply_steer |= 0xFF00; 
		
	} else{
		OPLkasActive = false;
		// TODO: send/set/notify something to show there was an error... 
	}
	OPSteeringControlMessageStatusPending = true;  //im not sure this should be there TODO: check if its right
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