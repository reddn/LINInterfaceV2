#ifndef GLOBALEXTERNH
#define GLOBALEXTERNH

#include "Arduino.h"
#include "struct.h"
// #include <FlexCAN_T4.h>
#include <eXoCAN.h>

extern  incomingLKASMessage incomingMsg;

extern  uint8_t lkas_off_array[][4];// =  { {0x00,0x80,0xc0,0xc0}, {0x20,0x80,0xc0,0xa0} };
extern uint8_t eps_off_array[][5];

//0x20= B‭0010000 0x80= ‭10000000‬0 0xc0=‭11000000‬ 0xa0= ‭10100000‬
//0x00 = 00000000  0x80= ‭10000000‬0 0xc0=‭11000000‬  0xc0=‭11000000‬
extern int16_t applySteer_global;  //TODO:change this


extern uint8_t EPStoLKASBuffer[5];
extern uint8_t EPStoLKASBufferModified[5];
extern uint8_t EPStoLKASBufferCounter;

extern uint8_t counterbit;


extern uint32_t readLEDblinkLastChange;
extern uint8_t LKASFrameSentByCreateLinMessage; // check to show if the current LKAS frame was sent by the CreateLinMessage function, if it was
											// do not relay the rest of the frame
//used to keep the last EPStoLKAS frame sent when Full MCU spoofing is occuring. sent when the 5th byte of the EPStoLKAS frame is received from the EPS
extern uint8_t EPStoLKASLastFrameSent[4]; 

extern uint8_t nextCounterBit;
 
extern int8_t LkasOnIntroCountDown; // sends 5 frames of LKAS on and 0 apply steer.. the stock LKAS does this. but I dont think its needed

// extern FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_128> FCAN;
// extern eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_WIRE_PULLUP); 


extern msgFrm canMsg;

extern bool OPLkasActive ;
extern uint8_t OPBigSteer ;
extern uint8_t OPLittleSteer ;
extern int16_t OPApply_steer;
extern int16_t steerTorqueModified;
extern uint8_t steerTorqueModifiedBigValue;
extern uint8_t steerTorqueModifiedLittleValue;

extern unsigned long OPTimeLastCANRecieved ;

extern uint8_t EPStoLKASCanFrameCounter ;

extern uint8_t LkasFromCanCounter ;
extern uint8_t LkasFromCanCounterErrorCount ;

extern uint8_t LkasFromCanChecksumErrorCount ;

extern uint8_t LkasFromCanFatalError ;

extern uint8_t OPCanCounter ;

extern uint16_t mainLedBlinkTimer ;


extern unsigned long lastDigitalReadTime;

extern uint8_t lastLittleSteer1bit;

extern uint8_t OPSteeringControlMessageActive ;

extern uint8_t LkasFromCanStatus ;

extern int16_t steerTorque;

extern uint8_t OPSteeringControlMessageStatusPending;

extern uint8_t LkasFromCanChecksumError;


extern uint8_t canSteerChecksumError;
extern uint8_t canSteerChecksumFatalError;
extern uint8_t EPStoLKASChecksumError;
extern uint8_t EPStoLKASChecksumFatalError;
extern uint8_t OPSteeringControlMsgActive;
extern uint8_t LKAStoEPSForwarding;
extern uint8_t OPSteeringMsgLate;
extern uint8_t OPSteeringMsgFatalLate ;
extern uint8_t canSteerCounterError ;
extern uint8_t canSteerCounterFatalError ;
extern uint8_t EPStoLKASLkasDisabled;

#endif