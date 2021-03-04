#ifndef GLOBALVARSH
#define GLOBALVARSH

#include "Arduino.h"
// #include <FlexCAN_T4.h>
#include <eXoCAN.h>
#include "struct.h"

//(rx, tx)   USART2  tx=PA2  rx= pA3   
//USART1 tx= PA9  rx=PA10

//PCB: UART3_TX is PA9    (im guessing rx is PA10)  ITs called 'Serial' in the framework

HardwareSerial EPStoLKAS_Serial(PA3,PA2); // rx, tx          
HardwareSerial LKAStoEPS_Serial(PA10,PA9);


incomingLKASMessage incomingMsg;

uint8_t lkas_off_array[][4] =  { {0x00,0x80,0xc0,0xc0}, {0x20,0x80,0xc0,0xa0} };
uint8_t eps_off_array[][5] = {
	{B00001111, B10011110, B11000000, B10000000, B10010011},  
	{B00101111, B10011110, B11000000, B10000000, B11110011} //    
};

//0x20= B‭0010000 0x80= ‭10000000‬0 0xc0=‭11000000‬ 0xa0= ‭10100000‬
//0x00 = 00000000  0x80= ‭10000000‬0 0xc0=‭11000000‬  0xc0=‭11000000‬
int16_t applySteer_global = 0;  //TODO:change this


uint8_t EPStoLKASBuffer[5];
uint8_t EPStoLKASBufferModified[5];
uint8_t EPStoLKASBufferCounter = 0;

uint8_t counterbit = 0;


uint32_t readLEDblinkLastChange = 0;
uint8_t LKASFrameSentByCreateLinMessage = 0; // check to show if the current LKAS frame was sent by the CreateLinMessage function, if it was
											// do not relay the rest of the frame
//used to keep the last EPStoLKAS frame sent when Full MCU spoofing is occuring. sent when the 5th byte of the EPStoLKAS frame is received from the EPS
uint8_t EPStoLKASLastFrameSent[4]; 

uint8_t nextCounterBit = 0;
 
int8_t LkasOnIntroCountDown = 5; // sends 5 frames of LKAS on and 0 apply steer.. the stock LKAS does this. but I dont think its needed

eXoCAN can(STD_ID_LEN, BR500K, PORTB_8_9_XCVR); 


msgFrm canMsg;


bool OPLkasActive = false;
uint8_t OPBigSteer = 0;
uint8_t OPLittleSteer = 0;
int16_t OPApply_steer = 0;
int16_t steerTorqueModified = 0;
uint8_t steerTorqueModifiedBigValue = 0;
uint8_t steerTorqueModifiedLittleValue = 0;

unsigned long OPTimeLastCANRecieved = 0;

uint8_t EPStoLKASCanFrameCounter = 0;

uint8_t LkasFromCanCounter = 0;
uint8_t LkasFromCanCounterErrorCount = 0;

uint8_t LkasFromCanChecksumErrorCount = 0;

uint8_t LkasFromCanFatalError = 0;

uint8_t OPCanCounter = 0;

uint16_t mainLedBlinkTimer = 2000;

unsigned long lastDigitalReadTime = 0;

unsigned long lastRedLedToggle = 0;

uint8_t lastLittleSteer1bit = 0;

uint8_t OPSteeringControlMessageActive = 0;

uint8_t LkasFromCanStatus = 0; //1 = missed 5 consecutive frames .. 2= 2 bad checksums in 250 ms .. 3 = EPS disabled

int16_t steerTorque = 0;

uint8_t OPSteeringControlMessageStatusPending = false;
uint8_t OPSteeringControlMessageStatusPendingData = false;


uint8_t LkasFromCanChecksumError = 0;

uint8_t canSteerChecksumError = 0;
uint8_t canSteerChecksumFatalError = 0;
uint8_t EPStoLKASChecksumError = 0;
uint8_t EPStoLKASChecksumFatalError = 0;
uint8_t OPSteeringControlMsgActive = 0;
uint8_t LKAStoEPSForwarding = 0;
uint8_t OPSteeringMsgLate = 0;
uint8_t OPSteeringMsgFatalLate = 0;
uint8_t canSteerCounterError = 0;
uint8_t canSteerCounterFatalError = 0;
uint8_t EPStoLKASLkasDisabled = 0;




#endif