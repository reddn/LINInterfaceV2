//INFO
// CTX3 31
// CRX3 30
// CTX1 22
// CRX1 23
// EPS TO LKAS TX >> UART2 RX
// EPS TO LKAS TX1 >> UART2 TX
// LKAS TO EPS TX >> UART3 RX
// LKAS TO EPS TX1 >> UART3 TX


// #define LKAStoEPS_Serial UART3  // was serial2

// #define EPStoLKAS_Serial UART2  // was serial

#define outputSerial Serial

#define sendEPStoLKASRxToCan false
#define sendLKAStoEPSRxToCan false

#define OUTPUTSERIAL_BAUD 921600
#define STATUS_LED PC14
#define BLUE_LED PC13 //Used for signifying "manipulated" data
#define TIME_BETWEEN_DIGITIAL_READS 250 //in millis

#define EPStoLKASCanMsgId 0x201
#define LKAStoEPSCanMsgId 0x1AB //this is Steer torque request DEC 427

#define SteerTorqueSensorCanMsgId 0x199 // TODO:  is in DEC 399

#define LKAStoEPSLinDataRxMsgId 0x202 //this is the data originated by the MCU that is suppose to go to the EPS.  this is only for testing and to further figure out what the EPStoLKAS data stream is (bytes 3 and 4 of the frame)
#define EPStoLKASLinDataRxMsgId 0x203 // see above


