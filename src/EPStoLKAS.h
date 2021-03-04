#ifndef EPSTOLKASH
#define EPSTOLKASH

#include "globalExtern.h"
#include "canMessages.h"
#include "sendSerial.h"

void handleEPStoLKASKeepMcuHappy(uint8_t rcvdByte);



void handleEPStoLKAS(){

    if(EPStoLKAS_Serial.available())
	{
		uint8_t rcvdByte = EPStoLKAS_Serial.read();	

		uint8_t offset4 = rcvdByte >> 4;
        
        if(offset4 < 4 ){ //first frame
            EPStoLKASBufferCounter = 0;
        }  
        EPStoLKASBuffer[EPStoLKASBufferCounter] = rcvdByte;

        

        if(OPSteeringControlMessageActive){
            switch(EPStoLKASBufferCounter){
                case 0:
                    rcvdByte &= B00100000;
                    rcvdByte |= steerTorqueModifiedBigValue;
                    break;
                case 1:
                    rcvdByte = eps_off_array[0][1] | steerTorqueModifiedLittleValue; 
                    break;
                case 2:
                    rcvdByte = eps_off_array[0][2];
                    break;
                case 3:
                    rcvdByte = eps_off_array[0][3];
                    break;
                case 4:
                    rcvdByte = chksm(&EPStoLKASBufferModified[0],4);
                    // OPSteeringControlMessageActive = OPSteeringControlMessageStatusPending; // only should happen in 1 place. frist byte of LKAStoEPS frame
            }
            EPStoLKASBufferModified[EPStoLKASBufferCounter] = rcvdByte;

        } 
        EPStoLKAS_Serial.write(EPStoLKASBuffer[EPStoLKASBufferCounter]); // just forward the data that came in from the EPS.. turns out it doesnt care if its showing LKAS_ACTIVE

        EPStoLKASBufferCounter++;
        if(EPStoLKASBufferCounter < 5) return;  //only reason for using 5 is im using the incrmeent, so its really still on 3, but is incremented to 4 early, so it really needs to be 5 

        //TODO: run checksum on data
        // This function builds the 2 CAN messages for MOTOR_TORQUE and STEER_TORQUE (input) from the EPStoLKAS 5 byte frame
        // but only does it after the whole frame is received and checksum'd

        if( ( (EPStoLKASBuffer[2] >> 2 ) & 1U ) == 1) {
            EPStoLKASLkasDisabled = 1;
            LkasFromCanFatalError = 1; // todo fix uncomment for prodeuction
        }

        buildSteerMotorTorqueCanMsg();
        buildSteerStatusCanMsg();
        if(!OPSteeringControlMessageActive) buildSendAllLinDataCanMsg();
        else buildSendAllLinDataCanMsg(); // remove this.. only for testing
        buildSendEps2LkasValuesWhole();     // this is for testing.. to be removed.  remove this
        steerTorque =  (EPStoLKASBuffer[0] << 5 )  & B11100000;
        steerTorque |= EPStoLKASBuffer[1] & B00011111;
        if ( (EPStoLKASBuffer[0] >> 3) == 1 ) { //its negative  // this isn't right... needs to &  B00000001
            steerTorque |= 0xFF00;
        } 
        steerTorqueModified = steerTorque + OPApply_steer / 3;
        steerTorqueModifiedBigValue = (uint8_t) ( steerTorqueModified >> 12 ) & B00001000;
        steerTorqueModifiedBigValue |= (uint8_t) ( steerTorqueModified >> 5 ) & B11100000;
        steerTorqueModifiedLittleValue = (uint8_t) steerTorqueModified & B0001111;
    
        if(++OPCanCounter > 3) OPCanCounter = 0;
        EPStoLKASBufferCounter = 0; //reset EPStoLKASBufferCounter to zero
    } // end if EPStoLKAS_Serial.available()
} // end handleEPStoLKAS()




void handleEPStoLKASKeepMcuHappy(uint8_t rcvdByte){ // this is dead

	if(EPStoLKASBufferCounter == 0){
		//TODO: send actual steering torque values
			sendArrayToEPStoLKASSerial(&eps_off_array[incomingMsg.counterBit][0]);

		
		#ifdef DEBUG_PRINT_EPStoLKAS_LIN_OUTPUT
		outputSerial.print("\nE-O:");
		printArrayInBinary(&eps_off_array[incomingMsg.counterBit][0],5);
		outputSerial.print("  ");
		#endif
	}
	// EPStoLKAS_Serial.write(eps_off_array[incomingMsg.counterBit][EPStoLKASBufferCounter]);
}



#endif