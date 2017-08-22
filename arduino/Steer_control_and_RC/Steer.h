#ifndef Steer_h
#define Steer_h

#include "Arduino.h"

class Steer{
	public:
		Steer();
		void moveAngle(int angle);
		void CANInit();
		int readCAN();
		double CANtoAngle(); 
		double getAngle();
		int getCAN();
	pribate:
		double steerAngle;
		unsigned char CANValue[5];
    unsigned char CANLen = 5;
		int CANPort;
};
#endif
