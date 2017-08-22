#include "Arduino.h"
#include "Steer.h"

Steer::Steer(int CANPort, double angle = 0.0){
	Steer::CANInit(CANPort);
	this->angle = angle;
}

void Steer::moveAngle(int angle){
}

void Steer::CANInit(int CANPort){
	this->CANPort = CANPort;
}

void Steer::readCAN(){
	if(CAN_MSGAVAIL == CAN.checkReceive()){
		CAN.readMsgBuf(&CANLen, CANValue);    // read data,  len: data length, buf: data buf
		unsigned int CANId = CAN.getCANId();
	}
}

void Steer::CANtoAngle(){
	unsigned char first_byte = 0x0000;
	unsigned char second_byte = 0x0000;
	first_byte = CANValue[1];
	second_byte = CANValue[0];
	steerAngle = (first_byte << 8) | second_byte;
}	

double Steer::getAngle(){
	return steerAngle;
}

unsigned char[] Steer::getCANvalue(){
	return CANValue;
}
