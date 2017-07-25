#include <Stepper.h>

const int stepsPerRevolution = 5000;  // change this to fit the number of steps per revolution
// for your motor

//1 pulse: 0.072 degree
int angle2pulse(float angle){
  float pulse = angle / 0.072;
  return (int)pulse;
}
void pulseOut(int pin, int us)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(us);
  digitalWrite(pin, LOW);
  delayMicroseconds(us);
}

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  // set the speed at 60 rpm:
  // initialize the serial port:
  Serial.begin(9600);
  for(int i = 0; i < angle2pulse(20); i++)
    pulseOut(3, 500);
  delay(1000);
}

void loop() {
  // step one revolution  in one direction:
//  for(int i = 0; i < stepsPerRevolution; i++)
//    pulseOut(2, 500);
//  delay(1000);

  // step one revolution in the other direction:  
//  for(int i = 0; i < stepsPerRevolution; i++)
//    pulseOut(3, 500);
//  delay(1000);
  //  Serial.println("counterclockwise");
  //  myStepper.step(-stepsPerRevolution);
  //  delay(500);
}
