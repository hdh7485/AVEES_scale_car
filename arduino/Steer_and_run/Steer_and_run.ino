// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>
#include <string.h>

#define PRINT_STEER_DATA  0
#define PRINT_RECEIVER    0
#define PROCESSING_ON 0

float float_value = 0;
float ch1;
float ch2;
float ch1Filter = 0.0f;
float ch2Filter = 0.0f;
double prev_error = 0.0f;
double error = 0.0f;
double kp = 0.0007;
double ki = 0.0;
double kd = 0.0045;

double steeringOutput;
double steeringTarget;
int steeringCurrent;

void float2Bytes(float float_variable, byte* bytes_temp);
float receiverScaler(float input, float max_value, float middle_value, float width);
void getAngleSignal();
int getMotorSignal();
int getRemoteSignal(int pinNumber);
float voltScaler(float input, float inputMin, float inputMax, float outputMin, float outputMax);
float LPFilter(float input, float pre_result);
void sendProcessing();

long int totalStartTime = 0;
long int totalEndTime = 0;
long int startTime = 0;
long int endTime = 0;
long int inTime = 0;
long int outTime = 0;

unsigned char total_rotate[8] = {0,};
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

//    Motor Command Part
unsigned char motor_1_front[4] = {0x1C, 0x72, 0x00, 0x01}; // Select Motor
unsigned char motor_1_back[4] = {0x00, 0x00, 0x00, 0x00}; // Command to control Motor
unsigned char motor_2_front[4] = {0x1C, 0x72, 0x00, 0x02}; // Select Motor
unsigned char motor_2_back[4] = {0x00, 0x00, 0x00, 0x00}; // Command to control Motor
unsigned char rotate[8] =      {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x50, 0x41};
unsigned char st[8] =          {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
unsigned char motor_2_on[8] =  {0x14, 0x65, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00};// steering motor on command
unsigned char motor_2_off[8] = {0x14, 0x65, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
unsigned char motor_2_quick_stop[6] = {0x14, 0x65, 0x00, 0x02, 0x00, 0x07};

float rearVoltage;

int statement_flag = 1;
//0: motor power on
//1: motor power stop
//2: quick stop (like emergency stop)

void setup() {
  Serial.begin(9600);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(12, OUTPUT);
  int CANStart = CAN.begin(CAN_500KBPS);
  if (!CANStart)
    Serial.println("CAN BUS Shield init ok!");
  else
    Serial.println("CAN FAIL");

  CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3ff);
  CAN.init_Filt(0, 0, 0x2B0);                          // there are 6 filter in mcp2515
  CAN.init_Filt(1, 0, 0x2B0);                          // there are 6 filter in mcp2515
}

void loop() {
  totalStartTime = micros();

  startTime = micros();

  getAngleSignal();

  inTime = micros();
  ch1 = getRemoteSignal(6);
  outTime = micros();
  Serial.print("float2Byte: ");
  Serial.println(outTime - inTime);
  
  ch1Filter = LPFilter(ch1, ch1Filter, 1, 1);

  if (ch1Filter < 100) {
    rearVoltage = 0;
  }
  else {
    rearVoltage = receiverScaler(ch1Filter, 19, 1500, 300);
  }
  float2Bytes(rearVoltage, motor_1_back);
  memcpy(total_rotate,     motor_1_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total_rotate + 4, motor_1_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]
  CAN.sendMsgBuf(0x01, 0, 8, total_rotate);

  endTime = micros();
  Serial.print("run delay: ");
  Serial.println(endTime - startTime);


  startTime = micros();
  ch2 = getRemoteSignal(5);
  ch2Filter = LPFilter(ch2, ch2Filter, 1, 1);
  if (ch2Filter < 100) {
    steeringTarget = 0;
  }
  else {
    steeringTarget = receiverScaler(ch2Filter, 600, 1500, 400);
  }
  error = (steeringTarget - (double)steeringCurrent);
  steeringOutput = error * kp + (error - prev_error) * kd;
  if (steeringOutput > 0) {
    steeringOutput = voltScaler(steeringOutput, 0, 12, 3.1, 5);
  }
  else {
    steeringOutput = voltScaler(steeringOutput, -12, 0, -11.00, -9.2);
  }
#if PROCESSING_ON
  sendProcessing();
#endif
  if (abs(error) < 25) {
    steeringOutput = 0;
  }
  float2Bytes(steeringOutput, motor_2_back);
  memcpy(total_rotate,     motor_2_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total_rotate + 4, motor_2_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]
  CAN.sendMsgBuf(0x01, 0, 8, total_rotate);

  endTime = micros();
  Serial.print("steer Delay: ");
  Serial.println(endTime - startTime);

#if PRINT_RECEIVER == 1
  Serial.print("ch1: ");
  Serial.print(ch1);
  Serial.print('\t');
  Serial.print(ch1Filter);
  Serial.print("\t ch2: ");
  Serial.print(ch2);
  Serial.print('\t');
  Serial.println(ch2Filter);
#endif

#if PRINT_STEER_DATA == 1
  Serial.print("Steering_target: ");
  Serial.print(steeringTarget);
  Serial.print(" \tSteering_current: ");
  Serial.print(steeringCurrent);
  Serial.print("\tSteering_output: ");
  Serial.println(steeringOutput);
#endif

  prev_error = error;

  totalEndTime = micros();
  Serial.print("TOTAL: ");
  Serial.println(totalEndTime - totalStartTime);
}
//END LOOP

int getRemoteSignal(int pinNumber) {
  int pwmData = pulseIn(pinNumber, HIGH, 25000); // each channel
  return pwmData;
}

float receiverScaler(float input, float max_value, float middle_value, float width) {
  float result;
  result = (input - middle_value) / width * max_value;
  result = (result > max_value) ? max_value : ((result < -max_value) ? -max_value : result);
  return result;
}

void float2Bytes(float float_variable, byte* bytes_temp) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

void getAngleSignal() {
  unsigned char len = 0;
  unsigned char buf[5];
  unsigned char firstByte = 0x0000;
  unsigned char secondByte = 0x0000;
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned char canId = CAN.getCanId();
    firstByte = buf[1];
    secondByte = buf[0];
    steeringCurrent = (firstByte << 8) | secondByte;
    steeringCurrent *= -1;
  }
}

int getMotorSignal() {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char firstByte = 0x0000;
  unsigned char secondByte = 0x0000;
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned char canId = CAN.getCanId();
    Serial.print("CAN ID:");
    Serial.println(canId);
    if (canId == 1) {}
    return 0;
  }
}

float voltScaler(float input, float inputMin, float inputMax, float outputMin, float outputMax) {
  float output;
  if (input < inputMin) {
    input = inputMin;
  }
  else if (input > inputMax) {
    input = inputMax;
  }

  output = (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;

  if (output < outputMin) {
    output = outputMin;
  }
  else if (output > outputMax) {
    output = outputMax;
  }
  return output;
}

float LPFilter(float input, float pre_result, float tau, float ts) {
  float y = ( tau * pre_result + ts * input ) / (tau + ts) ;
  return y;
}

void sendProcessing() {
  Serial.print((int)steeringTarget);
  Serial.print(',');
  Serial.print(steeringCurrent);
  Serial.print(',');
  Serial.print(steeringOutput * 10);
  Serial.print(';');
}
// END FILE
