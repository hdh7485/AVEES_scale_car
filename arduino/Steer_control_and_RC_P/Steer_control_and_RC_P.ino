// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>
#include <string.h>

#define PRINT_STEER_DATA  0
#define PRINT_RECEIVER    0

float float_value = 0;
float ch1;
float ch2;
float ch1Filter = 0.0f;
float ch2Filter = 0.0f;
double prev_error = 0.0f;
double error = 0.0f;
double kp = 0.01;
double ki = 0.0;
double kd = 0.002;

double steering_output, steering_target;
int steering_current;

void float2Bytes(float float_variable, byte* bytes_temp);
float receiver_scaler(float input, float max_value, float middle_value, float width);
void getAngleSignal();
int getMotorSignal();
int getRemoteSignal(int pinNumber);
float voltScaler(float input);
float LPFilter(float input, float pre_result);

unsigned char total_rotate[8] = {0,};
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

//    Motor Command Part
unsigned char motor_2_front[4] = {0x1C, 0x72, 0x00, 0x02}; // Select Motor
unsigned char motor_2_back[4] = {0x00, 0x00, 0x00, 0x00}; // Command to control Motor
unsigned char rotate[8] =      {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x50, 0x41};
unsigned char st[8] =          {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
unsigned char motor_2_on[8] =  {0x14, 0x65, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00};// steering motor on command
unsigned char motor_2_off[8] = {0x14, 0x65, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
unsigned char motor_2_quick_stop[6] = {0x14, 0x65, 0x00, 0x02, 0x00, 0x07};

float voltage;

int statement_flag = 1;
//0: motor power on
//1: motor power stop
//2: quick stop (like emergency stop)

void setup() {
  Serial.begin(9600);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
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
  getAngleSignal();

  ch1 = getRemoteSignal(6);
  ch2 = getRemoteSignal(5);

  ch1Filter = LPFilter(ch1, ch1Filter, 1, 1);
  ch2Filter = LPFilter(ch2, ch2Filter, 1, 1);

  voltage = receiver_scaler(ch1Filter, 19, 1550, 300);
  steering_target = receiver_scaler(ch2Filter, 710, 1500, 400);

  error = (steering_target - (double)steering_current);

  steering_output = error * kp + (error - prev_error) * kd;
  if (steering_output > 0)
  {
    steering_output = map(steering_output, 0, 12, 3.3, 5);
  }
  else
  {
    steering_output = map(steering_output, 0, -12, -9.50, -11.00);
  }
  
  //  Serial.print("Target: ");
  Serial.print((int)steering_target);
  Serial.print(',');

  //  Serial.print(" CurrentAngle: ");
  Serial.print(steering_current);
  Serial.print(',');

  Serial.print((int)steering_output);
  Serial.print('.');
  //  Serial.print(" Error: ");
  //  Serial.println(error);

  //  if (error > 90) {
  //    steering_output = 3.20;
  //  }
  //  else if (error < -90) {
  //    steering_output = -9.3;
  //  }
  //  else {
  //    steering_output = 0;
  //  }


  float2Bytes(steering_output, motor_2_back);
  memcpy(total_rotate,     motor_2_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total_rotate + 4, motor_2_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]
  //steering motor Sendmsg

  CAN.sendMsgBuf(0x01, 0, 8, total_rotate);
  //  getMotorSignal();
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
  Serial.print(steering_target);
  Serial.print(" \tSteering_current: ");
  Serial.print(steering_current);
  Serial.print("\tSteering_output: ");
  Serial.println(steering_output);
#endif

  prev_error = error;
}

int getRemoteSignal(int pinNumber) {
  int pwmData = pulseIn(pinNumber, HIGH, 25000); // each channel
  return pwmData;
}

float receiver_scaler(float input, float max_value, float middle_value, float width) {
  float result = (input - middle_value) / width * max_value;
  result = (result > max_value) ? max_value : ((result < -max_value) ? -max_value : result);
  return result;
}

void float2Bytes(float float_variable, byte* bytes_temp) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

void getAngleSignal() {
  unsigned char len = 0;
  unsigned char buf[5];
  unsigned char first_byte = 0x0000;
  unsigned char second_byte = 0x0000;

  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned char canId = CAN.getCanId();

    if (canId == 176) {
      first_byte = buf[1];
      second_byte = buf[0];
      steering_current = (first_byte << 8) | second_byte;
      steering_current *= -1;
    }
    //    Serial.print("steering_angle:");
    //    Serial.println(steering_current);
  }
}

int getMotorSignal() {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char first_byte = 0x0000;
  unsigned char second_byte = 0x0000;
  int steering_angle;
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
  float steering_output = input;
  if (steering_output < 0)
    steering_output = input * 2.1;

  if (steering_output > 5) {
    steering_output = 5;
  }
  else if (steering_output < -12) {
    steering_output = -12;
  }
  return steering_output;
}

float LPFilter(float input, float pre_result, float tau, float ts) {
  float y = ( tau * pre_result + ts * input ) / (tau + ts) ;
  return y;
}
// END FILE
