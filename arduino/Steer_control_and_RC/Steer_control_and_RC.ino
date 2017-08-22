// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>
#include <string.h>

float float_value = 0;
float ch1;
float ch2;
double prev_error = 0.0f;
double error = 0.0f;
double kp = 0.007;
double ki = 0.0;
double kd = 0.002;

double steering_output, steering_target;
int steering_current;

void float2Bytes(float float_variable, byte* bytes_temp) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

float receiver_scaler(float input, float max_value, float middle_value, float width) {
  float result = (input - middle_value) / width * max_value;
  result = (result > max_value) ? max_value : ((result < -max_value) ? -max_value : result);
  return result;
}

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

int statement_flag = 1;
//0: motor power on
//1: motor power stop
//2: quick stop (like emergency stop)

void setup() {
  Serial.begin(9600);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  //Can Communication Part
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  //  CAN.sendMsgBuf(0x01, 0, 8, motor_1_on);

}

void loop() {
  ///receive steering can msg
  unsigned char len = 0;
  unsigned char buf[5];
  unsigned char first_byte = 0x0000;
  unsigned char second_byte = 0x0000;
  int steer_angle = 0x0000;

  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
    first_byte = buf[1];
    second_byte = buf[0];
    steering_current = (first_byte << 8) | second_byte;
    steering_current *= -1;
  }


  //    RC Receive Part
  ch1 = pulseIn(6, HIGH, 25000); // each channel
  ch2 = pulseIn(5, HIGH, 25000); // each channel
  float voltage = receiver_scaler(ch1, 19, 1550, 300);
  float steering_target = receiver_scaler(ch2, 710, 1500, 400);

  error = (steering_target - (double)steering_current);
  steering_output = error * kp + (error - prev_error) * kd;


  ///////////////////////////
  if (steering_output < 0)
    steering_output *= 2.1;
    
  if (steering_output > 5) {
    steering_output = 5;
  }
  else if (steering_output < -12) {
    steering_output = -12;
  }

  float2Bytes(steering_output, motor_2_back);
  unsigned char* total_rotate = (unsigned char*)malloc(8 * sizeof(char)); // array to hold the result
  memcpy(total_rotate,     motor_2_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total_rotate + 4, motor_2_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]
  //steering motor Sendmsg
  CAN.sendMsgBuf(0x01, 0, 8, total_rotate);

  Serial.print('\n');
  Serial.print("Steering_target: ");
  Serial.print(steering_target);
  Serial.print('\n');
  Serial.print("Steering_current: ");
  Serial.print(steering_current);
  Serial.print('\n');
  Serial.print("Steering_output: ");
  Serial.println(steering_output);

  free(total_rotate);
  prev_error = error;
}

// END FILE
