// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>
#include <string.h>

float float_value = 0;
float ch1;

void float2Bytes(byte* bytes_temp, float float_variable) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char motor_1_front[4] = {0x1C, 0x72, 0x00, 0x01};
unsigned char motor_1_back[4] = {0x00, 0x00, 0x00, 0x00};
unsigned char rotate[8] =      {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x50, 0x41};
unsigned char st[8] =          {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

unsigned char motor_1_on[8] =  {0x14, 0x65, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
unsigned char motor_1_off[8] = {0x14, 0x65, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

unsigned char motor_1_quick_stop[6] = {0x14, 0x65, 0x00, 0x01, 0x00, 0x07};

int statement_flag = 1;
//0: motor power on
//1: motor power stop
//2: quick stop (like emergency stop)

void setup() {
  Serial.begin(9600);
  pinMode(6, INPUT);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  //  CAN.sendMsgBuf(0x01, 0, 8, motor_1_on);
}

void loop() {
  //  CAN.sendMsgBuf(0x01, 0, 8, rotate);
  ch1 = pulseIn(6, HIGH, 25000); // each channel
  float voltage = map(ch1, 1550.0, 1850.0, 0.0, 19.0);
  voltage = (voltage > 19) ? 19 : ((voltage < 0) ? 0 : voltage);
  
//  if (Serial.available() > 0) {
//    float_value = Serial.parseFloat();
//  }

  float2Bytes(motor_1_back, voltage);
  unsigned char* total_rotate = (unsigned char*)malloc(8 * sizeof(char)); // array to hold the result
  memcpy(total_rotate,     motor_1_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total_rotate + 4, motor_1_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]

  CAN.sendMsgBuf(0x01, 0, 8, total_rotate);

  for (int i = 0; i < 8; i++) {
    Serial.print(total_rotate[i], HEX);
    Serial.print('\t');
  }
  Serial.print('\n');

  free(total_rotate);
  delay(10);
}

// END FILE
