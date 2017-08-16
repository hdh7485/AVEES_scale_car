// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char rotate[8] =      {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x50, 0x41};
unsigned char st[8] =          {0x1C, 0x72, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

unsigned char motor_1_on[6] =  {0x14, 0x65, 0x00, 0x01, 0x00, 0x01};
unsigned char motor_1_off[8] = {0x14, 0x65, 0x00, 0x01, 0x00, 0x00};
unsigned char motor_1_quick_stop[6] = {0x14, 0x65, 0x00, 0x01, 0x00, 0x07};

int statement_flag = 1;
//0: motor power on
//1: motor power stop
//2: quick stop (like emergency stop)

void setup(){
  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
//  CAN.sendMsgBuf(0x01, 0, 8, motor_1_on);
}

void loop(){
//  CAN.sendMsgBuf(0x01, 0, 8, rotate);
  if(Serial.available() > 0) {
    statement_flag = Serial.read() - 48;
  }
  
  switch (statement_flag) {
    case 0:
      CAN.sendMsgBuf(0x01, 0, 8, rotate);
      Serial.println(statement_flag);
      break;
    case 1:
      CAN.sendMsgBuf(0x01, 0, 8, st);
      Serial.println(statement_flag);
      break;
  }
  delay(100);
}

// END FILE
