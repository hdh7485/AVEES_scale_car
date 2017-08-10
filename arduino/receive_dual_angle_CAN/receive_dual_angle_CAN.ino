// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int SPI_CS_PIN2 = 10;

MCP_CAN CAN1(SPI_CS_PIN);                                    // Set CS pin
MCP_CAN CAN2(SPI_CS_PIN2);                                    // Set CS pin

int decodeEncoder(unsigned char *buf){
  int steer_angle = 0;
  unsigned char first_byte = buf[1];
  unsigned char second_byte = buf[0];   
  steer_angle = (first_byte << 8) | second_byte;
  
  return steer_angle;
}

void setup()
{
  Serial.begin(115200);
//  while ( CAN_OK != CAN2.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
//  while (CAN_OK != CAN1.begin(CAN_500KBPS) && CAN_OK != CAN2.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
//  {
//    Serial.println("CAN BUS Shield init fail");
//    Serial.println(" Init CAN BUS Shield again");
//    delay(100);
//  }
  CAN1.begin(CAN_500KBPS);
  CAN2.begin(CAN_500KBPS);
  
  Serial.println("CAN BUS Shield init ok!");
}

int switcher = 1;

void loop()
{
  unsigned char len = 5;
  unsigned char buf[5];
  unsigned char buf2[5];
  int angle1 = 0;
  int angle2 = 0;
  if (CAN_MSGAVAIL == CAN1.checkReceive())           // check if data coming
  {
    CAN1.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned int canId = CAN1.getCanId();
    Serial.println("-----------------------------");
    Serial.print("CAN1: Get data from ID: ");
    Serial.println(canId, HEX);

    for (int i = 0; i < len; i++) // print the data
    {
      Serial.print(buf[i], HEX);
      Serial.print("\t");
    }
    angle1 = decodeEncoder(buf);
    Serial.print(angle1);
    Serial.println();
    switcher = 1;
  }
  
  if (CAN_MSGAVAIL == CAN2.checkReceive())           // check if data coming
//  if (CAN_MSGAVAIL == CAN2.checkReceive())           // check if data coming
  {
    CAN2.readMsgBuf(&len, buf2);    // read data,  len: data length, buf: data buf
    
    unsigned int canId2 = CAN2.getCanId();
    Serial.println("-----------------------------");
    Serial.print("CAN2: Get data from ID: ");
    Serial.println(canId2, HEX);
    
    for (int i = 0; i < len; i++) // print the data
    {
      Serial.print(buf2[i], HEX);
      Serial.print("\t");
    }
    angle2 = decodeEncoder(buf2);
    Serial.print(angle2);
    Serial.println();
    switcher = 0;
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
