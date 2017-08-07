// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int SPI_CS_PIN2 = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
MCP_CAN CAN2(SPI_CS_PIN2);                                    // Set CS pin

void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS) && CAN_OK != CAN2.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
}

int switcher = 1;

void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char buf2[8];

  if (switcher == 0 && CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned int canId = CAN.getCanId();
    Serial.println("-----------------------------");
    Serial.print("CAN1: Get data from ID: ");
    Serial.println(canId, HEX);

    for (int i = 0; i < len; i++) // print the data
    {
      Serial.print(buf[i], HEX);
      Serial.print("\t");
    }
    Serial.println();
    switcher = 1;
  }
  
  if (switcher == 1 && CAN_MSGAVAIL == CAN2.checkReceive())           // check if data coming
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
    Serial.println();
    switcher = 0;
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
