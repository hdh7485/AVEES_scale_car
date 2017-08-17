#include <stdlib.h>

float float_value = 13;

void float2Bytes(byte* bytes_temp, float float_variable) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

//unsigned char concatenate_uchar(unsigned char front, unsigned char back){
//  unsigned char conc[8];
//
//  return conc;
//}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  unsigned char motor_1_front[4] = {0x1C, 0x72, 0x00, 0x01};
  unsigned char motor_1_back[4] = {0x00, 0x00, 0x00, 0x00};

  if (  Serial.available() > 0) {
    float_value = Serial.parseFloat();
  }

  float2Bytes(motor_1_back, float_value);
  // put your main code here, to run repeatedly:
  Serial.println(float_value);
  Serial.println("Little Endian");

  unsigned char* total = (unsigned char*)malloc(8 * sizeof(char)); // array to hold the result
  memcpy(total,     motor_1_front, 4 * sizeof(unsigned char)); // copy 4 floats from x to total[0]...total[3]
  memcpy(total + 4, motor_1_back, 4 * sizeof(unsigned char)); // copy 4 floats from y to total[4]...total[7]

  for (int i = 0; i < 8; i++) {
    Serial.print(total[i], HEX);
    Serial.print('\t');
  }
  Serial.println('\n');
  
  free(total);
}
