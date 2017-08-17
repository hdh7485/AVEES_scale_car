float float_value = 13;

void float2Bytes(byte* bytes_temp, float float_variable) {
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}

byte result_lsb_value[4] = {0x00, 0x00, 0x00, 0x00};
unsigned char result_msb_value[4] = {0x00, 0x00, 0x00, 0x00};

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  if (  Serial.available() > 0) {
    float_value = Serial.parseFloat();
  }
  float2Bytes(result_lsb_value, float_value);
  // put your main code here, to run repeatedly:
  Serial.println(float_value);
  Serial.println("Little Endian");
  for (int i = 0; i < 4; i++) {
    Serial.print(result_lsb_value[i], HEX);
    Serial.print('\t');
  }
  Serial.println('\n');
}


