char data[4] = {0x00, 0x00, 0x00, 0x00};
boolean stringComplete = false;  // whether the string is complete

void setup() {
  //start serial connection
  Serial.begin(115200);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(13, OUTPUT);
}

void loop() {
  if (stringComplete) {
    Serial.print(data);
    stringComplete = false;
  }
}
/*
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == 0x02) {
      Serial.readBytes(data, 4);
    }
    stringComplete = true;
  }
}*/
