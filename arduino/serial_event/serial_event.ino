#define LEDPIN_STOP    2
#define LEDPIN_TIMEOUT 4
#define LEDPIN_NOISE   3
#define LEDPIN_NORMAL  5

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

boolean checksumStopFlag = 0;
boolean checksumTimeoutFlag = 0;
boolean checksumNoiseFlag = 0;
boolean checksumNormalFlag = 0;

boolean firstLoop = 1;

char preChecksum = 0;
char curChecksum = 0;

long curTime = 0;

void setup() {
  // initialize serial:
  pinMode(LEDPIN_STOP, OUTPUT);
  pinMode(LEDPIN_TIMEOUT, OUTPUT);
  pinMode(LEDPIN_NOISE, OUTPUT);
  pinMode(LEDPIN_NORMAL, OUTPUT);

  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    curTime = micros();
    curChecksum = inputString[10];
    Serial.print(curChecksum);

    if (curChecksum != 0 && curChecksum - preChecksum >= 2) {
      checksumNoiseFlag = 1;
    }
    else {
      checksumNoiseFlag = 0;
    }
    preChecksum = curChecksum;
    inputString = "";
    stringComplete = false;
  }

  if (micros() - curTime > 1000) {
    checksumStopFlag = 1;
  } else {
    checksumStopFlag = 0;
  }

  if (!(checksumStopFlag || checksumNoiseFlag)) {
    checksumNormalFlag = 1;
  } else {
    checksumNormalFlag = 0;
  }
  
  digitalWrite(LEDPIN_STOP, checksumStopFlag);
  digitalWrite(LEDPIN_NOISE, checksumNoiseFlag);
  digitalWrite(LEDPIN_NORMAL, checksumNormalFlag);

}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == 0x03) {
      stringComplete = true;
    }
  }
}
