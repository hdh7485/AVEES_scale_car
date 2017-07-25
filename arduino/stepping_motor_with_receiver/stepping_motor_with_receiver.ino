const int stepsPerRevolution = 5000;  // change this to fit the number of steps per revolution
// for your motor

#define BRAKE_ON  true
#define BRAKE_OFF false

int brake_channel;
bool current_brake_state = BRAKE_OFF;

//1 pulse: 0.072 degree
int angle2pulse(float angle) {
  float pulse = angle / 0.072;
  return (int)pulse;
}

void pulseOut(int pin, int us)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(us);
  digitalWrite(pin, LOW);
  delayMicroseconds(us);
}

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  //channel 2 = pin 7 = brake;
  pinMode(7, INPUT);
  pinMode(6, INPUT);

  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  brake_channel = pulseIn(7, HIGH, 25000);
  Serial.println(brake_channel);
  //brake on
  if (current_brake_state == BRAKE_OFF && brake_channel > 1500) {
    Serial.println("brake on");
    for (int i = 0; i < angle2pulse(30); i++)
      pulseOut(2, 500);
    current_brake_state = BRAKE_ON;
  }
  //brake off
  else if (current_brake_state == BRAKE_ON && brake_channel < 1500) {
        Serial.println("brake off");
    for (int i = 0; i < angle2pulse(30); i++)
      pulseOut(3, 500);
    current_brake_state = BRAKE_OFF;
  }
}
