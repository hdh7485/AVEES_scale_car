/*
 RC PulseIn Serial Read out
 By: Nick Poole
 SparkFun Electronics
 Date: 5
 License: CC-BY SA 3.0 - Creative commons share-alike 3.0
 use this code however you'd like, just keep this license and
 attribute. Let me know if you make hugely, awesome, great changes.
 */

//int ch1; // Here's where we'll keep our channel values
int ch1;
int ch3;

void setup() {

//  pinMode(5, INPUT); // Set our input pins as such
  pinMode(6, INPUT);
  pinMode(5, INPUT);

  Serial.begin(9600); // Pour a bowl of Serial

}

void loop() {

//  ch1 = pulseIn(5, HIGH, 25000); // Read the pulse width of
  ch1 = pulseIn(6, HIGH, 25000); // each channel
  ch3 = pulseIn(5, HIGH, 25000);
//
//  Serial.print("Channel 1:"); // Print the value of
//  Serial.println(ch1);        // each channel

  Serial.print("Channel 1:");
  Serial.println(ch1);

  Serial.print("Channel 3:");
  Serial.println(ch3);
  Serial.println(" ");

  float y = map(ch1, 1550.0, 1850.0, 0.0, 19.0);
  y = (y > 19) ? 19 : ((y < 0) ? 0 : y);
//  Serial.println(y);
  
  delay(100); // I put this here just to make the terminal
  // window happier
}
