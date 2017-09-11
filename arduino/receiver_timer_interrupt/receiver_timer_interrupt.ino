#include <MsTimer2.h>

static   byte rcOld;        // Prev. states of inputs
volatile unsigned long rcRises[4]; // times of prev. rising edges
volatile unsigned long rcTimes[4]; // recent pulse lengths
volatile unsigned int  rcChange=0; // Change-counter

void flash(){
  static boolean output = HIGH;
  digitalWrite(13, output);
  output =! output;
}

// Be sure to call setup_rcTiming() from setup()
void setup_rcTiming() {
  rcOld = 0;
  pinMode(A0, INPUT);  // pin 14, A0, PC0, for pin-change interrupt
  pinMode(A1, INPUT);  // pin 15, A1, PC1, for pin-change interrupt
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  PCMSK1 |= 0x0F;       // Four-bit mask for four channels
  PCIFR  |= 0x02;       // clear pin-change interrupts if any
  PCICR  |= 0x02;       // enable pin-change interrupts
}
// Define the service routine for PCI vector 1
ISR(PCINT1_vect) {
  byte rcNew = PINC & 15;   // Get low 4 bits, A0-A3
  byte changes = rcNew^rcOld;   // Notice changed bits
  byte channel = 0;
  unsigned long now = micros(); // micros() is ok in int routine
  while (changes) {
    if ((changes & 1)) {  // Did current channel change?
      if ((rcNew & (1<<channel))) { // Check rising edge
        rcRises[channel] = now;     // Is rising edge
      } else {              // Is falling edge
        rcTimes[channel] = now-rcRises[channel];
      }
    }
    changes >>= 1;      // shift out the done bit
    ++channel;
    ++rcChange;
  }
  rcOld = rcNew;        // Save new state
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting RC Timing Test");
  setup_rcTiming();
  pinMode(13, OUTPUT);
  MsTimer2::set(10, flash);
  MsTimer2::start();
}

void loop() {
  unsigned long rcT[4]; // copy of recent pulse lengths
  unsigned int rcN;
  if (rcChange) {

    // Data is subject to races if interrupted, so off interrupts
    cli();          // Disable interrupts
    rcN = rcChange;
    rcChange = 0;       // Zero the change counter
    rcT[0] = rcTimes[0];
    rcT[1] = rcTimes[1];
    rcT[2] = rcTimes[2];
    rcT[3] = rcTimes[3];
    sei();          // reenable interrupts

    Serial.print(rcT[0]);
    Serial.print(' ');
    Serial.println(rcT[1]);
  }
  sei();            // reenable interrupts
}
