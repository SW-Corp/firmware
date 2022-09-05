#include <Arduino.h>
#include "config.h"

void setupPins() {

  /*
   * Setup pins for automatic relay control
   * Relay board is driven by low-level logic, so we need to write 'HIGH' state to turn off all relays.
   */
  pinMode(PIN_P1, OUTPUT);
  digitalWrite(PIN_P1, HIGH);

  pinMode(PIN_P2, OUTPUT);
  digitalWrite(PIN_P2, HIGH);

  pinMode(PIN_P3, OUTPUT);
  digitalWrite(PIN_P3, HIGH);

  pinMode(PIN_P4, OUTPUT);
  digitalWrite(PIN_P4, HIGH);

  pinMode(PIN_V1, OUTPUT);
  digitalWrite(PIN_V1, HIGH);

  pinMode(PIN_V2, OUTPUT);
  digitalWrite(PIN_V2, HIGH);

  pinMode(PIN_V3, OUTPUT);
  digitalWrite(PIN_V3, HIGH);

  /*
   * Setup pins for float switch position detection
   */
  pinMode(FS_C1, INPUT);
  pinMode(FS_C2, INPUT);
  pinMode(FS_C3, INPUT);
  pinMode(FS_C4, INPUT);
  pinMode(FS_C5, INPUT);
}

void setup() {
  setupPins();
}

void loop() {
  // put your main code here, to run repeatedly:
}