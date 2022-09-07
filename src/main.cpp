#include <Arduino.h>
#include <Wire.h>
#include <INA219_WE.h> // TODO: test that library
#include <BMP280_DEV.h>  
#include <ErriezSerialTerminal.h>
#include "firmware.h"

float pressures[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

char newlineChar = '\n';
char delimiterChar = ' ';

SerialTerminal term(newlineChar, delimiterChar);


void unknownCommand(const char* command) {
  Serial.print(F(">Unknown command: "));
  Serial.println(command);
}

component P1 = {"P1", PIN_P1, INA219_WE(P1_INA_ADDR), FS_C2};
component P2 = {"P2", PIN_P2, INA219_WE(P2_INA_ADDR), FS_C3};
component P3 = {"P3", PIN_P3, INA219_WE(P3_INA_ADDR), FS_C4};
component P4 = {"P4", PIN_P4, INA219_WE(P4_INA_ADDR), FS_C1};
component V1 = {"V1", PIN_V1, INA219_WE(V1_INA_ADDR), FS_C5};
component V2 = {"V2", PIN_V2, INA219_WE(V2_INA_ADDR), FS_C5};
component V3 = {"V3", PIN_V3, INA219_WE(V3_INA_ADDR), FS_C5};

component pumps[] = {P1, P2, P3, P4};
component valves[] = {V1, V2, V3};

pressure_sensor C1_PS = {7, DEFAULT_BMP280_ADDR, BMP280_DEV(), 0, FS_C1, "C1"};
pressure_sensor C2_PS = {6, DEFAULT_BMP280_ADDR, BMP280_DEV(), 1, FS_C2, "C2"};
pressure_sensor C3_PS = {5, DEFAULT_BMP280_ADDR, BMP280_DEV(), 2, FS_C3, "C3"};
pressure_sensor C4_PS = {4, DEFAULT_BMP280_ADDR, BMP280_DEV(), 3, FS_C4, "C4"};
pressure_sensor C5_PS = {3, DEFAULT_BMP280_ADDR, BMP280_DEV(), 4, FS_C5, "C5"};
pressure_sensor REFERENCE_PS = {3, ALT_BMP280_ADDR, BMP280_DEV(), 5, FS_C5, "REF"};

pressure_sensor pressure_sensors[] = {C1_PS, C2_PS, C3_PS, C4_PS, C5_PS}; //, REFERENCE_PS};

uint8_t tca_select(uint8_t slot) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << slot);
  return Wire.endTransmission();
}

void reportContainer(uint8_t id) {

  Serial.print(F("$"));
  Serial.print(pressure_sensors[id].container);
  Serial.print(F(" "));
  Serial.print(digitalRead(pressure_sensors[id].FS_PIN));
  Serial.print(F(" "));
  Serial.println(pressures[id]); //pressures[pressure_sensors[id].idx]);
}

void reportComponent(component* comp) {
  Serial.print(F("$"));
  Serial.print(comp->id);
  Serial.print(F(" "));
  Serial.print(comp->INA_SENSOR.getCurrent_mA());
  Serial.print(F(" "));
  Serial.println(comp->INA_SENSOR.getBusVoltage_V()+(comp->INA_SENSOR.getShuntVoltage_mV()/1000));
}

void report_status() {
  for (uint8_t i=0; i<5; i++) {
    reportContainer(i);
  }
  for (uint8_t i=0; i<4; i++) {
    reportComponent(&pumps[i]);
  }
  for (uint8_t i=0; i<3; i++) {
    reportComponent(&valves[i]);
  }
}

// We assume that relay board uses low-level trigger logic.
// TODO: That logic should be configurable from config.h
void set_device() {
  char* device = term.getNext();
  char* newState = term.getNext();
  uint8_t newLogicState = HIGH; // assume that default state is "turned off"

  if (device == NULL || newState == NULL ) {
    Serial.println(">Invalid command");
    return;
  }

  if (!strcmp_P(newState, PSTR("ON"))) {
    newLogicState = LOW;
  }

  if (device[0]=='P') {
    for (uint8_t i=0; i<4; i++) {
      if (!strcmp(pumps[i].id, device)) {
        // found requested pump
        digitalWrite(pumps[i].AC_PIN, newLogicState);
        Serial.println(F(">OK"));
        return;
      }
    }
  } else if (device[0]=='V') {
    for (uint8_t i=0; i<3; i++) {
      if (!strcmp(valves[i].id, device)) {
        // found requested valve
        digitalWrite(valves[i].AC_PIN, newLogicState);
        Serial.println(F(">OK"));
        return;
      }
    }
  }
  
  Serial.println(F(">INVALID DEVICE"));

}

void setup_pins() {

  /*
   * Setup pins for automatic relay control
   * Relay board is driven by low-level logic, so we need to write 'HIGH' state to turn off all relays.
   */

  // TODO: replace that with iteration over components' lists
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

uint8_t init_INA_sensors() {

  for (uint8_t i=0; i<4; i++) {
    if (!pumps[i].INA_SENSOR.init()) {
      Serial.print(F(">INA219 P"));
      Serial.print(i+1);
      Serial.println(F(" INIT FAIL"));
      return 1;
    }
  }

  for (uint8_t i=0; i<3; i++) {
    if (!valves[i].INA_SENSOR.init()) {
      Serial.print(F(">V"));
      Serial.print(i+1);
      Serial.println(F(" INIT FAIL"));
      return 1;
    }
  }

  Serial.println(F(">INA219 OK"));
  return 0;
}

uint8_t init_BMP_sensors() {

  for (uint8_t i=0; i<5; i++) { // remember to set to 6 with ref sensor
    if (tca_select(pressure_sensors[i].TCA_SLOT)) {
      Serial.println(">TCA9548 FAIL");
      return 1;
    }    
    if (!pressure_sensors[i].device.begin(FORCED_MODE, pressure_sensors[i].I2C_ADDR)) {
      Serial.print(">BMP280 ");
      Serial.print(pressure_sensors[i].container);
      Serial.println(" INIT FAIL");
      return 1;
    }
    // pressure_sensors[i].device.setTimeStandby(TIME_STANDBY_2000MS);
    // pressure_sensors[i].device.startNormalConversion();
  }

  Serial.println(F(">BMP280 OK"));
  return 0;
}

void setup() {
  // cli();

  setup_pins();
  Wire.begin();
  Serial.begin(BAUD_RATE);
  Serial.println(F("Water Treatment Lab Controller v1.0"));

  term.setDefaultHandler(unknownCommand);
  term.setSerialEcho(false);

  term.addCommand("?", report_status);
  term.addCommand("SET", set_device);

  uint8_t res = init_INA_sensors();

  if (res && !DEBUG) {
    while(1); // Force user to restart device when one of the INA sensors fails. Skip that in DEBUG mode.
  }

  res = init_BMP_sensors();

  if (res && !DEBUG) {
    while(1);
  }


  
  /*
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  */
  
  // sei();
  
  // sei(); 
}

float temp, pres, alt;
uint8_t current_sensor_id = 0;

void loop() {
  term.readSerial();
  
  tca_select(pressure_sensors[current_sensor_id].TCA_SLOT);
  delay(1);
  if(pressure_sensors[current_sensor_id].device.getMeasurements(temp, pres, alt)) {
      pressures[current_sensor_id] = pres;
  }
  delay(1);
  tca_select(pressure_sensors[1].TCA_SLOT);
  delay(1);
  if(pressure_sensors[1].device.getMeasurements(temp, pres, alt)) {
      pressures[1] = pres;
  }
  delay(1);
  
  tca_select(pressure_sensors[3].TCA_SLOT);
  delay(1);
  if(pressure_sensors[3].device.getMeasurements(temp, pres, alt)) {
      pressures[3] = pres;
  }
  delay(1);
  tca_select(pressure_sensors[2].TCA_SLOT);
  delay(1);
  if(pressure_sensors[2].device.getMeasurements(temp, pres, alt)) {
      pressures[2] = pres;
  }
  delay(1);
  tca_select(pressure_sensors[4].TCA_SLOT);
  delay(1);
  if(pressure_sensors[4].device.getMeasurements(temp, pres, alt)) {
      pressures[4] = pres;
  }
  delay(1);
  /*
  if (current_sensor_id==4) {
    current_sensor_id = 0;
  } else {
    current_sensor_id++;
  }
  */

  /*
  for (uint8_t j=0; j<5; j++) {
    float temp, pres, alt;
    tca_select(pressure_sensors[j].TCA_SLOT);
    // Serial.print("Analyzing ");
    // Serial.println(i);
    // pressure_sensors[i].device.startForcedConversion();
    if(pressure_sensors[j].device.getMeasurements(temp, pres, alt)) {
      pressures[j] = pres;
    }
  }
  delay(5);
  tca_select(pressure_sensors[0].TCA_SLOT);
  if (pressure_sensors[0].device.getMeasurements(temp1, pres1, alt1)) {
    pressures[0] = pres1;
  }
  delay(5);
  */
}

/*
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  tca_select(pressure_sensors[current_sensor_id].TCA_SLOT);
  if (pressure_sensors[current_sensor_id].device.getMeasurements(temp, pres, alt)) {
    pressures[current_sensor_id] = pres;
  }

  current_sensor_id = current_sensor_id==4? 0 : current_sensor_id+1;
}
*/