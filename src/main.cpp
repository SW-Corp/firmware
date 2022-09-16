#include <Arduino.h>
#include <Wire.h>
#include <INA219_WE.h> // TODO: test that library
#include <BMP280_DEV.h>  
#include "firmware.h"

float pressures[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

char newlineChar = '\n';
char delimiterChar = ' ';


// void unknownCommand(const char* command) {
  // Serial.print(F(">Unknown command: "));
  // Serial.println(command);
// }

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
pressure_sensor REFERENCE_PS = {3, ALT_BMP280_ADDR, BMP280_DEV(), 5, FS_C5, "RF"};

pressure_sensor pressure_sensors[] = {C1_PS, C2_PS, C3_PS, C4_PS, C5_PS, REFERENCE_PS};

uint8_t tca_select(uint8_t slot) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << slot);
  return Wire.endTransmission();
}

void reportContainer(uint8_t id) {
  printPgmString(PSTR("$"));
  printString(pressure_sensors[id].container);
  printPgmString(PSTR(" "));
  printInteger(digitalRead(pressure_sensors[id].FS_PIN));
  printPgmString(PSTR(" "));
  printFloat(pressures[id], 2); 
  printPgmString(PSTR("\r\n"));
}

void reportComponent(component* comp) {
  printPgmString(PSTR("$"));
  printString(comp->id);
  printPgmString(PSTR(" "));
  printFloat(comp->INA_SENSOR.getCurrent_mA(), 2);
  printPgmString(PSTR(" "));
  printFloat(comp->INA_SENSOR.getBusVoltage_V()+(comp->INA_SENSOR.getShuntVoltage_mV()/1000), 2);
  printPgmString(PSTR("\r\n"));
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
  // char* device = term.getNext();
  // char* newState = term.getNext();
  // uint8_t newLogicState = HIGH; // assume that default state is "turned off"
// 
  // if (device == NULL || newState == NULL ) {
    // Serial.println(">Invalid command");
    // return;
  // }
// 
  // if (!strcmp_P(newState, PSTR("ON"))) {
    // newLogicState = LOW;
  // }
// 
  // if (device[0]=='P') {
    // for (uint8_t i=0; i<4; i++) {
      // if (!strcmp(pumps[i].id, device)) {
        // found requested pump
        // digitalWrite(pumps[i].AC_PIN, newLogicState);
        // Serial.println(F(">OK"));
        // return;
      // }
    // }
  // } else if (device[0]=='V') {
    // for (uint8_t i=0; i<3; i++) {
      // if (!strcmp(valves[i].id, device)) {
        // found requested valve
        // digitalWrite(valves[i].AC_PIN, newLogicState);
        // Serial.println(F(">OK"));
        // return;
      // }
    // }
  // }
  // 
  printPgmString(PSTR(">INVALID DEVICE"));

}

void execute_line(char* line) {

  if (line[0]=='?') {
    report_status();
  } else {

  }

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
      // Serial.print(F(">INA219 P"));
      printPgmString(PSTR(">INA219 P"));
      print_uint8_base10(i+1);
      printPgmString(PSTR(" INIT FAIL\r\n"));
      // Serial.print(i+1);
      // Serial.println(F(" INIT FAIL"));
      return 1;
    }
  }

  for (uint8_t i=0; i<3; i++) {
    if (!valves[i].INA_SENSOR.init()) {
      printPgmString(PSTR(">INA219 V"));
      print_uint8_base10(i+1);
      printPgmString(PSTR(" INIT FAIL\r\n"));
      // Serial.print(F(">V"));
      // Serial.print(i+1);
      // Serial.println(F(" INIT FAIL"));
      return 1;
    }
  }

  // Serial.println(F(">INA219 OK"));
  printPgmString(PSTR(">INA219 OK\r\n"));
  return 0;
}

uint8_t init_BMP_sensors() {

  for (uint8_t i=0; i<6; i++) { // remember to set to 6 with ref sensor
    if (tca_select(pressure_sensors[i].TCA_SLOT)) {
      // Serial.println(">TCA9548 FAIL");
      printPgmString(PSTR(">TCA9548 FAIL\r\n"));
      return 1;
    }    
    if (!pressure_sensors[i].device.begin(pressure_sensors[i].I2C_ADDR)) {
      printPgmString(PSTR(">BMP280"));
      printString(pressure_sensors[i].container);
      printPgmString(PSTR(" INIT FAIL\r\n"));
      // Serial.print(">BMP280 ");
      // Serial.print(pressure_sensors[i].container);
      // Serial.println(" INIT FAIL");
      return 1;
    }
    pressure_sensors[i].device.setTimeStandby(TIME_STANDBY_05MS);
    pressure_sensors[i].device.startNormalConversion();
  }

  // Serial.println(F(">BMP280 OK"));
  printPgmString(PSTR(">BMP280 OK\r\n"));
  return 0;
}

void setup() {
  // cli();

  setup_pins();
  Wire.begin();
  serial_init();
  serial_reset_read_buffer();
  // Serial.begin(BAUD_RATE);
  // Serial.println(F("Water Treatment Lab Controller v1.0"));
  printPgmString(PSTR("Water Treatment Lab Controller v1.0\r\n"));

  init_INA_sensors();

  init_BMP_sensors();

}

float temp, pres, alt;
uint8_t current_sensor_id = 0;

uint8_t char_counter = 0;
static char line[13];
uint8_t c;

void loop() {

  while((c = serial_read()) != SERIAL_NO_DATA) {
    if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        execute_line(line); // Line is complete. Execute it!
        char_counter = 0;
    } else if (char_counter >= 12) {  
      char_counter = 0;
    } else if (c >= 'a' && c <= 'z') {
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
  
  /*
  tca_select(pressure_sensors[current_sensor_id].TCA_SLOT);
  delay(1);
  if (pressure_sensors[current_sensor_id].device.getMeasurements(temp, pres, alt)) {
    pressures[current_sensor_id] = pres;
  }
  delay(2);
  current_sensor_id = current_sensor_id>3 ? 0 : current_sensor_id+1;
  */

}