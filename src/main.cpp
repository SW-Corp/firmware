#include <Arduino.h>
#include <Wire.h>
#include <INA219_WE.h> // TODO: test that library
#include <BMP280_DEV.h>  
// #include <ErriezSerialTerminal.h>
// #include "config.h"
#include "firmware.h"

// char newlineChar = '\n';
// char delimiterChar = ' ';

// SerialTerminal term(newlineChar, delimiterChar);

/*
void unknownCommand(const char* command) {
  Serial.print(F("Unknown command: "));
  Serial.println(command);
}
*/

component P1 = {PIN_P1, INA219_WE(P1_INA_ADDR), FS_C2};
component P2 = {PIN_P2, INA219_WE(P2_INA_ADDR), FS_C3};
component P3 = {PIN_P3, INA219_WE(P3_INA_ADDR), FS_C4};
component P4 = {PIN_P4, INA219_WE(P4_INA_ADDR), FS_C1};
component V1 = {PIN_V1, INA219_WE(V1_INA_ADDR), FS_C5};
component V2 = {PIN_V2, INA219_WE(V2_INA_ADDR), FS_C5};
component V3 = {PIN_V3, INA219_WE(V3_INA_ADDR), FS_C5};

component pumps[] = {P1, P2, P3, P4};
component valves[] = {V1, V2, V3};

pressure_sensor C1_PS = {7, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C1"};
pressure_sensor C2_PS = {6, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C2"};
pressure_sensor C3_PS = {5, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C3"};
pressure_sensor C4_PS = {4, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C4"};
pressure_sensor C5_PS = {3, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C5"};
pressure_sensor REFERENCE_PS = {3, ALT_BMP280_ADDR, BMP280_DEV(), "REF"};

pressure_sensor pressure_sensors[] = {C1_PS, C2_PS, C3_PS, C4_PS, C5_PS, REFERENCE_PS};

static char line[LINE_BUFFER_SIZE];

uint8_t char_counter = 0;
uint8_t c;

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
      // sprintf(result->message, "P%d INIT FAIL", i);
      printPgmString(PSTR(">P"));
      print_uint8_base10(i+1);
      printPgmStringLn(PSTR(" INIT FAIL"));
      return 1;
    }
  }

  for (uint8_t i=0; i<3; i++) {
    if (!valves[i].INA_SENSOR.init()) {
      // sprintf(result->message, ">V%d INIT FAIL", i+1);
      printPgmString(PSTR(">V"));
      print_uint8_base10(i+1);
      printPgmStringLn(PSTR(" INIT FAIL"));
      return 1;
    }
  }

  printPgmStringLn(PSTR(">INA219 OK"));
  return 0;
}

uint8_t init_BMP_sensors() {

  for (uint8_t i=0; i<6; i++) {
    if (!pressure_sensors[i].device.begin(pressure_sensors[i].I2C_ADDR)) {
      // sprintf(result->message, ">BMP280 %s INIT FAIL", pressure_sensors[i].container);      
      // result->code = 1;
      return 1;
    }
  }

  
  // sprintf(result->message, ">BMP280 OK");
  return 0;
}

void setup() {
  setup_pins();
  Wire.begin();
  serial_init();
  // Serial.begin(115200);
  // Serial.println(F("Water Treatment Lab Controller v1.0"));
  printPgmStringLn(PSTR("Water Treatment Lab Controller v1.0"));

  uint8_t res = init_INA_sensors();
  // Serial.println(result.message);
  // if (res) {
    // while(1);
  // }

  res = 0;

  res = init_BMP_sensors();

  if (res) {
    // Serial.println(result.message);
    // while(1);
  }

  sei();

  serial_reset_read_buffer();


}

void execute_line(char *line) {

  switch(line[0]) {
    case '$':
      printPgmStringLn(PSTR("REPORTING STATUS"));
      break;
  }

}

void loop() {

  while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        execute_line(line); // Line is complete. Execute it!
        // comment = COMMENT_NONE;
        char_counter = 0;
      } else {

          if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow. Report error and reset line buffer.
            // report_status_message(STATUS_OVERFLOW);
            // comment = COMMENT_NONE;
            char_counter = 0;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        // }
      }
    }
}