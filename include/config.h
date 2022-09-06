#ifndef config_h
#define config_h

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <INA219_WE.h>
#include <BMP280_DEV.h>

#define BAUD_RATE 115200

// Define serial port pins and interrupt vectors.
#define SERIAL_RX     USART_RX_vect
#define SERIAL_UDRE   USART_UDRE_vect

#define LINE_BUFFER_SIZE 20

/* 
 * Definitions of pins for automatic control. 
 * Px - Pump x 
 * Vx - Valve x
 */
#define PIN_P1 8
#define PIN_P2 7
#define PIN_P3 6
#define PIN_P4 5
#define PIN_V1 4
#define PIN_V2 3
#define PIN_V3 2

// Definitions of pins for float switch position detection
// FS_Cx - Float Switch Container x
#define FS_C1 12  // Defined by the float switch connected to the driver of pump 4
#define FS_C2 9   // . . . connected to the driver of pump 1
#define FS_C3 10  // . . . connected to the driver of pump 2
#define FS_C4 11  // . . . connected to the driver of pump 3
#define FS_C5 13  // . . . connected to the driver of the valves

#define TCA_ADDR 0x70
#define DEFAULT_BMP280_ADDR 0x76
#define ALT_BMP280_ADDR 0x77

/*
 * Definitions of I2C addresses of INA219 modules
 * for real-time measurement of current and voltage of each component
 */
#define P1_INA_ADDR 0x40
#define P2_INA_ADDR 0x41
#define P3_INA_ADDR 0x42
#define P4_INA_ADDR 0x43
#define V1_INA_ADDR 0x44
#define V2_INA_ADDR 0x45
#define V3_INA_ADDR 0x46

typedef struct  {
    uint8_t AC_PIN; // pin for automatic control
    INA219_WE INA_SENSOR;     
    uint8_t CORRESPONDING_FS;
} component;

typedef struct {
    uint8_t TCA_SLOT;
    uint8_t I2C_ADDR;
    BMP280_DEV device;
    char container[4];
} pressure_sensor;

typedef struct status {
    uint8_t code;
    char message[22];
} status_t;

/*
 * Components definitions
 */

extern component P1; // = {PIN_P1, INA219_WE(P1_INA_ADDR), FS_C2};
extern component P2; // = {PIN_P2, INA219_WE(P2_INA_ADDR), FS_C3};
extern component P3; // = {PIN_P3, INA219_WE(P3_INA_ADDR), FS_C4};
extern component P4; // = {PIN_P4, INA219_WE(P4_INA_ADDR), FS_C1};
extern component V1; // = {PIN_V1, INA219_WE(V1_INA_ADDR), FS_C5};
extern component V2; // = {PIN_V2, INA219_WE(V2_INA_ADDR), FS_C5};
extern component V3; // = {PIN_V3, INA219_WE(V3_INA_ADDR), FS_C5};

// component pumps[] = {P1, P2, P3, P4};
// component valves[] = {V1, V2, V3};

extern pressure_sensor C1_PS; // = {7, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C1"};
extern pressure_sensor C2_PS; // = {6, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C2"};
extern pressure_sensor C3_PS; // = {5, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C3"};
extern pressure_sensor C4_PS; // = {4, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C4"};
extern pressure_sensor C5_PS; // = {3, DEFAULT_BMP280_ADDR, BMP280_DEV(), "C5"};
extern pressure_sensor REFERENCE_PS; // = {3, ALT_BMP280_ADDR, BMP280_DEV(), "REF"};

// extern pressure_sensor pressure_sensors[] = {C1_PS, C2_PS, C3_PS, C4_PS, C5_PS, REFERENCE_PS};

#endif