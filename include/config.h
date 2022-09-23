#ifndef config_h
#define config_h

#include <INA219_WE.h>
#include <BMP280_DEV.h>

#define BAUD_RATE 115200

#define DEBUG 1

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

#define EXEC_RESET bit(4)

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
    const char id[3];
    uint8_t AC_PIN; // pin for automatic control
    INA219_WE INA_SENSOR;     
    uint8_t CORRESPONDING_FS;
} component;

typedef struct {
    uint8_t TCA_SLOT;
    uint8_t I2C_ADDR;
    BMP280_DEV device;
    uint8_t idx;
    uint8_t FS_PIN;
    char container[3];
} pressure_sensor;

/*
 * Components definitions
 */

extern component P1;
extern component P2;
extern component P3;
extern component P4;
extern component V1;
extern component V2;
extern component V3;

extern component pumps[];
extern component valves[];

extern pressure_sensor C1_PS;
extern pressure_sensor C2_PS;
extern pressure_sensor C3_PS;
extern pressure_sensor C4_PS;
extern pressure_sensor C5_PS;
extern pressure_sensor REFERENCE_PS;

extern pressure_sensor pressure_sensors[];

#endif