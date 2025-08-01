#include <Arduino.h>

#include <BLECharacteristic.h>

#include "debug.h"
#include "storage.h"

#define CONTROL_0 47
#define CONTROL_1 48
#define CONTROL_2 45
#define CONTROL_3 35
#define CONTROL_4 36
#define CONTROL_5 37
#define CONTROL_6 38
#define CONTROL_7 39
#define CONTROL_8 40
#define CONTROL_9 41
#define CONTROL_A 42
#define CONTROL_B 14
#define CONTROL_C 13
#define CONTROL_D 12
#define CONTROL_E 11
#define CONTROL_F 10

#define PINS_COUNT 16

const uint8_t controlPins[PINS_COUNT] = {
  CONTROL_0, CONTROL_1, CONTROL_2, CONTROL_3,
  CONTROL_4, CONTROL_5, CONTROL_6, CONTROL_7,
  CONTROL_8, CONTROL_9, CONTROL_A, CONTROL_B,
  CONTROL_C, CONTROL_D, CONTROL_E, CONTROL_F
};

void initialize_control(BLECharacteristic *controlOutputsCharacteristic) {
    for (int i = 0; i < PINS_COUNT; i++) {
        pinMode(controlPins[i], OUTPUT);
    }

    uint16_t controlBits = get_uint16(PREF_CONTROL_BITS);
    for (int i = 0; i < PINS_COUNT; i++) {
        bool value = (controlBits >> i) & 0x01;
        digitalWrite(controlPins[i], value ? HIGH : LOW);
    }

    controlOutputsCharacteristic->setValue((uint8_t*)&controlBits, sizeof(controlBits));
}

void bit_control(uint16_t bits) {
    infof("Setting control bits: 0x%04X", bits);
    for (int i = 0; i < PINS_COUNT; i++) {
        bool isHigh = (bits >> i) & 0x01;
        debugf("Control pin %d: %s", i, isHigh ? "HIGH" : "LOW");
        // Set the pin state based on the bit value
        digitalWrite(controlPins[i], isHigh ? HIGH : LOW);
    }
}
