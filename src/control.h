#ifndef FURGO_CONTROL__h
#define FURGO_CONTROL__h

#include <Arduino.h>

#include <BLECharacteristic.h>

void initialize_control(BLECharacteristic *controlOutputsCharacteristic);

void bit_control(uint16_t bits);

#endif
