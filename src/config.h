#ifndef FURGO_CONFIG__h
#define FURGO_CONFIG__h

#include <Arduino.h>

enum AccelConfig : uint8_t { S2G = 0, S4G = 1, S8G = 2, S16G = 3 };
enum GyroConfig  : uint8_t { S250 = 0, S500 = 1, S1000 = 2, S2000 = 3 };

#endif