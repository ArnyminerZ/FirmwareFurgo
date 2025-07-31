#ifndef FURGO_SENSORS__h
#define FURGO_SENSORS__h

void configure_mpu(
    AccelConfig accelConfig,
    BLECharacteristic* accelConfigCharacteristic,
    GyroConfig gyroConfig,
    BLECharacteristic* gyroConfigCharacteristic
);
bool initialize_mpu();

void wake_mpu();
void sleep_mpu();

void read_mpu(
    BLECharacteristic* tempCharacteristic,
    BLECharacteristic* accelDataCharacteristic,
    BLECharacteristic* gyroDataCharacteristic,
    BLECharacteristic* pitchRollYawCharacteristic,
    AccelConfig accelConfig,
    GyroConfig gyroConfig
);

#endif
