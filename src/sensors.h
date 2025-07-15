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
    AccelConfig accelConfig,
    GyroConfig gyroConfig
);

/**
 * Calibrates the gyroscope by taking `samples` measures, after waiting `wait` ms between measurements.
 */
void calibrate_gyro(
    BLECharacteristic* progressCharacteristic,
    BLECharacteristic* offsetsCharacteristic,
    int samples = 500,
    int wait = 2
);

/**
 * Fetches the calibration values of the MPU.
 */
float* calibration_mpu();

#endif
