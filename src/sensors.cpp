#include <Arduino.h>

#include <Wire.h>

#include <BLECharacteristic.h>

#include <MadgwickAHRS.h>

#include "config.h"
#include "debug.h"
#include "sensors.h"

#define I2C_SDA 6
#define I2C_SCL 7

#define MPU6050_ADDR 0x68

// Comment to disable
// #define DEBUG_SENSOR_VALUES

/**
 * How often to update the BLE characteristic of the MPU values.
 * In milliseconds.
 */
#define UPDATE_MPU_BLE_EVERY_MILLIS 1000

/**
 * Take a measurement from the MPU every x milliseconds.
 */
#define READ_MPU_EVERY_MILLIS 20 // 50Hz

// BEGIN::MPU Configuration Constants
#define REG_PWR_MGMT 0x6B
#define PWR_MGMT_WAKE 0x00
#define PWR_MGMT_SLEEP 0x40

#define REG_ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2G 0x00  // +- 2G
#define ACCEL_CONFIG_4G 0x08  // +- 4G
#define ACCEL_CONFIG_8G 0x10  // +- 8G
#define ACCEL_CONFIG_16G 0x18 // +- 16G

#define REG_GYRO_CONFIG 0x1B
#define GYRO_CONFIG_250  0x00
#define GYRO_CONFIG_500  0x08
#define GYRO_CONFIG_1000 0x10
#define GYRO_CONFIG_2000 0x18

// Scaling factors (LSB/unit)
#define SCALE_2G_LSB_ACCEL   16384.0
#define SCALE_4G_LSB_ACCEL    8192.0
#define SCALE_8G_LSB_ACCEL    4096.0
#define SCALE_16G_LSB_ACCEL   2048.0

#define SCALE_250_LSB_GYRO   131.0
#define SCALE_500_LSB_GYRO    65.5
#define SCALE_1000_LSB_GYRO   32.8
#define SCALE_2000_LSB_GYRO   16.4
// END::MPU Configuration Constants

#define PREF_MPU_CALIBRATION "mpu-cal"
#define PREF_MPU_CAL_OFFSET_GX "gx_offset"
#define PREF_MPU_CAL_OFFSET_GY "gy_offset"
#define PREF_MPU_CAL_OFFSET_GZ "gz_offset"

#define MPU_CAL_ITERATIONS 500
#define MPU_MADGWICH_FREQ 1000.0 / READ_MPU_EVERY_MILLIS // Convert from millis to hertz
#define DEG_TO_RAD 3.14159265359f / 180.0f
Madgwick mpu_filter;
static float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float axf = 0, ayf = 0, azf = 0;
float gxf = 0, gyf = 0, gzf = 0;
float temperature = 0;
float pitch = 0, roll = 0, yaw = 0;

static bool wire_initialized = false;

static unsigned long last_mpu_ble_update = millis();
static unsigned long last_mpu_read = millis();

// MPU filter state
unsigned long mpu_last_update_time = 0;

void writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Restart for read
  Wire.requestFrom(MPU6050_ADDR, count);
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

void configure_mpu(
    AccelConfig accelConfig,
    BLECharacteristic* accelConfigCharacteristic,
    GyroConfig gyroConfig,
    BLECharacteristic* gyroConfigCharacteristic
) {
    uint8_t accelReg = 0;
    switch (accelConfig) {
        case S2G:  accelReg = ACCEL_CONFIG_2G;  break;
        case S4G:  accelReg = ACCEL_CONFIG_4G;  break;
        case S8G:  accelReg = ACCEL_CONFIG_8G;  break;
        case S16G: accelReg = ACCEL_CONFIG_16G; break;
    }

    uint8_t gyroReg = 0;
    switch (gyroConfig) {
        case S250:  gyroReg = GYRO_CONFIG_250;  break;
        case S500:  gyroReg = GYRO_CONFIG_500;  break;
        case S1000: gyroReg = GYRO_CONFIG_1000; break;
        case S2000: gyroReg = GYRO_CONFIG_2000; break;
    }

    writeRegister(REG_ACCEL_CONFIG, accelReg);
    writeRegister(REG_GYRO_CONFIG, gyroReg);

    accelConfigCharacteristic->setValue((uint8_t*)&accelConfig, 1);
    gyroConfigCharacteristic->setValue((uint8_t*)&gyroConfig, 1);
}

void calibrate_mpu() {
    info("Calibrating MPU...");

    // Run this during calibration
    for (int i = 0; i < MPU_CAL_ITERATIONS; i++) {
        uint8_t rawData[14]; // accel(6), temp(2), gyro(6)
        readRegisters(0x3B, 14, rawData);

        // Convert data
        int16_t gx = (rawData[8]  << 8) | rawData[9];
        int16_t gy = (rawData[10] << 8) | rawData[11];
        int16_t gz = (rawData[12] << 8) | rawData[13];
        
        gx_offset += gx;
        gy_offset += gy;
        gz_offset += gz;
        delay(2);
    }
    gx_offset /= MPU_CAL_ITERATIONS;
    gy_offset /= MPU_CAL_ITERATIONS;
    gz_offset /= MPU_CAL_ITERATIONS;
    info("MPU calibration complete");

    debugf("Gyroscope calibration values: %f, %f, %f", gx_offset, gy_offset, gz_offset);
}

bool initialize_mpu() {
    if (!wire_initialized) {
        if (!Wire.begin(I2C_SDA, I2C_SCL)) {
            error("Wire.begin() failed");
            return false;
        }
        wire_initialized = true;
        delay(100);
    }

    // Wake sensor
    writeRegister(REG_PWR_MGMT, PWR_MGMT_WAKE);
    delay(20);

    infof("Initializing Madgwick filter at %f Hz", MPU_MADGWICH_FREQ);
    mpu_filter.begin(MPU_MADGWICH_FREQ);

    calibrate_mpu();

    return true;
}

void set_float_array_to_characteristic(
    BLECharacteristic* characteristic,
    float* array,
    size_t size
) {
    uint8_t data[size*2];
    for (int c = 0; c < size; c++) {
        int16_t value_x100 = (int16_t) (array[c] * 100);
        data[c*2] = value_x100 & 0xFF;
        data[c*2 + 1] = (value_x100 >> 8) & 0xFF;
    }
    characteristic->setValue(data, size*2);
    characteristic->notify();
}

void read_mpu(
    BLECharacteristic* tempCharacteristic,
    BLECharacteristic* accelDataCharacteristic,
    BLECharacteristic* gyroDataCharacteristic,
    BLECharacteristic* pitchRollYawCharacteristic,
    AccelConfig accelConfig,
    GyroConfig gyroConfig
) {
    if (!wire_initialized && !initialize_mpu()) return;

    if (millis() - last_mpu_read > READ_MPU_EVERY_MILLIS) {
        uint8_t rawData[14]; // accel(6), temp(2), gyro(6)
        readRegisters(0x3B, 14, rawData);

        // Convert data
        int16_t ax = (rawData[0] << 8) | rawData[1];
        int16_t ay = (rawData[2] << 8) | rawData[3];
        int16_t az = (rawData[4] << 8) | rawData[5];

        int16_t temp_raw = (rawData[6] << 8) | rawData[7];

        int16_t gx = (rawData[8]  << 8) | rawData[9];
        int16_t gy = (rawData[10] << 8) | rawData[11];
        int16_t gz = (rawData[12] << 8) | rawData[13];

        // Determine scaling
        float accel_scale = 1.0;
        switch (accelConfig) {
            case S2G:  accel_scale = 1.0 / SCALE_2G_LSB_ACCEL;  break;
            case S4G:  accel_scale = 1.0 / SCALE_4G_LSB_ACCEL;  break;
            case S8G:  accel_scale = 1.0 / SCALE_8G_LSB_ACCEL;  break;
            case S16G: accel_scale = 1.0 / SCALE_16G_LSB_ACCEL; break;
        }

        float gyro_scale = 1.0;
        switch (gyroConfig) {
            case S250:  gyro_scale = 1.0 / SCALE_250_LSB_GYRO;  break;
            case S500:  gyro_scale = 1.0 / SCALE_500_LSB_GYRO;  break;
            case S1000: gyro_scale = 1.0 / SCALE_1000_LSB_GYRO; break;
            case S2000: gyro_scale = 1.0 / SCALE_2000_LSB_GYRO; break;
        }

        // Scaled output
        axf = ax * accel_scale;
        ayf = ay * accel_scale;
        azf = az * accel_scale;
        gxf = (gx - gx_offset) * gyro_scale;
        gyf = (gy - gy_offset) * gyro_scale;
        gzf = (gz - gz_offset) * gyro_scale;
        temperature = (temp_raw / 340.0) + 36.53;

        // Compute pitch and roll from accelerometer
        mpu_filter.updateIMU(gxf*DEG_TO_RAD, gyf*DEG_TO_RAD, gzf*DEG_TO_RAD, axf, ayf, azf);
        pitch = mpu_filter.getPitch();
        roll = mpu_filter.getRoll();
        yaw = mpu_filter.getYaw();

#ifdef DEBUG_SENSOR_VALUES
        Serial.print("Accel [g]  X: "); Serial.print(axf, 2);
        Serial.print(" Y: "); Serial.print(ayf, 2);
        Serial.print(" Z: "); Serial.println(azf, 2);

        Serial.print("Gyro [°/s] X: "); Serial.print(gxf, 2);
        Serial.print(" Y: "); Serial.print(gyf, 2);
        Serial.print(" Z: "); Serial.println(gzf, 2);

        Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print(" °\t");
        Serial.print("Roll: "); Serial.print(roll, 2); Serial.print(" °\t");
        Serial.print("Yaw: "); Serial.print(yaw, 2); Serial.println(" °");

        Serial.print("Temp: "); Serial.print(temperature, 2); Serial.println(" °C\n");
#endif

        last_mpu_read = millis();
    }

    if (millis() - last_mpu_ble_update > UPDATE_MPU_BLE_EVERY_MILLIS) {
        int16_t temp_c_x100 = (int16_t) (temperature * 100);
        uint8_t tempData[2];
        tempData[0] = temp_c_x100 & 0xFF;
        tempData[1] = (temp_c_x100 >> 8) & 0xFF;
        tempCharacteristic->setValue(tempData, 2);
        tempCharacteristic->notify();

        // Pack accel and gyro data as x6 8-bit ints (24 bytes total)
        float accelData[3] = { axf, ayf, azf };
        float gyroData[3]  = { gxf, gyf, gzf };
        float pitchRollYaw[3]  = { pitch, roll, yaw };

        set_float_array_to_characteristic(accelDataCharacteristic, accelData, 3);
        set_float_array_to_characteristic(gyroDataCharacteristic, gyroData, 3);
        set_float_array_to_characteristic(pitchRollYawCharacteristic, pitchRollYaw, 3);

        last_mpu_ble_update = millis();
    }
}
