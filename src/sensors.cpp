#include <Arduino.h>

#include <Wire.h>

#include <BLECharacteristic.h>

#include <Preferences.h>

#include "config.h"
#include "sensors.h"

#define I2C_SDA 6
#define I2C_SCL 7

#define MPU6050_ADDR 0x68

/**
 * How often to update the BLE characteristic of the MPU values.
 * In milliseconds.
 */
#define UPDATE_MPU_BLE_EVERY_MILLIS 1000

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

static bool wire_initialized = false;
static bool offsets_loaded = false;

// Allows storing sensor calibration values
// Preferences prefs;

static unsigned long last_mpu_ble_update = 0;

// MPU filter state
float mpu_pitch = 0.0;
float mpu_roll = 0.0;
unsigned long mpu_last_update_time = 0;
// Gyro offsets (calibration bias)
float mpu_gx_offset = 0.0;
float mpu_gy_offset = 0.0;
float mpu_gz_offset = 0.0;
// Complementary filter weight
const float mpu_alpha = 0.98;
// Flag to block readings while calibrating
bool mpu_calibrating = false;

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

void wake_mpu() {
    writeRegister(REG_PWR_MGMT, PWR_MGMT_WAKE); // Clear sleep bit (6), enable all sensors
}

void sleep_mpu() {
    writeRegister(REG_PWR_MGMT, PWR_MGMT_SLEEP); // Set MPU to sleep mode
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

    wake_mpu();
    writeRegister(REG_ACCEL_CONFIG, accelReg);
    writeRegister(REG_GYRO_CONFIG, gyroReg);
    sleep_mpu();

    accelConfigCharacteristic->setValue((uint8_t*)&accelConfig, 1);
    gyroConfigCharacteristic->setValue((uint8_t*)&gyroConfig, 1);
}

void load_calibration() {
    // do not set to read-only because the ns may not be initialized yet
    /*if (!prefs.begin(PREF_MPU_CALIBRATION, false)) {
        Serial.println("Could not initialize prefs environment.");
        return;
    }
    
    if (!prefs.isKey(PREF_MPU_CAL_OFFSET_GX) || !prefs.isKey(PREF_MPU_CAL_OFFSET_GY) || !prefs.isKey(PREF_MPU_CAL_OFFSET_GZ)) {
        Serial.println("No calibration data entries found in flash.");
        prefs.end();
        return;
    }*/

    float gx_offset = 0.0; // prefs.getFloat(PREF_MPU_CAL_OFFSET_GX, NAN);
    float gy_offset = 0.0; // prefs.getFloat(PREF_MPU_CAL_OFFSET_GY, NAN);
    float gz_offset = 0.0; // prefs.getFloat(PREF_MPU_CAL_OFFSET_GZ, NAN);
    // prefs.end();

    if (isnan(gx_offset) || isnan(gy_offset) || isnan(gz_offset)) {
        Serial.println("No calibration data found in flash.");
    } else {
        mpu_gx_offset = gx_offset;
        mpu_gy_offset = gy_offset;
        mpu_gz_offset = gz_offset;

        Serial.println("Loaded calibration from flash:");
        Serial.print("GX: "); Serial.print(gx_offset);
        Serial.print(" GY: "); Serial.print(gy_offset);
        Serial.print(" GZ: "); Serial.println(gz_offset);
    }
}

bool initialize_mpu() {
    if (!wire_initialized) {
        if (!Wire.begin(I2C_SDA, I2C_SCL)) {
            Serial.println("Wire.begin() failed");
            return false;
        }
        Wire.begin(I2C_SDA, I2C_SCL);
        wire_initialized = true;
        delay(100);
    }
    if (!offsets_loaded) {
        load_calibration();
        offsets_loaded = true;
    }

    return true;
}

void calibrate_gyro(
    BLECharacteristic* progressCharacteristic,
    BLECharacteristic* offsetsCharacteristic,
    int samples,
    int wait
) {
    Serial.print("Calibrating gyro... ");
    mpu_calibrating = true;
    long gx_total = 0, gy_total = 0, gz_total = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t data[6];
        readRegisters(0x43, 6, data);

        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];
        int16_t gz = (data[4] << 8) | data[5];

        gx_total += gx;
        gy_total += gy;
        gz_total += gz;

        int progress = ((float)i / samples) * 100;
        progressCharacteristic->setValue(progress);
        progressCharacteristic->notify();

        delay(wait);  // ~1 kHz sampling
    }

    mpu_gx_offset = (float)gx_total / samples;
    mpu_gy_offset = (float)gy_total / samples;
    mpu_gz_offset = (float)gz_total / samples;

    // Save to flash
    /*prefs.begin(PREF_MPU_CALIBRATION, false);
    prefs.putFloat(PREF_MPU_CAL_OFFSET_GX, mpu_gx_offset);
    prefs.putFloat(PREF_MPU_CAL_OFFSET_GY, mpu_gy_offset);
    prefs.putFloat(PREF_MPU_CAL_OFFSET_GZ, mpu_gz_offset);
    prefs.end();*/

    float offsets[3] = {mpu_gx_offset, mpu_gy_offset, mpu_gz_offset};
    offsetsCharacteristic->setValue((uint8_t*)offsets, sizeof(offsets));
    offsetsCharacteristic->notify();

    Serial.println("Done.");
    Serial.print("Offsets: GX: "); Serial.print(mpu_gx_offset);
    Serial.print(" GY: "); Serial.print(mpu_gy_offset);
    Serial.print(" GZ: "); Serial.println(mpu_gz_offset);

    mpu_calibrating = false;
}

float* calibration_mpu() {
    float offsets[3] = {mpu_gx_offset, mpu_gy_offset, mpu_gz_offset};
    return offsets;
}

void compute_pitch_roll(float axf, float ayf, float azf, float gxf, float gyf) {
    // Compute pitch and roll from accelerometer
    float pitch_acc = atan2(axf, sqrt(ayf * ayf + azf * azf)) * 180.0 / PI;
    float roll_acc  = atan2(ayf, sqrt(axf * axf + azf * azf)) * 180.0 / PI;

    // Time delta
    unsigned long now = millis();
    float dt = (now - mpu_last_update_time) / 1000.0; // seconds
    mpu_last_update_time = now;

    // Integrate gyro data
    mpu_pitch += gxf * dt;
    mpu_roll  += gyf * dt;

    // Apply complementary filter
    mpu_pitch = mpu_alpha * mpu_pitch + (1 - mpu_alpha) * pitch_acc;
    mpu_roll  = mpu_alpha * mpu_roll  + (1 - mpu_alpha) * roll_acc;
}

void read_mpu(
    BLECharacteristic* tempCharacteristic,
    BLECharacteristic* accelDataCharacteristic,
    BLECharacteristic* gyroDataCharacteristic,
    AccelConfig accelConfig,
    GyroConfig gyroConfig
) {
    if (!wire_initialized && !initialize_mpu()) return;
    if (mpu_calibrating) return;

    if (millis() - last_mpu_ble_update > UPDATE_MPU_BLE_EVERY_MILLIS) {
        uint8_t rawData[14]; // accel(6), temp(2), gyro(6)

        wake_mpu();
        readRegisters(0x3B, 14, rawData);
        sleep_mpu();

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
        float axf = ax * accel_scale;
        float ayf = ay * accel_scale;
        float azf = az * accel_scale;
        float gxf = (gx - mpu_gx_offset) * gyro_scale;
        float gyf = (gy - mpu_gy_offset) * gyro_scale;
        float gzf = (gz - mpu_gz_offset) * gyro_scale;
        float temperature = (temp_raw / 340.0) + 36.53;

        // Compute pitch and roll from accelerometer
        compute_pitch_roll(axf, ayf, azf, gxf, gyf);

        Serial.print("Accel [g]  X: "); Serial.print(axf, 2);
        Serial.print(" Y: "); Serial.print(ayf, 2);
        Serial.print(" Z: "); Serial.println(azf, 2);

        Serial.print("Gyro [°/s] X: "); Serial.print(gxf, 2);
        Serial.print(" Y: "); Serial.print(gyf, 2);
        Serial.print(" Z: "); Serial.println(gzf, 2);

        Serial.print("Pitch: "); Serial.print(mpu_pitch, 2); Serial.print(" °\t");
        Serial.print("Roll: "); Serial.print(mpu_roll, 2); Serial.println(" °");

        Serial.print("Temp: "); Serial.print(temperature, 2); Serial.println(" °C\n");

        int16_t temp_c_x100 = (int16_t) (temperature * 100);
        uint8_t tempData[2];
        tempData[0] = temp_c_x100 & 0xFF;
        tempData[1] = (temp_c_x100 >> 8) & 0xFF;
        tempCharacteristic->setValue(tempData, 2);
        tempCharacteristic->notify();

        // Pack accel and gyro data as x6 8-bit ints (24 bytes total)
        uint8_t accelData[3] = { axf*100.0, ayf*100.0, azf*100.0 };
        uint8_t gyroData[3]  = { gxf*100.0, gyf*100.0, gzf*100.0 };

        accelDataCharacteristic->setValue(accelData, sizeof(accelData));
        accelDataCharacteristic->notify();

        gyroDataCharacteristic->setValue(gyroData, sizeof(gyroData));
        gyroDataCharacteristic->notify();

        last_mpu_ble_update = millis();
    }
}
