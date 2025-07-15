#include <Arduino.h>

#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Wire.h>

#include "config.h"
#include "sensors.h"

#define SERVICE_UUID        "cead3375-5f7c-41ad-a322-edbdb3079788"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SERVICE_MPU_UUID "12345678-9012-3456-7890-1234567890AB"
#define CHARACTERISTIC_MPU_ACCEL_CONFIG_UUID "12345678-9012-3456-7890-1234567890B0"
#define CHARACTERISTIC_MPU_ACCEL_DATA_UUID   "12345678-9012-3456-7890-1234567890B1"
#define CHARACTERISTIC_MPU_GYRO_CONFIG_UUID  "12345678-9012-3456-7890-1234567890C0"
#define CHARACTERISTIC_MPU_GYRO_DATA_UUID    "12345678-9012-3456-7890-1234567890C1"
#define CHARACTERISTIC_MPU_GYRO_CAL_TRI_UUID   "12345678-9012-3456-7890-1234567890C2" // Calibration trigger
#define CHARACTERISTIC_MPU_GYRO_CAL_PRO_UUID   "12345678-9012-3456-7890-1234567890C3" // Calibration progress
#define CHARACTERISTIC_MPU_GYRO_OFFSET_UUID   "12345678-9012-3456-7890-1234567890C4"  // Calibration offsets

AccelConfig accelConfig = S8G;
GyroConfig gyroConfig = S500;

BLECharacteristic *tempCharacteristic;
BLECharacteristic *accelDataCharacteristic;
BLECharacteristic *gyroDataCharacteristic;
BLECharacteristic *gyroCalibTrigCharacteristic;
BLECharacteristic *gyroCalibProgCharacteristic;
BLECharacteristic *gyroOffsetCharacteristic;

BLECharacteristic *mpuConfigAccelCharacteristic;
BLECharacteristic *mpuConfigGyroCharacteristic;

class AccelConfigCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 1) {
      uint8_t raw = value[0];
      if (raw <= 3) {
        accelConfig = static_cast<AccelConfig>(raw);
        configure_mpu(accelConfig, mpuConfigAccelCharacteristic, gyroConfig, mpuConfigGyroCharacteristic);

        Serial.printf("Updated AccelConfig: %u\n", accelConfig);
      } else {
        Serial.println("Invalid AccelConfig value");
      }
    }
  }
};

class GyroConfigCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 1) {
      uint8_t raw = value[0];
      if (raw <= 3) {
        gyroConfig = static_cast<GyroConfig>(raw);
        configure_mpu(accelConfig, mpuConfigAccelCharacteristic, gyroConfig, mpuConfigGyroCharacteristic);

        Serial.printf("Updated GyroConfig: %u\n", gyroConfig);
      } else {
        Serial.println("Invalid GyroConfig value");
      }
    }
  }
};

class CalibTriggerCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty() && value[0] == 1) {
      Serial.println("Starting MPU calibration...");
      calibrate_gyro(gyroCalibProgCharacteristic, gyroOffsetCharacteristic);
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");


  Serial.print("Booting MPU... ");
  if (initialize_mpu()) {
    configure_mpu(accelConfig, mpuConfigAccelCharacteristic, gyroConfig, mpuConfigGyroCharacteristic);
  }


  BLEDevice::init("FURGO");
  BLEServer *server = BLEDevice::createServer();

  //
  // Environmental Sensing Service
  //
  BLEService *envService = server->createService(BLEUUID((uint16_t)0x181A));
  
  tempCharacteristic = envService->createCharacteristic(
    BLEUUID((uint16_t)0x2A6E),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // Add CCCD to enable notifications
  tempCharacteristic->addDescriptor(new BLE2902());


  // Encode temperature in °C × 100 (e.g., 25.34°C = 2534)
  int16_t temp_c_x100 = 2534;
  uint8_t tempData[2];
  tempData[0] = temp_c_x100 & 0xFF;
  tempData[1] = (temp_c_x100 >> 8) & 0xFF;
  tempCharacteristic->setValue(tempData, 2);

  envService->start();


  //
  // Device Information Service
  //
  BLEService *deviceInfoService = server->createService(BLEUUID((uint16_t)0x180A));

  BLECharacteristic *manufacturerChar = deviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A29),
    BLECharacteristic::PROPERTY_READ
  );
  manufacturerChar->setValue("Arnau Mora Gras");

  BLECharacteristic *fwVersionChar = deviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A26),
    BLECharacteristic::PROPERTY_READ
  );
  fwVersionChar->setValue("1.0.0");

  deviceInfoService->start();


  //
  // MPU Configuration Service
  //
  BLEService *mpuService = server->createService(SERVICE_MPU_UUID);
  float *mpu_offsets = calibration_mpu();

  // Accel Config Characteristic
  mpuConfigAccelCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_ACCEL_CONFIG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  mpuConfigAccelCharacteristic->setCallbacks(new AccelConfigCallback());
  // Accelerometer Data Characteristic
  accelDataCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_ACCEL_DATA_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  accelDataCharacteristic->addDescriptor(new BLE2902());

  // Gyro Config Characteristic
  mpuConfigGyroCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_GYRO_CONFIG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  mpuConfigGyroCharacteristic->setCallbacks(new GyroConfigCallback());
  // Gyroscope Data Characteristic
  gyroDataCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_GYRO_DATA_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  gyroDataCharacteristic->addDescriptor(new BLE2902());
  // Gyroscope Calibration Trigger Characteristic
  gyroCalibTrigCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_GYRO_CAL_TRI_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  gyroCalibTrigCharacteristic->setCallbacks(new CalibTriggerCallback());
  // Gyroscope Calibration Progress Characteristic
  gyroCalibProgCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_GYRO_CAL_PRO_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  gyroCalibProgCharacteristic->addDescriptor(new BLE2902());
  // Gyroscope Calibration Data (3 floats)
  gyroOffsetCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_GYRO_OFFSET_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  gyroOffsetCharacteristic->setValue((uint8_t*)mpu_offsets, sizeof(mpu_offsets));
  gyroOffsetCharacteristic->notify();

  mpuService->start();


  // Start advertising
  BLEAdvertising *advertising = server->getAdvertising();
  advertising->addServiceUUID(envService->getUUID());
  advertising->addServiceUUID(deviceInfoService->getUUID());
  advertising->addServiceUUID(mpuService->getUUID());
  advertising->start();

  Serial.println("BLE device started, advertising...");
}

void loop() {
  read_mpu(
    tempCharacteristic,
    accelDataCharacteristic,
    gyroDataCharacteristic,
    accelConfig,
    gyroConfig
  );

  delay(500);
}
