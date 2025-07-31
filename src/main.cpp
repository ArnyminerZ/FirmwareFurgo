#include <Arduino.h>

#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "config.h"
#include "control.h"
#include "debug.h"
#include "remotes.h"
#include "sensors.h"
#include "wifi.h"

#define SERVICE_MPU_UUID "12345678-9012-3456-7890-1234567890AB"
#define CHARACTERISTIC_MPU_ACCEL_CONFIG_UUID "12345678-9012-3456-7890-1234567890B0"
#define CHARACTERISTIC_MPU_ACCEL_DATA_UUID   "12345678-9012-3456-7890-1234567890B1"
#define CHARACTERISTIC_MPU_GYRO_CONFIG_UUID  "12345678-9012-3456-7890-1234567890C0"
#define CHARACTERISTIC_MPU_GYRO_DATA_UUID    "12345678-9012-3456-7890-1234567890C1"
#define CHARACTERISTIC_MPU_PITCH_ROLL_UUID   "12345678-9012-3456-7890-1234567890C5"

#define SERVICE_WIFI_UUID             "0000AA3F-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_WIFI_SSID_UUID "0000AA30-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_WIFI_PASS_UUID "0000AA31-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_WIFI_CONN_UUID "0000AA32-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_WIFI_STAT_UUID "0000AA33-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_WIFI_ADDR_UUID "0000AA34-0000-1000-8000-00805F9B34FB"

#define SERVICE_CONTROL_UUID "12345678-9012-3456-7890-1234567890D0"
#define CHARACTERISTIC_CTRL_OUTPUT "12345678-9012-3456-7890-1234567890D1"

#define SSID "Wifi Casa Domotica"
#define PASSWORD "rgo74amm75amg02rmg07"

AccelConfig accelConfig = S8G;
GyroConfig gyroConfig = S500;

BLEServer* pServer = nullptr;
BLEAdvertising* pAdvertising = nullptr;

BLECharacteristic *tempCharacteristic;
BLECharacteristic *accelDataCharacteristic;
BLECharacteristic *gyroDataCharacteristic;
BLECharacteristic *pitchRollYawCharacteristic;

BLECharacteristic *mpuConfigAccelCharacteristic;
BLECharacteristic *mpuConfigGyroCharacteristic;

BLECharacteristic *controlOutputsCharacteristic;

BLECharacteristic* wifiSsidChar;
BLECharacteristic* wifiPassChar;
BLECharacteristic* wifiConnectChar;
BLECharacteristic* wifiStatusChar;
BLECharacteristic* wifiAddressChar;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    info("Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    info("Client disconnected, restarting advertising...");
    delay(100); // optional small delay to avoid issues
    pAdvertising->start();
  }
};

class AccelConfigCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 1) {
      uint8_t raw = value[0];
      if (raw <= 3) {
        accelConfig = static_cast<AccelConfig>(raw);
        configure_mpu(accelConfig, mpuConfigAccelCharacteristic, gyroConfig, mpuConfigGyroCharacteristic);

        successf("Updated AccelConfig: %u", accelConfig);
      } else {
        error("Invalid AccelConfig value");
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

        successf("Updated GyroConfig: %u", gyroConfig);
      } else {
        error("Invalid GyroConfig value");
      }
    }
  }
};

class ControlCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty() && value.length() >= 2) {
      uint8_t bits1 = value[0];
      uint8_t bits2 = value[1];
      uint16_t bits = static_cast<unsigned>(bits1) << 8 | static_cast<unsigned>(bits2);
      bit_control(bits);
    } else {
      errorf("Received invalid control value: %s", value.c_str());
    }
  }
};

class WifiConnectCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty() && value[0] == 1) {
      std::string ssid = wifiSsidChar->getValue();
      std::string pass = wifiPassChar->getValue();
      connectToWifi(ssid.c_str(), pass.c_str());
    }
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect
  }
  info("Starting BLE work!");

  delay(1000);


  BLEDevice::init("FURGO");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  //
  // Environmental Sensing Service
  //
  BLEService *envService = pServer->createService(BLEUUID((uint16_t)0x181A));
  
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
  BLEService *deviceInfoService = pServer->createService(BLEUUID((uint16_t)0x180A));

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
  BLEService *mpuService = pServer->createService(SERVICE_MPU_UUID);

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
  pitchRollYawCharacteristic = mpuService->createCharacteristic(
    CHARACTERISTIC_MPU_PITCH_ROLL_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pitchRollYawCharacteristic->addDescriptor(new BLE2902());

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

  mpuService->start();


  //
  // Control Service
  //
  BLEService *controlService = pServer->createService(SERVICE_CONTROL_UUID);

  // Accel Config Characteristic
  controlOutputsCharacteristic = controlService->createCharacteristic(
    CHARACTERISTIC_CTRL_OUTPUT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  controlOutputsCharacteristic->setCallbacks(new ControlCallback());

  controlService->start();


  //
  // WIFI Configuration Service
  //
  BLEService *wifiService = pServer->createService("0000AA3F-0000-1000-8000-00805F9B34FB");
  wifiSsidChar = wifiService->createCharacteristic(
    CHARACTERISTIC_WIFI_SSID_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  wifiSsidChar->setValue(SSID);
  wifiPassChar = wifiService->createCharacteristic(
    CHARACTERISTIC_WIFI_PASS_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  wifiPassChar->setValue(PASSWORD);
  wifiConnectChar = wifiService->createCharacteristic(
    CHARACTERISTIC_WIFI_CONN_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  wifiConnectChar->setCallbacks(new WifiConnectCallback());
  wifiStatusChar = wifiService->createCharacteristic(
    CHARACTERISTIC_WIFI_STAT_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  wifiStatusChar->addDescriptor(new BLE2902());
  wifiAddressChar = wifiService->createCharacteristic(
    CHARACTERISTIC_WIFI_ADDR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  wifiAddressChar->addDescriptor(new BLE2902());
  wifiAddressChar->setValue("");

  wifiService->start();


  // Start advertising
  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(envService->getUUID());
  pAdvertising->addServiceUUID(deviceInfoService->getUUID());
  pAdvertising->addServiceUUID(mpuService->getUUID());
  pAdvertising->addServiceUUID(wifiService->getUUID());
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();

  info("BLE device started, advertising...");


  info("Booting MPU... ");
  if (initialize_mpu()) {
    configure_mpu(accelConfig, mpuConfigAccelCharacteristic, gyroConfig, mpuConfigGyroCharacteristic);
  }

  configure_wifi(SSID, PASSWORD, wifiStatusChar, wifiAddressChar);

  configure_espnow();

  info("Booting control outputs...");
  initialize_control(controlOutputsCharacteristic);
}

void loop() {
  read_mpu(
    tempCharacteristic,
    accelDataCharacteristic,
    gyroDataCharacteristic,
    pitchRollYawCharacteristic,
    accelConfig,
    gyroConfig
  );

  wifi_loop();
}
