#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <BLECharacteristic.h>

#include "debug.h"

#define CONNECTION_CHECK_MILLIS 500
#define CONNECTION_TIMEOUT 10000

bool isConnecting = false;
unsigned long connectionAttemptStart = 0;
unsigned long connectionLastCheck = 0;

BLECharacteristic* _statusChar;
BLECharacteristic* _addrChar;

void connectToWifi(const char* ssid, const char* password) {
    debugf("Connecting to WiFi SSID: %s\n", ssid);

    uint8_t status = 1; // Connecting
    _statusChar->setValue(&status, 1);
    _statusChar->notify();

    WiFi.disconnect(true); // Clear previous

    WiFi.begin(ssid, password);

    isConnecting = true;
    connectionAttemptStart = millis();
    connectionLastCheck = millis();
}

void configure_wifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar) {
    info("Initializing WiFi...");
    WiFi.mode(WIFI_STA);

    _statusChar = statusChar;
    _addrChar = addrChar;

    connectToWifi(ssid, password);

    infof("Connected to: %s", ssid);
    infof("IP Address: %s", WiFi.localIP().toString().c_str());

    info("Preparing OTA...");
    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            infof("Start updating %s", type.c_str());
        })
        .onEnd([]() {
            info("End");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            debugf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            errorf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) errorf("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) errorf("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) errorf("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) errorf("Receive Failed");
            else if (error == OTA_END_ERROR) errorf("End Failed");
        });

    ArduinoOTA.begin();
    info("OTA ready");
}

void wifi_loop() {
    ArduinoOTA.handle();

    if (isConnecting){
        if (millis() - connectionLastCheck > CONNECTION_CHECK_MILLIS)
        {
            if (WiFi.status() == WL_CONNECTED || millis() - connectionAttemptStart > CONNECTION_TIMEOUT)
            {
                // Connection succeed or timeout
                uint8_t status = (WiFi.status() == WL_CONNECTED) ? 2 : 3;
                _statusChar->setValue(&status, 1);
                _statusChar->notify();

                if (status == 2) {
                    infof("WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
                    _addrChar->setValue(WiFi.localIP().toString().c_str());
                    _addrChar->notify();
                } else {
                    error("WiFi connection failed.");
                }
                isConnecting = false;
            } else {
                // Still not connected
                debug("Connecting...");
            }

            connectionLastCheck = millis();
        }
    }
    
}
