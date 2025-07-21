#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <BLECharacteristic.h>

#define CONNECTION_CHECK_MILLIS 500
#define CONNECTION_TIMEOUT 10000

bool isConnecting = false;
unsigned long connectionAttemptStart = 0;
unsigned long connectionLastCheck = 0;

BLECharacteristic* _statusChar;
BLECharacteristic* _addrChar;

void connectToWifi(const char* ssid, const char* password) {
    Serial.printf("Connecting to WiFi SSID: %s\n", ssid);

    uint8_t status = 1; // Connecting
    _statusChar->setValue(&status, 1);
    _statusChar->notify();

    WiFi.disconnect(true); // Clear previous

    Serial.print("Connecting to WiFi. SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    isConnecting = true;
    connectionAttemptStart = millis();
    connectionLastCheck = millis();
}

void configure_wifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar) {
    Serial.print("Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    Serial.println("ok");

    _statusChar = statusChar;
    _addrChar = addrChar;

    connectToWifi(ssid, password);

    Serial.print("Connected to: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("Preparing OTA...");
    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("ok");
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
                    Serial.print("WiFi connected. IP: ");
                    Serial.println(WiFi.localIP());
                    _addrChar->setValue(WiFi.localIP().toString().c_str());
                    _addrChar->notify();
                } else {
                    Serial.println("WiFi connection failed.");
                }
                isConnecting = false;
            } else {
                // Still not connected
                Serial.println("Connecting...");
            }

            connectionLastCheck = millis();
        }
    }
    
}
