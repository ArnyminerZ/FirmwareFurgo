#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <BLECharacteristic.h>

void connectToWifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar) {
    Serial.printf("Connecting to WiFi SSID: %s\n", ssid);

    uint8_t status = 1; // Connecting
    statusChar->setValue(&status, 1);
    statusChar->notify();

    WiFi.disconnect(true); // Clear previous

    Serial.print("Connecting to WiFi. SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { // timeout of 10 seconds
        delay(500);
    }

    status = (WiFi.status() == WL_CONNECTED) ? 2 : 3;
    statusChar->setValue(&status, 1);
    statusChar->notify();

    if (status == 2) {
        Serial.print("WiFi connected. IP: ");
        Serial.println(WiFi.localIP());
        addrChar->setValue(WiFi.localIP().toString().c_str());
        addrChar->notify();
    } else {
        Serial.println("WiFi connection failed.");
    }
}

void configure_wifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar) {
    Serial.print("Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    Serial.println("ok");

    connectToWifi(ssid, password, statusChar, addrChar);

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

void ota_handle() {
    ArduinoOTA.handle();
}
