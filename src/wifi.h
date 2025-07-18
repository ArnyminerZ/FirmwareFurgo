#ifndef FURGO_WIFI__h
#define FURGO_WIFI__h

void connectToWifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar);

void configure_wifi(const char* ssid, const char* password, BLECharacteristic* statusChar, BLECharacteristic* addrChar);

void ota_handle();

#endif
