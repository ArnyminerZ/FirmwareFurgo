#include "Arduino.h"

#include <esp_now.h>
#include <WiFi.h>

#include "debug.h"

// ESP-NOW Receiver MACs
// const uint8_t MAC_RECEIVER_1[] = {0xB8, 0xF8, 0x62, 0xE2, 0xD2, 0xD0};

const uint8_t *RECEIVERS_MACS[] = {};
const uint8_t RECEIVERS_COUNT = sizeof(RECEIVERS_MACS) / sizeof(uint8_t *);

bool receiversRegistered[RECEIVERS_COUNT] = {};
bool receiversPinged[RECEIVERS_COUNT] = {};
unsigned long pingSentAt[RECEIVERS_COUNT] = {};

#define MSG_REQ_ACK 0x00 // Request acknowledgment

#define MSG_RES_ACK 0x01 // Response acknowledgment

#define HEALTHCHECK_MILLIS 5000
#define RESPONSE_TIMEOUT 2000

TaskHandle_t healthcheckTaskHandle;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    debug("\r\nLast Packet Send Status:\t");
    debug(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnMessageReceived(const uint8_t *mac, const uint8_t *data, int len)
{
    debugf("Packet received from: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    debugf("Bytes received: %d", len);

    uint8_t sender_id = -1;
    for (uint8_t i = 0; i < RECEIVERS_COUNT; i++)
    {
        if (memcmp(mac, RECEIVERS_MACS[i], 6) == 0)
        {
            sender_id = i;
            break;
        }
    }

    if (sender_id == -1)
    {
        error("Unknown sender");
        return;
    }

    if (data[0] == MSG_RES_ACK)
    {
        success("Response ACK received");
        receiversPinged[sender_id] = true;
    }
}

void SendMessage(uint8_t peek_id, uint8_t* payload, size_t length)
{
    esp_err_t result = esp_now_send(RECEIVERS_MACS[peek_id], payload, length);

    debugf("Sending message to peek %d with MAC: %02X:%02X:%02X:%02X:%02X:%02X. Size: %d", peek_id,
           RECEIVERS_MACS[peek_id][0], RECEIVERS_MACS[peek_id][1], RECEIVERS_MACS[peek_id][2],
           RECEIVERS_MACS[peek_id][3], RECEIVERS_MACS[peek_id][4], RECEIVERS_MACS[peek_id][5], length);
    if (result == ESP_OK)
    {
        success("Sent with success");
    }
    else
    {
        error("Error sending the data");
    }
}

bool registerPeek(uint8_t id)
{
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, RECEIVERS_MACS[id], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    debugf("Registering peer %d with MAC: %02X:%02X:%02X:%02X:%02X:%02X", id,
           RECEIVERS_MACS[id][0], RECEIVERS_MACS[id][1], RECEIVERS_MACS[id][2],
           RECEIVERS_MACS[id][3], RECEIVERS_MACS[id][4], RECEIVERS_MACS[id][5]);

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        error("Failed to add peer");
        receiversRegistered[id] = false;
        return false;
    }
    else
    {
        successf("Registered peer %d", id);
        receiversRegistered[id] = true;
        return true;
    }
}

void unregisterPeek(uint8_t id)
{
    if (esp_now_del_peer(RECEIVERS_MACS[id]) != ESP_OK)
    {
        error("Failed to remove peer");
    }
    else
    {
        successf("Unregistered peer %d", id);
        receiversRegistered[id] = false;
    }
}

void pingPeek(uint8_t id)
{
    if (!receiversRegistered[id])
    {
        debugf("Peek %d is not registered, trying to register...", id);
        if (!registerPeek(id))
            return;
    }

    debugf("Pinging peek %d", id);
    receiversPinged[id] = false;
    pingSentAt[id] = millis();

    uint16_t data = MSG_REQ_ACK;
	uint8_t payload[sizeof(uint16_t)];
	memcpy(payload, &data, sizeof(uint16_t));

    SendMessage(id, payload, sizeof(payload));
}

void registerPeeks()
{
    debug("Registering peers...");
    for (uint8_t i = 0; i < RECEIVERS_COUNT; i++)
    {
        registerPeek(i);
    }
}

void healthcheckTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        for (uint8_t i = 0; i < RECEIVERS_COUNT; i++)
        {
            pingPeek(i);

            unsigned long checkStart = millis();
            while (millis() - checkStart < RESPONSE_TIMEOUT)
            {
                if (receiversPinged[i])
                {
                    infof("Peek %d responded successfully", i);
                    break; // Exit the waiting loop if response received
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // non-blocking wait
            }
        }

        // Check for missed responses
        for (uint8_t i = 0; i < RECEIVERS_COUNT; i++)
        {
            if (!receiversPinged[i])
            {
                errorf("Peek %d did not respond", i);
                unregisterPeek(i);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HEALTHCHECK_MILLIS));
    }
}

void configure_espnow()
{
    if (esp_now_init() != ESP_OK)
    {
        error("Error initializing ESP-NOW");
    }
    else
    {
        success("ESP-NOW initialized successfully");
        esp_now_register_send_cb(OnDataSent);
        esp_now_register_recv_cb(OnMessageReceived);
        info("ESP-NOW callbacks registered");

        registerPeeks();

        // Create FreeRTOS task
        xTaskCreatePinnedToCore(
            healthcheckTask,     // task function
            "HealthCheckTask",   // name
            4096,                // stack size
            NULL,                // parameters
            1,                   // priority
            &healthcheckTaskHandle, // task handle
            1                    // core
        );
    }
}
