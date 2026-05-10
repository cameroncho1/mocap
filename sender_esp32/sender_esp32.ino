#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

uint8_t broadcastAddresses[][6] = {
  { 0xC0, 0x4E, 0x30, 0x4B, 0x61, 0x3A },
  { 0xC0, 0x4E, 0x30, 0x4B, 0x80, 0x3B },
};

esp_now_peer_info_t peerInfo;

const int TARGET_DRONE_INDEX = 1;  // which drone the shortcuts target

void OnDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendToDrone(int droneIndex, const char* payload) {
  Serial.printf("\nsending to drone %d: %s\n", droneIndex, payload);
  esp_err_t result = esp_now_send(broadcastAddresses[droneIndex], 
                                  (const uint8_t *)payload, strlen(payload) + 1);
  if (result) {
    Serial.println(esp_err_to_name(result));
  }
}

void setup() {
  Serial.begin(1000000);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_wifi_start();

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddresses[0], 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer 0");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddresses[1], 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer 1");
    return;
  }

  Serial.println("Sender ready");
  Serial.println("Commands:");
  Serial.println("  d         -> deploy");
  Serial.println("  h         -> hit confirmed");
  Serial.println("  N{...}    -> raw JSON to drone N");
}

char buffer[1024];

void loop() {
  if (!Serial.available()) {
    yield();
    return;
  }

  int availableBytes = Serial.available();
  Serial.readBytes(buffer, availableBytes);
  buffer[availableBytes] = '\0';

  // strip trailing newline / cr
  while (availableBytes > 0 && (buffer[availableBytes-1] == '\n' || buffer[availableBytes-1] == '\r')) {
    buffer[--availableBytes] = '\0';
  }

  if (availableBytes == 0) return;

  // shortcut: 'd' = deploy, 'h' = hit
  if (availableBytes == 1 && buffer[0] == 'd') {
    sendToDrone(TARGET_DRONE_INDEX, "{\"deploy\":true}");
    return;
  }
  if (availableBytes == 1 && buffer[0] == 'h') {
    sendToDrone(TARGET_DRONE_INDEX, "{\"hit\":true}");
    return;
  }

  // raw JSON path: "N{...}" where N is digit drone index
  if (availableBytes > 1 && buffer[0] >= '0' && buffer[0] <= '9') {
    int droneIndex = buffer[0] - '0';
    if (droneIndex < 0 || droneIndex >= (int)(sizeof(broadcastAddresses) / 6)) {
      Serial.printf("Invalid drone index: %d\n", droneIndex);
      return;
    }
    sendToDrone(droneIndex, &buffer[1]);
    return;
  }

  Serial.printf("Unrecognized input: %s\n", buffer);
}