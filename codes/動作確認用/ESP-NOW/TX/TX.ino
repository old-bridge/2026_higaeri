// ESP-NOW 距離確認用 TX
// マイコン単体。500ms 周期でブロードキャスト送信し続ける。
// 起動時に自身の MAC アドレスをシリアル出力する。

#include <esp_now.h>
#include <WiFi.h>

constexpr uint32_t kSendIntervalMs = 500;
constexpr uint32_t kDebugBaudRate  = 115200;

struct RangeTestPacket {
  uint8_t  deviceId;      // 0xAA 固定
  uint32_t sequenceNumber;
  uint32_t timestampMs;
};

namespace {
const uint8_t kBroadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t g_sequence = 0;
unsigned long g_lastSendAt = 0;

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
  // ブロードキャストは常に ESP_NOW_SEND_SUCCESS が返るため参考程度
  (void)mac;
  (void)status;
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();

  Serial.print("[TX] MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX] ESP-NOW init FAILED");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kBroadcastAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("[TX] ESP-NOW init OK. Sending...");
}

void loop() {
  if (millis() - g_lastSendAt < kSendIntervalMs) {
    return;
  }
  g_lastSendAt = millis();

  RangeTestPacket packet;
  packet.deviceId       = 0xAA;
  packet.sequenceNumber = ++g_sequence;
  packet.timestampMs    = millis();

  esp_now_send(kBroadcastAddress,
               reinterpret_cast<const uint8_t*>(&packet),
               sizeof(packet));

  Serial.printf("[TX] seq=%lu\n", static_cast<unsigned long>(g_sequence));
}
