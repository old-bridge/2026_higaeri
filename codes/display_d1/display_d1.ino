/*
 * display_d1.ino
 * 表示マイコン D-1（Slave ID: 2）
 * 
 * 機能：
 * - TFTディスプレイにセンサーデータを表示
 * - Modbus Slaveとして動作
 * - I2C経由でセンサーデータ受け取り
 */

#include <HardwareSerial.h>
#include <ModbusRTU.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>

// ===================================================================
// 定義
// ===================================================================
#define SLAVE_ID 2
#define DE_PIN D3
#define BAUDRATE 5000000
#define I2C_ADDR_D2 0x30

#define TFT_GREY 0x5AEB

// Modbusレジスタ構成（Slave 2: DISPLAY_D1）
// アドレス 0-2: 読み取り専用レジスタ
//   0: 大気圧高度 (cm)
//   1: ポテンショメーター①
//   2: ポテンショメーター②

// ===================================================================
// グローバル変数
// ===================================================================
ModbusRTUServer mb;
HardwareSerial MySerial0(0);
TFT_eSPI tft = TFT_eSPI();

// センサーデータ
uint32_t rotationCount = 0;
uint16_t potentiometer1 = 500;
uint16_t potentiometer2 = 1200;
uint16_t batteryVoltage = 3800;
uint16_t ultrasonicAlt = 1250;
uint16_t baroAlt = 0;  // D2から受け取る値で更新

// I2C通信ステータス
struct I2CStatus {
  bool success = false;
  unsigned long lastSuccessTime = 0;
  uint32_t successCount = 0;
  uint32_t failureCount = 0;
  char message[32] = "No Data";
} i2cStatus;

// Modbus通信ステータス
struct ModbusStatus {
  bool masterCommActive = false;
  unsigned long lastRequestTime = 0;
  uint32_t requestCount = 0;
  char message[32] = "No Request";
} modbusStatus;

// タイミング
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastI2CReadTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;
const unsigned long I2C_READ_INTERVAL = 500;

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[DISPLAY_D1] --- System Start ---");

  // SPI初期化（TFT用）
  Serial.println("[DISPLAY_D1] Initializing SPI...");
  SPI.begin(8, 9, 10);  // SCK=8, MISO=9, MOSI=10
  
  // TFT初期化
  Serial.println("[DISPLAY_D1] Initializing TFT...");
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.drawCentreString("Display D-1", 120, 30, 4);
  tft.drawCentreString("Starting...", 120, 100, 2);
  Serial.println("[DISPLAY_D1] TFT initialized");

  // I2C初期化
  Serial.println("[DISPLAY_D1] Initializing I2C...");
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("[DISPLAY_D1] I2C initialized");

  // Modbus Slave初期化
  Serial.println("[DISPLAY_D1] Initializing Modbus Slave...");
  MySerial0.begin(BAUDRATE, SERIAL_8N1, 5, 4);  // RX: D7(GPIO5), TX: D6(GPIO4)
  mb.startModbusServer(SLAVE_ID, BAUDRATE, MySerial0, false);
  
  Serial.printf("[DISPLAY_D1] Initialized\n");
  Serial.printf("  Slave ID: %d\n", SLAVE_ID);
  Serial.printf("  Baudrate: %lu\n", BAUDRATE);
  Serial.printf("  DE pin: %d\n", DE_PIN);
  Serial.println("[DISPLAY_D1] All systems initialized - Ready!");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // Modbus通信処理（常に実行）
  mb.communicationLoop();
  
  // D-2からセンサーデータを読み込み
  readSensorDataFromD2();
  
  // Modbusレジスタにセンサーデータをセット
  updateModbusRegisters();

  // TFTディスプレイ更新
  if (millis() - lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }
}

// ===================================================================
// Modbusレジスタにセンサーデータをセット
// ===================================================================
void updateModbusRegisters() {
  mb.setHoldingValue(0, baroAlt);           // レジスタ 0: 大気圧高度
  mb.setHoldingValue(1, potentiometer1);    // レジスタ 1: ポテンショ①
  mb.setHoldingValue(2, potentiometer2);    // レジスタ 2: ポテンショ②
}

// ===================================================================
// D-2からセンサーデータを読み込む
// ===================================================================
void readSensorDataFromD2() {
  if (millis() - lastI2CReadTime < I2C_READ_INTERVAL) {
    return;
  }
  lastI2CReadTime = millis();

  // I2C通信試行
  Wire.beginTransmission(I2C_ADDR_D2);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    // リクエスト送信失敗
    i2cStatus.success = false;
    i2cStatus.failureCount++;
    snprintf(i2cStatus.message, sizeof(i2cStatus.message), "Req Failed");
    return;
  }

  int bytesRead = Wire.requestFrom(I2C_ADDR_D2, 12);
  if (bytesRead != 12) {
    // データ受信失敗
    i2cStatus.success = false;
    i2cStatus.failureCount++;
    snprintf(i2cStatus.message, sizeof(i2cStatus.message), "Rx Got %d", bytesRead);
    return;
  }

  // データ読み込み（リトルエンディアン）
  uint8_t buffer[12];
  for (int i = 0; i < 12; i++) {
    buffer[i] = Wire.read();
  }

  potentiometer1 = (buffer[1] << 8) | buffer[0];
  potentiometer2 = (buffer[3] << 8) | buffer[2];
  batteryVoltage = (buffer[5] << 8) | buffer[4];
  ultrasonicAlt = (buffer[7] << 8) | buffer[6];
  baroAlt = (buffer[9] << 8) | buffer[8];

  // I2C通信成功
  i2cStatus.success = true;
  i2cStatus.lastSuccessTime = millis();
  i2cStatus.successCount++;
  snprintf(i2cStatus.message, sizeof(i2cStatus.message), "OK");

  Serial.printf("[DISPLAY_D1] I2C OK: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d, Baro_Alt=%d\n",
                potentiometer1, potentiometer2, batteryVoltage, ultrasonicAlt, baroAlt);
}

// ===================================================================
// TFTディスプレイ更新
// ===================================================================
void updateDisplay() {
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);

  // Title
  tft.drawCentreString("=== FLIGHT DATA ===", 120, 2, 1);

  char buf[64];
  int ypos = 20;
  
  // ========== Rotation ==========
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  sprintf(buf, "Rotation: %lu rpm", rotationCount);
  tft.drawString(buf, 10, ypos, 1);
  ypos += 16;

  // ========== Potentiometers ==========
  tft.setTextColor(TFT_CYAN, TFT_GREY);
  sprintf(buf, "Pot1: %d", potentiometer1);
  tft.drawString(buf, 10, ypos, 1);
  sprintf(buf, "Pot2: %d", potentiometer2);
  tft.drawString(buf, 120, ypos, 1);
  ypos += 16;

  // ========== Battery ==========
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  sprintf(buf, "Battery: %dmV", batteryVoltage);
  tft.drawString(buf, 10, ypos, 1);
  ypos += 16;

  // ========== Altitude (Atmospheric & Ultrasonic) ==========
  tft.setTextColor(TFT_ORANGE, TFT_GREY);
  sprintf(buf, "Baro: %d cm", baroAlt);
  tft.drawString(buf, 10, ypos, 1);
  sprintf(buf, "US: NC");  // 超音波センサ未実装
  tft.drawString(buf, 120, ypos, 1);
  ypos += 16;

  // ========== Attitude (IMU) ==========
  tft.setTextColor(TFT_MAGENTA, TFT_GREY);
  sprintf(buf, "Y: NC");  // Yaw未実装
  tft.drawString(buf, 10, ypos, 1);
  sprintf(buf, "P: NC");  // Pitch未実装
  tft.drawString(buf, 80, ypos, 1);
  sprintf(buf, "R: NC");  // Roll未実装
  tft.drawString(buf, 150, ypos, 1);
  ypos += 16;

  // ========== Separator ==========
  tft.drawLine(0, ypos, 240, ypos, TFT_WHITE);
  ypos += 4;

  // ========== I2C (D-2 Sensor) Status ==========
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  tft.drawString("I2C(D-2):", 10, ypos, 1);
  
  if (i2cStatus.success) {
    tft.setTextColor(TFT_GREEN, TFT_GREY);
  } else {
    tft.setTextColor(TFT_RED, TFT_GREY);
  }
  sprintf(buf, "%s", i2cStatus.message);
  tft.drawString(buf, 80, ypos, 1);
  ypos += 14;
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  sprintf(buf, "OK:%lu Err:%lu", i2cStatus.successCount, i2cStatus.failureCount);
  tft.drawString(buf, 10, ypos, 1);
  ypos += 14;

  // ========== Modbus Master Status ==========
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  tft.drawString("Modbus:", 10, ypos, 1);
  
  if (modbusStatus.masterCommActive) {
    tft.setTextColor(TFT_GREEN, TFT_GREY);
  } else {
    tft.setTextColor(TFT_ORANGE, TFT_GREY);
  }
  sprintf(buf, "%s", modbusStatus.message);
  tft.drawString(buf, 80, ypos, 1);
  ypos += 14;
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  sprintf(buf, "Req: %lu", modbusStatus.requestCount);
  tft.drawString(buf, 10, ypos, 1);

  // ========== Footer ==========
  tft.drawLine(0, 220, 240, 220, TFT_WHITE);
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  tft.drawString("[Slave ID: 2]", 160, 225, 1);
}
