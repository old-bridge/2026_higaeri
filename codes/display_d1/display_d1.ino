/*
 * display_d1.ino
 * 表示マイコン D-1（Slave ID: 2）
 * 
 * 機能：
 * - TFTディスプレイにセンサーデータを表示
 * - Modbus Slaveとして動作
 * - I2C経由でセンサーデータ受け取り
 */

#define DE_PIN D3  // RS485制御ピン

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "ModbusConfig.h"
#include "ModbusSlave.h"

#define TFT_GREY 0x5AEB
#define I2C_ADDR_D2 0x30

TFT_eSPI tft = TFT_eSPI();

// ===================================================================
// センサーデータ
// ===================================================================
uint32_t rotationCount = 0;
uint16_t potentiometer1 = 500;
uint16_t potentiometer2 = 1200;
uint16_t batteryVoltage = 3800;
uint16_t ultrasonicAlt = 1250;
uint16_t baroAlt = 1245;

// Modbus通信ステータス
struct ModbusStatus {
  // I2C通信状況（D-2センサー）
  bool i2cSuccess = false;
  unsigned long i2cLastSuccessTime = 0;
  uint32_t i2cSuccessCount = 0;
  uint32_t i2cFailureCount = 0;
  
  // Modbusマスター通信状況（ロガーからのリクエスト受信）
  bool masterCommActive = false;
  unsigned long masterLastRequestTime = 0;
  uint32_t masterRequestCount = 0;
  
  char i2cStatusMessage[16] = "No Data";
  char masterStatusMessage[16] = "No Request";
} modbusStatus;

// TFT更新タイマー
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;

// Modbusスレーブインスタンス
ModbusSlaveBase* displaySlave = nullptr;

// ===================================================================
// DisplayD1Slave: ModbusSlaveBase の具体的な実装
// ===================================================================
class DisplayD1Slave : public ModbusSlaveBase {
public:
  DisplayD1Slave(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : ModbusSlaveBase(hwSerial, id, de) {}

protected:
  /**
   * レジスタのセットアップ
   */
  void setupRegisters() override {
    // 最小限の実装：何も初期化しない
    // ModbusRTUServer が自動的にレジスタバッファを管理
  }

  /**
   * コールバック関数の登録
   */
  void setupCallbacks() override {
    // 最小限の実装：何も登録しない
    // 必要に応じて後で追加可能
  }
};

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[DISPLAY_D1] --- System Start ---");

  // ★重要: XIAO ESP32C3のハードウェアSPIピンを明示的に割り当てる
  Serial.println("[DISPLAY_D1] Initializing SPI...");
  SPI.begin(8, 9, 10);  // SCK=8, MISO=9, MOSI=10
  Serial.println("[DISPLAY_D1] SPI initialized");
  
  // TFT初期化
  Serial.println("[DISPLAY_D1] Initializing TFT...");
  tft.init();
  Serial.println("[DISPLAY_D1] TFT init() completed");
  
  tft.setRotation(0);
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  
  // Draw initial message
  tft.drawCentreString("Display D-1", 120, 30, 4);
  tft.drawCentreString("Starting...", 120, 100, 2);
  
  Serial.println("[DISPLAY_D1] TFT display initialized");

  // I2C初期化
  Serial.println("[DISPLAY_D1] Initializing I2C...");
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("[DISPLAY_D1] I2C initialized");

  // Modbus Slave初期化
  Serial.println("[DISPLAY_D1] Initializing Modbus Slave...");
  displaySlave = new DisplayD1Slave(&Serial0, SLAVE_ID_DISPLAY_3_1, DE_PIN);
  displaySlave->begin();
  Serial.println("[DISPLAY_D1] Modbus Slave initialized");

  Serial.println("[DISPLAY_D1] All systems initialized - Ready!");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // Modbusスレーブタスク（常に実行必要）
  if (displaySlave) {
    displaySlave->task();
    
    // マスターからのリクエストを検出
    // ロギング用フラグをチェック
    static unsigned long lastMasterActivity = 0;
    static bool masterWasActive = false;
    
    // Serial0でデータが受信されているかチェック
    if (Serial0.available() > 0) {
      // Modbusマスターからのリクエストがある
      modbusStatus.masterCommActive = true;
      modbusStatus.masterLastRequestTime = millis();
      modbusStatus.masterRequestCount++;
      lastMasterActivity = millis();
      masterWasActive = true;
      snprintf(modbusStatus.masterStatusMessage, sizeof(modbusStatus.masterStatusMessage), "Active");
    }
    
    // タイムアウト検出（3秒以上通信がない）
    if (millis() - lastMasterActivity > 3000 && masterWasActive) {
      modbusStatus.masterCommActive = false;
      snprintf(modbusStatus.masterStatusMessage, sizeof(modbusStatus.masterStatusMessage), "Idle");
    }
  }

  // D-2からセンサーデータを読み込み
  readSensorDataFromD2();

  // TFTディスプレイ更新
  if (millis() - lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }
}

// ===================================================================
// D-2からセンサーデータを読み込む
// ===================================================================
void readSensorDataFromD2() {
  static unsigned long lastI2CRead = 0;
  if (millis() - lastI2CRead < 500) return;  // 500ms毎に読み込み
  lastI2CRead = millis();

  // I2C通信試行
  Wire.beginTransmission(I2C_ADDR_D2);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    // リクエスト送信失敗
    modbusStatus.i2cSuccess = false;
    modbusStatus.i2cFailureCount++;
    snprintf(modbusStatus.i2cStatusMessage, sizeof(modbusStatus.i2cStatusMessage), "Req Failed");
    return;
  }

  int bytesRead = Wire.requestFrom(I2C_ADDR_D2, 8);
  if (bytesRead != 8) {
    // データ受信失敗
    modbusStatus.i2cSuccess = false;
    modbusStatus.i2cFailureCount++;
    snprintf(modbusStatus.i2cStatusMessage, sizeof(modbusStatus.i2cStatusMessage), "Rx Got %d", bytesRead);
    return;
  }

  // データ読み込み（リトルエンディアン）
  uint8_t buffer[8];
  for (int i = 0; i < 8; i++) {
    buffer[i] = Wire.read();
  }

  potentiometer1 = (buffer[1] << 8) | buffer[0];
  potentiometer2 = (buffer[3] << 8) | buffer[2];
  batteryVoltage = (buffer[5] << 8) | buffer[4];
  ultrasonicAlt = (buffer[7] << 8) | buffer[6];

  // I2C通信成功
  modbusStatus.i2cSuccess = true;
  modbusStatus.i2cLastSuccessTime = millis();
  modbusStatus.i2cSuccessCount++;
  snprintf(modbusStatus.i2cStatusMessage, sizeof(modbusStatus.i2cStatusMessage), "OK");

  Serial.printf("[DISPLAY_D1] I2C OK: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d\n",
                potentiometer1, potentiometer2, batteryVoltage, ultrasonicAlt);
}

// ===================================================================
// TFTディスプレイ更新
// ===================================================================
void updateDisplay() {
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);

  // Title
  tft.drawCentreString("=== FLIGHT DATA ===", 120, 2, 1);

  // ========== Rotation (Smaller size) ==========
  char buf[64];
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  sprintf(buf, "%lu", rotationCount);
  tft.drawString(buf, 10, 20, 4);  // Smaller than before
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.drawString("Rotation (rpm)", 10, 60, 1);

  // ========== Potentiometers ==========
  tft.setTextColor(TFT_CYAN, TFT_GREY);
  sprintf(buf, "Pot1: %d", potentiometer1);
  tft.drawString(buf, 10, 80, 1);

  sprintf(buf, "Pot2: %d", potentiometer2);
  tft.drawString(buf, 120, 80, 1);

  // ========== Battery ==========
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  sprintf(buf, "Battery: %dmV", batteryVoltage);
  tft.drawString(buf, 10, 100, 1);

  // ========== Altitude ==========
  tft.setTextColor(TFT_ORANGE, TFT_GREY);
  sprintf(buf, "US Alt: %dm", ultrasonicAlt);
  tft.drawString(buf, 10, 120, 1);

  sprintf(buf, "Baro: %dm", baroAlt);
  tft.drawString(buf, 120, 120, 1);

  // ========== I2C (D-2 Sensor) Status ==========
  tft.drawLine(0, 140, 240, 140, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  tft.drawString("I2C(D-2):", 10, 150, 1);
  
  if (modbusStatus.i2cSuccess) {
    tft.setTextColor(TFT_GREEN, TFT_GREY);
  } else {
    tft.setTextColor(TFT_RED, TFT_GREY);
  }
  sprintf(buf, "%s", modbusStatus.i2cStatusMessage);
  tft.drawString(buf, 100, 150, 1);
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  sprintf(buf, "OK:%lu Err:%lu", modbusStatus.i2cSuccessCount, modbusStatus.i2cFailureCount);
  tft.drawString(buf, 10, 165, 1);

  // ========== Modbus Master Status ==========
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  tft.drawString("Master:", 10, 185, 1);
  
  if (modbusStatus.masterCommActive) {
    tft.setTextColor(TFT_GREEN, TFT_GREY);
  } else {
    tft.setTextColor(TFT_ORANGE, TFT_GREY);
  }
  sprintf(buf, "%s", modbusStatus.masterStatusMessage);
  tft.drawString(buf, 100, 185, 1);
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  sprintf(buf, "Req: %lu", modbusStatus.masterRequestCount);
  tft.drawString(buf, 10, 202, 1);

  // ========== Footer ==========
  tft.drawLine(0, 220, 240, 220, TFT_WHITE);
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  tft.drawString("[Slave ID: 2]", 160, 225, 1);
}
