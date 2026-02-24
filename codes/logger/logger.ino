/**
 * logger.ino
 * ロガーマイコン（Modbus Master）
 * 
 * 機能：
 * - エアデータと表示マイコンからデータを読み込み
 * - MicroSD に全データをログ記録
 * - IMU から姿勢データを読む（I2C）
 * - RTC から時刻を取得（I2C）
 * - ボーレート動的切り替え制御
 */

#define DE_PIN D2  // RS485制御ピン

#include "../shared/ModbusConfig.h"
#include "../shared/ModbusMaster.h"

// ===================================================================
// グローバル変数
// ===================================================================

ModbusMaster master(&Serial0, DE_PIN);

// センサー値バッファ
uint16_t airDataBuf[AIR_REG_READ_SIZE];      // エアデータから読む
uint16_t displayBuf[DISP_REG_READ_SIZE];     // 表示から読む

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;   // 1秒ごと

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  master.begin();
  
  // TODO: MicroSD初期化
  // TODO: IMU初期化
  // TODO: RTC初期化
  // TODO: コマ1ンダ入力初期化（シリアルモニタなど）
  
  Serial.println("[LOGGER] All systems initialized");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // Modbusマスタータスク（常に実行必要）
  master.task();

  // 定期的にデータを読み込み
  if (millis() - lastReadTime > READ_INTERVAL) {
    lastReadTime = millis();
    
    // エアデータマイコンから読む
    if (master.readRegistersSync(SLAVE_ID_AIR_DATA, AIR_REG_READ, airDataBuf, AIR_REG_READ_SIZE)) {
      Serial.printf("[LOGGER] Air Data: Rotation=%d, AS5600_1=%d, AS5600_2=%d, Batt=%d\n",
                    airDataBuf[0], airDataBuf[1], airDataBuf[2], airDataBuf[3]);
      
      // TODO: MicroSDに記録
    } else {
      Serial.println("[LOGGER] FAILED to read Air Data");
    }
    
    // 表示マイコンから読む
    if (master.readRegistersSync(SLAVE_ID_DISPLAY, DISP_REG_READ, displayBuf, DISP_REG_READ_SIZE)) {
      Serial.printf("[LOGGER] Display Data: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d, Pressure_Alt=%d\n",
                    displayBuf[0], displayBuf[1], displayBuf[2], displayBuf[3], displayBuf[4]);
      
      // TODO: MicroSDに記録
    } else {
      Serial.println("[LOGGER] FAILED to read Display Data");
    }
  }

  // TODO: シリアルコマンド処理（ボーレート切り替え指令など）
  // TODO: IMU/RTCデータ読み込み
}

/**
 * ボーレート切り替える例（シリアルコマンドで指令される想定）
 */
void changeMasterBaud(int baudIdx) {
  if (baudIdx < 0 || baudIdx >= NUM_BAUDS) {
    Serial.printf("Invalid baud index: %d\n", baudIdx);
    return;
  }
  
  // 最初にエアデータマイコンを切り替え
  Serial.printf("Changing Air Data baud to index %d...\n", baudIdx);
  master.requestBaudChange(SLAVE_ID_AIR_DATA, baudIdx);
  delay(2000);
  
  // 次に表示マイコンを切り替え
  Serial.printf("Changing Display baud to index %d...\n", baudIdx);
  master.requestBaudChange(SLAVE_ID_DISPLAY, baudIdx);
  delay(2000);
  
  Serial.printf("All slaves now at %ld baud\n", master.getCurrentBaud());
}

/**
 * シリアルモニタからのコマンド処理（オプション）
 */
void handleSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'r':
        Serial.println("Requesting READ test...");
        // テスト読み込み
        break;
      case 'w':
        Serial.println("Requesting WRITE test...");
        // テスト書き込み
        break;
      case 'b':
        Serial.println("Enter baud index (0-4): ");
        while (!Serial.available());
        int baudIdx = Serial.read() - '0';
        changeMasterBaud(baudIdx);
        break;
      default:
        break;
    }
  }
}
