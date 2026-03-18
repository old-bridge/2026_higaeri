/**
 * logger.ino
 * ロガーマイコン（Modbus Master）
 * 
 * 機能：
 * - エアデータマイコンからセンサーデータを読み込み
 * - Display D-1（TFT + 大気圧センサ）からデータを読み込み
 *   ※ Display D-2のセンサーデータはD-1経由で取得
 * - MicroSD に全データをログ記録
 * - IMU から姿勢データを読む（I2C）
 * - RTC から時刻を取得（I2C）
 * - ボーレート動的切り替え制御
 */

#define DE_PIN D2  // RS485制御ピン

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "../shared/ModbusConfig.h"
#include "../shared/ModbusMaster.h"

// ===================================================================
// グローバル変数
// ===================================================================

ModbusMaster master(&Serial0, DE_PIN);

// センサー値バッファ
uint16_t airDataBuf[AIR_REG_READ_SIZE];          // エアデータから読む
uint16_t displayD1Buf[DISP_D1_REG_READ_SIZE];   // Display D-1から読む

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;   // 1秒ごと

// 送信用バッファ
uint16_t rotationCountToSend = 0;  // display_d1に送信する回転数

// SD カード関連
const char* LOG_FILE = "/flight_log.csv";  // ログファイル名
bool sdCardReady = false;                   // SD カード初期化状態

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  master.begin();
  
  // SD カード初期化
  initSDCard();
  
  // TODO: IMU初期化
  // TODO: RTC初期化
  // TODO: コマンド入力初期化（シリアルモニタなど）
  
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
      
      // 回転数を display_d1 に送信
      rotationCountToSend = airDataBuf[0];
      if (master.writeRegistersSync(SLAVE_ID_DISPLAY_3_1, DISP_D1_REG_WRITE, &rotationCountToSend, 1)) {
        Serial.printf("[LOGGER] Sent rotation count to Display D-1: %d\n", rotationCountToSend);
      } else {
        Serial.println("[LOGGER] FAILED to send rotation count to Display D-1");
      }
      
      // SDカードに記録
      if (sdCardReady) {
        logAirData(airDataBuf);
      }
    } else {
      Serial.println("[LOGGER] FAILED to read Air Data");
    }
    
    // Display D-1から読む（D-2のセンサーデータはD-1経由で受け取られている）
    if (master.readRegistersSync(SLAVE_ID_DISPLAY_3_1, DISP_D1_REG_READ, displayD1Buf, DISP_D1_REG_READ_SIZE)) {
      Serial.printf("[LOGGER] Display D-1: Baro_Alt=%d\n", displayD1Buf[0]);
      
      // Display D-1内部には D-2 から受け取ったセンサーデータもある
      // D-1の displaySlave.getPotentiometer1() など経由でアクセス可能
      
      // SDカードに記録
      if (sdCardReady) {
        logDisplayData(displayD1Buf);
      }
    } else {
      Serial.println("[LOGGER] FAILED to read Display D-1");
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
  
  // 次に Display D-1 を切り替え
  Serial.printf("Changing Display D-1 baud to index %d...\n", baudIdx);
  master.requestBaudChange(SLAVE_ID_DISPLAY_3_1, baudIdx);
  delay(2000);
  
  Serial.printf("All slaves now at correct baud\n");
}

// ===================================================================
// SDカード関連関数
// ===================================================================

/**
 * SDカード初期化
 */
void initSDCard() {
  Serial.println("[LOGGER] Initializing SD card...");
  
  if (!SD.begin()) {
    Serial.println("[LOGGER] SD card mount failed");
    sdCardReady = false;
    return;
  }
  
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[LOGGER] No SD card attached");
    sdCardReady = false;
    return;
  }
  
  // カード情報を表示
  Serial.print("[LOGGER] SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("[LOGGER] SD Card Size: %lluMB\n", cardSize);
  Serial.printf("[LOGGER] Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  
  // ログファイルが存在しない場合はヘッダを書き込み
  if (!SD.exists(LOG_FILE)) {
    File file = SD.open(LOG_FILE, FILE_WRITE);
    if (file) {
      file.println("Timestamp,Rotation,AS5600_1,AS5600_2,Battery,Baro_Alt");
      file.close();
      Serial.printf("[LOGGER] Created log file: %s\n", LOG_FILE);
    }
  }
  
  sdCardReady = true;
  Serial.println("[LOGGER] SD card ready");
}

/**
 * エアデータをSDカードに記録
 */
void logAirData(uint16_t* buf) {
  if (!sdCardReady) return;
  
  char logEntry[128];
  unsigned long ts = millis();
  
  sprintf(logEntry, "%lu,%u,%u,%u,%u,",
          ts, buf[0], buf[1], buf[2], buf[3]);
  
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (file) {
    file.print(logEntry);
  } else {
    Serial.printf("[LOGGER] Failed to open log file for writing\n");
  }
  file.close();
}

/**
 * ディスプレイデータをSDカードに記録
 */
void logDisplayData(uint16_t* buf) {
  if (!sdCardReady) return;
  
  char logEntry[32];
  sprintf(logEntry, "%u\n", buf[0]);  // Baro_Alt
  
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (file) {
    file.print(logEntry);
  } else {
    Serial.printf("[LOGGER] Failed to append to log file\n");
  }
  file.close();
}
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
