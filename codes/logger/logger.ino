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
#define LED_PIN D1  // LED制御ピン

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "ModbusConfig.h"
#include "ModbusMaster.h"
#include "ModbusSlave.h"
// Note: .cpp files are compiled separately by Arduino IDE

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

// LED制御用
#define LED_BLINK_SUCCESS 1  // 点滅（成功）
#define LED_ON_ERROR 2       // 点灯（エラー）
uint16_t ledControlValue = 0;  // D1に送信するLED制御値
unsigned long ledLastToggleTime = 0;
const unsigned long LED_BLINK_INTERVAL = 500;  // 500ms間隔で点滅
static bool ledBlinkState = false;  // LED点滅の状態

// SD カード関連
const char* LOG_FILE = "/flight_log.csv";  // ログファイル名
bool sdCardReady = false;                   // SD カード初期化状態

// 書き込みステータス
struct WriteStatus {
  bool lastWriteSuccess = false;
  unsigned long lastWriteTime = 0;
  uint32_t successCount = 0;
  uint32_t failureCount = 0;
  char statusMessage[32] = "Initializing";
} writeStatus;

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(100);  // 最小限の待機（デバッグ用）
  Serial.println("\n--- Logger Start ---");

  // LEDピン初期化
  Serial.println("[LOGGER] Initializing LED pin...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // 初期状態はOFF
  Serial.println("[LOGGER] LED pin initialized");

  // ★重要: XIAO ESP32C3のハードウェアSPIピンを明示的に割り当てる
  Serial.println("[LOGGER] Initializing SPI...");
  SPI.begin(8, 9, 10, 5);  // SCK=8, MISO=9, MOSI=10, CS=5
  Serial.println("[LOGGER] SPI initialized");

  // SD カード初期化
  initSDCard();
  
  // Modbus マスタ初期化
  master.begin();
  
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
    bool airDataReadSuccess = false;
    if (master.readRegistersSync(SLAVE_ID_AIR_DATA, AIR_REG_READ, airDataBuf, AIR_REG_READ_SIZE)) {
      Serial.printf("[LOGGER] Air Data: Rotation=%d, AS5600_1=%d, AS5600_2=%d, Batt=%d\n",
                    airDataBuf[0], airDataBuf[1], airDataBuf[2], airDataBuf[3]);
      airDataReadSuccess = true;
      
      // 回転数を display_d1 に送信
      rotationCountToSend = airDataBuf[0];
      if (master.writeRegistersSync(SLAVE_ID_DISPLAY_3_1, DISP_D1_REG_WRITE, &rotationCountToSend, 1)) {
        Serial.printf("[LOGGER] Sent rotation count to Display D-1: %d\n", rotationCountToSend);
      } else {
        Serial.println("[LOGGER] FAILED to send rotation count to Display D-1");
      }
    } else {
      Serial.println("[LOGGER] FAILED to read Air Data - will log 0");
      airDataReadSuccess = false;
      // 失敗時はバッファを0に初期化
      for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
        airDataBuf[i] = 0;
      }
    }
    
    // Display D-1から読む（D-2のセンサーデータはD-1経由で受け取られている）
    bool displayD1ReadSuccess = false;
    if (master.readRegistersSync(SLAVE_ID_DISPLAY_3_1, DISP_D1_REG_READ, displayD1Buf, DISP_D1_REG_READ_SIZE)) {
      Serial.printf("[LOGGER] Display D-1: Baro_Alt=%d\n", displayD1Buf[0]);
      displayD1ReadSuccess = true;
    } else {
      Serial.println("[LOGGER] FAILED to read Display D-1 - will log 0");
      displayD1ReadSuccess = false;
      // 失敗時はバッファを0に初期化
      for (int i = 0; i < DISP_D1_REG_READ_SIZE; i++) {
        displayD1Buf[i] = 0;
      }
    }
    
    // SDカードに統合ログレコードを記録（成功失敗に関わらず常に記録）
    if (sdCardReady) {
      logFlightData(airDataBuf, displayD1Buf);
    }
    
    // 書き込み結果に基づいてLED制御値を設定
    if (writeStatus.lastWriteSuccess) {
      ledControlValue = LED_BLINK_SUCCESS;  // 成功時は点滅
      Serial.println("[LOGGER] LED control: BLINK (Write Success)");
    } else {
      ledControlValue = LED_ON_ERROR;       // 失敗時は点灯
      Serial.println("[LOGGER] LED control: ON (Write Error)");
    }
  }

  // ロガー自身のLED制御
  updateLoggerLED();

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
  
  // ★重要: SDライブラリの初期化時に、割り当てたCSピン(5)を渡す
  if(!SD.begin(5)){
    Serial.println("[LOGGER] Card Mount Failed");
    sdCardReady = false;
    return;
  }
  
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("[LOGGER] No SD card attached");
    sdCardReady = false;
    return;
  }

  Serial.print("[LOGGER] SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
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
 * フライトデータを統合してSDカードに記録
 * センサー読み込み失敗時も0を記録
 */
void logFlightData(uint16_t* airBuf, uint16_t* displayBuf) {
  if (!sdCardReady) {
    writeStatus.lastWriteSuccess = false;
    writeStatus.failureCount++;
    snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "SD Not Ready");
    return;
  }
  
  char logEntry[128];
  unsigned long ts = millis();
  
  // CSV形式: Timestamp,Rotation,AS5600_1,AS5600_2,Battery,Baro_Alt
  sprintf(logEntry, "%lu,%u,%u,%u,%u,%u\n",
          ts, 
          airBuf[0],      // Rotation
          airBuf[1],      // AS5600_1
          airBuf[2],      // AS5600_2
          airBuf[3],      // Battery
          displayBuf[0]   // Baro_Alt
         );
  
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (file) {
    if (file.print(logEntry)) {
      writeStatus.lastWriteSuccess = true;
      writeStatus.successCount++;
      snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Log: OK");
      Serial.printf("[LOGGER] Flight log written OK: %s", logEntry);
    } else {
      writeStatus.lastWriteSuccess = false;
      writeStatus.failureCount++;
      snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Log: Write Err");
      Serial.println("[LOGGER] Flight log write FAILED");
    }
    file.close();
  } else {
    writeStatus.lastWriteSuccess = false;
    writeStatus.failureCount++;
    snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Log: Open Err");
    Serial.println("[LOGGER] Failed to open log file");
  }
  
  writeStatus.lastWriteTime = millis();
}

/**
 * エアデータをSDカードに記録（非推奨：logFlightData()を使用）
 */
void logAirData(uint16_t* buf) {
  if (!sdCardReady) {
    writeStatus.lastWriteSuccess = false;
    writeStatus.failureCount++;
    snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "SD Not Ready");
    return;
  }
  
  char logEntry[128];
  unsigned long ts = millis();
  
  sprintf(logEntry, "%lu,%u,%u,%u,%u,",
          ts, buf[0], buf[1], buf[2], buf[3]);
  
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (file) {
    if (file.print(logEntry)) {
      writeStatus.lastWriteSuccess = true;
      writeStatus.successCount++;
      snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Air: OK");
      Serial.println("[LOGGER] Air data written OK");
    } else {
      writeStatus.lastWriteSuccess = false;
      writeStatus.failureCount++;
      snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Air: Write Err");
      Serial.println("[LOGGER] Air data write FAILED");
    }
    file.close();
  } else {
    writeStatus.lastWriteSuccess = false;
    writeStatus.failureCount++;
    snprintf(writeStatus.statusMessage, sizeof(writeStatus.statusMessage), "Air: Open Err");
    Serial.println("[LOGGER] Failed to open log file");
  }
  
  writeStatus.lastWriteTime = millis();
}

/**
 * ロガーのLED制御（GPIO上のLED）
 * SD書き込み成功時は点滅、失敗時は常時点灯
 */
void updateLoggerLED() {
  if (ledControlValue == LED_BLINK_SUCCESS) {
    // 成功時：LED点滅（500ms間隔）
    unsigned long currentTime = millis();
    if (currentTime - ledLastToggleTime >= LED_BLINK_INTERVAL) {
      ledLastToggleTime = currentTime;
      ledBlinkState = !ledBlinkState;  // 状態切り替え
      digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW);
      Serial.printf("[LOGGER] LED BLINK: %s\n", ledBlinkState ? "ON" : "OFF");
    }
  } else if (ledControlValue == LED_ON_ERROR) {
    // エラー時：LED常時点灯
    digitalWrite(LED_PIN, HIGH);
  } else {
    // 初期状態：LED OFF
    digitalWrite(LED_PIN, LOW);
  }
}
