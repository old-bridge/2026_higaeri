/*
 * TFT Display Test Demo - display_d1.ino
 * TFT表示動作テスト専用コード
 * 
 * 機能:
 * - 数値カウントアップのデモ表示
 * - テキストサイズの表示イメージ確認
 * - 色分けフォーマットテスト
 * 
 * 注: Modbus/I2C/その他通信は一切不使用
 * 純粋にTFT表示の動作確認用
 */

#include <SPI.h>
#include <TFT_eSPI.h>  // Hardware-specific library

#define TFT_GREY 0x5AEB

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

// Demo variables
uint32_t rotationCount = 0;
uint16_t potentiometer1 = 500;
uint16_t potentiometer2 = 1200;
uint16_t batteryVoltage = 3800;
uint16_t ultrasonicAlt = 1250;
uint16_t baroAlt = 1245;

uint32_t lastUpdateTime = 0;
const uint32_t UPDATE_INTERVAL = 500;  // Update every 500ms
uint8_t direction = 1;  // For cycling values

void setup() {
  // シリアル通信の初期化とモニタ起動待ちのウェイト
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- TFT Demo Start ---");

  Serial.println("Step 1: tft.init() 実行前");
  
  tft.init();  // Initialize TFT display
  
  Serial.println("Step 2: tft.init() 完了");

  tft.setRotation(0);
  tft.fillScreen(TFT_GREY);
  
  Serial.println("Step 3: 画面塗りつぶし完了");

  tft.setTextColor(TFT_WHITE, TFT_GREY);
  
  // Draw title
  tft.drawCentreString("Display D-1 Test", 120, 10, 2);
  tft.drawCentreString("Rotation Demo", 120, 30, 1);

  Serial.println("Step 4: 初期表示完了");
}

void loop() {
  // Update demo values
  if (millis() - lastUpdateTime > UPDATE_INTERVAL) {
    lastUpdateTime = millis();
    
    // Increment/decrement values for demo
    rotationCount += 5;
    if (rotationCount > 9999) rotationCount = 0;
    
    potentiometer1 += direction * 50;
    if (potentiometer1 > 4095) {
      potentiometer1 = 4095;
      direction = -1;
    } else if (potentiometer1 < 0) {
      potentiometer1 = 0;
      direction = 1;
    }
    
    potentiometer2 = 4095 - potentiometer1;
    batteryVoltage = 3600 + (potentiometer1 / 10);
    ultrasonicAlt = 800 + (potentiometer1 / 5);
    baroAlt = ultrasonicAlt + 5;
    
    updateDisplay();
    
    Serial.printf("Rotation: %d, Pot1: %d, Pot2: %d\n", 
                  rotationCount, potentiometer1, potentiometer2);
  }
}

void updateDisplay() {
  // Clear screen and fill with background color
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);

  // Title
  tft.drawCentreString("=== FLIGHT DATA ===", 120, 10, 2);

  // ========== Rotation (displayed largest) ==========
  char buf[64];
  
  // Rotation counter - Large display
  tft.setTextColor(TFT_YELLOW, TFT_GREY);
  sprintf(buf, "%d", rotationCount);
  tft.drawString(buf, 10, 45, 8);  // Massive size
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.drawString("Rotation (rpm)", 10, 115, 2);

  // ========== Potentiometers ==========
  tft.setTextColor(TFT_CYAN, TFT_GREY);
  sprintf(buf, "Pot1: %d", potentiometer1);
  tft.drawString(buf, 10, 145, 2);

  sprintf(buf, "Pot2: %d", potentiometer2);
  tft.drawString(buf, 10, 165, 2);

  // ========== Battery Voltage ==========
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  sprintf(buf, "Battery: %dmV", batteryVoltage);
  tft.drawString(buf, 10, 195, 2);

  // ========== Altitude information ==========
  tft.setTextColor(TFT_ORANGE, TFT_GREY);
  sprintf(buf, "US Alt: %dm", ultrasonicAlt);
  tft.drawString(buf, 10, 225, 2);

  sprintf(buf, "Baro Alt: %dm", baroAlt);
  tft.drawString(buf, 10, 245, 2);

  // ========== Status bar ==========
  tft.drawLine(0, 270, 240, 270, TFT_WHITE);  // Separator line
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  tft.drawString("[Demo Mode]", 10, 280, 1);
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.drawString("[Slave ID: 2]", 160, 280, 1);
}
