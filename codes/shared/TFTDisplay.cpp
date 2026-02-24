#include "TFTDisplay.h"

/**
 * DisplayManager::begin()
 * TFTの初期化
 */
void DisplayManager::begin() {
  tft.init();
  tft.setRotation(1);  // 横向き
  tft.fillScreen(backgroundColor);
  Serial.println("[DISPLAY] TFT initialized");
}

/**
 * DisplayManager::clear()
 * 画面をクリア
 */
void DisplayManager::clear() {
  tft.fillScreen(backgroundColor);
}

/**
 * DisplayManager::drawText()
 * テキストを描画（汎用）
 */
void DisplayManager::drawText(int x, int y, const char* text, int size, uint16_t color) {
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.println(text);
}

/**
 * DisplayManager::drawValue()
 * 数値を描画（汎用）
 */
void DisplayManager::drawValue(int x, int y, int value, int size, uint16_t color) {
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.println(value);
}

/**
 * DisplayManager::drawRect()
 * 矩形を描画（背景クリア用）
 */
void DisplayManager::drawRect(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
}

/**
 * DisplayManager::drawVerticalLine()
 * 縦線を描画（区切り線など）
 */
void DisplayManager::drawVerticalLine(int x, int y, int length, uint16_t color) {
  tft.drawFastVLine(x, y, length, color);
}

/**
 * DisplayManager::drawHorizontalLine()
 * 横線を描画（区切り線など）
 */
void DisplayManager::drawHorizontalLine(int x, int y, int length, uint16_t color) {
  tft.drawFastHLine(x, y, length, color);
}

/**
 * DisplayManager::drawCircle()
 * 円を描画
 */
void DisplayManager::drawCircle(int x, int y, int r, uint16_t color) {
  tft.drawCircle(x, y, r, color);
}

/**
 * DisplayManager::setBackgroundColor()
 * 背景色を設定
 */
void DisplayManager::setBackgroundColor(uint16_t color) {
  backgroundColor = color;
}

/**
 * DisplayManager::setTextColor()
 * テキスト色を設定
 */
void DisplayManager::setTextColor(uint16_t color) {
  textColor = color;
}

/**
 * DisplayManager::setUpdateColor()
 * 更新時の強調色を設定
 */
void DisplayManager::setUpdateColor(uint16_t color) {
  updateColor = color;
}

/**
 * DisplayManager::getTFT()
 * TFT_eSPIオブジェクトに直接アクセス
 */
TFT_eSPI* DisplayManager::getTFT() {
  return &tft;
}

/**
 * DisplayManager::getWidth()
 * 画面の幅を取得
 */
int DisplayManager::getWidth() const {
  return tft.width();
}

/**
 * DisplayManager::getHeight()
 * 画面の高さを取得
 */
int DisplayManager::getHeight() const {
  return tft.height();
}

/**
 * DisplayManager::drawString()
 * TFT_eSPIの drawString メソッドをラッパー
 */
void DisplayManager::drawString(const char* text, int x, int y, int font) {
  tft.drawString(text, x, y, font);
}
