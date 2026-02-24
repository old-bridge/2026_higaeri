#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <TFT_eSPI.h>

/**
 * DisplayManager
 * TFT_eSPIを使用した表示管理クラス
 * 柔軟性を重視し、子クラスでカスタマイズ可能
 */
class DisplayManager {
protected:
  TFT_eSPI tft;
  
  // 表示領域管理用
  uint16_t backgroundColor;
  uint16_t textColor;
  uint16_t updateColor;

public:
  /**
   * コンストラクタ
   */
  DisplayManager() 
    : backgroundColor(TFT_BLACK), 
      textColor(TFT_WHITE), 
      updateColor(TFT_GREEN) {}

  /**
   * TFTの初期化
   * setup()内から呼び出すこと
   */
  void begin() {
    tft.init();
    tft.setRotation(1);  // 横向き
    tft.fillScreen(backgroundColor);
    Serial.println("[DISPLAY] TFT initialized");
  }

  /**
   * 画面をクリア
   */
  void clear() {
    tft.fillScreen(backgroundColor);
  }

  /**
   * テキストを描画（汎用）
   * x, y: 左上座標
   * text: 表示するテキスト
   * size: テキストサイズ（1-8）
   * color: テキスト色
   */
  void drawText(int x, int y, const char* text, int size = 2, uint16_t color = TFT_WHITE) {
    tft.setTextColor(color);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    tft.println(text);
  }

  /**
   * 数値を描画（汎用）
   * x, y: 左上座標
   * value: 表示する数値
   * size: テキストサイズ
   * color: テキスト色
   */
  void drawValue(int x, int y, int value, int size = 2, uint16_t color = TFT_WHITE) {
    tft.setTextColor(color);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    tft.println(value);
  }

  /**
   * 矩形を描画（背景クリア用）
   * x, y: 左上座標
   * w, h: 幅、高さ
   * color: 塗りつぶし色
   */
  void drawRect(int x, int y, int w, int h, uint16_t color) {
    tft.fillRect(x, y, w, h, color);
  }

  /**
   * 縦線を描画（区切り線など）
   */
  void drawVerticalLine(int x, int y, int length, uint16_t color) {
    tft.drawFastVLine(x, y, length, color);
  }

  /**
   * 横線を描画（区切り線など）
   */
  void drawHorizontalLine(int x, int y, int length, uint16_t color) {
    tft.drawFastHLine(x, y, length, color);
  }

  /**
   * 円を描画
   */
  void drawCircle(int x, int y, int r, uint16_t color) {
    tft.drawCircle(x, y, r, color);
  }

  /**
   * 背景色を設定
   */
  void setBackgroundColor(uint16_t color) {
    backgroundColor = color;
  }

  /**
   * テキスト色を設定
   */
  void setTextColor(uint16_t color) {
    textColor = color;
  }

  /**
   * 更新時の強調色を設定
   */
  void setUpdateColor(uint16_t color) {
    updateColor = color;
  }

  /**
   * TFT_eSPIオブジェクトに直接アクセス
   * より細かい制御が必要な場合用
   */
  TFT_eSPI* getTFT() {
    return &tft;
  }

  /**
   * 画面の幅を取得
   */
  int getWidth() const {
    return tft.width();
  }

  /**
   * 画面の高さを取得
   */
  int getHeight() const {
    return tft.height();
  }
};

#endif  // TFT_DISPLAY_H
