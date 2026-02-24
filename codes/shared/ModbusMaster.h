#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <ModbusRTU.h>
#include "ModbusConfig.h"

/**
 * ModbusMaster - Modbus RTUマスター
 * ロガーマイコン用
 */
class ModbusMaster {
private:
  ModbusRTU mb;
  HardwareSerial* serial;
  uint8_t dePin;
  int currentBaudIdx;

  unsigned long lastActionTime;
  const unsigned long ACTION_INTERVAL = 5000;  // 5秒待機

  /**
   * ボーレート変更（内部用）
   */
  void changeBaud(int baudIdx);

public:
  ModbusMaster(HardwareSerial* hwSerial, uint8_t de)
    : serial(hwSerial), dePin(de), currentBaudIdx(0), lastActionTime(0) {}

  /**
   * 初期化
   * setup()内から呼び出すこと
   */
  void begin();

  /**
   * Modbusタスク実行
   * loop()内から毎フレーム呼び出すこと
   */
  void task();

  /**
   * ボーレート切り替えをリクエスト
   * スレーブ側に切り替え指令を送信し、マスター側も切り替える
   * requestSlaveId: 切り替え対象スレーブのID
   * newBaudIdx: 新しいボーレートインデックス
   */
  void requestBaudChange(uint8_t requestSlaveId, int newBaudIdx);

  /**
   * スレーブから複数レジスタを読み込む（同期的に待機）
   * slaveId: スレーブのID
   * startAddr: 開始アドレス
   * buffer: データを格納するバッファ
   * count: 読み込むレジスタ数
   * 戻り値: 成功時true、タイムアウト時false
   */
  bool readRegistersSync(uint8_t slaveId, uint16_t startAddr, uint16_t* buffer, int count);

  /**
   * スレーブに複数レジスタを書き込む（同期的に待機）
   * slaveId: スレーブのID
   * startAddr: 開始アドレス
   * buffer: 書き込むデータ
   * count: 書き込むレジスタ数
   * 戻り値: 成功時true、タイムアウト時false
   */
  bool writeRegistersSync(uint8_t slaveId, uint16_t startAddr, const uint16_t* buffer, int count);

  /**
   * 単一レジスタを読み込む（便利メソッド）
   */
  bool readRegister(uint8_t slaveId, uint16_t addr, uint16_t& value);

  /**
   * 単一レジスタに書き込む（便利メソッド）
   */
  bool writeRegister(uint8_t slaveId, uint16_t addr, uint16_t value);

  /**
   * 現在のボーレート取得
   */
  long getCurrentBaud() const;

  /**
   * 現在のボーレートインデックス取得
   */
  int getCurrentBaudIdx() const;
};

#endif  // MODBUS_MASTER_H
