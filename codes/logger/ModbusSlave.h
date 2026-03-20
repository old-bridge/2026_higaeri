#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "ModbusRTU_wrapper.h"
#include "ModbusConfig.h"

/**
 * ModbusSlaveBase
 * ModbusRTUServer を使用したスレーブマイコン用の基本クラス
 * 各スレーブはこのクラスを継承して具体的な処理を実装
 * 
 * 注意: ModbusRTUServer は公開コンストラクタを持つため、
 * メンバ変数として直接宣言できます
 */
class ModbusSlaveBase {
protected:
  ModbusRTUServer mb;
  HardwareSerial* serial;
  uint8_t slaveId;
  uint8_t dePin;
  int currentBaudIdx;

  // 各スレーブが実装すべき仮想メソッド
  virtual void setupRegisters() = 0;
  virtual void setupCallbacks() = 0;

public:
  ModbusSlaveBase(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : serial(hwSerial), slaveId(id), dePin(de), currentBaudIdx(0) {}

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
   * ボーレート切り替え
   * REG_BAUD_CTRLへの書き込みコールバックから呼ばれる
   */
  void changeBaud(int baudIdx);

  /**
   * 現在のボーレート取得
   */
  long getCurrentBaud() const;

  /**
   * 現在のボーレートインデックス取得
   */
  int getCurrentBaudIdx() const;

  /**
   * レジスタに値を書き込む
   */
  void setRegisterValue(uint16_t address, uint16_t value);

  /**
   * レジスタの値を読み込む
   */
  uint16_t getRegisterValue(uint16_t address) const;
};

#endif  // MODBUS_SLAVE_H
