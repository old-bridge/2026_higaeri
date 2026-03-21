# Modbus通信トラブルシューティング

## 現状の実装
- **マスター**: logger マイコン（XIAO ESP32C3）
- **スレーブ①**: air_data マイコン（Slave ID: 1）
- **スレーブ②**: display_d1 マイコン（Slave ID: 2）
- **通信方式**: RS485（ハードウェアシリアル Serial0）
- **ボーレート**: 複数対応（9600, 38400, 115200, 921600, 5000000 bps）

---

## ソフトウェア的な原因と対策

### 1. **ボーレート不一致**
**症状**: 全くデータが受け取れない、ガベージデータが出現

**原因**:
- logger（マスター）と各スレーブのボーレートが異なっている
- 初期化時のボーレート変更に失敗している
- EEPROM に残された古いボーレート設定

**対策**:
```cpp
// 全スレーブを同じボーレートに統一
// logger.ino: changeMasterBaud(1);  // 38400 bpsに変更
// （ボーレート配列: {9600, 38400, 115200, 921600, 5000000}）
```

---

### 2. **Modbus RTUチェックサム（CRC）エラー**
**症状**: 通信は試みているが、返答がない または 頻繁に失敗する

**原因**:
- RS485レベル変換（MAX485等）のピン接続不正
  - DI（ドライバーイン）: TX
  - RO（レシーバーアウト）: RX
  - DE（ドライバーイネーブル）: DE_PIN
  - /RE（レシーバーイネーブル）: GND
- RS485ノイズ（長いケーブル、不完全な終端抵抗）
- メモリ破損によるCRC計算エラー

**対策**:
- ロジックアナライザーで実際のRS485信号を確認
- 終端抵抗（120Ω）を確認
- 最後のノードに必ず終端抵抗を接続

---

### 3. **DE_PIN（RS485制御ピン）設定エラー**
**症状**: データは送信されるが、返答が受信できない

**原因**:
- DE_PIN（ドライバーイネーブル）のピン番号が間違っている
- DE_PIN をHIGHに設定している時間が短い（返答受信前にLOWに戻る）
- マスター側 vs スレーブ側で異なるピンデジニッションを使用

**対策**:
```cpp
// logger.ino の場合：
#define DE_PIN D2  // RS485制御ピン

// ModbusMaster 内部では自動的に：
// - 送信前：digitalWrite(DE_PIN, HIGH)
// - 受信前：digitalWrite(DE_PIN, LOW)
```

---

### 4. **Slave ID 不一致**
**症状**: 特定スレーブだけ通信できない

**原因**:
- ModbusConfig.h で定義した Slave ID と
- 実際のスレーブが期待する Slave ID が異なる
- 複数スレーブを同じID で起動している

**対策**:
```cpp
// ModbusConfig.h
#define SLAVE_ID_AIR_DATA      1  // air_data.ino で使用
#define SLAVE_ID_DISPLAY_3_1   2  // display_d1.ino で使用
```

---

### 5. **レジスタアドレス・サイズ定義エラー**
**症状**: データは受信するが、値が完全におかしい（ゴミ値）

**原因**:
- AIR_REG_READ, DISP_D1_REG_READ のアドレス定義エラー
- AIR_REG_READ_SIZE, DISP_D1_REG_READ_SIZE と
  実際のレジスタバッファサイズが不一致
- ModbusConfig.h を編集したが、すべてのファイルを再コンパイルしていない

**対策**:
```cpp
// ModbusConfig.h
#define DISP_D1_REG_READ       0    // アドレス 0～2
#define DISP_D1_REG_READ_SIZE  3    // 3 レジスタ

// logger.ino
uint16_t displayD1Buf[DISP_D1_REG_READ_SIZE];  // サイズが一致
master.readRegistersSync(..., DISP_D1_REG_READ_SIZE);  // 一致
```

---

### 6. **タイムアウト設定過短**
**症状**: 時々通信成功するが、頻繁に失敗する

**原因**:
- ModbusConfig.h の MODBUS_TIMEOUT が短すぎる
- スレーブの処理が遅い、RS485遅延が大きい
- 複数スレーブがある場合、順序待ちが長い

**対策**:
```cpp
// ModbusConfig.h
const unsigned long MODBUS_TIMEOUT = 1000;  // 1秒に設定

// 必要に応じて増加：
// const unsigned long MODBUS_TIMEOUT = 2000;  // 2秒
```

---

### 7. **ModbusSlaveBase の初期化不完全**
**症状**: スレーブが受信しているはずだが、書き込みが反映されない

**原因**:
- `displaySlave->begin()` が呼ばれていない
- `setRegisterValue()` を呼び出す前に レジスタが初期化されていない
- ModbusRTUServer のバッファが初期化されていない

**対策**:
```cpp
// display_d1.ino
void setup() {
  // ...
  displaySlave = new DisplayD1Slave(&Serial0, SLAVE_ID_DISPLAY_3_1, DE_PIN);
  displaySlave->begin();  // 必須！
  // ...
}

void loop() {
  displaySlave->task();  // 常に呼び出す
  // ...
}
```

---

### 8. **Serial0 ハードウェアシリアル初期化エラー**
**症状**: 全く動作しない、ハング状態

**原因**:
- Serial0 が既に別の目的に使用されている
- XIAO ESP32C3 の場合、Serial0 がUSB/UARTと共有
- ボーレート設定漏れ

**対策**:
```cpp
// ModbusMaster / ModbusSlave のコンストラクタ
// ModbusMaster master(&Serial0, DE_PIN);  // 必ず Serial0 を使用

// デバッグ用の Serial は Serial.begin(115200) で OK
```

---

### 9. **master.task() の呼び出し漏れ**
**症状**: マスター側はRequest/Responseを送っているが、返答待ちのまま

**原因**:
- `loop()` 内で `master.task()` を呼び出していない
- Modbus RTU ライブラリの内部状態機械が更新されない
- タイムアウト後も復帰しない

**対策**:
```cpp
void loop() {
  master.task();  // 毎回呼び出す（これが無いと通信ハング）
  
  if (millis() - lastReadTime > READ_INTERVAL) {
    // センサー読み込み処理
  }
}
```

---

### 10. **バッファサイズ不一致**
**症状**: 特定のレジスタ値だけ化ける

**原因**:
- readRegistersSync() の第4引数（読み込みサイズ）が
  実際のバッファサイズと異なる
- logger 側: 3 uint16_t を要求
- スレーブ側: 12 uint8_t を送信
- メモリスオーバーフロー

**対策**:
```cpp
// logger.ino
uint16_t displayD1Buf[DISP_D1_REG_READ_SIZE];  // uint16_t × 3

master.readRegistersSync(
  SLAVE_ID_DISPLAY_3_1,
  DISP_D1_REG_READ,
  displayD1Buf,
  DISP_D1_REG_READ_SIZE  // 3を指定
);
```

---

## デバッグ手順

### ステップ1: シリアルモニタの確認
```
[MODBUS_STATUS]
  Air Data (ID:1): CONNECTED | OK:45 Err:0 | Last: 1230ms ago
  Display D-1 (ID:2): DISCONNECTED | OK:12 Err:33 | Last: 65000ms ago
  Overall: ✗ SOME SLAVES DISCONNECTED
```

### ステップ2: 各スレーブのボーレート確認
- air_data のシリアルモニタで初期化メッセージが出ているか
- display_d1 のシリアルモニタで初期化メッセージが出ているか
- ボーレート設定は一致しているか

### ステップ3: RS485 レベル変換確認
- ロジックアナライザーで UART 信号を確認
  - TX (logger) → DI (MAX485)
  - RX (logger) ← RO (MAX485)
  - DE (logger) → DE (MAX485)
- CRC エラー: RS485バス上のノイズを確認

### ステップ4: 物理的な接続確認
- RS485 A・B ラインが短絡していないか
- 全ノード間で適切に親機-子機で接続されているか
- 終端抵抗（120Ω）が正確に接続されているか

---

## 推奨設定

| 項目 | 推奨値 | 理由 |
|------|--------|------|
| ボーレート | 115200 bps | バランスの取れた速度、確実性 |
| Modbus Timeout | 1000 ms | 十分な余裕 |
| タスク呼び出し間隔 | 毎ループ | 時間遅延防止 |
| DE_PIN 制御 | 自動（ライブラリ） | 信頼性向上 |
| 終端抵抗 | 120Ω（最後のノード） | RS485 High-Z への対応 |

