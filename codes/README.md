# Modbus RTU マルチマイコン通信システム

このプロジェクトは、4つのマイコンがModbus RTUとI2Cで連携する飛行データ収集システムです。

## 📋 システム構成

### マイコン4つ

| マイコン | 役割 | 通信役割 | Slave ID | DE Pin |
|---------|------|----------|----------|--------|
| **air_data** | 回転数と機体側センサー値の提供 | Modbus Slave | 1 | D10 |
| **logger** | Modbus 集約と SD 記録 | Modbus Master | - | D2 |
| **display_d1** | TFT 表示と display_d2 取り込み | Modbus Slave / I2C Master | 2 | D3 |
| **display_d2** | 表示用センサー取得 | I2C Slave | - | - |

### ハードウェア

- **マイコンボード**: Seeduino XIAO ESP32C3 × 4
- **通信**: RS485（Modbus RTU/9600bps固定）
- **表示**: TFT_eSPIライブラリ対応
- **I2C**: display_d2からdisplay_d1へ12byteのセンサペイロードを転送
- **ADC**: ポテンショメータ、バッテリー電圧監視

---

## 📦 フォルダー構成

```
codes/
├── shared/
│   ├── ModbusConfig.h
│   ├── ModbusSlave.h
│   └── ModbusMaster.h
├── air_data/
│   ├── air_data.ino              ← センサー読み取りと Modbus 反映
│   └── ModbusSlave.h/.cpp
├── logger/
│   ├── logger.ino                ← ログ処理と Modbus ポーリング
│   └── ModbusMaster.h/.cpp
├── display_d1/
│   ├── display_d1.ino            ← I2C 読み取りと TFT 表示
│   └── ModbusSlave.h/.cpp
└── display_d2/
   ├── display_d2.ino            ← センサー読み取りと I2C 応答
   └── DisplayD2Config.h
```

---

## 🔧 Modbusレジスタマップ

### エアデータマイコン（Slave ID: 1）

#### 読み込み用レジスタ（アドレス 0～3）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 0 | 回転数 | ロータリーエンコーダー値 |
| 0 | 対気速度 | 0.5秒間のパルス数を一次関数で変換した値 |
| 1 | AS5600①角度 | I2Cセンサー |
| 2 | AS5600②角度 | I2Cセンサー |
| 3 | バッテリー電圧 | ADC値 |

#### 書き込み用レジスタ（アドレス 10）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 10 | コマンド | 1=LED点灯、0=LED消灯、書き込み後は 0 に戻す |

---

### 表示マイコン（Slave ID: 2）

#### 読み込み用レジスタ（アドレス 0～4）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 0 | 気圧高度 | display_d2 から取得した値 |
| 1 | ポテンショメータ① | ADC値 |
| 2 | ポテンショメータ② | ADC値 |
| 3 | バッテリー電圧 | ADC値 |
| 4 | 超音波高度 | 現状はプレースホルダー0 |

#### 書き込み用レジスタ（アドレス 10）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 10 | 回転数 | logger から転送される回転数 |
| 10 | 対気速度 | logger から転送される対気速度 |

---

## 🚀 セットアップ手順

### 1. ライブラリのインストール

Arduino IDEで以下をインストール:
- `ModbusRTU` (مكتبة Modbus)
- `TFT_eSPI`（TFTディスプレイ表示用）

### 2. 各マイコンへのアップロード

```bash
# エアデータ
# → air_data/air_data.ino をアップロード

# ロガー
# → logger/logger.ino をアップロード

# 表示1
# → display_d1/display_d1.ino をアップロード

# 表示2
# → display_d2/display_d2.ino をアップロード
```

### 3. 通信テスト

ロガーマイコンのシリアルモニターで各スレーブからの読み込みテストを実行。

---

## 📝 使用例

### エアデータマイコンでセンサー値を提供

```cpp
// air_data.ino の loop() 内
void loop() {
  airDataSlave.updateSensorValues();  // センサー値をレジスタに格納
  airDataSlave.task();                 // Modbusタスク実行
}
```

### ロガーがデータを読み込み

```cpp
// logger.ino の loop() 内
uint16_t airDataBuf[AIR_REG_READ_SIZE];
master.readRegistersSync(SLAVE_ID_AIR_DATA, AIR_REG_READ, airDataBuf, AIR_REG_READ_SIZE);

// airDataBuf[0] = 回転数
// airDataBuf[1] = AS5600①
// airDataBuf[2] = AS5600②
// airDataBuf[3] = バッテリー電圧
```

### ロガーがボーレートを切り替え

```cpp
// logger.ino の loop() 内
master.requestBaudChange(SLAVE_ID_AIR_DATA, 2);  // 115200 bps に切り替え
```

---

## 🛠️ カスタマイズのコツ

### レジスタマップの変更

`shared/ModbusConfig.h` で定数を編集するだけで、全マイコンに反映：

```cpp
#define AIR_REG_READ 0
#define AIR_REG_READ_SIZE 4
```

### 新しいセンサーを追加

1. `ModbusConfig.h` にレジスタを追加
2. 各スレーブの `setupRegisters()` で登録
3. 各スレーブの `updateSensorValues()` で値を更新

### TFT表示をカスタマイズ

`display.ino` の `updateDisplay()` 関数を編集：

```cpp
void updateDisplay() {
  display.clear();
  display.drawText(x, y, "Custom Text", size, color);
  // ... 他の描画コマンド
}
```

---

## 📚 API リファレンス

### ModbusMaster クラス

```cpp
// 初期化
master.begin();

// タスク実行（毎フレーム）
master.task();

// 単一レジスタ読み込み
uint16_t value;
master.readRegister(SLAVE_ID, address, value);

// 複数レジスタ読み込み
uint16_t buffer[10];
master.readRegistersSync(SLAVE_ID, address, buffer, 10);

// 単一レジスタ書き込み
master.writeRegister(SLAVE_ID, address, value);

// ボーレート切り替えリクエスト
master.requestBaudChange(SLAVE_ID, baudIndex);
```

### ModbusSlave基本クラス

```cpp
// 初期化（子クラスで実装）
class MySlave : public ModbusSlaveBase {
  void setupRegisters() override { ... }
  void setupCallbacks() override { ... }
};

// タスク実行（毎フレーム）
slave.task();

// ボーレート切り替え
slave.changeBaud(baudIndex);
```

### DisplayManager クラス

```cpp
// 初期化
display.begin();

// 画面クリア
display.clear();

// テキスト描画
display.drawText(x, y, "Text", size, color);

// 数値描画
display.drawValue(x, y, 123, size, color);

// 直線描画
display.drawHorizontalLine(x, y, length, color);
```

---

## ⚠️ トラブルシューティング

### Modbus通信が失敗する

1. **ボーレートが一致しているか確認**
   - 全マイコンで同じボーレートを使う
   - 動的切り替え後は確実に全マイコンが切り替わるまで待つ

2. **RS485配線を確認**
   - D+ (A ピン) と D- (B ピン) が正しく接続されているか
   - 終端抵抗（120Ω）がバス両端に接続されているか

3. **DE ピン設定を確認**
   - 各マイコンの DE ピン設定が正しいか
   - `#define DE_PIN D10` など各INOファイルの先頭で定義済みか

### TFT表示が映らない

1. **User_Setup.h が正しい場所にあるか**
   - `display/User_Setup.h` に TFT_eSPI 設定があるか

2. **SPI ピン配置を確認**
   - TFT_eSPI のピン設定がマイコンに合っているか

---

## 📞 サポート

質問や問題がある場合は、シリアルモニタの出力を確認してください。

各マイコンは `Serial.printf()` で詳細なログを出力しています。

---

**作成日**: 2026年2月16日  
**マイコン**: Seeduino XIAO ESP32C3  
**通信プロトコル**: Modbus RTU  
