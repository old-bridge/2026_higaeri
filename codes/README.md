# Modbus RTU マルチマイコン通信システム

このプロジェクトは、3つのマイコンが Modbus RTU で連携する飛行制御システムです。

## 📋 システム構成

### マイコン3つ

| マイコン | 役割 | Modbus Role | Slave ID | DE Pin |
|---------|------|-------------|----------|--------|
| **エアデータ** | センサー値提供 | Slave | 1 | D10 |
| **ロガー** | データ記録管理 | **Master** | - | D2 |
| **表示** | GUI表示 | Slave | 2 | D3 |

### ハードウェア

- **マイコンボード**: Seeduino XIAO ESP32C3 × 3
- **通信**: RS485 (Modbus RTU / 9600～5000000 bps)
- **表示**: TFT_eSPI ライブラリ対応
- **I2C**: 各種センサー（AS5600、気圧センサ、IMU等）
- **ADC**: ポテンショメータ、バッテリー電圧監視

---

## 📦 フォルダ構成

```
codes/
├── shared/                        ← 全マイコン共用モジュール
│   ├── ModbusConfig.h            ← レジスタ定義、Slave ID、ボーレート定数
│   ├── ModbusSlave.h             ← Slave基本クラス
│   ├── ModbusMaster.h            ← Master基本クラス
│   └── TFTDisplay.h              ← TFT表示管理クラス
│
├── air_data/                      ← エアデータマイコン
│   └── air_data.ino
│
├── logger/                        ← ロガーマイコン（Master）
│   └── logger.ino
│
└── display/                       ← 表示マイコン
    ├── display.ino
    └── User_Setup.h               ← TFT_eSPI設定（ユーザー提供）
```

---

## 🔧 Modbusレジスタマップ

### エアデータマイコン（Slave ID: 1）

#### 読み込み用レジスタ（アドレス 0～3）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 0 | 回転数 | ロータリーエンコーダ値 |
| 1 | AS5600①角度 | I2Cセンサ |
| 2 | AS5600②角度 | I2Cセンサ |
| 3 | バッテリー電圧 | ADC値 |

#### 書き込み用レジスタ（アドレス 10）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 10 | LED制御 | 1=点灯、0=消灯 |

#### 制御用レジスタ（アドレス 300）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 300 | ボーレート制御 | 0～4（ボーレートインデックス） |

---

### 表示マイコン（Slave ID: 2）

#### 読み込み用レジスタ（アドレス 0～4）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 0 | ポテンショメータ①角度 | ADC値 |
| 1 | ポテンショメータ②角度 | ADC値 |
| 2 | バッテリー電圧 | ADC値 |
| 3 | 超音波センサ高度 | Soft UART値 |
| 4 | 大気圧センサ高度 | I2C値 |

#### 書き込み用レジスタ（アドレス 10）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 10 | LED制御 | 1=点灯、0=消灯 |

#### 制御用レジスタ（アドレス 300）
| アドレス | 内容 | 説明 |
|---------|------|------|
| 300 | ボーレート制御 | 0～4（ボーレートインデックス） |

---

## ⚙️ ボーレート一覧

| インデックス | ボーレート |
|------------|-----------|
| 0 | 9600 |
| 1 | 38400 |
| 2 | 115200 |
| 3 | 921600 |
| 4 | 5000000 |

---

## 🚀 セットアップ手順

### 1. ライブラリのインストール

Arduino IDE で以下をインストール：
- `ModbusRTU` (مكتبة Modbus)
- `TFT_eSPI` (TFT ディスプレイ表示用)

### 2. 各マイコンへのアップロード

```bash
# エアデータ
# → air_data/air_data.ino をアップロード

# ロガー
# → logger/logger.ino をアップロード

# 表示
# → display/display.ino をアップロード
```

### 3. 通信テスト

ロガーマイコンのシリアルモニタで各スレーブからの読み込みテストを実行。

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
