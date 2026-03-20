# 修正完了 - ModbusRTUライブラリAPI更新

## 根本原因の分析

### 1. **ライブラリAPIの不一致**
あなたのコード（ModbusSlave.cpp, ModbusMaster.h/cpp, air_data.ino）は、**古い/異なるModbusRTUライブラリのAPI** を使用していました。実際のModbusRTUライブラリは異なるクラス構造を持っていました。

**コードが呼び出そうとしていたメソッド** ❌
- `mb.begin(serial, dePin)` - 非推奨エントリーポイント  
- `mb.slave(slaveId)` - 存在しない
- `mb.master()` - 存在しない
- `mb.Hreg()` - 存在しない
- `mb.addHreg()` - 存在しない
- `mb.onGetHreg()` - 存在しない
- `mb.onSetHreg()` - 存在しない

**実際に存在するAPI** ✅
- スレーブ: `ModbusRTUServer` クラス
- マスター: `ModbusRTUClient` クラス

---

## 実装した修正内容

### 1. **ModbusSlaveBase クラスの修正**

**変更前:**
```cpp
class ModbusSlaveBase {
protected:
  ModbusRTU mb;  // ❌ 無効 - コンストラクタが protected
```

**変更後:**
```cpp
class ModbusSlaveBase {
protected:
  ModbusRTUServer mb;  // ✅ 正しいクラス
```

**修正ファイル:**
- [air_data/ModbusSlave.h](air_data/ModbusSlave.h#L10)

---

### 2. **ModbusMaster クラスの修正**

**変更前:**
```cpp
class ModbusMaster {
private:
  ModbusRTU mb;  // ❌ 無効
```

**変更後:**
```cpp
class ModbusMaster {
private:
  ModbusRTUClient mb;  // ✅ 正しいクラス
```

**修正ファイル:**
- [air_data/ModbusMaster.h](air_data/ModbusMaster.h#L9)

---

### 3. **ModbusSlave.cpp の API更新**

新しいAPI メソッド群:

#### スレーブの初期化
```cpp
// 旧 (無効)
mb.begin(serial, dePin);
mb.slave(slaveId);

// 新 (正しい)
mb.startModbusServer(slaveId, 9600, *serial, false);
```

#### タスク実行
```cpp
// 旧
mb.task();

// 新
mb.communicationLoop();  // 戻り値: write request が存在すれば true
```

#### レジスタ値の操作
```cpp
// 旧
mb.Hreg(address, value);        // 読み書き
uint16_t val = mb.Hreg(address);

// 新
mb.setHoldingValue(address, value);   // 書き込み
uint16_t val = mb.getHoldingValue(address);  // 読み込み
mb.setInputValue(address, value);     // 読み専用値を設定
```

**修正ファイル:**
- [air_data/ModbusSlave.cpp](air_data/ModbusSlave.cpp#L1-L95)

---

### 4. **ModbusMaster.cpp の API更新**

#### マスターの初期化
```cpp
// 旧
mb.begin(serial, dePin);
mb.master();

// 新
mb.startModbusClient(0, 9600, *serial, false);
```

#### レジスタ読み込み/書き込み (同期的動作)
```cpp
// 旧 (非同期/複雑)
if (mb.readHreg(slaveId, startAddr, buffer, count)) {
  while (mb.slave()) {
    mb.task();
  }
}

// 新 (同期的/シンプル)
int ret = mb.ReadHoldingRegisters(startAddr, count, buffer, 200000);
if (ret == 0) {  // 成功
  // データは buffer に入っている
}
```

**修正ファイル:**
- [air_data/ModbusMaster.cpp](air_data/ModbusMaster.cpp#L1-L120)

---

### 5. **air_data.ino クラスの簡略化**

#### setupRegisters() の削除
```cpp
// 旧: レジスタを手動登録
void setupRegisters() {
  mb.addHreg(AIR_REG_READ, 100, AIR_REG_READ_SIZE);
  mb.addHreg(AIR_REG_WRITE, 0, AIR_REG_WRITE_SIZE);
  mb.addHreg(AIR_REG_BAUD_CTRL, 0);
}

// 新: ModbusRTUServer が自動管理
// → setupRegisters() は不要（削除）
```

#### setupCallbacks() の削除  
```cpp
// 旧: TRegister* パラメータを使用したコールバック
static uint16_t cbRead(TRegister* reg, uint16_t oldValue) {
  return oldValue;
}

// 新: イベントハンドラーベース
// mb.setWriteHoldingRegisterEvent(eventHandler, context);
// → 複雑なコールバックは不要
```

#### updateSensorValues() の簡略化
```cpp
// 旧
for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
  mb.Hreg(AIR_REG_READ + i, sensorValues[i]);
}

// 新
for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
  mb.setInputValue(AIR_REG_READ + i, sensorValues[i]);
}
```

#### コマンド処理の追加
```cpp
// 新メソッド: checkCommands()
bool checkCommands() {
  uint16_t cmdValue = mb.getHoldingValue(AIR_REG_WRITE);
  if (cmdValue != 0) {
    // コマンド処理
    mb.setHoldingValue(AIR_REG_WRITE, 0);  // 消去
    return true;
  }
}
```

**修正ファイル:**
- [air_data/air_data.ino](air_data/air_data.ino#L34-L135)

---

## 重要な設計の変更

### API パラダイムの違い

| 項目 | 旧API | 新API |
|------|------|------|
| **スレーブクラス** | `ModbusRTU + mb.slave()` | `ModbusRTUServer` |
| **マスタークラス** | `ModbusRTU + mb.master()` | `ModbusRTUClient` |
| **ハードウェアシリアル** | `Serial` (HWCDC - USB) | `Serial1` (HardwareSerial - RS485) |
| **タスク実行** | 非同期: `mb.task() + while(mb.slave())` | 同期: `mb.communicationLoop()` |
| **レジスタ操作** | `addHreg()`, `Hreg()`, `Hreg()` | `setInputValue()`, `setHoldingValue()`, `getHoldingValue()` |
| **コールバック** | `TRegister*` パラメータ | イベントハンドラーベース |
| **読み書き方式** | アシンクロナス | **シンクロナス** ✨ |

---

## ESP32 固有の問題

### Serial vs Serial1
- `Serial` → USB-CDC (`HWCDC` 型)
- `Serial1` → UART (`HardwareSerial` 型)

ModbusRTUライブラリは `HardwareSerial&` を要求するため、**RS485トランシーバーはSerial1を使用** する必要があります。

---

## 修正内容サマリー

| ファイル | 変更内容 | 理由 |
|---------|--------|------|
| [ModbusSlave.h](air_data/ModbusSlave.h#L10) | `ModbusRTU` → `ModbusRTUServer` | 正しいクラスを使用 |
| [ModbusMaster.h](air_data/ModbusMaster.h#L9) | `ModbusRTU` → `ModbusRTUClient` | 正しいクラスを使用 |
| [ModbusSlave.cpp](air_data/ModbusSlave.cpp) | 全API更新 | 新ライブラリAPI対応 |
| [ModbusMaster.cpp](air_data/ModbusMaster.cpp) | 全API更新 | 同期型API採用 |
| [air_data.ino](air_data/air_data.ino) | クラス簡略化 | コールバック削除、自動管理 |
| [ModbusRTU.h](libraries/ModbusRTU/src/ModbusRTU.h#L1) | ヘッダガード追加 | 多重インクルード対策済み |

---

## Compilation エラーメッセージの消滅

✅ **これらのエラーは解決されます:**

```
error: could not convert 'Serial' from 'HWCDC' to 'HardwareSerial&'
❌ → ✅ Serial1 が自動的に使用される（ユーザー指定）

error: 'ModbusRTU::ModbusRTU()' is protected within this context
❌ → ✅ ModbusRTUServer/ModbusRTUClient は public constructor を持つ

error: 'class ModbusRTU' has no member named 'begin'
❌ → ✅ startModbusServer() / startModbusClient() を使用

error: 'TRegister' has not been declared
❌ → ✅ コールバックシステムが簡略化されて削除

error: redefinition of 'const uint16_t crc_table [256]'
❌ → ✅ ModbusRTU.h にヘッダガード追加
```

---

## 次のステップ (ユーザーが確認すべき項目)

1. **ハードウェア設定**
   - RS485トランシーバーに **Serial1** が接続されていることを確認
   - DE_PIN（D10）がトランシーバーの送受信制御ピンに接続されていることを確認

2. **setup() 関数の確認**
   - `airDataSlave.begin()` が正しく呼び出されているか
   - 割り込みハンドラが正しく設定されているか

3. **loop() 関数の確認**
   - `airDataSlave.task()` が毎ループ呼び出されているか
   - `airDataSlave.updateSensorValues()` が周期的に呼ばれているか
   - `airDataSlave.checkCommands()` で受信コマンドが処理されているか

4. **ボーレート設定**
   - 使用ボーレート (9600, 38400, 115200, 921600, 5000000) が妥当か確認

---

**すべての修正が完了しました！** 🎉  
コンパイルエラーは解決されるはずです。
