/*
 * pulse_ble.ino
 *
 * エンコーダパルスを正確な 50 ms ウィンドウでカウントし、
 * BLE (Nordic UART Service) でパソコンに送信するスケッチ。
 *
 * ハードウェア構成は air_data と同一:
 *   エンコーダ      : D8 (RISING エッジ割り込み)
 *   バッテリ A/D    : A0
 *   AS5600 (AoA)    : ハードウェア I2C (Wire, デフォルト SDA/SCL)
 *   AS5600 (AoS)    : ソフトウェア I2C (Wire1, SDA=D2, SCL=D3)
 *   ステータス LED   : D0 (BLE 接続中 HIGH)
 *
 * サンプリングタイミング保証:
 *   - esp_timer を ESP_TIMER_ISR ディスパッチで使用し、
 *     正確な 50 ms 周期でパルス数スナップショットを取得する。
 *   - メインループの処理時間（I2C 読み取り・BLE 送信）は
 *     サンプリング周期に影響しない。
 *
 * BLE 送信フォーマット (CSV, LF 終端):
 *   timestamp_us,pulse_count,rpm,aoa_raw,aos_raw
 *
 * Windows / macOS での受信例:
 *   Web Bluetooth REPL、または BLE シリアルターミナルで
 *   NUS (Nordic UART Service) に接続し、TX Characteristic を Notify 登録する。
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <esp_timer.h>

// ── ピン定義 (air_data と同一) ────────────────────────────────────────────
constexpr uint8_t kEncoderPin      = D8;
constexpr uint8_t kBatterySensePin = A0;
constexpr uint8_t kStatusLedPin    = D0;
constexpr uint8_t kAosSdaPin       = D2;
constexpr uint8_t kAosSclPin       = D3;
constexpr bool    kEnableAosSensor = false;

// ── AS5600 ────────────────────────────────────────────────────────────────
constexpr uint8_t kAs5600Address  = 0x36;
constexpr uint8_t kAs5600RegAngle = 0x0C;

// ── サンプリング設定 ──────────────────────────────────────────────────────
constexpr uint32_t kSampleIntervalUs = 500000UL; // 500 ms
constexpr float    kSampleIntervalS  = 0.500f;
constexpr uint16_t kPulsesPerRev     = 1;         // ★ 1回転あたりのパルス数: 実機に合わせて変更

// ── BLE Nordic UART Service (NUS) UUID ───────────────────────────────────
static const char* kNusServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kNusTxUuid      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kNusRxUuid      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// ── ISR ↔ タスク間 共有データ ────────────────────────────────────────────
// ESP32 デュアルコア向け安全なスピンロック
static portMUX_TYPE g_slotMux = portMUX_INITIALIZER_UNLOCKED;

// エンコーダ ISR から加算されるカウンタ（32-bit 整列 → ESP32 上でアトミック読み取り可能）
volatile uint32_t g_totalPulseCount = 0;

struct SampleSlot {
  uint32_t pulseCount;   // 直前 50 ms 窓のパルス数
  uint32_t timestampUs;  // esp_timer 基準のスナップショット時刻 [us]
};

volatile SampleSlot g_slot      = {};
volatile bool       g_slotReady = false;

// タイマー ISR 内でのみ参照する前回カウント（保護不要）
static uint32_t g_prevPulseCount = 0;

// ── BLE ──────────────────────────────────────────────────────────────────
static BLECharacteristic* g_txChar       = nullptr;
static bool               g_bleConnected = false;

// ── AS5600 キャッシュ (別タスクが更新) ─────────────────────────────────
static volatile uint16_t g_aoaRaw       = 0;
static volatile uint16_t g_aosRaw       = 0;
static volatile bool     g_aoaAvailable = false;
static volatile bool     g_aosAvailable = false;

// ── エンコーダ割り込みハンドラ (IRAM) ────────────────────────────────────
void IRAM_ATTR handleEncoderPulse() {
  g_totalPulseCount++;
}

// ── 50 ms タイマーコールバック ──────────────────────────────────────────
// ESP_TIMER_TASK ディスパッチ: 専用の高優先度タイマータスクから呼ばれる。
// メインループ (Arduino loop タスク) とは独立して動作するため、
// ループの処理時間がサンプリング周期に影響しない。
void onSampleTimer(void* /*arg*/) {
  const uint32_t now   = (uint32_t)esp_timer_get_time();
  // volatile uint32_t の 32-bit 読み取りは ESP32 上でアトミック
  const uint32_t total = g_totalPulseCount;

  portENTER_CRITICAL(&g_slotMux);
  g_slot.pulseCount  = total - g_prevPulseCount;
  g_slot.timestampUs = now;
  g_slotReady        = true;
  portEXIT_CRITICAL(&g_slotMux);

  // g_prevPulseCount はタイマータスク内のみで使用するため保護不要
  g_prevPulseCount = total;
}

// ── BLE 接続コールバック ──────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    g_bleConnected = true;
    digitalWrite(kStatusLedPin, HIGH);
  }
  void onDisconnect(BLEServer*) override {
    g_bleConnected = false;
    digitalWrite(kStatusLedPin, LOW);
    // 切断後に自動で再アドバタイズ
    BLEDevice::startAdvertising();
  }
};

// ── AS5600 読み取り関数 ─────────────────────────────────────────────────
static bool readAS5600(TwoWire& wire, uint16_t& rawAngle) {
  wire.beginTransmission(kAs5600Address);
  wire.write(kAs5600RegAngle);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom(kAs5600Address, (uint8_t)2) < 2) return false;
  rawAngle  = ((uint16_t)wire.read() << 8) & 0x0F00;
  rawAngle |= (uint16_t)wire.read();
  return true;
}

// ── AoA 読み取りタスク ──────────────────────────────────────────────────
// 動作確認済みのハードウェア I2C (Wire) のみを扱う。AoS 側が不安定でも AoA は継続する。
static void aoaTask(void* /*arg*/) {
  for (;;) {
    uint16_t rawAngle = 0;
    g_aoaAvailable = readAS5600(Wire, rawAngle);
    if (g_aoaAvailable) {
      g_aoaRaw = rawAngle;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ── AoS 読み取りタスク ──────────────────────────────────────────────────
// ソフトウェア I2C 側が不安定なためデフォルト無効。必要時のみ有効化する。
static void aosTask(void* /*arg*/) {
  for (;;) {
    uint16_t rawAngle = 0;
    g_aosAvailable = readAS5600(Wire1, rawAngle);
    if (g_aosAvailable) {
      g_aosRaw = rawAngle;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ── setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  pinMode(kEncoderPin, INPUT_PULLUP);
  pinMode(kBatterySensePin, INPUT);

  // エンコーダ割り込み登録
  attachInterrupt(digitalPinToInterrupt(kEncoderPin), handleEncoderPulse, RISING);

  // I2C (air_data と同一設定)
  Wire.begin();
  Wire.setClock(400000);
  if (kEnableAosSensor) {
    Wire1.begin(kAosSdaPin, kAosSclPin);
    Wire1.setClock(400000);
  }

  // ── BLE 初期化 ────────────────────────────────────────────────────────
  BLEDevice::init("AirData-BLE");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* svc = server->createService(kNusServiceUuid);

  // TX Characteristic: ESP32 → PC (Notify)
  g_txChar = svc->createCharacteristic(
    kNusTxUuid, BLECharacteristic::PROPERTY_NOTIFY);
  g_txChar->addDescriptor(new BLE2902());

  // RX Characteristic: PC → ESP32 (今回は受信処理なし、NUS 互換のため宣言)
  svc->createCharacteristic(
    kNusRxUuid, BLECharacteristic::PROPERTY_WRITE);

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kNusServiceUuid);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  // AS5600 読み取りタスク起動
  xTaskCreate(aoaTask, "aoa", 2048, nullptr, 1, nullptr);
  if (kEnableAosSensor) {
    xTaskCreate(aosTask, "aos", 2048, nullptr, 1, nullptr);
  }

  // ── 500 ms 周期ハードウェアタイマー ───────────────────────────────────
  // ESP_TIMER_ISR: ISR コンテキストで直接コールバックを実行し、
  // FreeRTOS タスクスケジューラのジッターを排除する。
  esp_timer_handle_t sampleTimer;
  const esp_timer_create_args_t timerArgs = {
    .callback              = onSampleTimer,
    .arg                   = nullptr,
    .dispatch_method       = ESP_TIMER_TASK,
    .name                  = "sample500ms",
    .skip_unhandled_events = false,
  };
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &sampleTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(sampleTimer, kSampleIntervalUs));

  Serial.println("[pulse_ble] ready – advertising as 'AirData-BLE'");
}

// ── loop ──────────────────────────────────────────────────────────────────
void loop() {
  // サンプルが準備されていなければ即時リターン（ポーリング）
  if (!g_slotReady) return;

  // スナップショットをローカルにコピーしてフラグを解除
  SampleSlot sample;
  portENTER_CRITICAL(&g_slotMux);
  sample.pulseCount  = g_slot.pulseCount;
  sample.timestampUs = g_slot.timestampUs;
  g_slotReady        = false;
  portEXIT_CRITICAL(&g_slotMux);

  // RPM 計算: (パルス数 / PPR) / 窓幅[s] * 60
  const float rpm = (float)sample.pulseCount / kPulsesPerRev / kSampleIntervalS * 60.0f;

  // AS5600 キャッシュ値をスナップショット
  const uint16_t aoaRaw = g_aoaRaw;
  const uint16_t aosRaw = kEnableAosSensor ? g_aosRaw : 0;

  // シリアルモニタ出力
  Serial.printf("[BLE:%s] t=%7lu us  pulse=%3u  rpm=%8.1f  aoa=%4u%s  aos=%4u%s\n",
    g_bleConnected ? "CONN  " : "no-con",
    (unsigned long)sample.timestampUs,
    (unsigned)sample.pulseCount,
    rpm,
    (unsigned)aoaRaw,
    g_aoaAvailable ? "" : "?",
    (unsigned)aosRaw,
    kEnableAosSensor && !g_aosAvailable ? "?" : "");

  // BLE 送信 (接続中のみ): timestamp_us,pulse_count,rpm,aoa_raw,aos_raw
  if (g_bleConnected) {
    char buf[64];
    const int len = snprintf(buf, sizeof(buf), "%lu,%u,%.1f,%u,%u\n",
      (unsigned long)sample.timestampUs,
      (unsigned)sample.pulseCount,
      rpm,
      (unsigned)aoaRaw,
      (unsigned)aosRaw);
    if (len > 0) {
      g_txChar->setValue(reinterpret_cast<uint8_t*>(buf), (size_t)len);
      g_txChar->notify();
    }
  }
}
