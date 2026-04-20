"""
receive_ble.py
==============
AirData-BLE (Nordic UART Service) から pulse_ble.ino のデータを受信し、
CSV として保存するスクリプト。

依存ライブラリ:
    pip install bleak

送受信フォーマット (ESP32 → PC):
    timestamp_us,pulse_count,rpm,aoa_raw,aos_raw\n

出力 CSV 列:
    pc_time_iso      : PC 側受信時刻 (ISO 8601, 例: 2026-04-11T12:34:56.789012)
    esp_timestamp_us : ESP32 側 esp_timer 基準のタイムスタンプ [µs]
    pulse_count      : 直前 50 ms ウィンドウのパルス数
    rpm              : 回転数 [rpm] (ESP32 側で計算済み)
    aoa_raw          : AS5600 角度 (迎角側) 生値 [0–4095]
    aos_raw          : AS5600 角度 (滑り角側) 生値 [0–4095]

使い方:
    python receive_ble.py                       # デバイス自動スキャン
    python receive_ble.py --out my_log.csv      # 保存先を指定
    python receive_ble.py --addr AA:BB:CC:DD:EE:FF  # アドレス直指定
"""

import argparse
import asyncio
import csv
import datetime
import sys
from pathlib import Path

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

# ── Nordic UART Service UUID ───────────────────────────────────────────────
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32→PC (Notify)
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC→ESP32 (Write)

DEVICE_NAME        = "AirData-BLE"
RECONNECT_DELAY    = 3.0    # 秒
SCAN_TIMEOUT       = 10.0   # 秒
SAMPLE_INTERVAL_S  = 0.500  # ESP32 側サンプリング周期 [s]

CSV_HEADER = [
    "pc_time_iso",
    "esp_timestamp_us",
    "pulse_count",
    "rpm",
    "aoa_raw",
    "aos_raw",
]


# ── BLE スキャン ───────────────────────────────────────────────────────────
async def find_device(addr: str | None) -> str:
    """アドレスが指定されていればそのまま返す。未指定なら名前でスキャン。"""
    if addr:
        return addr

    print(f"[scan] '{DEVICE_NAME}' をスキャン中... (最大 {SCAN_TIMEOUT:.0f} 秒)")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=SCAN_TIMEOUT)
    if device is None:
        print(f"[error] '{DEVICE_NAME}' が見つかりませんでした。")
        sys.exit(1)
    print(f"[scan] 発見: {device.name}  ({device.address})")
    return device.address


# ── 受信ループ ────────────────────────────────────────────────────────────
async def run(address: str, csv_path: Path) -> None:
    buf = ""  # 受信途中の断片バッファ
    file_new = not csv_path.exists()
    csv_file = csv_path.open("a", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    if file_new:
        writer.writerow(CSV_HEADER)
    csv_file.flush()

    print(f"[csv]  保存先: {csv_path.resolve()}")

    # asyncio.Queue を使ってスレッドセーフに通知データを渡す
    queue: asyncio.Queue[bytearray] = asyncio.Queue()

    def on_notify(_char: BleakGATTCharacteristic, data: bytearray) -> None:
        # bleak が別スレッドからこの関数を呼ぶ場合でも安全なように
        # Queue.put_nowait を使用する
        queue.put_nowait(data)

    async def process_queue() -> None:
        nonlocal buf
        while True:
            data = await queue.get()
            buf += data.decode("utf-8", errors="replace")

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                pc_time = datetime.datetime.now().isoformat(timespec="microseconds")
                parts = line.split(",")

                if len(parts) != 5:
                    print(f"[warn] 不正なデータ ({len(parts)} フィールド): {line!r}")
                    continue

                try:
                    esp_ts, pulse = int(parts[0]), int(parts[1])
                    rpm = float(parts[2])
                    aoa_raw, aos_raw = int(parts[3]), int(parts[4])
                except ValueError:
                    print(f"[warn] パース失敗: {line!r}")
                    continue

                writer.writerow([pc_time, esp_ts, pulse, f"{rpm:.1f}", aoa_raw, aos_raw])
                csv_file.flush()
                print(
                    f"  {pc_time}  pulse={pulse:4d}  rpm={rpm:8.1f}  aoa={aoa_raw:4d}  aos={aos_raw:4d}"
                )

    # 切断後に自動再接続するループ
    while True:
        print(f"[ble]  接続中: {address}")
        try:
            async with BleakClient(address, timeout=15.0) as client:
                print(f"[ble]  接続完了 (MTU={client.mtu_size})")
                await client.start_notify(NUS_TX_UUID, on_notify)
                print("[ble]  通知受信中... (Ctrl+C で終了)")
                # process_queue タスクを起動して接続中は継続
                processor = asyncio.ensure_future(process_queue())
                try:
                    while client.is_connected:
                        await asyncio.sleep(0.5)
                finally:
                    processor.cancel()
        except Exception as exc:
            print(f"[ble]  エラー: {exc}")

        print(f"[ble]  {RECONNECT_DELAY:.0f} 秒後に再接続します...")
        await asyncio.sleep(RECONNECT_DELAY)


# ── エントリーポイント ─────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="AirData-BLE から NUS 経由でデータを受信して CSV に保存する"
    )
    parser.add_argument(
        "--addr",
        metavar="MAC_ADDRESS",
        default=None,
        help="BLE アドレスを直接指定 (省略時はデバイス名でスキャン)",
    )
    parser.add_argument(
        "--out",
        metavar="FILE",
        default=None,
        help="出力 CSV ファイルパス (省略時: airdata_YYYYMMDD_HHMMSS.csv)",
    )
    args = parser.parse_args()

    if args.out is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = Path(f"airdata_{timestamp}.csv")
    else:
        out_path = Path(args.out)

    async def _main() -> None:
        address = await find_device(args.addr)
        await run(address, out_path)

    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\n[info] 終了しました。")


if __name__ == "__main__":
    main()
