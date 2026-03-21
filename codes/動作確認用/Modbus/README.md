# Modbus 通信テスト

このフォルダは、Modbus RTU の通信確認だけを行うためのテスト用コードです。
センサーや表示処理は入れず、RS485 上での読み書きと応答確認に絞っています。

## 構成

- [air_data/air_data.ino](air_data/air_data.ino)
  - Slave ID: 1
  - エアデータ側の通信確認用
- [logger/logger.ino](logger/logger.ino)
  - Master
  - 2 台のスレーブを読み書き確認する用
- [display_d1/display_d1.ino](display_d1/display_d1.ino)
  - Slave ID: 2
  - 表示側の通信確認用

## 書き込み先

Arduino IDE では、各フォルダをそれぞれ別のスケッチとして開いて書き込みます。

## 通信条件

- ボード: Seeduino XIAO ESP32C3
- 通信: RS485 / Modbus RTU
- 既定ボーレート: 115200 bps
- DE ピン:
  - air_data: D10
  - logger: D2
  - display_d1: D3

## テスト内容

- slave 側はダミー値を holding register に公開
- master 側は定期的に読み取り
- master 側は register 10 へ LED 制御値を書き込み
- register 300 は通信確認用の制御レジスタとして読み書き可能

## 補足

このテスト版では、register 300 は実際のボーレート変更には使わず、
値の書き込みと読み戻しを確認するための制御レジスタとして扱います。
実運用での動的ボーレート切り替えは、通常版のコードを使ってください。