# VibeStackChan

**日本語** | [English](README.en.md)

VibeStackChanは、M5Stack CoreS3ベースのｽﾀｯｸﾁｬﾝからCodexを操作するための
ファームウェアです。画面と背面タッチセンサーを使って、エージェントの選択、
音声入力、承認操作などを手元から行えます。

## デモ動画

https://github.com/user-attachments/assets/2e9146ba-471e-43bf-ac56-51009befd163

## 操作方法

| 入力 | 動作 |
| --- | --- |
| Agent／Actionタイルをタップ | 対応する操作を送信 |
| 中央のAgent／Actionボタンをタップ | 操作レイヤーを切り替え |
| 待機中に背面を短くタップ | 音声入力を開始 |
| 録音中に背面を短くタップ | 音声入力を終了（まだ送信しない） |
| 音声入力終了後に背面を650 ms長押し | 音声入力を送信 |
| 画面の`MIC`／`REC` | 音声入力を切り替え |
| 承認画面で背面タッチ | 承認 |
| 画面の`APPROVE`／`REJECT` | 承認または拒否 |
| `SET`をタップ | 音量とBLE接続先を設定 |
| 設定画面の`LIGHTS ON/OFF` | 本体ライトを点灯／消灯 |
| 電源ボタンを短押し | 画面と本体ライトを消灯／復帰 |
| 電源ボタンを長押し | 本体ライトを消して電源OFF |

Codexが承認またはユーザー入力を待っている間は、ｽﾀｯｸﾁｬﾝが首を左右へ振って
知らせます。音声入力中はモーター音を拾わないよう、サーボの動作を停止します。

## インストール

1. [Releases](https://github.com/Corvelis/VibeStackChan/releases)から
   `VibeStackChan-vX.Y.Z-CoreS3.factory.bin`をダウンロードします。
2. Python 3と[esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/)を用意します。

   ```sh
   python3 -m pip install --upgrade esptool
   ```

3. CoreS3をUSB接続します。書き込みモードにならない場合はRESETを約2〜3秒長押しし、
   緑色LEDが点灯したら離します。
4. ポート名とファイル名を実際の値に置き換えて書き込みます。

   macOS:

   ```sh
   python3 -m esptool --chip esp32s3 --port /dev/cu.usbmodemXXXX write-flash 0x0 VibeStackChan-vX.Y.Z-CoreS3.factory.bin
   ```

   Windows:

   ```powershell
   py -m esptool --chip esp32s3 --port COM5 write-flash 0x0 VibeStackChan-vX.Y.Z-CoreS3.factory.bin
   ```

   esptool 4.xでは`write-flash`の代わりに`write_flash`を使います。

書き込みを行うと、CoreS3に入っているファームウェアと保存済みのBLE設定は
上書きされます。

## BLEペアリング

CoreS3を再起動し、Codexを実行しているMacのBluetooth設定から
`VibeStackChan #1`を選択します。接続できない場合は、Macに残っている以前の
VibeStackChan登録を削除してからペアリングし直してください。

`SET`画面では、接続スロットを`#1`、`#2`、`#3`から選択できます。

## ソースからビルド

[PlatformIO](https://platformio.org/)をインストールして実行します。

```sh
pio run -e m5stack-cores3
pio run -e m5stack-cores3 -t upload
```

## 謝辞とライセンス

BLE HIDによるCodex操作、操作体系、UIおよびビジュアルデザインは、
GOROmanさんの[VibeWatch](https://github.com/GOROman/vibewatch)を基にしています。
VibeWatchを公開してくださったGOROmanさんに感謝します。

VibeStackChanはMIT Licenseです。詳しくは[LICENSE](LICENSE)と
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)を参照してください。
