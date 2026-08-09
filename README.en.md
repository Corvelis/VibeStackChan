# VibeStackChan

[日本語](README.md) | **English**

VibeStackChan is firmware for controlling Codex from a CoreS3-based Stack-chan.
Use its display and rear touch sensor to select agents, dictate prompts, and
respond to approval requests.

## Demo video

https://github.com/user-attachments/assets/8990cb8f-bef3-4873-8256-471760a9596f

## Controls

| Input | Result |
| --- | --- |
| Tap an Agent or Action tile | Send the corresponding action |
| Tap the center Agent/Action button | Switch control layers |
| Rear tap while idle | Start voice input |
| Rear tap while recording | Stop voice input without sending |
| Rear hold for 650 ms after stopping | Send the prepared voice input |
| On-screen `MIC` / `REC` | Toggle voice input |
| Rear touch on the approval prompt | Approve |
| On-screen `APPROVE` / `REJECT` | Approve or reject |
| Tap `SET` | Configure volume and BLE slot |
| `LIGHTS ON/OFF` in settings | Enable or disable the body lights |
| Short-press the power button | Turn the display and body lights off or on |
| Long-press the power button | Clear the body lights before power-off |

Stack-chan shakes its head when Codex needs approval or user input. Servo motion
stops during voice input to prevent motor noise from reaching the microphone.

## Install

1. Download `VibeStackChan-vX.Y.Z-CoreS3.factory.bin` from
   [Releases](https://github.com/Corvelis/VibeStackChan/releases).
2. Install Python 3 and [esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/esptool/).

   ```sh
   python3 -m pip install --upgrade esptool
   ```

3. Connect the CoreS3 over USB. If it does not enter download mode, hold RESET
   for about 2–3 seconds and release it when the green LED lights.
4. Replace the port and filename with the actual values, then flash the image.

   macOS:

   ```sh
   python3 -m esptool --chip esp32s3 --port /dev/cu.usbmodemXXXX write-flash 0x0 VibeStackChan-vX.Y.Z-CoreS3.factory.bin
   ```

   Windows:

   ```powershell
   py -m esptool --chip esp32s3 --port COM5 write-flash 0x0 VibeStackChan-vX.Y.Z-CoreS3.factory.bin
   ```

   With esptool 4.x, use `write_flash` instead of `write-flash`.

Flashing replaces the existing firmware and saved BLE settings on the CoreS3.

## BLE pairing

Restart the CoreS3 and select `VibeStackChan #1` in Bluetooth settings on the
Mac running Codex. If it does not connect, forget any previous VibeStackChan
entry on the Mac and pair it again.

The `SET` screen lets you select pairing slot `#1`, `#2`, or `#3`.

## Build from source

Install [PlatformIO](https://platformio.org/) and run:

```sh
pio run -e m5stack-cores3
pio run -e m5stack-cores3 -t upload
```

## Acknowledgements and license

The BLE HID controls for Codex, interaction model, UI, and visual design are
based on GOROman's [VibeWatch](https://github.com/GOROman/vibewatch). Thank you
to GOROman for making VibeWatch available.

VibeStackChan is released under the MIT License. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for details.
