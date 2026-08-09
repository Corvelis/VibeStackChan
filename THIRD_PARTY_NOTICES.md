# Third-Party Notices

VibeStackChan incorporates and depends on the software listed below. The root
MIT License covers the VibeStackChan project. Adapted third-party code retains
its upstream notice.

## VibeWatch

Source: <https://github.com/GOROman/vibewatch>

Pinned source revision: `f2520875a61fe087cb7e5b63a2a61ddcc2e79cb2`

License: MIT, Copyright (c) 2026 GOROman. The required upstream license is
retained at [`licenses/GOROman-MIT.txt`](licenses/GOROman-MIT.txt).

The BLE HID protocol, interaction model, and UI/design elements were adapted
for CoreS3 and Stack-chan controls.

## PlatformIO dependencies

| Dependency | License | Notes |
| --- | --- | --- |
| `m5stack/M5Unified` | MIT | M5Stack unified device library. |
| `m5stack/M5GFX` | MIT and bundled permissive/font licenses | Includes efont, GFXFF, and IPA font notices in its upstream source. |
| `bblanchon/ArduinoJson` | MIT | JSON library. |
| `h2zero/NimBLE-Arduino` | Apache-2.0 | BLE stack wrapper; its upstream distribution includes required notices. |
| `m5stack/StackChan-BSP` | MIT | Stack-chan board support package. |

PlatformIO downloads these dependencies during setup; they are not copied into
this repository.
