#pragma once

#include <Arduino.h>

namespace VibeStackChanApp {

// Starts the CoreS3 UI and BLE HID transport.
bool begin();

// Updates screen input, BLE RPC processing, feedback, and rendering.
void update(uint32_t now);

// CoreS3 Stack-chan uses one rear tap to start microphone input, another tap
// to stop it, and a subsequent rear hold to send the prepared prompt. During
// an approval prompt, a rear touch approves instead.
void updateBackTouch(bool detected);

// Turns the LCD and body lights off or on while keeping BLE connected.
void setDisplayEnabled(bool enabled);
bool displayEnabled();

// Handles a short power-key event, including the synthetic click that the
// CoreS3 PMIC reports after a long-press event.
void handlePowerButtonClick();

// Clears latched outputs before the CoreS3 power controller cuts power after
// a long press. The Stack-chan body's RGB controller otherwise retains its
// last color after the CoreS3 shuts down.
void prepareForPowerOff();

}  // namespace VibeStackChanApp
