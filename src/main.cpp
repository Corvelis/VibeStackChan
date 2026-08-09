#include <Arduino.h>
#include <M5StackChan.h>

#include "VibeStackChanApp.h"

namespace {

constexpr uint8_t kBackTouchIntensityThreshold = 2;
constexpr uint32_t kBackTouchStartupReleaseMs = 1000;

bool backTouchReady = false;
uint32_t backTouchReleasedAt = 0;

bool rearTouchDetected() {
  const auto& intensities = M5StackChan.TouchSensor.getIntensities();
  const uint8_t maximum = max(intensities[0], max(intensities[1], intensities[2]));
  return maximum >= kBackTouchIntensityThreshold;
}

void updateRearTouch(uint32_t now) {
  const bool detected = rearTouchDetected();
  if (!backTouchReady) {
    if (detected) {
      backTouchReleasedAt = 0;
      return;
    }
    if (backTouchReleasedAt == 0) {
      backTouchReleasedAt = now;
      return;
    }
    if (now - backTouchReleasedAt < kBackTouchStartupReleaseMs) {
      return;
    }
    backTouchReady = true;
    Serial.println("[touch] rear sensor ready");
  }
  VibeStackChanApp::updateBackTouch(detected);
}

void updatePowerButton() {
  if (M5.BtnPWR.wasHold()) {
    VibeStackChanApp::prepareForPowerOff();
    return;
  }
  if (M5.BtnPWR.wasClicked()) {
    VibeStackChanApp::handlePowerButtonClick();
  }
}

void showStartupFailure() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(TFT_RED, TFT_BLACK);
  M5.Display.drawString("VibeStackChan", M5.Display.width() / 2,
                        M5.Display.height() / 2 - 14);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.drawString("BLE startup failed", M5.Display.width() / 2,
                        M5.Display.height() / 2 + 16);
}

}  // namespace

void setup() {
  M5StackChan.begin();
  Serial.begin(115200);
  Serial.println("[boot] VibeStackChan standalone");

  if (!VibeStackChanApp::begin()) {
    Serial.println("[boot] VibeStackChan startup failed");
    showStartupFailure();
  }
}

void loop() {
  M5StackChan.update();
  const uint32_t now = millis();
  updatePowerButton();
  updateRearTouch(now);
  VibeStackChanApp::update(now);
  delay(1);
}
