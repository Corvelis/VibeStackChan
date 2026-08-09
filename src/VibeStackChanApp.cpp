#include "VibeStackChanApp.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <M5StackChan.h>
#include <Preferences.h>

class NimBLEServer;
class NimBLEConnInfo;
bool configureBleIdentity();

namespace VibeStackChanApp {
enum class TransportPowerProfile : uint8_t {
    Active,
    Background,
    ScreenOff,
};

void setTransportPowerProfile(TransportPowerProfile profile);
bool hidConnected();
void onHidSubscribed(NimBLEConnInfo& connection);
void onHidConnParamsUpdated(NimBLEConnInfo& connection);
}

// BLE transport, host protocol, and shared application state are private to
// this translation unit.
#include "internal/HidRuntime.inc"

bool configureBleIdentity() {
    const std::uint64_t chipMac = ESP.getEfuseMac();
    char addressText[18] = {};
    std::snprintf(addressText, sizeof(addressText),
                  "C2:%02X:%02X:%02X:%02X:%02X",
                  static_cast<unsigned>((chipMac >> 8) & 0xff),
                  static_cast<unsigned>((chipMac >> 16) & 0xff),
                  static_cast<unsigned>((chipMac >> 24) & 0xff),
                  static_cast<unsigned>((chipMac >> 32) & 0xff),
                  static_cast<unsigned>((chipMac >> 40) & 0xff));

    const NimBLEAddress address(addressText, BLE_ADDR_RANDOM);
    const bool addressSet = NimBLEDevice::setOwnAddr(address);
    const bool typeSet = addressSet &&
                         NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    Serial.printf("[app] BLE identity=%s static=%d ready=%d\n",
                  addressText,
                  address.isStatic() ? 1 : 0,
                  typeSet ? 1 : 0);
    return typeSet && address.isStatic();
}

namespace VibeStackChanApp {
namespace {

bool appActive = false;
bool uiVisible = true;
bool transportResident = false;
volatile TransportPowerProfile transportPowerProfile =
    TransportPowerProfile::Active;

struct ConnectionPowerParams {
    uint16_t minInterval;
    uint16_t maxInterval;
    uint16_t latency;
    uint16_t timeout;
    const char* name;
};

ConnectionPowerParams connectionPowerParams(TransportPowerProfile profile) {
    // Interval units are 1.25 ms and timeout units are 10 ms. macOS remains
    // the central and may clamp or reject these requests, so the negotiated
    // values are logged by onHidConnParamsUpdated().
    switch (profile) {
        case TransportPowerProfile::Background:
            return {48, 80, 4, 600, "background"};   // 60-100 ms
        case TransportPowerProfile::ScreenOff:
            return {80, 120, 8, 800, "screen_off"}; // 100-150 ms
        case TransportPowerProfile::Active:
        default:
            return {12, 24, 0, 180, "active"};       // 15-30 ms
    }
}

void requestConnectionPowerProfile(uint16_t connectionHandle) {
    if (g_server == nullptr || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const auto profile = static_cast<TransportPowerProfile>(
        transportPowerProfile);
    const ConnectionPowerParams params = connectionPowerParams(profile);
    g_server->updateConnParams(connectionHandle,
                               params.minInterval,
                               params.maxInterval,
                               params.latency,
                               params.timeout);
    Serial.printf("[app] HID power request profile=%s handle=%u interval=%u-%u latency=%u timeout=%u\n",
                  params.name,
                  static_cast<unsigned>(connectionHandle),
                  static_cast<unsigned>(params.minInterval),
                  static_cast<unsigned>(params.maxInterval),
                  static_cast<unsigned>(params.latency),
                  static_cast<unsigned>(params.timeout));
}

constexpr int kServoPanMin = 45;
constexpr int kServoPanMax = 135;
constexpr int kServoPanCenter = 90;
constexpr int kCoreTouchLayer = kTouchAgentStateVibe + 1;
constexpr int kCoreHeaderHeight = 52;
constexpr int kCoreStatusTop = 216;
constexpr uint32_t kCoreMicAutoStopMs = 60000;
constexpr uint32_t kCoreBackTouchReleaseMs = 180;
constexpr uint32_t kCoreBackTouchReleaseDebounceMs = 120;
constexpr uint32_t kCoreBackTouchHoldMs = 650;
constexpr uint32_t kCoreApprovalSwingIntervalMs = 650;
constexpr int kCoreApprovalPanOffset = 26;
constexpr int kCoreApprovalServoSpeed = 500;
constexpr int kCoreApprovalReturnSpeed = 450;
constexpr uint32_t kCoreApprovalTestDurationMs = 6000;
constexpr uint32_t kCoreApprovalDecisionLockMs = 1500;
constexpr int kCoreTouchApprovalAccept = kCoreTouchLayer + 1;
constexpr int kCoreTouchApprovalReject = kCoreTouchLayer + 2;
constexpr int kCoreTouchSwipePad = kCoreTouchLayer + 3;
constexpr int kCoreTouchLights = kCoreTouchLayer + 4;
constexpr int kCoreSwipePadSlot = 1;
constexpr uint32_t kCoreSwipeHoldMs = 360;
constexpr int kCoreSwipeThresholdPx = 24;
constexpr uint32_t kCoreAgentLedUpdatePeriodMs = 50;
constexpr uint8_t kCoreAgentLedMaxChannel = 96;
constexpr float kCoreAgentLedMinSaturation = 0.82f;

// Physical action-grid slots: FAST, swipe pad, OK / FORK, SEND, NG.
// Values are the upstream logical action indexes; -1 reserves the swipe pad.
constexpr int kCoreActionBySlot[kAgentCount] = {0, -1, 1, 3, 4, 2};
// Codex Micro joystick angles are normalized turns: right=0, down=.25,
// left=.5, and up=.75. This order matches coreSwipeDirectionForDelta().
constexpr std::uint16_t kCoreSwipeAnglesPermille[4] = {750, 250, 500, 0};

struct CoreRect {
    int x;
    int y;
    int w;
    int h;

    bool contains(int px, int py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

bool g_coreMicLatched = false;
bool g_coreVoiceReadyToSend = false;
uint32_t g_coreMicStartedAt = 0;
bool g_corePowerOffPrepared = false;
bool g_coreBackTouchArmed = false;
bool g_coreBackTouchPressed = false;
bool g_coreBackTouchHoldFired = false;
uint32_t g_coreBackTouchPressedAt = 0;
uint32_t g_coreBackTouchReleasedAt = 0;
uint32_t g_coreBackTouchReleaseCandidateAt = 0;
M5Canvas g_coreCanvas(&M5.Display);
bool g_coreCanvasReady = false;
bool g_coreSurfaceReady = false;
bool g_coreRenderedSettings = false;
bool g_coreRenderedActionLayer = false;
bool g_coreSettingsSnapshotValid = false;
bool g_coreRenderedConnected = false;
bool g_coreRenderedRestarting = false;
int g_coreRenderedDeviceSlot = 0;
int g_coreRenderedPendingDeviceSlot = 0;
uint8_t g_coreRenderedVolume = 0;
uint8_t g_coreRenderedBattery = 0;
bool g_coreRenderedCharging = false;
bool g_coreRenderedLightsEnabled = true;
bool g_coreRenderedApprovalPrompt = false;
bool g_coreRenderedSwipeGuide = false;
uint8_t g_coreApprovalMask = 0;
bool g_coreApprovalMotionPending = false;
bool g_coreApprovalMotionActive = false;
uint8_t g_coreApprovalMotionStep = 0;
uint32_t g_coreApprovalNextStepAt = 0;
bool g_coreApprovalReturningCenter = false;
bool g_coreApprovalPausedForMic = false;
uint32_t g_coreApprovalTestUntil = 0;
char g_coreApprovalTestSerialLine[32] = {};
size_t g_coreApprovalTestSerialLength = 0;
uint32_t g_coreApprovalDecisionSentAt = 0;
bool g_coreApprovalDecisionStopsMotion = false;
bool g_coreSwipeTracking = false;
bool g_coreSwipeGuideVisible = false;
uint32_t g_coreSwipeStartedAt = 0;
int g_coreSwipeStartX = 0;
int g_coreSwipeStartY = 0;
int g_coreSwipeDirection = -1;
uint32_t g_coreAgentLedLastUpdateAt = 0;
uint32_t g_coreAgentLedColors[kAgentCount] = {};
bool g_coreAgentLedsActive = false;
uint8_t g_coreDisplayBrightness = 80;
int g_coreYawOffset = 0;

void renderCoreUi(uint32_t now);
void setCoreMicLatched(bool enabled, bool feedback,
                       bool readyToSend = false);
bool sendCoreVoicePrompt();
void releasePressedControls();
bool isCoreApprovalColor(uint32_t packedColor);

void loadCoreServoCalibration() {
    Preferences preferences;
    preferences.begin("motion", true);
    g_coreYawOffset = constrain(preferences.getInt("yaw_offset", 0), -500, 500);
    preferences.end();
    Serial.printf("[app] servo yaw offset=%d\n", g_coreYawOffset);
}

void resetCoreBackTouchGesture() {
    g_coreBackTouchArmed = false;
    g_coreBackTouchPressed = false;
    g_coreBackTouchHoldFired = false;
    g_coreBackTouchPressedAt = 0;
    g_coreBackTouchReleasedAt = 0;
    g_coreBackTouchReleaseCandidateAt = 0;
}

void resetCoreSwipeGesture(bool invalidateSurface) {
    const bool wasVisible = g_coreSwipeGuideVisible;
    g_coreSwipeTracking = false;
    g_coreSwipeGuideVisible = false;
    g_coreSwipeStartedAt = 0;
    g_coreSwipeStartX = 0;
    g_coreSwipeStartY = 0;
    g_coreSwipeDirection = -1;
    if (invalidateSurface && wasVisible) {
        g_coreSurfaceReady = false;
    }
}

void clearCoreAgentLeds() {
    if (!g_coreAgentLedsActive) {
        return;
    }
    for (int led = 0; led < 12; ++led) {
        M5StackChan.setRgbColor(led, 0, 0, 0);
    }
    M5StackChan.refreshRgb();
    for (int i = 0; i < kAgentCount; ++i) {
        g_coreAgentLedColors[i] = 0;
    }
    g_coreAgentLedsActive = false;
    Serial.println("[app] CoreS3 agent LEDs off");
}

void forceCoreAgentLedsOff() {
    for (int led = 0; led < 12; ++led) {
        M5StackChan.setRgbColor(led, 0, 0, 0);
    }
    M5StackChan.refreshRgb();
    for (int i = 0; i < kAgentCount; ++i) {
        g_coreAgentLedColors[i] = 0;
    }
    g_coreAgentLedsActive = false;
}

uint32_t coreAgentLedColor(int index, uint32_t now) {
    const auto& state = g_agents[index];
    if (state.color == 0) {
        return 0;
    }
    float brightness = effectBrightness(state.effect, state.brightness,
                                        state.speed, now);
    if (brightness <= 0.01f && state.brightness > 0.0f) {
        brightness = state.brightness;
    }
    brightness = std::max(0.0f, std::min(1.0f, brightness));
    const uint32_t rgb = state.color & 0x00FFFFFFU;
    float red = static_cast<float>((rgb >> 16) & 0xFFU) / 255.0f;
    float green = static_cast<float>((rgb >> 8) & 0xFFU) / 255.0f;
    float blue = static_cast<float>(rgb & 0xFFU) / 255.0f;

    // The host palette is designed for an LCD and contains deliberately pale
    // colors. On the diffused RGB LEDs those colors blend together and look
    // almost white. Preserve the hue but raise HSV saturation for the LEDs
    // only; the screen continues to use the original host color.
    const float maximum = std::max(red, std::max(green, blue));
    const float minimum = std::min(red, std::min(green, blue));
    if (maximum <= 0.001f) {
        return 0;
    }
    const float saturation = (maximum - minimum) / maximum;
    red /= maximum;
    green /= maximum;
    blue /= maximum;
    if (saturation > 0.001f && saturation < kCoreAgentLedMinSaturation) {
        const float saturationScale = kCoreAgentLedMinSaturation / saturation;
        red = std::max(0.0f, 1.0f - (1.0f - red) * saturationScale);
        green = std::max(0.0f, 1.0f - (1.0f - green) * saturationScale);
        blue = std::max(0.0f, 1.0f - (1.0f - blue) * saturationScale);
    }

    const float outputScale = brightness * kCoreAgentLedMaxChannel;
    const uint8_t outputRed = static_cast<uint8_t>(red * outputScale + 0.5f);
    const uint8_t outputGreen = static_cast<uint8_t>(green * outputScale + 0.5f);
    const uint8_t outputBlue = static_cast<uint8_t>(blue * outputScale + 0.5f);
    return (static_cast<uint32_t>(outputRed) << 16) |
           (static_cast<uint32_t>(outputGreen) << 8) |
           outputBlue;
}

void updateCoreAgentLeds(uint32_t now) {
    if (!appActive || !uiVisible || !g_lightsEnabled) {
        if (g_coreAgentLedsActive ||
            now - g_coreAgentLedLastUpdateAt >= 250) {
            g_coreAgentLedLastUpdateAt = now;
            forceCoreAgentLedsOff();
        }
        return;
    }
    if (now - g_coreAgentLedLastUpdateAt < kCoreAgentLedUpdatePeriodMs) {
        return;
    }
    g_coreAgentLedLastUpdateAt = now;

    // One LED per agent only illuminates a small section at the base of each
    // light guide. Use the complete twelve-LED bar as a single status lamp so
    // the color is visible across Stack-chan's body. Awaiting approval always
    // wins; otherwise show the selected agent, falling back to an active one.
    int statusAgent = -1;
    for (int i = 0; i < kAgentCount; ++i) {
        if (g_agents[i].brightness > 0.02f &&
            isCoreApprovalColor(g_agents[i].color)) {
            statusAgent = i;
            break;
        }
    }
    if (statusAgent < 0 && g_selectedAgent >= 0 &&
        g_selectedAgent < kAgentCount &&
        g_agents[g_selectedAgent].effect != 0 &&
        g_agents[g_selectedAgent].brightness > 0.02f) {
        statusAgent = g_selectedAgent;
    }
    if (statusAgent < 0) {
        float strongestBrightness = 0.02f;
        for (int i = 0; i < kAgentCount; ++i) {
            if (g_agents[i].effect != 0 &&
                g_agents[i].brightness > strongestBrightness) {
                strongestBrightness = g_agents[i].brightness;
                statusAgent = i;
            }
        }
    }

    const uint32_t color = statusAgent >= 0
        ? coreAgentLedColor(statusAgent, now)
        : 0;
    if (g_coreAgentLedsActive && color == g_coreAgentLedColors[0]) {
        return;
    }
    const uint8_t red = static_cast<uint8_t>((color >> 16) & 0xFFU);
    const uint8_t green = static_cast<uint8_t>((color >> 8) & 0xFFU);
    const uint8_t blue = static_cast<uint8_t>(color & 0xFFU);
    for (int led = 0; led < 12; ++led) {
        M5StackChan.setRgbColor(led, red, green, blue);
    }
    for (int i = 0; i < kAgentCount; ++i) {
        g_coreAgentLedColors[i] = i == 0 ? color : 0;
    }
    M5StackChan.refreshRgb();
    g_coreAgentLedsActive = true;
}

bool ensureCoreCanvas() {
    if (g_coreCanvasReady &&
        g_coreCanvas.width() == M5.Display.width() &&
        g_coreCanvas.height() == M5.Display.height()) {
        return true;
    }
    if (g_coreCanvasReady) {
        g_coreCanvas.deleteSprite();
        g_coreCanvasReady = false;
    }
    g_coreCanvas.setPsram(true);
    g_coreCanvas.setColorDepth(16);
    g_coreCanvasReady =
        g_coreCanvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
    if (!g_coreCanvasReady) {
        Serial.println("[app] CoreS3 frame buffer allocation failed");
        return false;
    }
    g_coreCanvas.fillScreen(TFT_BLACK);
    Serial.printf("[app] CoreS3 frame buffer ready %dx%d\n",
                  g_coreCanvas.width(), g_coreCanvas.height());
    return true;
}

bool isCoreApprovalColor(uint32_t packedColor) {
    const uint32_t rgb = packedColor & 0x00FFFFFFU;
    const uint8_t red = static_cast<uint8_t>((rgb >> 16) & 0xFFU);
    const uint8_t green = static_cast<uint8_t>((rgb >> 8) & 0xFFU);
    const uint8_t blue = static_cast<uint8_t>(rgb & 0xFFU);
    // Codex Micro maps awaiting-* states (including approval/input required)
    // to orange #FF6D00. Keep a modest tolerance for host color tuning.
    return red >= 220 && green >= 70 && green <= 150 && blue <= 48;
}

uint8_t coreApprovalMask() {
    uint8_t mask = 0;
    for (int i = 0; i < kAgentCount; ++i) {
        if (g_agents[i].brightness > 0.02f &&
            isCoreApprovalColor(g_agents[i].color)) {
            mask |= static_cast<uint8_t>(1U << i);
        }
    }
    return mask;
}

int coreApprovalYawForPan(int pan) {
    const int clampedPan = constrain(pan, kServoPanMin, kServoPanMax);
    return constrain((kServoPanCenter - clampedPan) * 10 + g_coreYawOffset,
                     -1280, 1280);
}

void moveCoreApprovalPan(int pan, int speed) {
    M5StackChan.Motion.moveYaw(coreApprovalYawForPan(pan), speed);
}

bool coreApprovalTestActive(uint32_t now) {
    return g_coreApprovalTestUntil != 0 &&
           static_cast<int32_t>(now - g_coreApprovalTestUntil) < 0;
}

void handleCoreApprovalTestSerial(uint32_t now) {
    while (Serial.available() > 0) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\r') {
            continue;
        }
        if (value == '\n') {
            g_coreApprovalTestSerialLine[g_coreApprovalTestSerialLength] = '\0';
            if (std::strcmp(g_coreApprovalTestSerialLine,
                            "VIBE_HEAD_TEST") == 0) {
                g_coreApprovalTestUntil = now + kCoreApprovalTestDurationMs;
                g_coreApprovalMotionPending = true;
                Serial.printf("[app] approval head shake test started duration=%lums\n",
                              static_cast<unsigned long>(kCoreApprovalTestDurationMs));
            }
            g_coreApprovalTestSerialLength = 0;
            g_coreApprovalTestSerialLine[0] = '\0';
            continue;
        }
        if (value >= 0x20 && value <= 0x7e &&
            g_coreApprovalTestSerialLength + 1 <
                sizeof(g_coreApprovalTestSerialLine)) {
            g_coreApprovalTestSerialLine[g_coreApprovalTestSerialLength++] = value;
        } else if (value != '\0') {
            g_coreApprovalTestSerialLength = 0;
        }
    }
}

void updateCoreApprovalState(uint32_t now) {
    const uint8_t nextMask = coreApprovalMask();
    const uint8_t newlyAwaiting = nextMask & ~g_coreApprovalMask;
    const bool wasAwaiting = g_coreApprovalMask != 0 ||
                             coreApprovalTestActive(now);
    g_coreApprovalMask = nextMask;
    if (g_coreApprovalTestUntil != 0 && !coreApprovalTestActive(now)) {
        g_coreApprovalTestUntil = 0;
        Serial.println("[app] approval head shake test finished");
    }
    if (!appActive) {
        g_coreApprovalMotionPending = false;
        return;
    }
    if (newlyAwaiting != 0) {
        // Cancel any in-progress tile/microphone report before the dedicated
        // approval controls take ownership of display and rear touch input.
        releasePressedControls();
        g_coreApprovalDecisionSentAt = 0;
        g_coreApprovalDecisionStopsMotion = false;
        g_coreApprovalMotionPending = true;
        Serial.printf("[app] approval attention requested agents=0x%02X\n",
                      static_cast<unsigned>(newlyAwaiting));
    }
    const bool motionRequested =
        (nextMask != 0 && !g_coreApprovalDecisionStopsMotion) ||
        coreApprovalTestActive(now);
    if (motionRequested) {
        // Keep a request armed for as long as any agent is awaiting. The
        // motion loop consumes it only to start, then continues alternating
        // left/right until the awaiting state clears.
        g_coreApprovalMotionPending = true;
    } else if (nextMask == 0) {
        g_coreApprovalDecisionSentAt = 0;
        g_coreApprovalDecisionStopsMotion = false;
        g_coreApprovalMotionPending = false;
        if (g_coreApprovalMotionActive) {
            g_coreApprovalMotionActive = false;
            g_coreApprovalMotionStep = 0;
            g_coreApprovalNextStepAt = 0;
            g_coreApprovalPausedForMic = false;
            M5StackChan.Motion.stop();
            moveCoreApprovalPan(kServoPanCenter,
                                kCoreApprovalReturnSpeed);
            g_coreApprovalReturningCenter = true;
            Serial.println("[app] approval head shake stopped; returning center");
        } else if (wasAwaiting) {
            M5StackChan.Motion.stop();
            moveCoreApprovalPan(kServoPanCenter,
                                kCoreApprovalReturnSpeed);
            g_coreApprovalReturningCenter = true;
        }
    }
}

void setCoreApprovalMotionTarget(uint8_t step) {
    static constexpr int kPanOffsets[] = {
        -kCoreApprovalPanOffset,
         kCoreApprovalPanOffset,
    };
    moveCoreApprovalPan(kServoPanCenter + kPanOffsets[step],
                        kCoreApprovalServoSpeed);
    Serial.printf("[app] approval head shake step=%u pan=%d\n",
                  static_cast<unsigned>(step),
                  kServoPanCenter + kPanOffsets[step]);
}

void updateCoreApprovalMotion(uint32_t now) {
    if (g_coreApprovalReturningCenter) {
        if (M5StackChan.Motion.isYawMoving()) {
            return;
        }
        g_coreApprovalReturningCenter = false;
    }

    // Pause in place while recording so servo noise is not captured. Resume
    // the current fast target afterward if an agent is still awaiting.
    if (g_coreMicLatched) {
        if (g_coreApprovalMotionActive && !g_coreApprovalPausedForMic) {
            M5StackChan.Motion.stop();
            g_coreApprovalPausedForMic = true;
            Serial.println("[app] approval head shake paused for microphone");
        }
        return;
    }
    if (g_coreApprovalPausedForMic) {
        g_coreApprovalPausedForMic = false;
        if (g_coreApprovalMotionActive && g_coreApprovalMask != 0) {
            setCoreApprovalMotionTarget(g_coreApprovalMotionStep);
            g_coreApprovalNextStepAt = now + kCoreApprovalSwingIntervalMs;
            Serial.println("[app] approval head shake resumed after microphone");
            return;
        }
    }

    if (!g_coreApprovalMotionActive) {
        if (!g_coreApprovalMotionPending || g_coreMicLatched) {
            return;
        }
        g_coreApprovalMotionPending = false;
        g_coreApprovalMotionActive = true;
        g_coreApprovalMotionStep = 0;
        g_coreApprovalNextStepAt = now + kCoreApprovalSwingIntervalMs;
        M5StackChan.Motion.stop();
        setCoreApprovalMotionTarget(g_coreApprovalMotionStep);
        Serial.printf("[app] approval head shake started speed=%d offset=%d\n",
                      kCoreApprovalServoSpeed, kCoreApprovalPanOffset);
        return;
    }

    if (static_cast<int32_t>(now - g_coreApprovalNextStepAt) < 0) {
        return;
    }
    g_coreApprovalMotionStep = (g_coreApprovalMotionStep + 1) % 2;
    setCoreApprovalMotionTarget(g_coreApprovalMotionStep);
    g_coreApprovalNextStepAt = now + kCoreApprovalSwingIntervalMs;
}

void resetCoreApprovalMotion(bool returnToCenter) {
    g_coreApprovalMask = 0;
    g_coreApprovalMotionPending = false;
    g_coreApprovalMotionActive = false;
    g_coreApprovalMotionStep = 0;
    g_coreApprovalNextStepAt = 0;
    g_coreApprovalReturningCenter = false;
    g_coreApprovalPausedForMic = false;
    g_coreApprovalTestUntil = 0;
    g_coreApprovalTestSerialLength = 0;
    g_coreApprovalTestSerialLine[0] = '\0';
    g_coreApprovalDecisionSentAt = 0;
    g_coreApprovalDecisionStopsMotion = false;
    if (returnToCenter) {
        M5StackChan.Motion.stop();
        moveCoreApprovalPan(kServoPanCenter, kCoreApprovalReturnSpeed);
    }
}

void releasePressedControls() {
    if (g_activeTouch >= 0 && g_activeTouch < kAgentCount) {
        if (g_touchActionLayer) {
            sendOuterActionEvent(g_activeTouch, false);
        } else {
            sendAgentEvent(g_activeTouch, false);
        }
    } else if (g_activeTouch == kTouchMic) {
        sendMicEvent(false);
    }

    if (g_leftAgentPressed >= 0) {
        if (g_leftPressedActionLayer) {
            sendOuterActionEvent(g_leftAgentPressed, false);
        } else {
            sendAgentEvent(g_leftAgentPressed, false);
        }
    }
    if (g_rightActionPressed) {
        sendOuterActionEvent(kOkAction, false);
    }
    if (g_rightLongTriggered) {
        sendMicEvent(false);
    }
    resetCoreSwipeGesture(true);
    if (g_coreMicLatched) {
        sendMicEvent(false);
        g_coreMicLatched = false;
        g_coreMicStartedAt = 0;
    }
    g_coreVoiceReadyToSend = false;

    g_activeTouch = -1;
    g_leftAgentPressed = -1;
    g_leftPressedActionLayer = false;
    g_leftPressPending = false;
    g_rightLongTriggered = false;
    g_rightPhysicalPressedAt = 0;
    g_rightActionPending = false;
    g_rightActionPressed = false;
    g_buttonChordActive = false;
    g_vibrationOffAt = 0;
    M5.Power.setVibration(0);
    g_uiDirty = true;
}

void drainRpcQueue() {
    if (g_rpcQueue == nullptr) {
        return;
    }
    char* message = nullptr;
    while (xQueueReceive(g_rpcQueue, &message, 0) == pdTRUE) {
        std::free(message);
        message = nullptr;
    }
}

void stopBleTransport() {
    if (g_server != nullptr) {
        // The reference callback has static storage. Ensure NimBLE does not
        // take ownership of it when deinit(true) destroys the server.
        g_server->setCallbacks(nullptr, false);
        const auto peers = g_server->getPeerDevices();
        for (const auto connectionHandle : peers) {
            g_server->disconnect(connectionHandle);
        }
    }
    if (g_vendorOutput != nullptr) {
        g_vendorOutput->setCallbacks(nullptr);
    }
    if (g_vendorInput != nullptr) {
        g_vendorInput->setCallbacks(nullptr);
    }
    if (NimBLEDevice::isInitialized()) {
        NimBLEDevice::getAdvertising()->stop();
        delay(40);
        NimBLEDevice::deinit(true);
    }
    delete g_hid;
    g_server = nullptr;
    g_hid = nullptr;
    g_vendorInput = nullptr;
    g_vendorOutput = nullptr;
    g_connected = false;
    g_hidConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
    g_bleStarted = false;
    g_pairingSuccessPending = false;
    g_rxBuffer = "";
    transportResident = false;
}

bool startBleTransport() {
    initializeBle();
    if (g_server != nullptr) {
        // Keep advertising reusable after a disconnect.
        g_server->setCallbacks(&g_serverCallbacks, false);
    }
    const bool ready = g_bleStarted &&
           g_server != nullptr && g_hid != nullptr &&
           g_vendorInput != nullptr && g_vendorOutput != nullptr;
    transportResident = ready;
    return ready;
}

bool resumeAdvertising() {
    if (!transportResident || !NimBLEDevice::isInitialized() ||
        g_hid == nullptr) {
        return false;
    }
    if (g_connected) {
        return true;
    }

    auto* advertising = NimBLEDevice::getAdvertising();
    advertising->stop();
    advertising->clearData();
    advertising->setName(g_deviceName);
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(g_hid->getHidService()->getUUID());
    advertising->enableScanResponse(true);
    const bool started = advertising->start();
    Serial.printf("[app] resident advertising=%d name=%s\n",
                  started ? 1 : 0, g_deviceName);
    return started;
}

void restartBleTransportForPairing() {
    stopBleTransport();
    g_restartAt = 0;
    if (!startBleTransport()) {
        Serial.println("[app] BLE restart failed");
    }
    g_uiDirty = true;
}

CoreRect coreTileRect(int index) {
    const int column = index % 3;
    const int row = index / 3;
    return {6 + column * 104, 56 + row * 78, 100, 72};
}

CoreRect coreLayerRect() {
    return {66, 4, 188, 44};
}

CoreRect coreMicRect() {
    return {6, 4, 54, 44};
}

CoreRect coreSettingsRect() {
    return {260, 4, 54, 44};
}

CoreRect coreApprovalAcceptRect() {
    return {8, 66, 148, 166};
}

CoreRect coreApprovalRejectRect() {
    return {164, 66, 148, 166};
}

bool coreApprovalDecisionLocked(uint32_t now) {
    return g_coreApprovalDecisionSentAt != 0 &&
           now - g_coreApprovalDecisionSentAt <
               kCoreApprovalDecisionLockMs;
}

int hitTestCoreApproval(int x, int y) {
    if (coreApprovalAcceptRect().contains(x, y)) {
        return kCoreTouchApprovalAccept;
    }
    if (coreApprovalRejectRect().contains(x, y)) {
        return kCoreTouchApprovalReject;
    }
    return -1;
}

bool sendCoreApprovalDecision(bool approve, const char* source) {
    const uint32_t now = millis();
    if (g_coreApprovalMask == 0 || !g_connected ||
        coreApprovalDecisionLocked(now)) {
        return false;
    }
    const int action = approve ? kOkAction : kNgAction;
    g_selectedAction = action;
    sendOuterActionEvent(action, true);
    delay(18);
    sendOuterActionEvent(action, false);
    playOuterActionPressSe(action);
    g_coreApprovalDecisionSentAt = millis();
    g_coreApprovalDecisionStopsMotion = true;
    g_coreApprovalMotionPending = false;
    g_coreApprovalMotionActive = false;
    g_coreApprovalMotionStep = 0;
    g_coreApprovalNextStepAt = 0;
    g_coreApprovalPausedForMic = false;
    M5StackChan.Motion.stop();
    moveCoreApprovalPan(kServoPanCenter, kCoreApprovalReturnSpeed);
    g_coreApprovalReturningCenter = true;
    g_uiDirty = true;
    Serial.printf("[app] approval decision=%s source=%s; head shake stopped\n",
                  approve ? "accept" : "reject",
                  source != nullptr ? source : "unknown");
    return true;
}

CoreRect coreSettingsCloseRect() {
    return {266, 4, 48, 40};
}

CoreRect coreSlotRect(int index) {
    return {10 + index * 70, 53, 62, 38};
}

CoreRect corePairRect() {
    return {224, 53, 86, 38};
}

CoreRect coreVolumeRect() {
    return {20, 113, 280, 48};
}

CoreRect coreLightsRect() {
    return {80, 169, 160, 36};
}

void drawCoreBorder(const CoreRect& rect, int radius, int thickness,
                    uint16_t color) {
    for (int i = 0; i < thickness; ++i) {
        g_coreCanvas.drawRoundRect(rect.x + i, rect.y + i,
                                 rect.w - i * 2, rect.h - i * 2,
                                 std::max(1, radius - i), color);
    }
}

void drawCoreApprovalPrompt(uint32_t now) {
    const bool locked = coreApprovalDecisionLocked(now);
    const uint16_t background = g_coreCanvas.color565(10, 12, 17);
    const uint16_t orange = g_coreCanvas.color565(255, 109, 0);
    const uint16_t green = g_coreCanvas.color565(28, locked ? 104 : 176,
                                                 locked ? 76 : 92);
    const uint16_t red = g_coreCanvas.color565(locked ? 112 : 206,
                                               43, 51);
    g_coreCanvas.fillScreen(background);
    g_coreCanvas.fillRoundRect(8, 6, 304, 52, 14, orange);
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.88f);
    g_coreCanvas.setTextColor(TFT_WHITE, orange);
    g_coreCanvas.drawString(locked ? "DECISION SENT - WAITING"
                                   : "CODEX NEEDS APPROVAL",
                            160, 32);

    const CoreRect accept = coreApprovalAcceptRect();
    const CoreRect reject = coreApprovalRejectRect();
    g_coreCanvas.fillRoundRect(accept.x, accept.y, accept.w, accept.h, 18,
                               green);
    g_coreCanvas.fillRoundRect(reject.x, reject.y, reject.w, reject.h, 18,
                               red);
    drawCoreBorder(accept, 18,
                   g_activeTouch == kCoreTouchApprovalAccept ? 5 : 2,
                   TFT_WHITE);
    drawCoreBorder(reject, 18,
                   g_activeTouch == kCoreTouchApprovalReject ? 5 : 2,
                   TFT_WHITE);

    const int acceptCx = accept.x + accept.w / 2;
    const int rejectCx = reject.x + reject.w / 2;
    g_coreCanvas.drawWideLine(acceptCx - 25, 126, acceptCx - 8, 145,
                              9.0f, TFT_WHITE);
    g_coreCanvas.drawWideLine(acceptCx - 8, 145, acceptCx + 30, 104,
                              9.0f, TFT_WHITE);
    g_coreCanvas.drawWideLine(rejectCx - 24, 108, rejectCx + 24, 151,
                              9.0f, TFT_WHITE);
    g_coreCanvas.drawWideLine(rejectCx + 24, 108, rejectCx - 24, 151,
                              9.0f, TFT_WHITE);

    g_coreCanvas.setTextSize(0.86f);
    g_coreCanvas.setTextColor(TFT_WHITE, green);
    g_coreCanvas.drawString("APPROVE", acceptCx, 180);
    g_coreCanvas.setTextColor(TFT_WHITE, red);
    g_coreCanvas.drawString("REJECT", rejectCx, 180);
    g_coreCanvas.setTextSize(0.55f);
    g_coreCanvas.setTextColor(TFT_WHITE, green);
    g_coreCanvas.drawString("REAR TAP", acceptCx, 211);
    g_coreCanvas.setTextColor(TFT_WHITE, red);
    g_coreCanvas.drawString("SCREEN TAP", rejectCx, 211);
}

uint16_t coreAgentColor(int index, uint32_t now) {
    const auto& state = g_agents[index];
    float brightness = effectBrightness(state.effect, state.brightness,
                                        state.speed, now);
    if (brightness <= 0.01f && state.brightness > 0.0f) {
        brightness = state.brightness;
    }
    if (brightness <= 0.01f || state.color == 0) {
        return g_coreCanvas.color565(46, 51, 63);
    }
    return scaledColor(state.color, std::max(0.22f, brightness));
}

void drawCoreHeader() {
    const uint16_t header = g_coreCanvas.color565(18, 22, 30);
    const uint16_t micColor = g_coreMicLatched
        ? g_coreCanvas.color565(220, 54, 66)
        : g_coreCanvas.color565(38, 91, 205);
    g_coreCanvas.fillRect(0, 0, g_coreCanvas.width(), kCoreHeaderHeight, header);

    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    const CoreRect layer = coreLayerRect();
    const uint16_t layerFill = g_actionLayer
        ? g_coreCanvas.color565(112, 72, 194)
        : g_coreCanvas.color565(45, 91, 176);
    g_coreCanvas.fillRoundRect(layer.x, layer.y, layer.w, layer.h, 12,
                               layerFill);
    drawCoreBorder(layer, 12, 2, g_coreCanvas.color565(174, 184, 218));
    const int layerIconX = layer.x + 31;
    const int layerCy = layer.y + layer.h / 2;
    if (g_actionLayer) {
        g_coreCanvas.fillTriangle(layerIconX + 3, layerCy - 16,
                                  layerIconX - 9, layerCy + 1,
                                  layerIconX + 1, layerCy - 1, TFT_WHITE);
        g_coreCanvas.fillTriangle(layerIconX - 1, layerCy - 2,
                                  layerIconX + 10, layerCy - 4,
                                  layerIconX - 4, layerCy + 17, TFT_WHITE);
    } else {
        g_coreCanvas.fillCircle(layerIconX, layerCy - 8, 7, TFT_WHITE);
        g_coreCanvas.fillRoundRect(layerIconX - 12, layerCy + 1, 24, 16, 7,
                                   TFT_WHITE);
    }
    g_coreCanvas.setTextSize(0.96f);
    g_coreCanvas.setTextColor(TFT_WHITE, layerFill);
    g_coreCanvas.drawString(g_actionLayer ? "ACTION" : "AGENT",
                            layer.x + 120, layerCy);

    const CoreRect mic = coreMicRect();
    g_coreCanvas.fillRoundRect(mic.x, mic.y, mic.w, mic.h, 10, micColor);
    drawCoreBorder(mic, 10, 2, g_coreMicLatched ? TFT_WHITE : micColor);
    const int micIconX = mic.x + mic.w / 2;
    const int micIconY = mic.y + 16;
    g_coreCanvas.fillRoundRect(micIconX - 4, micIconY - 8, 8, 15, 4,
                               TFT_WHITE);
    g_coreCanvas.drawArc(micIconX, micIconY, 8, 6, 0, 180, TFT_WHITE);
    g_coreCanvas.drawFastVLine(micIconX, micIconY + 6, 5, TFT_WHITE);
    g_coreCanvas.setTextSize(0.46f);
    g_coreCanvas.setTextColor(TFT_WHITE, micColor);
    g_coreCanvas.drawString(g_coreMicLatched ? "REC" : "MIC",
                            micIconX, mic.y + 36);

    const CoreRect settings = coreSettingsRect();
    g_coreCanvas.fillRoundRect(settings.x, settings.y, settings.w, settings.h, 12,
                             g_coreCanvas.color565(45, 50, 62));
    drawCoreBorder(settings, 12, 2, g_coreCanvas.color565(96, 104, 124));
    const int gearX = settings.x + settings.w / 2;
    const int gearY = settings.y + 17;
    g_coreCanvas.drawCircle(gearX, gearY, 8, TFT_WHITE);
    g_coreCanvas.drawCircle(gearX, gearY, 3, TFT_WHITE);
    for (int i = 0; i < 8; ++i) {
        const float angle = i * PI / 4.0f;
        g_coreCanvas.drawWideLine(gearX + static_cast<int>(cosf(angle) * 8),
                                gearY + static_cast<int>(sinf(angle) * 8),
                                gearX + static_cast<int>(cosf(angle) * 11),
                                gearY + static_cast<int>(sinf(angle) * 11),
                                2.0f, TFT_WHITE);
    }
    g_coreCanvas.setTextSize(0.48f);
    g_coreCanvas.setTextColor(TFT_WHITE, g_coreCanvas.color565(45, 50, 62));
    g_coreCanvas.drawString("SET", settings.x + settings.w / 2,
                          settings.y + 36);
}

void drawCoreAgentTile(int index, uint32_t now) {
    const CoreRect rect = coreTileRect(index);
    const bool selected = index == g_selectedAgent;
    const uint16_t stateColor = coreAgentColor(index, now);
    const uint16_t fill = selected
        ? stateColor
        : g_coreCanvas.color565(25, 29, 38);
    const uint16_t border = selected ? TFT_WHITE : stateColor;
    g_coreCanvas.fillRoundRect(rect.x, rect.y, rect.w, rect.h, 12, fill);
    drawCoreBorder(rect, 12, selected ? 4 : 2, border);

    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::Orbitron_Light_32);
    g_coreCanvas.setTextSize(1.0f);
    g_coreCanvas.setTextColor(TFT_WHITE, fill);
    char number[3];
    std::snprintf(number, sizeof(number), "%d", index + 1);
    g_coreCanvas.drawString(number, rect.x + rect.w / 2, rect.y + rect.h / 2 - 5);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.58f);
    g_coreCanvas.drawString(selected ? "SELECTED" : "AGENT",
                          rect.x + rect.w / 2, rect.y + rect.h - 13);
}

void drawCoreActionTile(int slot) {
    static constexpr const char* labels[kActionCount] = {
        "FAST", "OK", "NG", "FORK", "SEND",
    };
    static constexpr uint32_t packedColors[kActionCount] = {
        0x2D8CFF, 0x2DD47D, 0xEF5350, 0x9575CD, 0x36C5F0,
    };
    const int action = kCoreActionBySlot[slot];
    if (action < 0 || action >= kActionCount) {
        return;
    }
    const CoreRect rect = coreTileRect(slot);
    const bool selected = action == g_selectedAction;
    // ACT09 is the host's FORK action and remains a momentary control.
    const bool toggled = false;
    const uint16_t accent = scaledColor(packedColors[action], 1.0f);
    const uint16_t fill = selected || toggled
        ? scaledColor(packedColors[action], toggled ? 0.72f : 0.46f)
        : g_coreCanvas.color565(25, 29, 38);
    g_coreCanvas.fillRoundRect(rect.x, rect.y, rect.w, rect.h, 12, fill);
    drawCoreBorder(rect, 12, selected ? 4 : 2, selected ? TFT_WHITE : accent);
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(1.0f);
    g_coreCanvas.setTextColor(TFT_WHITE, fill);
    g_coreCanvas.drawString(labels[action], rect.x + rect.w / 2,
                          rect.y + rect.h / 2);
    if (toggled) {
        g_coreCanvas.setFont(&fonts::DejaVu18);
        g_coreCanvas.setTextSize(0.55f);
        g_coreCanvas.drawString("ON", rect.x + rect.w / 2, rect.y + rect.h - 13);
    }
}

void drawCoreSwipePadTile() {
    const CoreRect rect = coreTileRect(kCoreSwipePadSlot);
    const bool pressed = g_coreSwipeTracking && !g_coreSwipeGuideVisible;
    const uint16_t fill = pressed
        ? g_coreCanvas.color565(39, 76, 125)
        : g_coreCanvas.color565(25, 34, 46);
    const uint16_t border = pressed
        ? TFT_WHITE
        : g_coreCanvas.color565(64, 154, 230);
    g_coreCanvas.fillRoundRect(rect.x, rect.y, rect.w, rect.h, 12, fill);
    drawCoreBorder(rect, 12, pressed ? 4 : 2, border);
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.65f);
    g_coreCanvas.setTextColor(TFT_WHITE, fill);
    g_coreCanvas.drawString("HOLD", rect.x + rect.w / 2,
                          rect.y + rect.h / 2 - 9);
    g_coreCanvas.setTextSize(0.56f);
    g_coreCanvas.setTextColor(g_coreCanvas.color565(108, 199, 255), fill);
    g_coreCanvas.drawString("SWIPE", rect.x + rect.w / 2,
                          rect.y + rect.h / 2 + 14);
}

void drawCoreSwipeGuide() {
    const uint16_t background = g_coreCanvas.color565(11, 16, 24);
    const uint16_t normalFill = g_coreCanvas.color565(28, 36, 50);
    const uint16_t normalBorder = g_coreCanvas.color565(70, 105, 145);
    const uint16_t activeFill = g_coreCanvas.color565(38, 112, 205);
    g_coreCanvas.fillRect(0, kCoreHeaderHeight,
                          g_coreCanvas.width(),
                          kCoreStatusTop - kCoreHeaderHeight,
                          background);

    // Keep the guide centered on the physical HOLD/SWIPE tile (160, 92)
    // rather than on the whole content area. This makes the choices appear
    // around the finger exactly where the gesture starts.
    constexpr int guideCenterX = 160;
    constexpr int guideCenterY = 92;
    const CoreRect directions[4] = {
        {guideCenterX - 38, 54, 76, 30},              // up
        {guideCenterX - 38, guideCenterY + 19, 76, 34}, // down
        {42, guideCenterY - 17, 86, 34},              // left
        {192, guideCenterY - 17, 86, 34},             // right
    };
    static constexpr const char* labels[4] = {
        "UP", "DOWN", "LEFT", "RIGHT",
    };
    for (int direction = 0; direction < 4; ++direction) {
        const bool active = direction == g_coreSwipeDirection;
        const CoreRect& rect = directions[direction];
        const uint16_t fill = active ? activeFill : normalFill;
        g_coreCanvas.fillRoundRect(rect.x, rect.y, rect.w, rect.h, 11, fill);
        drawCoreBorder(rect, 11, active ? 4 : 2,
                       active ? TFT_WHITE : normalBorder);
        g_coreCanvas.setTextDatum(middle_center);
        g_coreCanvas.setFont(&fonts::DejaVu18);
        g_coreCanvas.setTextSize(active ? 0.82f : 0.68f);
        g_coreCanvas.setTextColor(TFT_WHITE, fill);
        g_coreCanvas.drawString(labels[direction],
                              rect.x + rect.w / 2,
                              rect.y + rect.h / 2);
    }

    g_coreCanvas.fillCircle(guideCenterX, guideCenterY, 18,
                            g_coreCanvas.color565(43, 50, 65));
    g_coreCanvas.drawCircle(guideCenterX, guideCenterY, 18,
                            g_coreCanvas.color565(120, 132, 157));
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.46f);
    g_coreCanvas.setTextColor(g_coreCanvas.color565(213, 220, 234),
                              g_coreCanvas.color565(43, 50, 65));
    g_coreCanvas.drawString("HOLD", guideCenterX, guideCenterY);

    g_coreCanvas.setTextSize(0.48f);
    g_coreCanvas.setTextColor(g_coreCanvas.color565(150, 163, 188),
                              background);
    g_coreCanvas.drawString("SLIDE + RELEASE", guideCenterX, 158);
}

void drawCoreStatus() {
    const uint16_t panel = g_coreCanvas.color565(18, 22, 30);
    const uint16_t state = g_connected
        ? g_coreCanvas.color565(55, 220, 132)
        : g_coreCanvas.color565(255, 174, 54);
    g_coreCanvas.fillRect(0, kCoreStatusTop, g_coreCanvas.width(),
                        g_coreCanvas.height() - kCoreStatusTop, panel);
    g_coreCanvas.fillCircle(10, kCoreStatusTop + 12, 4, state);
    g_coreCanvas.setTextDatum(middle_left);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.58f);
    g_coreCanvas.setTextColor(TFT_WHITE, panel);
    char status[64];
    if (g_restartAt != 0) {
        std::snprintf(status, sizeof(status), "RESTART  #%d", g_deviceSlot);
    } else {
        std::snprintf(status, sizeof(status), "%s  #%d  %u%%%s",
                      g_connected ? "ON" : "PAIR", g_deviceSlot,
                      g_batteryLevel, g_isCharging ? "+" : "");
    }
    g_coreCanvas.drawString(status, 19, kCoreStatusTop + 12);
    g_coreCanvas.setTextDatum(middle_right);
    g_coreCanvas.setTextColor(g_coreCanvas.color565(176, 182, 199), panel);
    g_coreCanvas.drawString("BACK: TAP/HOLD", g_coreCanvas.width() - 7,
                          kCoreStatusTop + 12);
}

void renderCoreSettings(bool clearBackground) {
    const uint16_t background = g_coreCanvas.color565(12, 15, 21);
    const uint16_t panel = g_coreCanvas.color565(29, 34, 44);
    const uint16_t purple = g_coreCanvas.color565(128, 103, 240);
    if (clearBackground) {
        g_coreCanvas.fillScreen(background);
    } else {
        // Clear only the mutable settings body. Avoid the full-screen black
        // flash that previously happened on every touch update.
        g_coreCanvas.fillRect(0, 96, g_coreCanvas.width(),
                            kCoreStatusTop - 96, background);
    }
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.70f);
    g_coreCanvas.setTextColor(TFT_WHITE, background);
    g_coreCanvas.drawString("VibeStackChan", 132, 14);
    g_coreCanvas.setTextSize(0.48f);
    g_coreCanvas.setTextColor(g_coreCanvas.color565(172, 180, 201), background);
    g_coreCanvas.drawString("SETTINGS", 132, 34);

    const CoreRect close = coreSettingsCloseRect();
    g_coreCanvas.fillRoundRect(close.x, close.y, close.w, close.h, 8, panel);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.8f);
    g_coreCanvas.setTextColor(TFT_WHITE, panel);
    g_coreCanvas.drawString("X", close.x + close.w / 2, close.y + close.h / 2);

    for (int i = 0; i < 3; ++i) {
        const CoreRect slot = coreSlotRect(i);
        const bool selected = g_pendingDeviceSlot == i + 1;
        const uint16_t fill = selected ? purple : panel;
        g_coreCanvas.fillRoundRect(slot.x, slot.y, slot.w, slot.h, 9, fill);
        drawCoreBorder(slot, 9, selected ? 3 : 1,
                       selected ? TFT_WHITE : g_coreCanvas.color565(91, 98, 116));
        g_coreCanvas.setTextColor(TFT_WHITE, fill);
        char label[4];
        std::snprintf(label, sizeof(label), "#%d", i + 1);
        g_coreCanvas.drawString(label, slot.x + slot.w / 2,
                              slot.y + slot.h / 2);
    }

    const CoreRect pair = corePairRect();
    g_coreCanvas.fillRoundRect(pair.x, pair.y, pair.w, pair.h, 9, purple);
    drawCoreBorder(pair, 9, 2, TFT_WHITE);
    g_coreCanvas.setTextColor(TFT_WHITE, purple);
    g_coreCanvas.drawString("PAIR", pair.x + pair.w / 2,
                          pair.y + pair.h / 2);

    g_coreCanvas.setTextColor(TFT_WHITE, background);
    g_coreCanvas.setTextDatum(middle_left);
    char volume[28];
    std::snprintf(volume, sizeof(volume), "SE VOLUME  %u%%",
                  static_cast<unsigned>(g_seVolume) * 100U / 255U);
    g_coreCanvas.drawString(volume, 20, 108);
    const int sliderX = 25 + static_cast<int>(g_seVolume) * 270 / 255;
    g_coreCanvas.drawWideLine(25, 142, 295, 142, 6.0f,
                            g_coreCanvas.color565(78, 84, 101));
    g_coreCanvas.drawWideLine(25, 142, sliderX, 142, 6.0f, purple);
    g_coreCanvas.fillCircle(sliderX, 142, 10, TFT_WHITE);

    const CoreRect lights = coreLightsRect();
    const uint16_t lightsFill = g_lightsEnabled
        ? g_coreCanvas.color565(42, 133, 88)
        : panel;
    g_coreCanvas.fillRoundRect(lights.x, lights.y, lights.w, lights.h, 9,
                               lightsFill);
    drawCoreBorder(lights, 9, 2, g_lightsEnabled ? TFT_WHITE
                                                 : g_coreCanvas.color565(91, 98, 116));
    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setTextSize(0.56f);
    g_coreCanvas.setTextColor(TFT_WHITE, lightsFill);
    g_coreCanvas.drawString(g_lightsEnabled ? "LIGHTS ON" : "LIGHTS OFF",
                            lights.x + lights.w / 2,
                            lights.y + lights.h / 2);

    drawCoreStatus();
}

void renderCoreUi(uint32_t now) {
    if (!uiVisible) {
        return;
    }
    if (!ensureCoreCanvas()) {
        return;
    }
    const bool approvalPrompt = g_coreApprovalMask != 0;
    const bool fullRedraw = !g_coreSurfaceReady ||
                            g_coreRenderedApprovalPrompt != approvalPrompt ||
                            g_coreRenderedSettings != g_settingsOpen ||
                            g_coreRenderedSwipeGuide != g_coreSwipeGuideVisible ||
                            (!g_settingsOpen &&
                             g_coreRenderedActionLayer != g_actionLayer);
    const bool settingsStateChanged = !g_coreSettingsSnapshotValid ||
        g_coreRenderedConnected != static_cast<bool>(g_connected) ||
        g_coreRenderedRestarting != (g_restartAt != 0) ||
        g_coreRenderedDeviceSlot != g_deviceSlot ||
        g_coreRenderedPendingDeviceSlot != g_pendingDeviceSlot ||
        g_coreRenderedVolume != g_seVolume ||
        g_coreRenderedLightsEnabled != g_lightsEnabled ||
        g_coreRenderedBattery != g_batteryLevel ||
        g_coreRenderedCharging != g_isCharging;

    // Agent RPC and breathing effects continue arriving after pairing, but
    // none of them are visible in Settings. Do not clear and repaint the
    // settings body for those unrelated updates; doing so caused a rapid
    // full-panel flicker while Codex was connected.
    if (!approvalPrompt && g_settingsOpen && !fullRedraw &&
        !settingsStateChanged) {
        g_lastUiDraw = now;
        g_uiDirty = false;
        return;
    }
    g_coreCanvas.startWrite();
    if (approvalPrompt) {
        drawCoreApprovalPrompt(now);
    } else if (g_settingsOpen) {
        renderCoreSettings(fullRedraw);
        g_coreSettingsSnapshotValid = true;
        g_coreRenderedConnected = g_connected;
        g_coreRenderedRestarting = g_restartAt != 0;
        g_coreRenderedDeviceSlot = g_deviceSlot;
        g_coreRenderedPendingDeviceSlot = g_pendingDeviceSlot;
        g_coreRenderedVolume = g_seVolume;
        g_coreRenderedLightsEnabled = g_lightsEnabled;
        g_coreRenderedBattery = g_batteryLevel;
        g_coreRenderedCharging = g_isCharging;
    } else {
        if (fullRedraw) {
            g_coreCanvas.fillScreen(TFT_BLACK);
        }
        drawCoreHeader();
        if (g_actionLayer) {
            if (g_coreSwipeGuideVisible) {
                drawCoreSwipeGuide();
            } else {
                for (int slot = 0; slot < kAgentCount; ++slot) {
                    if (slot == kCoreSwipePadSlot) {
                        drawCoreSwipePadTile();
                    } else {
                        drawCoreActionTile(slot);
                    }
                }
            }
        } else {
            for (int i = 0; i < kAgentCount; ++i) {
                drawCoreAgentTile(i, now);
            }
        }
        drawCoreStatus();
    }
    g_coreCanvas.endWrite();
    // Present only the completed frame. The physical LCD never sees the
    // intermediate background clears and tile-by-tile repaint sequence.
    g_coreCanvas.pushSprite(&M5.Display, 0, 0);
    g_coreSurfaceReady = true;
    g_coreRenderedApprovalPrompt = approvalPrompt;
    g_coreRenderedSettings = g_settingsOpen;
    g_coreRenderedActionLayer = g_actionLayer;
    g_coreRenderedSwipeGuide = g_coreSwipeGuideVisible;
    g_lastUiDraw = now;
    g_uiDirty = false;
}

int hitTestCoreMain(int x, int y) {
    if (coreLayerRect().contains(x, y)) {
        return kCoreTouchLayer;
    }
    if (coreMicRect().contains(x, y)) {
        return kTouchMic;
    }
    if (coreSettingsRect().contains(x, y)) {
        return kTouchSettings;
    }
    if (g_actionLayer) {
        for (int slot = 0; slot < kAgentCount; ++slot) {
            if (!coreTileRect(slot).contains(x, y)) {
                continue;
            }
            if (slot == kCoreSwipePadSlot) {
                return kCoreTouchSwipePad;
            }
            return kCoreActionBySlot[slot];
        }
    } else {
        for (int agent = 0; agent < kAgentCount; ++agent) {
            if (coreTileRect(agent).contains(x, y)) {
                return agent;
            }
        }
    }
    return -1;
}

int hitTestCoreSettings(int x, int y) {
    if (coreSettingsCloseRect().contains(x, y)) {
        return kTouchSettingsBack;
    }
    for (int i = 0; i < 3; ++i) {
        if (coreSlotRect(i).contains(x, y)) {
            return kTouchSlot1 + i;
        }
    }
    if (corePairRect().contains(x, y)) {
        return kTouchPair;
    }
    if (coreVolumeRect().contains(x, y)) {
        return kTouchVolume;
    }
    if (coreLightsRect().contains(x, y)) {
        return kCoreTouchLights;
    }
    return -1;
}

void updateCoreVolume(int x) {
    const int bounded = std::max(25, std::min(295, x));
    g_seVolume = static_cast<uint8_t>((bounded - 25) * 255 / 270);
    M5.Speaker.setVolume(g_seVolume);
    g_uiDirty = true;
}

void releaseCoreTileIfNeeded() {
    if (g_activeTouch < 0 || g_activeTouch >= kAgentCount) {
        g_activeTouch = -1;
        return;
    }
    if (g_touchActionLayer && g_activeTouch < kActionCount) {
        sendOuterActionEvent(g_activeTouch, false);
    } else if (!g_touchActionLayer) {
        sendAgentEvent(g_activeTouch, false);
    }
    g_activeTouch = -1;
}

int coreSwipeDirectionForDelta(int dx, int dy) {
    const int absoluteX = std::abs(dx);
    const int absoluteY = std::abs(dy);
    if (std::max(absoluteX, absoluteY) < kCoreSwipeThresholdPx) {
        return -1;
    }
    if (absoluteX > absoluteY) {
        return dx < 0 ? 2 : 3;  // left : right
    }
    return dy < 0 ? 0 : 1;      // up : down
}

void sendCoreSwipeAction(int direction) {
    if (direction < 0 || direction >= 4 || !g_connected) {
        if (!g_connected) {
            playSe(360.0f, 90);
            Serial.println("[app] swipe ignored while disconnected");
        }
        return;
    }
    static constexpr const char* names[4] = {
        "up", "down", "left", "right",
    };
    const std::uint16_t angle = kCoreSwipeAnglesPermille[direction];
    // Cross the app's 0.5 activation threshold, then return to the 0.1 center
    // dead zone. The center report rearms the same direction for the next
    // swipe, matching a physical Codex Micro stick flick.
    sendJoystickEvent(angle, 1000);
    delay(24);
    sendJoystickEvent(0, 0);
    playSe(760.0f + direction * 105.0f, 46);
    Serial.printf("[app] swipe direction=%s joystick-angle=%u.%03u\n",
                  names[direction],
                  static_cast<unsigned>(angle / 1000),
                  static_cast<unsigned>(angle % 1000));
}

void toggleCoreLayer() {
    releaseCoreTileIfNeeded();
    resetCoreSwipeGesture(true);
    g_actionLayer = !g_actionLayer;
    g_selectionAnimating = false;
    playSe(g_actionLayer ? 1120.0f : 680.0f, 48);
    g_uiDirty = true;
    renderCoreUi(millis());
    Serial.printf("[app] CoreS3 layer=%s\n",
                  g_actionLayer ? "action" : "agent");
}

void setCoreMicLatched(bool enabled, bool feedback, bool readyToSend) {
    if (enabled == g_coreMicLatched) {
        if (!enabled && !readyToSend && g_coreVoiceReadyToSend) {
            g_coreVoiceReadyToSend = false;
            g_uiDirty = true;
        }
        return;
    }
    if (enabled && !g_connected) {
        if (feedback) {
            playSe(360.0f, 90);
        }
        Serial.println("[app] MIC start ignored while disconnected");
        return;
    }
    sendMicEvent(enabled);
    g_coreMicLatched = enabled;
    g_coreVoiceReadyToSend = !enabled && readyToSend;
    g_coreMicStartedAt = enabled ? millis() : 0;
    if (feedback) {
        playMicSe(enabled);
    }
    g_uiDirty = true;
    Serial.printf("[app] CoreS3 microphone=%s\n",
                  enabled ? "started" : "stopped");
}

bool sendCoreVoicePrompt() {
    if (!g_coreVoiceReadyToSend) {
        Serial.println("[app] voice send ignored; stop recording with a tap first");
        return false;
    }
    if (!g_connected) {
        playSe(360.0f, 90);
        Serial.println("[app] voice send ignored while disconnected");
        return false;
    }

    // ACT12 is the dedicated CODEX key. Releasing the paired microphone keys
    // only finishes dictation; this separate click submits the prepared prompt.
    sendActionEvent(12, true);
    delay(24);
    sendActionEvent(12, false);
    g_coreVoiceReadyToSend = false;
    playSe(987.77f, 82);
    g_uiDirty = true;
    Serial.println("[app] CoreS3 voice prompt sent with ACT12 CODEX key");
    return true;
}

void handleCoreSettingsTouch(const m5::Touch_Class::touch_detail_t& touch) {
    if (touch.wasPressed()) {
        g_activeTouch = hitTestCoreSettings(touch.x, touch.y);
        if (g_activeTouch == kTouchVolume) {
            updateCoreVolume(touch.x);
        }
        g_uiDirty = true;
    }
    if (touch.isPressed() && g_activeTouch == kTouchVolume) {
        updateCoreVolume(touch.x);
    }
    if (!touch.wasReleased() || g_activeTouch < 0) {
        return;
    }
    const int releasedTarget = hitTestCoreSettings(touch.x, touch.y);
    if (releasedTarget == g_activeTouch) {
        if (g_activeTouch == kTouchSettingsBack) {
            g_settingsOpen = false;
            playSe(540.0f);
        } else if (g_activeTouch >= kTouchSlot1 &&
                   g_activeTouch <= kTouchSlot3) {
            g_pendingDeviceSlot = 1 + g_activeTouch - kTouchSlot1;
            playSe(760.0f + g_pendingDeviceSlot * 90.0f);
        } else if (g_activeTouch == kTouchPair) {
            playSe(1100.0f, 55);
            beginPairing();
        } else if (g_activeTouch == kTouchVolume) {
            saveSeVolume();
            playSe(980.0f, 70);
        } else if (g_activeTouch == kCoreTouchLights) {
            g_lightsEnabled = !g_lightsEnabled;
            saveLightsEnabled();
            if (g_lightsEnabled) {
                g_coreAgentLedLastUpdateAt = 0;
            } else {
                clearCoreAgentLeds();
            }
            playSe(g_lightsEnabled ? 1040.0f : 520.0f, 55);
            Serial.printf("[app] body lights=%s\n",
                          g_lightsEnabled ? "on" : "off");
        }
    }
    g_activeTouch = -1;
    g_uiDirty = true;
    renderCoreUi(millis());
}

void handleCoreTouch() {
    const auto touch = M5.Touch.getDetail();
    if (g_coreApprovalMask != 0) {
        if (touch.wasPressed()) {
            g_activeTouch = coreApprovalDecisionLocked(millis())
                                ? -1
                                : hitTestCoreApproval(touch.x, touch.y);
            g_uiDirty = true;
            renderCoreUi(millis());
            return;
        }
        if (!touch.wasReleased()) {
            return;
        }
        const int pressedTarget = g_activeTouch;
        const int releasedTarget = hitTestCoreApproval(touch.x, touch.y);
        g_activeTouch = -1;
        if (pressedTarget == releasedTarget) {
            if (pressedTarget == kCoreTouchApprovalAccept) {
                sendCoreApprovalDecision(true, "screen");
            } else if (pressedTarget == kCoreTouchApprovalReject) {
                sendCoreApprovalDecision(false, "screen");
            }
        }
        g_uiDirty = true;
        renderCoreUi(millis());
        return;
    }
    if (g_settingsOpen) {
        handleCoreSettingsTouch(touch);
        return;
    }

    if (touch.wasPressed()) {
        g_activeTouch = hitTestCoreMain(touch.x, touch.y);
        g_touchActionLayer = g_actionLayer;
        if (g_activeTouch == kCoreTouchSwipePad && g_actionLayer) {
            g_coreSwipeTracking = true;
            g_coreSwipeGuideVisible = false;
            g_coreSwipeStartedAt = millis();
            g_coreSwipeStartX = touch.x;
            g_coreSwipeStartY = touch.y;
            g_coreSwipeDirection = -1;
            Serial.println("[app] swipe pad hold started");
        } else if (g_activeTouch >= 0 && g_activeTouch < kAgentCount) {
            if (g_touchActionLayer && g_activeTouch < kActionCount) {
                g_selectedAction = g_activeTouch;
                sendOuterActionEvent(g_activeTouch, true);
                playOuterActionPressSe(g_activeTouch);
            } else if (!g_touchActionLayer) {
                g_selectedAgent = g_activeTouch;
                g_selectionAnimating = false;
                sendAgentEvent(g_activeTouch, true);
                playSe(820.0f + g_activeTouch * 55.0f);
            }
        }
        g_uiDirty = true;
    }

    if (g_coreSwipeTracking) {
        const uint32_t now = millis();
        if (touch.isPressed()) {
            if (!g_coreSwipeGuideVisible &&
                now - g_coreSwipeStartedAt >= kCoreSwipeHoldMs) {
                g_coreSwipeGuideVisible = true;
                g_coreSwipeDirection = coreSwipeDirectionForDelta(
                    touch.x - g_coreSwipeStartX,
                    touch.y - g_coreSwipeStartY);
                g_coreSurfaceReady = false;
                g_uiDirty = true;
                Serial.println("[app] swipe pad armed");
            } else if (g_coreSwipeGuideVisible) {
                const int direction = coreSwipeDirectionForDelta(
                    touch.x - g_coreSwipeStartX,
                    touch.y - g_coreSwipeStartY);
                if (direction != g_coreSwipeDirection) {
                    g_coreSwipeDirection = direction;
                    g_uiDirty = true;
                }
            }
            return;
        }
        if (touch.wasReleased()) {
            const int direction = g_coreSwipeGuideVisible
                ? coreSwipeDirectionForDelta(touch.x - g_coreSwipeStartX,
                                             touch.y - g_coreSwipeStartY)
                : -1;
            const bool shouldSend = g_coreSwipeGuideVisible && direction >= 0;
            resetCoreSwipeGesture(true);
            g_activeTouch = -1;
            if (shouldSend) {
                sendCoreSwipeAction(direction);
            } else {
                Serial.println("[app] swipe pad released without action");
            }
            g_uiDirty = true;
            renderCoreUi(millis());
            return;
        }
        return;
    }

    if (!touch.wasReleased() || g_activeTouch < 0) {
        return;
    }
    const int pressedTarget = g_activeTouch;
    const int releasedTarget = hitTestCoreMain(touch.x, touch.y);
    if (pressedTarget < kAgentCount) {
        releaseCoreTileIfNeeded();
    } else {
        g_activeTouch = -1;
    }
    if (releasedTarget == pressedTarget) {
        if (pressedTarget == kCoreTouchLayer) {
            toggleCoreLayer();
        } else if (pressedTarget == kTouchMic) {
            if (g_coreMicLatched) {
                setCoreMicLatched(false, true, true);
            } else {
                setCoreMicLatched(true, true);
            }
        } else if (pressedTarget == kTouchSettings) {
            setCoreMicLatched(false, false);
            g_settingsOpen = true;
            g_pendingDeviceSlot = g_deviceSlot;
            playSe(760.0f);
        }
    }
    g_uiDirty = true;
    renderCoreUi(millis());
}

void drawCoreSplashFrame(float progress) {
    if (!ensureCoreCanvas()) {
        return;
    }
    const float eased = 1.0f - std::pow(1.0f - clamp01(progress), 3.0f);
    const float fade = clamp01(progress / 0.58f);
    const float pulse = 0.76f + 0.24f * std::sin(progress * PI * 4.0f);
    const uint16_t purple = scaledColor(0x9D74FF, fade * pulse);
    const uint16_t cyan = scaledColor(0x33C4E8, fade);
    const uint16_t muted = scaledColor(0xAAB4C8, fade * 0.86f);
    const int cx = g_coreCanvas.width() / 2;
    constexpr int cy = 104;

    g_coreCanvas.startWrite();
    g_coreCanvas.fillScreen(TFT_BLACK);
    const int ringRadius = 29 + static_cast<int>(31.0f * eased);
    g_coreCanvas.drawCircle(cx, cy, ringRadius + 6,
                            scaledColor(0x34284F, fade));
    g_coreCanvas.drawCircle(cx, cy, ringRadius + 5,
                            scaledColor(0x34284F, fade));
    g_coreCanvas.drawCircle(cx, cy, ringRadius, purple);
    g_coreCanvas.drawCircle(cx, cy, ringRadius - 1, purple);
    g_coreCanvas.drawCircle(cx, cy, ringRadius - 2, purple);

    for (int i = 0; i < kAgentCount; ++i) {
        const float revealAt = 0.08f + i * 0.07f;
        if (progress < revealAt) {
            continue;
        }
        const float dotFade = clamp01((progress - revealAt) / 0.18f);
        const float angle = (-90.0f + i * 60.0f) * PI / 180.0f;
        const int x = cx + static_cast<int>(std::cos(angle) * 82.0f);
        const int y = cy + static_cast<int>(std::sin(angle) * 82.0f);
        g_coreCanvas.fillCircle(x, y, 3 + static_cast<int>(3.0f * dotFade),
                                scaledColor(i % 2 == 0 ? 0x9D74FF : 0x33C4E8,
                                            dotFade));
    }

    g_coreCanvas.setTextDatum(middle_center);
    g_coreCanvas.setFont(&fonts::Orbitron_Light_24);
    g_coreCanvas.setTextSize(0.70f);
    g_coreCanvas.setTextColor(purple, TFT_BLACK);
    g_coreCanvas.drawString("VibeStackChan", cx, 106);
    g_coreCanvas.setFont(&fonts::DejaVu18);
    g_coreCanvas.setTextSize(0.50f);
    g_coreCanvas.setTextColor(muted, TFT_BLACK);
    g_coreCanvas.drawString("AI CONTROL SURFACE", cx, 151);

    constexpr int barX = 50;
    constexpr int barY = 204;
    constexpr int barWidth = 220;
    constexpr int barHeight = 8;
    const int ready = static_cast<int>(std::lround(eased * 100.0f));
    char readyLabel[20];
    std::snprintf(readyLabel, sizeof(readyLabel), "PREPARING  %d%%", ready);
    g_coreCanvas.setTextSize(0.48f);
    g_coreCanvas.setTextColor(cyan, TFT_BLACK);
    g_coreCanvas.drawString(readyLabel, cx, 185);
    g_coreCanvas.fillRoundRect(barX, barY, barWidth, barHeight, 4,
                               scaledColor(0x566071, fade * 0.65f));
    const int fillWidth = static_cast<int>(barWidth * eased);
    if (fillWidth > 0) {
        g_coreCanvas.fillRoundRect(barX, barY, fillWidth, barHeight, 4, cyan);
    }
    g_coreCanvas.endWrite();
    g_coreCanvas.pushSprite(&M5.Display, 0, 0);
}

void showCoreSplashScreen() {
    constexpr uint32_t kAnimationMs = 1450;
    constexpr uint32_t kHoldMs = 320;
    struct Note {
        uint32_t atMs;
        float frequency;
        uint32_t durationMs;
    };
    static constexpr Note notes[] = {
        {80, 329.63f, 70},
        {235, 493.88f, 75},
        {410, 659.25f, 85},
        {630, 783.99f, 95},
        {900, 987.77f, 210},
    };

    drawCoreSplashFrame(0.0f);
    const uint32_t startedAt = millis();
    size_t nextNote = 0;
    while (millis() - startedAt < kAnimationMs) {
        const uint32_t elapsed = millis() - startedAt;
        while (nextNote < sizeof(notes) / sizeof(notes[0]) &&
               elapsed >= notes[nextNote].atMs) {
            playSe(notes[nextNote].frequency, notes[nextNote].durationMs);
            ++nextNote;
        }
        drawCoreSplashFrame(static_cast<float>(elapsed) /
                            static_cast<float>(kAnimationMs));
        delay(24);
    }
    drawCoreSplashFrame(1.0f);
    delay(kHoldMs);
}

}  // namespace

bool begin() {
    if (appActive) {
        return true;
    }

    setTransportPowerProfile(TransportPowerProfile::Active);

    loadPreferences();
    const uint8_t currentBrightness = M5.Display.getBrightness();
    if (currentBrightness > 0) {
        g_coreDisplayBrightness = currentBrightness;
    }
    M5.Speaker.setVolume(g_seVolume);
    updateBattery(false);
    uiVisible = true;
    g_settingsOpen = false;
    g_actionLayer = false;
    g_activeTouch = -1;
    g_selectionAnimating = false;
    g_coreMicLatched = false;
    g_coreVoiceReadyToSend = false;
    g_coreMicStartedAt = 0;
    resetCoreBackTouchGesture();
    resetCoreSwipeGesture(false);
    g_coreRenderedSwipeGuide = false;
    resetCoreApprovalMotion(false);
    g_coreAgentLedLastUpdateAt = 0;
    g_coreAgentLedsActive = false;
    for (int i = 0; i < kAgentCount; ++i) {
        g_coreAgentLedColors[i] = 0;
    }
    g_coreSurfaceReady = false;
    loadCoreServoCalibration();
    showCoreSplashScreen();

    if (transportResident && g_rpcQueue != nullptr &&
        NimBLEDevice::isInitialized() && g_bleStarted) {
        if (!resumeAdvertising()) {
            Serial.println("[app] resident BLE resume failed");
            return false;
        }
        appActive = true;
        uiVisible = true;
        g_uiDirty = true;
        Serial.println("[app] application resumed without HID disconnect");
        return true;
    }

    drainRpcQueue();
    if (g_rpcQueue != nullptr) {
        vQueueDelete(g_rpcQueue);
    }
    g_rpcQueue = xQueueCreate(6, sizeof(char*));
    if (g_rpcQueue == nullptr) {
        Serial.println("[app] RPC queue allocation failed");
        return false;
    }

    renderCoreUi(millis());
    if (!startBleTransport()) {
        stopBleTransport();
        vQueueDelete(g_rpcQueue);
        g_rpcQueue = nullptr;
        return false;
    }

    appActive = true;
    uiVisible = true;
    g_uiDirty = true;
    Serial.println("[app] application started");
    return true;
}

void update(uint32_t loopNow) {
    (void)loopNow;
    const uint32_t now = millis();
    if (!transportResident || g_rpcQueue == nullptr) {
        return;
    }

    if (appActive && uiVisible) {
        handleCoreTouch();
    }

    char* message = nullptr;
    while (xQueueReceive(g_rpcQueue, &message, 0) == pdTRUE) {
        processRpc(message);
        std::free(message);
        message = nullptr;
    }

    // processRpc() updates g_agents from v.oai.thstatus. Evaluate the final
    // state after draining this loop iteration so orange awaiting-* edges are
    // observed immediately without retriggering for every status refresh.
    handleCoreApprovalTestSerial(now);
    updateCoreApprovalState(now);
    if (g_coreApprovalDecisionSentAt != 0 &&
        now - g_coreApprovalDecisionSentAt >=
            kCoreApprovalDecisionLockMs) {
        g_coreApprovalDecisionSentAt = 0;
        g_uiDirty = true;
    }

    if (appActive && g_pairingSuccessPending) {
        g_pairingSuccessPending = false;
        playSe(1320.0f, 95);
    }

    if (appActive && g_restartAt != 0 &&
        static_cast<std::int32_t>(now - g_restartAt) >= 0) {
        // Rebuild only the BLE transport so the display and servos stay live.
        restartBleTransportForPairing();
    }
    if (g_vibrationOffAt != 0 &&
        static_cast<std::int32_t>(now - g_vibrationOffAt) >= 0) {
        M5.Power.setVibration(0);
        g_vibrationOffAt = 0;
    }
    if (now - g_lastBatteryUpdate >= kBatteryUpdatePeriodMs) {
        updateBattery(true);
    }
    if (appActive && !g_connected &&
        (g_coreMicLatched || g_coreVoiceReadyToSend)) {
        g_coreMicLatched = false;
        g_coreVoiceReadyToSend = false;
        g_coreMicStartedAt = 0;
        g_uiDirty = true;
        Serial.println("[app] CoreS3 microphone stopped on disconnect");
    } else if (appActive && g_coreMicLatched &&
               now - g_coreMicStartedAt >= kCoreMicAutoStopMs) {
        setCoreMicLatched(false, true, true);
        Serial.println("[app] CoreS3 microphone auto-stop after 60000ms");
    }
    if (appActive) {
        updateCoreApprovalMotion(now);
    }
    updateCoreAgentLeds(now);
    const bool animateVisibleUi =
        !g_settingsOpen &&
        uiIsAnimated();
    const std::uint32_t uiPeriod = g_selectionAnimating
                                       ? kSelectionAnimationPeriodMs
                                       : kUiAnimationPeriodMs;
    if (appActive && uiVisible &&
        (g_uiDirty || animateVisibleUi) && now - g_lastUiDraw >= uiPeriod) {
        renderCoreUi(now);
    }
}

void updateBackTouch(bool detected) {
    if (!appActive || !uiVisible || g_settingsOpen) {
        resetCoreBackTouchGesture();
        return;
    }
    const uint32_t now = millis();
    if (!detected) {
        if (g_coreBackTouchPressed) {
            if (g_coreBackTouchReleaseCandidateAt == 0) {
                g_coreBackTouchReleaseCandidateAt = now;
                return;
            }
            if (now - g_coreBackTouchReleaseCandidateAt <
                kCoreBackTouchReleaseDebounceMs) {
                return;
            }
            const bool shortTap = !g_coreBackTouchHoldFired;
            const uint32_t pressedDuration = now - g_coreBackTouchPressedAt;
            g_coreBackTouchPressed = false;
            g_coreBackTouchPressedAt = 0;
            g_coreBackTouchReleaseCandidateAt = 0;
            g_coreBackTouchReleasedAt = now;
            Serial.printf("[touch] rear release duration=%lums hold=%d\n",
                          static_cast<unsigned long>(pressedDuration),
                          g_coreBackTouchHoldFired ? 1 : 0);
            if (shortTap && g_coreApprovalMask == 0) {
                if (!g_coreMicLatched) {
                    setCoreMicLatched(true, true);
                    Serial.println("[app] CoreS3 rear tap microphone started");
                } else {
                    setCoreMicLatched(false, true, true);
                    Serial.println("[app] CoreS3 rear tap microphone stopped; ready to send");
                }
            }
            g_coreBackTouchHoldFired = false;
            return;
        }
        if (g_coreBackTouchArmed) {
            return;
        }
        if (g_coreBackTouchReleasedAt == 0) {
            g_coreBackTouchReleasedAt = now;
            return;
        }
        if (now - g_coreBackTouchReleasedAt >= kCoreBackTouchReleaseMs) {
            g_coreBackTouchArmed = true;
            g_coreBackTouchReleasedAt = 0;
        }
        return;
    }
    g_coreBackTouchReleaseCandidateAt = 0;
    g_coreBackTouchReleasedAt = 0;
    if (!g_coreBackTouchPressed) {
        if (!g_coreBackTouchArmed) {
            return;
        }
        g_coreBackTouchArmed = false;
        g_coreBackTouchPressed = true;
        g_coreBackTouchHoldFired = false;
        g_coreBackTouchPressedAt = now;
        Serial.printf("[touch] rear press mic=%d send_ready=%d approval=%u\n",
                      g_coreMicLatched ? 1 : 0,
                      g_coreVoiceReadyToSend ? 1 : 0,
                      static_cast<unsigned>(g_coreApprovalMask));
        if (g_coreApprovalMask != 0) {
            g_coreBackTouchHoldFired = true;
            sendCoreApprovalDecision(true, "rear_touch");
            g_uiDirty = true;
            renderCoreUi(millis());
        }
        return;
    }
    if (g_coreBackTouchHoldFired ||
        now - g_coreBackTouchPressedAt < kCoreBackTouchHoldMs) {
        return;
    }
    g_coreBackTouchHoldFired = true;
    if (sendCoreVoicePrompt()) {
        Serial.println("[app] CoreS3 rear hold sent stopped voice input");
    }
}

void setDisplayEnabled(bool enabled) {
    if (uiVisible == enabled) {
        return;
    }
    if (!enabled) {
        setCoreMicLatched(false, false);
        resetCoreBackTouchGesture();
        resetCoreSwipeGesture(false);
        uiVisible = false;
        forceCoreAgentLedsOff();
        g_coreAgentLedLastUpdateAt = millis();
        setTransportPowerProfile(TransportPowerProfile::ScreenOff);
        M5.Display.setBrightness(0);
        M5.Display.sleep();
        Serial.println("[display] screen and body lights off");
        return;
    }

    M5.Display.wakeup();
    M5.Display.setBrightness(g_coreDisplayBrightness);
    M5StackChan.setServoPowerEnabled(true);
    g_corePowerOffPrepared = false;
    uiVisible = true;
    g_coreSurfaceReady = false;
    g_coreSettingsSnapshotValid = false;
    g_coreAgentLedLastUpdateAt = 0;
    g_uiDirty = true;
    setTransportPowerProfile(TransportPowerProfile::Active);
    renderCoreUi(millis());
    Serial.println("[display] screen on");
}

bool displayEnabled() {
    return uiVisible;
}

void handlePowerButtonClick() {
    if (g_corePowerOffPrepared) {
        // AXP2101 reports a click after its long-press IRQ. Consuming that
        // trailing event prevents the just-cleared RGB pixels from being
        // restored while the PMIC is completing hardware shutdown.
        g_corePowerOffPrepared = false;
        Serial.println("[power] trailing click after long press ignored");
        return;
    }
    setDisplayEnabled(!displayEnabled());
}

void prepareForPowerOff() {
    setCoreMicLatched(false, false);
    resetCoreBackTouchGesture();
    resetCoreSwipeGesture(false);
    uiVisible = false;
    M5.Power.setVibration(0);
    M5StackChan.Motion.stop();
    M5StackChan.setServoPowerEnabled(false);
    g_corePowerOffPrepared = true;

    // RGB is driven by the Stack-chan body's IO expander. Its last value
    // survives after the CoreS3 PMIC cuts power, so commit black twice before
    // shutdown instead of relying on CoreS3 power loss to clear the pixels.
    forceCoreAgentLedsOff();
    delay(20);
    forceCoreAgentLedsOff();
    g_coreAgentLedLastUpdateAt = millis();

    M5.Display.setBrightness(0);
    M5.Display.sleep();
    Serial.println("[power] long press: outputs cleared for shutdown");
}

void setTransportPowerProfile(TransportPowerProfile profile) {
    const auto previous = static_cast<TransportPowerProfile>(
        transportPowerProfile);
    transportPowerProfile = profile;
    if (previous == profile) {
        return;
    }
    const ConnectionPowerParams params = connectionPowerParams(profile);
    Serial.printf("[app] HID power profile=%s connected=%d\n",
                  params.name, hidConnected() ? 1 : 0);
    if (hidConnected()) {
        requestConnectionPowerProfile(g_hidConnectionHandle);
    }
}

bool hidConnected() {
    return g_connected &&
           g_hidConnectionHandle != BLE_HS_CONN_HANDLE_NONE;
}

void onHidSubscribed(NimBLEConnInfo& connection) {
    requestConnectionPowerProfile(connection.getConnHandle());
}

void onHidConnParamsUpdated(NimBLEConnInfo& connection) {
    if (connection.getConnHandle() != g_hidConnectionHandle) {
        return;
    }
    const ConnectionPowerParams requested = connectionPowerParams(
        static_cast<TransportPowerProfile>(transportPowerProfile));
    Serial.printf("[app] HID power negotiated profile=%s handle=%u interval=%u(%.1fms) latency=%u timeout=%u(%ums)\n",
                  requested.name,
                  static_cast<unsigned>(connection.getConnHandle()),
                  static_cast<unsigned>(connection.getConnInterval()),
                  static_cast<double>(connection.getConnInterval()) * 1.25,
                  static_cast<unsigned>(connection.getConnLatency()),
                  static_cast<unsigned>(connection.getConnTimeout()),
                  static_cast<unsigned>(connection.getConnTimeout()) * 10U);
}

}  // namespace VibeStackChanApp
