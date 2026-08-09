#include <Arduino.h>
#include <ArduinoJson.h>
#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <Preferences.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../HidProtocol.h"
#include "../Sound.h"

namespace {

// -----------------------------------------------------------------------------
// UI geometry, timing, and persistent-setting keys
// -----------------------------------------------------------------------------

// Shared control and protocol constants.
constexpr int kAgentCount = 6;
constexpr int kActionCount = 5;
constexpr int kOkAction = 1;
constexpr int kNgAction = 2;
constexpr int kScreenCenter = 233;
constexpr int kAgentOrbitRadius = 160;
constexpr int kAgentButtonRadius = 55;
constexpr int kMicButtonRadius = 67;
constexpr int kSettingsX = 72;
constexpr int kSettingsY = 370;
constexpr int kSettingsRadius = 22;
constexpr int kSettingsVisualRadius = 16;
constexpr int kSettingsCloseX = 365;
constexpr int kSettingsCloseY = 55;
constexpr int kSettingsCloseRadius = 25;
constexpr int kSettingsSliderLeft = 103;
constexpr int kSettingsSliderRight = 363;
constexpr std::uint32_t kLeftPhysicalButtonColor = 0xFFAC28;
constexpr std::uint32_t kRightPhysicalButtonColor = 0x2D8CFF;
constexpr std::uint32_t kButtonChordGraceMs = 90;
constexpr std::uint32_t kPhysicalMicHoldMs = 350;
constexpr std::uint32_t kUiAnimationPeriodMs = 80;
constexpr std::uint32_t kSelectionAnimationPeriodMs = 16;
constexpr std::uint32_t kSelectionAnimationBaseMs = 88;
constexpr std::uint32_t kBatteryUpdatePeriodMs = 30000;
constexpr char kPreferencesNamespace[] = "vibestackchan";
constexpr char kDeviceSlotKey[] = "device-slot";
constexpr char kSeVolumeKey[] = "se-volume";
constexpr char kVibrationStrengthKey[] = "vibe-strength";
constexpr char kAgentStateVibeKey[] = "state-vibe";
constexpr char kLightsEnabledKey[] = "lights";

// Hit-test results share one integer space. Non-negative values below
// kAgentCount are outer-ring items; the remaining values identify fixed UI
// controls such as the microphone and settings widgets.
constexpr int kTouchMic = kAgentCount;
constexpr int kTouchSettings = kAgentCount + 1;
constexpr int kTouchSettingsBack = kAgentCount + 2;
constexpr int kTouchSlot1 = kAgentCount + 3;
constexpr int kTouchSlot2 = kAgentCount + 4;
constexpr int kTouchSlot3 = kAgentCount + 5;
constexpr int kTouchPair = kAgentCount + 6;
constexpr int kTouchVolume = kAgentCount + 7;
constexpr int kTouchVibrationStrength = kAgentCount + 8;
constexpr int kTouchAgentStateVibe = kAgentCount + 9;

struct AgentState {
    std::uint32_t color = 0;
    float brightness = 0.0f;
    int effect = 0;
    float speed = 0.0f;
};

struct AmbientState {
    std::uint32_t color = 0x304FFE;
    float brightness = 0.25f;
    int effect = 0;
    float speed = 0.4f;
};

// -----------------------------------------------------------------------------
// Runtime state
// -----------------------------------------------------------------------------

// Visual state received from the host. Agent colors communicate per-chat state;
// ambient data is retained for protocol compatibility but is not drawn at the bezel.
std::array<AgentState, kAgentCount> g_agents;
AmbientState g_ambient;
String g_focusedApp;

// BLE objects are created once during setup and remain valid for the lifetime
// of the firmware. RPC messages are moved out of the BLE callback through a
// FreeRTOS queue so JSON processing never blocks the NimBLE task.
NimBLEServer* g_server = nullptr;
NimBLEHIDDevice* g_hid = nullptr;
NimBLECharacteristic* g_vendorInput = nullptr;
NimBLECharacteristic* g_vendorOutput = nullptr;
QueueHandle_t g_rpcQueue = nullptr;

volatile bool g_connected = false;
volatile bool g_uiDirty = true;
volatile bool g_pairingSuccessPending = false;
bool g_bleStarted = false;
volatile std::uint16_t g_hidConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
String g_rxBuffer;

// Input state is intentionally explicit because a physical-button press may
// become a single action, a long press, or a two-button layer-switch chord.
int g_activeTouch = -1;
std::uint32_t g_vibrationOffAt = 0;
std::uint32_t g_lastUiDraw = 0;
std::uint32_t g_lastBatteryUpdate = 0;
std::uint8_t g_batteryLevel = 100;
bool g_isCharging = false;
bool g_settingsOpen = false;
int g_deviceSlot = 1;
int g_pendingDeviceSlot = 1;
int g_selectedAgent = 0;
int g_selectedAction = 0;
bool g_planModeEnabled = false;
float g_selectionX = 0.0f;
float g_selectionY = 0.0f;
float g_selectionFromX = 0.0f;
float g_selectionFromY = 0.0f;
float g_selectionToX = 0.0f;
float g_selectionToY = 0.0f;
float g_selectionFromAngle = 0.0f;
float g_selectionToAngle = 0.0f;
std::uint32_t g_selectionAnimationStartedAt = 0;
std::uint32_t g_selectionAnimationDurationMs = 260;
bool g_selectionAnimating = false;
int g_leftAgentPressed = -1;
bool g_leftPressedActionLayer = false;
bool g_leftPressPending = false;
std::uint32_t g_leftPressedAt = 0;
bool g_rightLongTriggered = false;
std::uint32_t g_rightPhysicalPressedAt = 0;
bool g_rightActionPending = false;
bool g_rightActionPressed = false;
std::uint32_t g_rightActionPressedAt = 0;
bool g_buttonChordActive = false;
bool g_actionLayer = false;
bool g_touchActionLayer = false;
std::uint32_t g_restartAt = 0;
char g_deviceName[24] = {};
std::uint8_t g_seVolume = 128;
std::uint8_t g_vibrationStrength = 255;
bool g_agentStateVibeEnabled = true;
bool g_lightsEnabled = true;
std::uint32_t g_lastAgentVibrationAt = 0;

std::array<int, kAgentCount> agentX{};
std::array<int, kAgentCount> agentY{};
std::array<int, kActionCount> actionX{};
std::array<int, kActionCount> actionY{};

// -----------------------------------------------------------------------------
// Preferences, sound, color, haptics, and battery helpers
// -----------------------------------------------------------------------------

void updateDeviceName() {
    std::snprintf(g_deviceName, sizeof(g_deviceName), "VibeStackChan #%d",
                  g_deviceSlot);
}

void loadPreferences() {
    Preferences preferences;
    preferences.begin(kPreferencesNamespace, true);
    g_deviceSlot = preferences.getUChar(kDeviceSlotKey, 1);
    g_seVolume = preferences.getUChar(kSeVolumeKey, 128);
    g_vibrationStrength = preferences.getUChar(kVibrationStrengthKey, 255);
    g_agentStateVibeEnabled = preferences.getBool(kAgentStateVibeKey, true);
    g_lightsEnabled = preferences.getBool(kLightsEnabledKey, true);
    preferences.end();
    if (g_deviceSlot < 1 || g_deviceSlot > 3) {
        g_deviceSlot = 1;
    }
    g_pendingDeviceSlot = g_deviceSlot;
    updateDeviceName();
}

void saveDeviceSlot(int slot) {
    Preferences preferences;
    preferences.begin(kPreferencesNamespace, false);
    preferences.putUChar(kDeviceSlotKey, static_cast<std::uint8_t>(slot));
    preferences.end();
}

void saveSeVolume() {
    Preferences preferences;
    preferences.begin(kPreferencesNamespace, false);
    preferences.putUChar(kSeVolumeKey, g_seVolume);
    preferences.end();
}

void saveFeedbackSettings() {
    Preferences preferences;
    preferences.begin(kPreferencesNamespace, false);
    preferences.putUChar(kVibrationStrengthKey, g_vibrationStrength);
    preferences.putBool(kAgentStateVibeKey, g_agentStateVibeEnabled);
    preferences.end();
}

void saveLightsEnabled() {
    Preferences preferences;
    preferences.begin(kPreferencesNamespace, false);
    preferences.putBool(kLightsEnabledKey, g_lightsEnabled);
    preferences.end();
}

void playSe(float frequency = 880.0f, std::uint32_t durationMs = 35) {
    sound::playSquare(frequency, durationMs, g_seVolume);
}

void playMicSe(bool pressed, std::uint8_t tempoMultiplier = 1) {
    sound::playEffect(
        pressed ? sound::Effect::Start : sound::Effect::StartReverse,
        g_seVolume, tempoMultiplier);
}

void renderUi(std::uint32_t now);

float clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
}

std::uint16_t scaledColor(std::uint32_t packed, float brightness) {
    const float scale = clamp01(brightness);
    const auto r = static_cast<std::uint8_t>(((packed >> 16) & 0xFF) * scale);
    const auto g = static_cast<std::uint8_t>(((packed >> 8) & 0xFF) * scale);
    const auto b = static_cast<std::uint8_t>((packed & 0xFF) * scale);
    return M5.Display.color565(r, g, b);
}

float effectBrightness(int effect, float brightness, float speed, std::uint32_t now) {
    if (effect == 0 || brightness <= 0.0f) {
        return 0.0f;
    }
    if (effect == 4 || effect == 6) {
        const float hz = 0.35f + clamp01(speed) * 1.4f;
        const float phase = static_cast<float>(now % 10000) * 0.001f * hz * 2.0f * PI;
        const float low = effect == 6 ? 0.5f : 0.15f;
        return brightness * (low + (1.0f - low) * (0.5f + 0.5f * std::sin(phase)));
    }
    return brightness;
}

bool uiIsAnimated() {
    if (g_selectionAnimating) {
        return true;
    }
    for (const auto& state : g_agents) {
        if (state.effect == 4 || state.effect == 6) {
            return true;
        }
    }
    return false;
}

void vibrate(std::uint8_t strength = 120, std::uint32_t durationMs = 25) {
    if (strength == 0 || g_vibrationStrength == 0) {
        return;
    }
    const auto scaledStrength = static_cast<std::uint8_t>(std::max(
        1U, (static_cast<unsigned>(strength) * g_vibrationStrength + 127U) / 255U));
    M5.Power.setVibration(scaledStrength);
    g_vibrationOffAt = millis() + durationMs;
}

void updateBattery(bool notify) {
    const int level = M5.Power.getBatteryLevel();
    if (level >= 0 && level <= 100) {
        g_batteryLevel = static_cast<std::uint8_t>(level);
    }
    g_isCharging = M5.Power.isCharging() == m5::Power_Class::is_charging;
    if (g_hid != nullptr) {
        g_hid->setBatteryLevel(g_batteryLevel, notify && g_connected);
    }
    g_lastBatteryUpdate = millis();
    g_uiDirty = true;
}

// -----------------------------------------------------------------------------
// Host communication
// -----------------------------------------------------------------------------

// Vendor JSON-RPC messages are split into fixed-size HID reports. Byte 0 is the
// channel, byte 1 is the payload length, and bytes 2..62 contain UTF-8 JSON.
void sendFramedJson(String payload, bool appendCrlf) {
    if (!g_connected || g_vendorInput == nullptr) {
        return;
    }
    if (appendCrlf && !payload.endsWith("\r\n")) {
        payload += "\r\n";
    }

    const std::size_t total = payload.length();
    std::size_t offset = 0;
    while (offset < total) {
        const std::size_t chunk = std::min(hid_protocol::kRpcChunkLength, total - offset);
        std::uint8_t report[hid_protocol::kBleReportLength] = {};
        report[0] = hid_protocol::kChannelJsonRpc;
        report[1] = static_cast<std::uint8_t>(chunk);
        std::memcpy(&report[2], payload.c_str() + offset, chunk);
        g_vendorInput->setValue(report, sizeof(report));
        if (!g_vendorInput->notify()) {
            Serial.println("BLE notify failed");
            return;
        }
        offset += chunk;
        if (offset < total) {
            delay(8);
        }
    }
}

void sendKeyEvent(const char* key, bool pressed) {
    if (!g_connected || g_vendorInput == nullptr) {
        return;
    }

    // Match the working M5Core2 implementation byte-for-byte: one complete
    // 63-byte vendor report with CRLF included in the payload length.
    std::uint8_t report[hid_protocol::kBleReportLength] = {};
    report[0] = hid_protocol::kChannelJsonRpc;
    const int written = std::snprintf(
        reinterpret_cast<char*>(&report[2]), hid_protocol::kRpcChunkLength,
        "{\"m\":\"v.oai.hid\",\"p\":{\"k\":\"%s\",\"act\":%u}}\r\n", key, pressed ? 1U : 0U);
    if (written < 0 || written >= static_cast<int>(hid_protocol::kRpcChunkLength)) {
        Serial.println("HID event payload overflow");
        return;
    }
    report[1] = static_cast<std::uint8_t>(written);
    g_vendorInput->setValue(report, sizeof(report));
    if (!g_vendorInput->notify()) {
        Serial.printf("HID notify failed: %s\n", key);
        return;
    }
    Serial.printf("HID %s %s len=%d\n", key, pressed ? "DOWN" : "UP", written);
}

void sendJoystickEvent(std::uint16_t anglePermille, std::uint16_t distancePermille) {
    if (!g_connected || g_vendorInput == nullptr) {
        return;
    }

    anglePermille = std::min<std::uint16_t>(anglePermille, 1000);
    distancePermille = std::min<std::uint16_t>(distancePermille, 1000);

    // Codex Micro reports both values normalized to 0..1 over the dedicated
    // joystick notification channel. Keep fixed-point inputs here so the wire
    // payload is deterministic even when floating-point printf is disabled.
    std::uint8_t report[hid_protocol::kBleReportLength] = {};
    report[0] = hid_protocol::kChannelJsonRpc;
    const int written = std::snprintf(
        reinterpret_cast<char*>(&report[2]), hid_protocol::kRpcChunkLength,
        "{\"m\":\"v.oai.rad\",\"p\":{\"a\":%u.%03u,\"d\":%u.%03u}}\r\n",
        static_cast<unsigned>(anglePermille / 1000),
        static_cast<unsigned>(anglePermille % 1000),
        static_cast<unsigned>(distancePermille / 1000),
        static_cast<unsigned>(distancePermille % 1000));
    if (written < 0 || written >= static_cast<int>(hid_protocol::kRpcChunkLength)) {
        Serial.println("Joystick event payload overflow");
        return;
    }
    report[1] = static_cast<std::uint8_t>(written);
    g_vendorInput->setValue(report, sizeof(report));
    if (!g_vendorInput->notify()) {
        Serial.println("Joystick notify failed");
        return;
    }
    Serial.printf("JOYSTICK angle=%u.%03u distance=%u.%03u len=%d\n",
                  static_cast<unsigned>(anglePermille / 1000),
                  static_cast<unsigned>(anglePermille % 1000),
                  static_cast<unsigned>(distancePermille / 1000),
                  static_cast<unsigned>(distancePermille % 1000),
                  written);
}

void sendAgentEvent(int index, bool pressed) {
    char key[5];
    std::snprintf(key, sizeof(key), "AG%02d", index);
    sendKeyEvent(key, pressed);
}

void sendActionEvent(int index, bool pressed) {
    char key[6];
    std::snprintf(key, sizeof(key), "ACT%02d", index);
    sendKeyEvent(key, pressed);
}

void sendMicEvent(bool pressed) {
    sendActionEvent(10, pressed);
    // NimBLE notifications are asynchronous. Pace the paired MIC reports so
    // ACT11 cannot overwrite ACT10 in the controller buffer before delivery.
    delay(12);
    sendActionEvent(11, pressed);
}

void sendOuterActionEvent(int index, bool pressed) {
    sendActionEvent(index == 4 ? 12 : 6 + index, pressed);
}

void playOuterActionPressSe(int index) {
    if (index == kOkAction) {
        playSe(659.25f, 34);
        delay(42);
        playSe(987.77f, 86);
        return;
    }
    if (index == kNgAction) {
        playSe(392.00f, 42);
        delay(50);
        playSe(293.66f, 105);
        return;
    }
    playSe(900.0f + index * 75.0f, 40);
}

// Apply the host's compact agent-state array directly to the six ring buttons.
void applyAgentStatus(JsonVariantConst params) {
    if (!params.is<JsonArrayConst>()) {
        return;
    }
    bool anyAgentChanged = false;
    for (JsonObjectConst item : params.as<JsonArrayConst>()) {
        const int id = item["id"] | -1;
        if (id < 0 || id >= kAgentCount) {
            continue;
        }
        AgentState next;
        next.color = item["c"] | 0U;
        next.brightness = item["b"] | 0.0f;
        next.effect = item["e"] | 0;
        next.speed = item["s"] | 0.0f;
        auto& state = g_agents[id];
        const bool changed = state.color != next.color ||
                             std::abs(state.brightness - next.brightness) > 0.001f ||
                             state.effect != next.effect ||
                             std::abs(state.speed - next.speed) > 0.001f;
        anyAgentChanged = anyAgentChanged || changed;
        state = next;
    }

    // Agent-state changes use haptics only. One compact cue represents a host
    // update even when several agents change together, with rapid updates throttled.
    const std::uint32_t now = millis();
    if (anyAgentChanged && g_agentStateVibeEnabled &&
        now - g_lastAgentVibrationAt >= 120) {
        vibrate(190, 42);
        g_lastAgentVibrationAt = now;
    }
    g_uiDirty = true;
}

void applyAmbientStatus(JsonVariantConst params) {
    JsonObjectConst ambient = params["ambient"].as<JsonObjectConst>();
    if (ambient.isNull()) {
        return;
    }
    g_ambient.color = ambient["c"] | 0U;
    g_ambient.brightness = ambient["b"] | 0.0f;
    g_ambient.effect = ambient["e"] | 0;
    g_ambient.speed = ambient["s"] | 0.0f;
    g_uiDirty = true;
}

void applyFocusedApp(JsonVariantConst params) {
    const char* appName = params["appName"] | "";
    g_focusedApp = appName;
    if (g_focusedApp.length() > 18) {
        g_focusedApp = g_focusedApp.substring(0, 17) + "…";
    }
    g_uiDirty = true;
}

void sendRpcResponse(const char* method, int id) {
    JsonDocument response;
    response["id"] = id;
    response["method"] = method;

    if (std::strcmp(method, "device.status") == 0) {
        updateBattery(false);
        JsonObject result = response["result"].to<JsonObject>();
        result["version"] = hid_protocol::kFirmwareVersion;
        result["profile_index"] = 0;
        result["layer_index"] = 1;
        result["battery"] = g_batteryLevel;
        result["is_charging"] = g_isCharging;
    } else if (std::strcmp(method, "sys.version") == 0) {
        response["result"]["version"] = hid_protocol::kFirmwareVersion;
    } else {
        response["result"]["ok"] = 1;
    }

    String json;
    serializeJson(response, json);
    sendFramedJson(json, true);
    Serial.printf("RPC response: %s id=%d\n", method, id);
}

void processRpc(const char* json) {
    JsonDocument request;
    const DeserializationError error = deserializeJson(request, json);
    if (error) {
        Serial.printf("RPC parse failed: %s\n", error.c_str());
        return;
    }

    const char* method = request["method"] | request["m"] | "";
    int id = request["id"] | request["i"] | -1;
    JsonVariantConst params = request["params"];
    if (params.isNull()) {
        params = request["p"];
    }

    if (std::strcmp(method, "v.oai.thstatus") == 0) {
        applyAgentStatus(params);
    } else if (std::strcmp(method, "v.oai.rgbcfg") == 0) {
        applyAmbientStatus(params);
    } else if (std::strcmp(method, "host.focused_app") == 0) {
        applyFocusedApp(params);
    }

    if (id >= 0 && method[0] != '\0') {
        sendRpcResponse(method, id);
    }
}

// BLE output callbacks run on NimBLE's task. This class only validates and
// reassembles frames, then queues a complete JSON message for the main loop.
class RpcOutputCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        const NimBLEAttValue value = characteristic->getValue();
        const auto* data = value.data();
        const std::size_t length = value.size();
        if (data == nullptr || length < 2 || data[0] != hid_protocol::kChannelJsonRpc) {
            return;
        }

        const std::size_t chunkLength = data[1];
        if (chunkLength > hid_protocol::kRpcChunkLength || chunkLength > length - 2) {
            Serial.println("Invalid RPC chunk");
            g_rxBuffer = "";
            return;
        }
        if (g_rxBuffer.length() + chunkLength > hid_protocol::kRpcBufferLength) {
            Serial.println("RPC request too large");
            g_rxBuffer = "";
            return;
        }

        for (std::size_t i = 0; i < chunkLength; ++i) {
            g_rxBuffer += static_cast<char>(data[i + 2]);
        }

        JsonDocument probe;
        const DeserializationError parseResult = deserializeJson(probe, g_rxBuffer);
        if (parseResult == DeserializationError::IncompleteInput) {
            return;
        }
        if (parseResult) {
            Serial.printf("Discarding malformed RPC: %s\n", parseResult.c_str());
            g_rxBuffer = "";
            return;
        }

        auto* message = static_cast<char*>(std::malloc(g_rxBuffer.length() + 1));
        if (message == nullptr) {
            Serial.println("RPC allocation failed");
            g_rxBuffer = "";
            return;
        }
        std::memcpy(message, g_rxBuffer.c_str(), g_rxBuffer.length());
        message[g_rxBuffer.length()] = '\0';
        if (xQueueSend(g_rpcQueue, &message, 0) != pdTRUE) {
            Serial.println("RPC queue full");
            std::free(message);
        }
        g_rxBuffer = "";
    }
};

// Connection callbacks keep the UI synchronized with pairing state and resume
// advertising automatically after a disconnect.
class HidServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connection) override {
        g_uiDirty = true;
        server->updateConnParams(connection.getConnHandle(), 12, 24, 0, 180);
        Serial.printf("BLE connected: %s\n", connection.getAddress().toString().c_str());
    }

    void onDisconnect(NimBLEServer*, NimBLEConnInfo& connection, int reason) override {
        if (g_hidConnectionHandle == connection.getConnHandle()) {
            g_connected = false;
            g_hidConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
        }
        g_activeTouch = -1;
        g_uiDirty = true;
        Serial.printf("BLE disconnected: %d\n", reason);
        NimBLEDevice::startAdvertising();
    }

    void onAuthenticationComplete(NimBLEConnInfo& connection) override {
        if (!connection.isEncrypted()) {
            Serial.println("BLE encryption failed");
            NimBLEDevice::getServer()->disconnect(connection.getConnHandle());
            return;
        }
        // Defer hardware feedback to Arduino's main loop; NimBLE owns this
        // callback task and should only publish the successful result.
        g_pairingSuccessPending = true;
        Serial.println("BLE pairing authenticated");
    }

    void onConnParamsUpdate(NimBLEConnInfo& connection) override {
        VibeStackChanApp::onHidConnParamsUpdated(connection);
    }
};

class HidInputCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& connection,
                     std::uint16_t subValue) override {
        if (subValue != 0) {
            g_hidConnectionHandle = connection.getConnHandle();
            g_connected = true;
            g_uiDirty = true;
            Serial.printf("BLE HID subscribed: handle=%u\n",
                          static_cast<unsigned>(g_hidConnectionHandle));
            VibeStackChanApp::onHidSubscribed(connection);
        } else if (g_hidConnectionHandle == connection.getConnHandle()) {
            g_connected = false;
            g_hidConnectionHandle = BLE_HS_CONN_HANDLE_NONE;
            g_uiDirty = true;
            Serial.println("BLE HID unsubscribed");
        }
    }
};

RpcOutputCallbacks g_rpcCallbacks;
HidServerCallbacks g_serverCallbacks;
HidInputCallbacks g_hidInputCallbacks;

void addDeviceInfoCharacteristic(std::uint16_t uuid, const char* value) {
    auto* characteristic = g_hid->getDeviceInfoService()->createCharacteristic(uuid, NIMBLE_PROPERTY::READ);
    characteristic->setValue(value);
}

void initializeBle() {
    // Expose standard keyboard/consumer/pointer reports plus the vendor report
    // used for agent status, actions, and request/response messages.
    g_bleStarted = false;
    if (!NimBLEDevice::init(g_deviceName)) {
        Serial.println("Failed to initialize BLE");
        return;
    }
    if (!configureBleIdentity()) {
        Serial.println("Failed to configure BLE identity");
        NimBLEDevice::deinit(true);
        return;
    }
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setSecurityAuth(true, false, true);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(&g_serverCallbacks);
    g_hid = new NimBLEHIDDevice(g_server);

    g_hid->setManufacturer(hid_protocol::kManufacturer);
    g_hid->setPnp(0x01, hid_protocol::kVendorId, hid_protocol::kProductId, hid_protocol::kProductVersion);
    g_hid->setHidInfo(0x00, 0x01);
    g_hid->setReportMap(hid_protocol::kReportMap, sizeof(hid_protocol::kReportMap));

    char serial[17];
    std::snprintf(serial, sizeof(serial), "%016llX", ESP.getEfuseMac());
    addDeviceInfoCharacteristic(0x2A24, hid_protocol::kModelNumber);
    addDeviceInfoCharacteristic(0x2A25, serial);
    addDeviceInfoCharacteristic(0x2A26, hid_protocol::kFirmwareVersion);

    auto* keyboardInput = g_hid->getInputReport(1);
    auto* consumerInput = g_hid->getInputReport(2);
    auto* pointerInput = g_hid->getInputReport(3);
    g_vendorInput = g_hid->getInputReport(hid_protocol::kVendorReportId);
    g_vendorOutput = g_hid->getOutputReport(hid_protocol::kVendorReportId);
    g_hid->getFeatureReport(hid_protocol::kVendorReportId);

    const std::uint8_t keyboardIdle[8] = {};
    const std::uint8_t consumerIdle[2] = {};
    const std::uint8_t pointerIdle[5] = {};
    const std::uint8_t vendorIdle[hid_protocol::kBleReportLength] = {};
    keyboardInput->setValue(keyboardIdle, sizeof(keyboardIdle));
    consumerInput->setValue(consumerIdle, sizeof(consumerIdle));
    pointerInput->setValue(pointerIdle, sizeof(pointerIdle));
    g_vendorInput->setValue(vendorIdle, sizeof(vendorIdle));
    g_vendorInput->setCallbacks(&g_hidInputCallbacks);
    g_vendorOutput->setCallbacks(&g_rpcCallbacks);

    updateBattery(false);
    if (!g_server->start()) {
        Serial.println("Failed to start BLE GATT server");
        return;
    }

    auto* advertising = NimBLEDevice::getAdvertising();
    advertising->setName(g_deviceName);
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(g_hid->getHidService()->getUUID());
    advertising->enableScanResponse(true);
    if (!advertising->start()) {
        Serial.println("Failed to start BLE advertising");
        return;
    }
    g_bleStarted = true;
    Serial.printf("BLE HID advertising started as %s\n", g_deviceName);
}

void beginPairing() {
    // A slot is a separate advertised identity. Clear every stored bond before
    // restarting so macOS discovers the selected slot with a clean GATT cache.
    saveDeviceSlot(g_pendingDeviceSlot);

    if (g_server != nullptr) {
        const auto peers = g_server->getPeerDevices();
        for (const auto connectionHandle : peers) {
            g_server->disconnect(connectionHandle);
        }
    }
    NimBLEDevice::deleteAllBonds();

    g_connected = false;
    g_deviceSlot = g_pendingDeviceSlot;
    updateDeviceName();
    g_restartAt = millis() + 900;
    g_uiDirty = true;
    Serial.printf("Pairing requested for %s; restarting\n", g_deviceName);
}

}  // namespace
