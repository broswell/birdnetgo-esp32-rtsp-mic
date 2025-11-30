#include <WiFi.h>
#include <WiFiManager.h>
#include "driver/i2s.h"
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <math.h>
#include "WebUI.h"
#include <WiFiUdp.h>

// ================== SETTINGS (ESP32 RTSP Mic for BirdNET-Go) ==================
#define FW_VERSION "1.4.4"
const char* FW_VERSION_STR = FW_VERSION;
#define MAX_CLIENTS 2  // Support 2 concurrent streams

// OTA password (optional):
// #define OTA_PASSWORD "1234"

// -- DEFAULT PARAMETERS (configurable via Web UI / API)
#define DEFAULT_SAMPLE_RATE 48000
#define DEFAULT_GAIN_FACTOR 1.2f
#define DEFAULT_BUFFER_SIZE 1024
#define DEFAULT_WIFI_TX_DBM 19.5f

// High-pass filter defaults
#define DEFAULT_HPF_ENABLED true
#define DEFAULT_HPF_CUTOFF_HZ 500

// Thermal protection defaults
#define DEFAULT_OVERHEAT_PROTECTION true
#define DEFAULT_OVERHEAT_LIMIT_C 80
#define OVERHEAT_MIN_LIMIT_C 30
#define OVERHEAT_MAX_LIMIT_C 95
#define OVERHEAT_LIMIT_STEP_C 5

// -- Pins
#define I2S_BCLK_PIN    21
#define I2S_LRCLK_PIN   1
#define I2S_DOUT_PIN    2

// -- RTSP/RTP Transport
enum TransportMode {
    TRANSPORT_TCP,
    TRANSPORT_UDP
};

// -- Client Session Structure
struct ClientSession {
    WiFiClient client;
    WiFiUDP udpSocket;
    TransportMode transport;
    IPAddress clientRtpAddress;
    uint16_t clientRtpPort;
    uint16_t serverRtpPort;
    String sessionId;
    bool streaming;
    uint16_t rtpSequence;
    uint32_t rtpTimestamp;
    unsigned long lastActivity;
    uint8_t parseBuffer[1024];
    int parseBufferPos;
    unsigned long packetsSent;
    
    void reset() {
        if (client.connected()) client.stop();
        if (transport == TRANSPORT_UDP) udpSocket.stop();
        transport = TRANSPORT_TCP;
        clientRtpPort = 0;
        serverRtpPort = 0;
        sessionId = "";
        streaming = false;
        rtpSequence = 0;
        rtpTimestamp = 0;
        lastActivity = 0;
        parseBufferPos = 0;
        packetsSent = 0;
    }
};

// -- Servers
WiFiServer rtspServer(8554);
ClientSession clients[MAX_CLIENTS];
uint32_t rtpSSRC = 0x43215678;

// -- WebUI Compatibility (for first active client)
WiFiClient rtspClient;  // Points to first active client
String rtspSessionId = "";
volatile bool isStreaming = false;
unsigned long audioPacketsSent = 0;

// -- Buffers
int32_t* i2s_32bit_buffer = nullptr;
int16_t* i2s_16bit_buffer = nullptr;

// -- Global state
unsigned long lastStatsReset = 0;
bool rtspServerEnabled = true;

// -- Audio parameters (runtime configurable)
uint32_t currentSampleRate = DEFAULT_SAMPLE_RATE;
float currentGainFactor = DEFAULT_GAIN_FACTOR;
uint16_t currentBufferSize = DEFAULT_BUFFER_SIZE;
uint8_t i2sShiftBits = 12;

// -- Audio metering / clipping diagnostics
uint16_t lastPeakAbs16 = 0;
uint32_t audioClipCount = 0;
bool audioClippedLastBlock = false;
uint16_t peakHoldAbs16 = 0;
unsigned long peakHoldUntilMs = 0;

// -- High-pass filter (biquad)
struct Biquad {
    float b0{1.0f}, b1{0.0f}, b2{0.0f}, a1{0.0f}, a2{0.0f};
    float x1{0.0f}, x2{0.0f}, y1{0.0f}, y2{0.0f};
    inline float process(float x) {
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y;
        return y;
    }
    inline void reset() { x1 = x2 = y1 = y2 = 0.0f; }
};

bool highpassEnabled = DEFAULT_HPF_ENABLED;
uint16_t highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
Biquad hpf;
uint32_t hpfConfigSampleRate = 0;
uint16_t hpfConfigCutoff = 0;

// -- Preferences for persistent settings
Preferences audioPrefs;

// -- Diagnostics and monitoring
unsigned long lastMemoryCheck = 0;
unsigned long lastPerformanceCheck = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastTempCheck = 0;
uint32_t minFreeHeap = 0xFFFFFFFF;
uint32_t maxPacketRate = 0;
uint32_t minPacketRate = 0xFFFFFFFF;
bool autoRecoveryEnabled = true;
bool autoThresholdEnabled = true;

// Deferred reboot scheduling
volatile bool scheduledFactoryReset = false;
volatile unsigned long scheduledRebootAt = 0;
unsigned long bootTime = 0;
unsigned long lastI2SReset = 0;
float maxTemperature = 0.0f;
float lastTemperatureC = 0.0f;
bool lastTemperatureValid = false;
bool overheatProtectionEnabled = DEFAULT_OVERHEAT_PROTECTION;
float overheatShutdownC = (float)DEFAULT_OVERHEAT_LIMIT_C;
bool overheatLockoutActive = false;
float overheatTripTemp = 0.0f;
unsigned long overheatTriggeredAt = 0;
String overheatLastReason = "";
String overheatLastTimestamp = "";
bool overheatSensorFault = false;
bool overheatLatched = false;

// -- Scheduled reset
bool scheduledResetEnabled = false;
uint32_t resetIntervalHours = 24;

// -- Configurable thresholds
uint32_t minAcceptableRate = 50;
uint32_t performanceCheckInterval = 15;
uint8_t cpuFrequencyMhz = 160;

// -- WiFi TX power
float wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
wifi_power_t currentWifiPowerLevel = WIFI_POWER_19_5dBm;

// -- RTSP statistics
unsigned long lastRtspClientConnectMs = 0;
unsigned long lastRtspPlayMs = 0;
uint32_t rtspConnectCount = 0;
uint32_t rtspPlayCount = 0;

// ===============================================
// Update WebUI compatibility variables
void updateWebUIVars() {
    isStreaming = false;
    audioPacketsSent = 0;
    rtspSessionId = "";
    
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].streaming) {
            isStreaming = true;
            audioPacketsSent += clients[i].packetsSent;
            if (rtspSessionId == "" && clients[i].sessionId != "") {
                rtspSessionId = clients[i].sessionId;
            }
        }
    }
    
    // Update rtspClient to point to first connected client
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].client.connected()) {
            rtspClient = clients[i].client;
            break;
        }
    }
}

// Helper functions
float wifiPowerLevelToDbm(wifi_power_t lvl) {
    switch (lvl) {
        case WIFI_POWER_19_5dBm:    return 19.5f;
        case WIFI_POWER_19dBm:      return 19.0f;
        case WIFI_POWER_18_5dBm:    return 18.5f;
        case WIFI_POWER_17dBm:      return 17.0f;
        case WIFI_POWER_15dBm:      return 15.0f;
        case WIFI_POWER_13dBm:      return 13.0f;
        case WIFI_POWER_11dBm:      return 11.0f;
        case WIFI_POWER_8_5dBm:     return 8.5f;
        case WIFI_POWER_7dBm:       return 7.0f;
        case WIFI_POWER_5dBm:       return 5.0f;
        case WIFI_POWER_2dBm:       return 2.0f;
        case WIFI_POWER_MINUS_1dBm: return -1.0f;
        default:                    return 19.5f;
    }
}

static wifi_power_t pickWifiPowerLevel(float dbm) {
    if (dbm <= -1.0f) return WIFI_POWER_MINUS_1dBm;
    if (dbm <= 2.0f)  return WIFI_POWER_2dBm;
    if (dbm <= 5.0f)  return WIFI_POWER_5dBm;
    if (dbm <= 7.0f)  return WIFI_POWER_7dBm;
    if (dbm <= 8.5f)  return WIFI_POWER_8_5dBm;
    if (dbm <= 11.0f) return WIFI_POWER_11dBm;
    if (dbm <= 13.0f) return WIFI_POWER_13dBm;
    if (dbm <= 15.0f) return WIFI_POWER_15dBm;
    if (dbm <= 17.0f) return WIFI_POWER_17dBm;
    if (dbm <= 18.5f) return WIFI_POWER_18_5dBm;
    if (dbm <= 19.0f) return WIFI_POWER_19dBm;
    return WIFI_POWER_19_5dBm;
}

void applyWifiTxPower(bool log = true) {
    wifi_power_t desired = pickWifiPowerLevel(wifiTxPowerDbm);
    if (desired != currentWifiPowerLevel) {
        WiFi.setTxPower(desired);
        currentWifiPowerLevel = desired;
        if (log) {
            simplePrintln("WiFi TX power set to " + String(wifiPowerLevelToDbm(currentWifiPowerLevel), 1) + " dBm");
        }
    }
}

void updateHighpassCoeffs() {
    if (!highpassEnabled) {
        hpf.reset();
        hpfConfigSampleRate = currentSampleRate;
        hpfConfigCutoff = highpassCutoffHz;
        return;
    }
    float fs = (float)currentSampleRate;
    float fc = (float)highpassCutoffHz;
    if (fc < 10.0f) fc = 10.0f;
    if (fc > fs * 0.45f) fc = fs * 0.45f;
    const float pi = 3.14159265358979323846f;
    float w0 = 2.0f * pi * (fc / fs);
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float Q = 0.70710678f;
    float alpha = sinw0 / (2.0f * Q);
    float b0 =  (1.0f + cosw0) * 0.5f;
    float b1 = -(1.0f + cosw0);
    float b2 =  (1.0f + cosw0) * 0.5f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cosw0;
    float a2 =  1.0f - alpha;
    hpf.b0 = b0 / a0;
    hpf.b1 = b1 / a0;
    hpf.b2 = b2 / a0;
    hpf.a1 = a1 / a0;
    hpf.a2 = a2 / a0;
    hpf.reset();
    hpfConfigSampleRate = currentSampleRate;
    hpfConfigCutoff = (uint16_t)fc;
}

String formatUptime(unsigned long seconds) {
    unsigned long days = seconds / 86400;
    seconds %= 86400;
    unsigned long hours = seconds / 3600;
    seconds %= 3600;
    unsigned long minutes = seconds / 60;
    seconds %= 60;
    String result = "";
    if (days > 0) result += String(days) + "d ";
    if (hours > 0 || days > 0) result += String(hours) + "h ";
    if (minutes > 0 || hours > 0 || days > 0) result += String(minutes) + "m ";
    result += String(seconds) + "s";
    return result;
}

String formatSince(unsigned long eventMs) {
    if (eventMs == 0) return String("never");
    unsigned long seconds = (millis() - eventMs) / 1000;
    return formatUptime(seconds) + " ago";
}

static bool isTemperatureValid(float temp) {
    if (isnan(temp) || isinf(temp)) return false;
    if (temp < -20.0f || temp > 130.0f) return false;
    return true;
}

static void persistOverheatNote() {
    audioPrefs.begin("audio", false);
    audioPrefs.putString("ohReason", overheatLastReason);
    audioPrefs.putString("ohStamp", overheatLastTimestamp);
    audioPrefs.putFloat("ohTripC", overheatTripTemp);
    audioPrefs.putBool("ohLatched", overheatLatched);
    audioPrefs.end();
}

void recordOverheatTrip(float temp) {
    unsigned long uptimeSeconds = (millis() - bootTime) / 1000;
    overheatTripTemp = temp;
    overheatTriggeredAt = millis();
    overheatLastTimestamp = formatUptime(uptimeSeconds);
    overheatLastReason = String("Thermal shutdown: ") + String(temp, 1) + " C reached (limit " +
                         String(overheatShutdownC, 1) + " C). Stream disabled; acknowledge in UI.";
    overheatLatched = true;
    simplePrintln("THERMAL PROTECTION: " + overheatLastReason);
    simplePrintln("TIP: Improve cooling or lower WiFi TX power/CPU MHz if overheating persists.");
    persistOverheatNote();
}

void checkTemperature() {
    float temp = temperatureRead();
    bool tempValid = isTemperatureValid(temp);
    if (!tempValid) {
        lastTemperatureValid = false;
        if (!overheatSensorFault) {
            overheatSensorFault = true;
            overheatLastReason = "Thermal protection disabled: temperature sensor unavailable.";
            overheatLastTimestamp = "";
            overheatTripTemp = 0.0f;
            overheatTriggeredAt = 0;
            persistOverheatNote();
            simplePrintln("WARNING: Temperature sensor unavailable. Thermal protection paused.");
        }
        return;
    }
    lastTemperatureC = temp;
    lastTemperatureValid = true;
    if (overheatSensorFault) {
        overheatSensorFault = false;
        overheatLastReason = "Thermal protection restored: temperature sensor reading valid.";
        overheatLastTimestamp = formatUptime((millis() - bootTime) / 1000);
        persistOverheatNote();
        simplePrintln("Temperature sensor restored. Thermal protection active again.");
    }
    if (temp > maxTemperature) {
        maxTemperature = temp;
    }
    bool protectionActive = overheatProtectionEnabled && !overheatSensorFault;
    if (protectionActive) {
        if (!overheatLockoutActive && temp >= overheatShutdownC) {
            overheatLockoutActive = true;
            recordOverheatTrip(temp);
            for (int i = 0; i < MAX_CLIENTS; i++) {
                clients[i].reset();
            }
            rtspServerEnabled = false;
            rtspServer.stop();
        } else if (overheatLockoutActive && temp <= (overheatShutdownC - OVERHEAT_LIMIT_STEP_C)) {
            overheatLockoutActive = false;
        }
    } else {
        overheatLockoutActive = false;
    }
    static unsigned long lastTempWarn = 0;
    float warnThreshold = max(overheatShutdownC - 5.0f, (float)OVERHEAT_MIN_LIMIT_C);
    if (temp > warnThreshold && (millis() - lastTempWarn) > 600000UL) {
        simplePrintln("WARNING: High temperature detected (" + String(temp, 1) + " C). Approaching shutdown limit.");
        lastTempWarn = millis();
    }
}

void checkPerformance() {
    uint32_t currentHeap = ESP.getFreeHeap();
    if (currentHeap < minFreeHeap) {
        minFreeHeap = currentHeap;
    }
    
    bool anyStreaming = false;
    unsigned long totalPackets = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].streaming) {
            anyStreaming = true;
            totalPackets += clients[i].packetsSent;
        }
    }
    
    if (anyStreaming && (millis() - lastStatsReset) > 30000) {
        uint32_t runtime = millis() - lastStatsReset;
        uint32_t currentRate = (totalPackets * 1000) / runtime;
        if (currentRate > maxPacketRate) maxPacketRate = currentRate;
        if (currentRate < minPacketRate) minPacketRate = currentRate;
        if (currentRate < minAcceptableRate) {
            simplePrintln("PERFORMANCE DEGRADATION DETECTED!");
            simplePrintln("Rate " + String(currentRate) + " < minimum " + String(minAcceptableRate) + " pkt/s");
            if (autoRecoveryEnabled) {
                simplePrintln("AUTO-RECOVERY: Restarting I2S...");
                restartI2S();
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    clients[i].packetsSent = 0;
                }
                lastStatsReset = millis();
                lastI2SReset = millis();
            }
        }
    }
}

void checkWiFiHealth() {
    if (WiFi.status() != WL_CONNECTED) {
        simplePrintln("WiFi disconnected! Reconnecting...");
        WiFi.reconnect();
    }
    applyWifiTxPower(false);
    int32_t rssi = WiFi.RSSI();
    if (rssi < -85) {
        simplePrintln("WARNING: Weak WiFi signal: " + String(rssi) + " dBm");
    }
}

void checkScheduledReset() {
    if (!scheduledResetEnabled) return;
    unsigned long uptimeHours = (millis() - bootTime) / 3600000;
    if (uptimeHours >= resetIntervalHours) {
        simplePrintln("SCHEDULED RESET: " + String(resetIntervalHours) + " hours reached");
        delay(1000);
        ESP.restart();
    }
}

void loadAudioSettings() {
    audioPrefs.begin("audio", false);
    currentSampleRate = audioPrefs.getUInt("sampleRate", DEFAULT_SAMPLE_RATE);
    currentGainFactor = audioPrefs.getFloat("gainFactor", DEFAULT_GAIN_FACTOR);
    currentBufferSize = audioPrefs.getUShort("bufferSize", DEFAULT_BUFFER_SIZE);
    i2sShiftBits = audioPrefs.getUChar("shiftBits", i2sShiftBits);
    autoRecoveryEnabled = audioPrefs.getBool("autoRecovery", true);
    scheduledResetEnabled = audioPrefs.getBool("schedReset", false);
    resetIntervalHours = audioPrefs.getUInt("resetHours", 24);
    minAcceptableRate = audioPrefs.getUInt("minRate", 50);
    performanceCheckInterval = audioPrefs.getUInt("checkInterval", 15);
    autoThresholdEnabled = audioPrefs.getBool("thrAuto", true);
    cpuFrequencyMhz = audioPrefs.getUChar("cpuFreq", 160);
    wifiTxPowerDbm = audioPrefs.getFloat("wifiTxDbm", DEFAULT_WIFI_TX_DBM);
    highpassEnabled = audioPrefs.getBool("hpEnable", DEFAULT_HPF_ENABLED);
    highpassCutoffHz = (uint16_t)audioPrefs.getUInt("hpCutoff", DEFAULT_HPF_CUTOFF_HZ);
    overheatProtectionEnabled = audioPrefs.getBool("ohEnable", DEFAULT_OVERHEAT_PROTECTION);
    uint32_t ohLimit = audioPrefs.getUInt("ohThresh", DEFAULT_OVERHEAT_LIMIT_C);
    if (ohLimit < OVERHEAT_MIN_LIMIT_C) ohLimit = OVERHEAT_MIN_LIMIT_C;
    if (ohLimit > OVERHEAT_MAX_LIMIT_C) ohLimit = OVERHEAT_MAX_LIMIT_C;
    ohLimit = OVERHEAT_MIN_LIMIT_C + ((ohLimit - OVERHEAT_MIN_LIMIT_C) / OVERHEAT_LIMIT_STEP_C) * OVERHEAT_LIMIT_STEP_C;
    overheatShutdownC = (float)ohLimit;
    overheatLastReason = audioPrefs.getString("ohReason", "");
    overheatLastTimestamp = audioPrefs.getString("ohStamp", "");
    overheatTripTemp = audioPrefs.getFloat("ohTripC", 0.0f);
    overheatLatched = audioPrefs.getBool("ohLatched", false);
    audioPrefs.end();
    if (autoThresholdEnabled) {
        minAcceptableRate = computeRecommendedMinRate();
    }
    if (overheatLatched) {
        rtspServerEnabled = false;
    }
    float txShown = wifiPowerLevelToDbm(pickWifiPowerLevel(wifiTxPowerDbm));
    simplePrintln("Loaded settings: Rate=" + String(currentSampleRate) +
                  ", Gain=" + String(currentGainFactor, 1) +
                  ", Buffer=" + String(currentBufferSize) +
                  ", WiFiTX=" + String(txShown, 1) + "dBm" +
                  ", shiftBits=" + String(i2sShiftBits) +
                  ", HPF=" + String(highpassEnabled?"on":"off") +
                  ", HPFcut=" + String(highpassCutoffHz) + "Hz");
}

void saveAudioSettings() {
    audioPrefs.begin("audio", false);
    audioPrefs.putUInt("sampleRate", currentSampleRate);
    audioPrefs.putFloat("gainFactor", currentGainFactor);
    audioPrefs.putUShort("bufferSize", currentBufferSize);
    audioPrefs.putUChar("shiftBits", i2sShiftBits);
    audioPrefs.putBool("autoRecovery", autoRecoveryEnabled);
    audioPrefs.putBool("schedReset", scheduledResetEnabled);
    audioPrefs.putUInt("resetHours", resetIntervalHours);
    audioPrefs.putUInt("minRate", minAcceptableRate);
    audioPrefs.putUInt("checkInterval", performanceCheckInterval);
    audioPrefs.putBool("thrAuto", autoThresholdEnabled);
    audioPrefs.putUChar("cpuFreq", cpuFrequencyMhz);
    audioPrefs.putFloat("wifiTxDbm", wifiTxPowerDbm);
    audioPrefs.putBool("hpEnable", highpassEnabled);
    audioPrefs.putUInt("hpCutoff", (uint32_t)highpassCutoffHz);
    audioPrefs.putBool("ohEnable", overheatProtectionEnabled);
    uint32_t ohLimit = (uint32_t)(overheatShutdownC + 0.5f);
    if (ohLimit < OVERHEAT_MIN_LIMIT_C) ohLimit = OVERHEAT_MIN_LIMIT_C;
    if (ohLimit > OVERHEAT_MAX_LIMIT_C) ohLimit = OVERHEAT_MAX_LIMIT_C;
    audioPrefs.putUInt("ohThresh", ohLimit);
    audioPrefs.putString("ohReason", overheatLastReason);
    audioPrefs.putString("ohStamp", overheatLastTimestamp);
    audioPrefs.putFloat("ohTripC", overheatTripTemp);
    audioPrefs.putBool("ohLatched", overheatLatched);
    audioPrefs.end();
    simplePrintln("Settings saved to flash");
}

void scheduleReboot(bool factoryReset, uint32_t delayMs) {
    scheduledFactoryReset = factoryReset;
    scheduledRebootAt = millis() + delayMs;
}

uint32_t computeRecommendedMinRate() {
    uint32_t buf = max((uint16_t)1, currentBufferSize);
    float expectedPktPerSec = (float)currentSampleRate / (float)buf;
    uint32_t rec = (uint32_t)(expectedPktPerSec * 0.7f + 0.5f);
    if (rec < 5) rec = 5;
    return rec;
}

void resetToDefaultSettings() {
    simplePrintln("FACTORY RESET: Restoring default settings...");
    audioPrefs.begin("audio", false);
    audioPrefs.clear();
    audioPrefs.end();
    currentSampleRate = DEFAULT_SAMPLE_RATE;
    currentGainFactor = DEFAULT_GAIN_FACTOR;
    currentBufferSize = DEFAULT_BUFFER_SIZE;
    i2sShiftBits = 12;
    autoRecoveryEnabled = true;
    autoThresholdEnabled = true;
    scheduledResetEnabled = false;
    resetIntervalHours = 24;
    minAcceptableRate = computeRecommendedMinRate();
    performanceCheckInterval = 15;
    cpuFrequencyMhz = 160;
    wifiTxPowerDbm = DEFAULT_WIFI_TX_DBM;
    highpassEnabled = DEFAULT_HPF_ENABLED;
    highpassCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
    overheatProtectionEnabled = DEFAULT_OVERHEAT_PROTECTION;
    overheatShutdownC = (float)DEFAULT_OVERHEAT_LIMIT_C;
    overheatLockoutActive = false;
    overheatTripTemp = 0.0f;
    overheatTriggeredAt = 0;
    overheatLastReason = "";
    overheatLastTimestamp = "";
    overheatSensorFault = false;
    overheatLatched = false;
    lastTemperatureC = 0.0f;
    lastTemperatureValid = false;
    saveAudioSettings();
    simplePrintln("Defaults applied. Device will reboot.");
}

void restartI2S() {
    simplePrintln("Restarting I2S with new parameters...");
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].streaming = false;
    }
    if (i2s_32bit_buffer) { free(i2s_32bit_buffer); i2s_32bit_buffer = nullptr; }
    if (i2s_16bit_buffer) { free(i2s_16bit_buffer); i2s_16bit_buffer = nullptr; }
    i2s_32bit_buffer = (int32_t*)malloc(currentBufferSize * sizeof(int32_t));
    i2s_16bit_buffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));
    if (!i2s_32bit_buffer || !i2s_16bit_buffer) {
        simplePrintln("FATAL: Memory allocation failed after parameter change!");
        ESP.restart();
    }
    setup_i2s_driver();
    updateHighpassCoeffs();
    maxPacketRate = 0;
    minPacketRate = 0xFFFFFFFF;
    simplePrintln("I2S restarted successfully");
}

void simplePrint(String message) {
    Serial.print(message);
}

void simplePrintln(String message) {
    Serial.println(message);
    webui_pushLog(message);
}

void setupOTA() {
    ArduinoOTA.setHostname("ESP32-RTSP-Mic");
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.begin();
}

void setup_i2s_driver() {
    i2s_driver_uninstall(I2S_NUM_0);
    uint16_t dma_buf_len = (currentBufferSize > 512) ? 512 : currentBufferSize;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = currentSampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = dma_buf_len,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCLK_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DOUT_PIN
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    simplePrintln("I2S ready: " + String(currentSampleRate) + "Hz, gain " +
                  String(currentGainFactor, 1) + ", buffer " + String(currentBufferSize) +
                  ", shiftBits " + String(i2sShiftBits));
}

static bool writeAll(WiFiClient &client, const uint8_t* data, size_t len) {
    size_t off = 0;
    while (off < len) {
        int w = client.write(data + off, len - off);
        if (w <= 0) return false;
        off += (size_t)w;
    }
    return true;
}

// Send RTP packet to specific client
void sendRTPPacket(ClientSession &session, int16_t* audioData, int numSamples) {
    const uint16_t payloadSize = (uint16_t)(numSamples * (int)sizeof(int16_t));
    const uint16_t packetSize = (uint16_t)(12 + payloadSize);
    
    uint8_t header[12];
    header[0] = 0x80;
    header[1] = 96;
    header[2] = (uint8_t)((session.rtpSequence >> 8) & 0xFF);
    header[3] = (uint8_t)(session.rtpSequence & 0xFF);
    header[4] = (uint8_t)((session.rtpTimestamp >> 24) & 0xFF);
    header[5] = (uint8_t)((session.rtpTimestamp >> 16) & 0xFF);
    header[6] = (uint8_t)((session.rtpTimestamp >> 8) & 0xFF);
    header[7] = (uint8_t)(session.rtpTimestamp & 0xFF);
    header[8]  = (uint8_t)((rtpSSRC >> 24) & 0xFF);
    header[9]  = (uint8_t)((rtpSSRC >> 16) & 0xFF);
    header[10] = (uint8_t)((rtpSSRC >> 8) & 0xFF);
    header[11] = (uint8_t)(rtpSSRC & 0xFF);
    
    // Create copy and byte-swap
    int16_t* clientData = (int16_t*)malloc(numSamples * sizeof(int16_t));
    if (!clientData) return;
    memcpy(clientData, audioData, numSamples * sizeof(int16_t));
    
    for (int i = 0; i < numSamples; ++i) {
        uint16_t s = (uint16_t)clientData[i];
        s = (uint16_t)((s << 8) | (s >> 8));
        clientData[i] = (int16_t)s;
    }
    
    bool success = false;
    if (session.transport == TRANSPORT_TCP) {
        if (!session.client.connected()) {
            session.streaming = false;
            free(clientData);
            return;
        }
        
        uint8_t inter[4];
        inter[0] = 0x24;
        inter[1] = 0x00;
        inter[2] = (uint8_t)((packetSize >> 8) & 0xFF);
        inter[3] = (uint8_t)(packetSize & 0xFF);
        
        if (writeAll(session.client, inter, sizeof(inter)) &&
            writeAll(session.client, header, sizeof(header)) &&
            writeAll(session.client, (uint8_t*)clientData, payloadSize)) {
            success = true;
        } else {
            session.streaming = false;
        }
    } else {
        if (session.clientRtpPort == 0) {
            session.streaming = false;
            free(clientData);
            return;
        }
        
        session.udpSocket.beginPacket(session.clientRtpAddress, session.clientRtpPort);
        session.udpSocket.write(header, sizeof(header));
        session.udpSocket.write((uint8_t*)clientData, payloadSize);
        if (session.udpSocket.endPacket()) {
            success = true;
        }
    }
    
    free(clientData);
    
    if (success) {
        session.rtpSequence++;
        session.rtpTimestamp += (uint32_t)numSamples;
        session.packetsSent++;
    }
}

// Stream audio to all active clients
void streamAudio() {
    bool anyStreaming = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].streaming) {
            anyStreaming = true;
            break;
        }
    }
    
    if (!anyStreaming) return;
    
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_NUM_0, i2s_32bit_buffer,
                                currentBufferSize * sizeof(int32_t),
                                &bytesRead, 50 / portTICK_PERIOD_MS);
    if (result == ESP_OK && bytesRead > 0) {
        int samplesRead = bytesRead / sizeof(int32_t);
        
        if (highpassEnabled && (hpfConfigSampleRate != currentSampleRate || hpfConfigCutoff != highpassCutoffHz)) {
            updateHighpassCoeffs();
        }
        
        bool clipped = false;
        float peakAbs = 0.0f;
        
        for (int i = 0; i < samplesRead; i++) {
            float sample = (float)(i2s_32bit_buffer[i] >> i2sShiftBits);
            if (highpassEnabled) sample = hpf.process(sample);
            float amplified = sample * currentGainFactor;
            float aabs = fabsf(amplified);
            if (aabs > peakAbs) peakAbs = aabs;
            if (aabs > 32767.0f) clipped = true;
            if (amplified > 32767.0f) amplified = 32767.0f;
            if (amplified < -32768.0f) amplified = -32768.0f;
            i2s_16bit_buffer[i] = (int16_t)amplified;
        }
        
        if (peakAbs > 32767.0f) peakAbs = 32767.0f;
        lastPeakAbs16 = (uint16_t)peakAbs;
        audioClippedLastBlock = clipped;
        if (clipped) audioClipCount++;
        
        if (lastPeakAbs16 > peakHoldAbs16) {
            peakHoldAbs16 = lastPeakAbs16;
            peakHoldUntilMs = millis() + 3000UL;
        } else if (peakHoldAbs16 > 0 && millis() > peakHoldUntilMs) {
            peakHoldAbs16 = 0;
        }
        
        // Send to all active clients
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].streaming) {
                sendRTPPacket(clients[i], i2s_16bit_buffer, samplesRead);
            }
        }
    }
}

void parseTransportHeader(ClientSession &session, String request, String &transportResponse, int clientIdx) {
    int transportPos = request.indexOf("Transport:");
    if (transportPos < 0) {
        session.transport = TRANSPORT_TCP;
        transportResponse = "RTP/AVP/TCP;unicast;interleaved=0-1";
        return;
    }
    
    String transportLine = request.substring(transportPos);
    int endPos = transportLine.indexOf("\r\n");
    if (endPos > 0) {
        transportLine = transportLine.substring(0, endPos);
    }
    
    transportLine.toLowerCase();
    
    if (transportLine.indexOf("rtp/avp/udp") >= 0 || transportLine.indexOf("rtp/avp;unicast") >= 0) {
        session.transport = TRANSPORT_UDP;
        
        int clientPortPos = transportLine.indexOf("client_port=");
        if (clientPortPos >= 0) {
            String portStr = transportLine.substring(clientPortPos + 12);
            int dashPos = portStr.indexOf("-");
            if (dashPos > 0) {
                portStr = portStr.substring(0, dashPos);
            }
            int spacePos = portStr.indexOf(";");
            if (spacePos > 0) {
                portStr = portStr.substring(0, spacePos);
            }
            portStr.trim();
            session.clientRtpPort = (uint16_t)portStr.toInt();
        }
        
        if (session.client.connected()) {
            session.clientRtpAddress = session.client.remoteIP();
        }
        
        session.serverRtpPort = 5004 + (clientIdx * 2);
        session.udpSocket.begin(session.serverRtpPort);
        
        transportResponse = "RTP/AVP/UDP;unicast;client_port=" + String(session.clientRtpPort) + "-" + String(session.clientRtpPort + 1);
        transportResponse += ";server_port=" + String(session.serverRtpPort) + "-" + String(session.serverRtpPort + 1);
        
        simplePrintln("Client " + String(clientIdx) + " UDP: " + session.clientRtpAddress.toString() + ":" + String(session.clientRtpPort));
    } else {
        session.transport = TRANSPORT_TCP;
        transportResponse = "RTP/AVP/TCP;unicast;interleaved=0-1";
        simplePrintln("Client " + String(clientIdx) + " TCP interleaved");
    }
}

void handleRTSPCommand(ClientSession &session, String request, int clientIdx) {
    String cseq = "1";
    int cseqPos = request.indexOf("CSeq: ");
    if (cseqPos >= 0) {
        cseq = request.substring(cseqPos + 6, request.indexOf("\r", cseqPos));
        cseq.trim();
    }
    session.lastActivity = millis();
    
    if (request.startsWith("OPTIONS")) {
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n");
        session.client.print("Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER\r\n\r\n");
    } else if (request.startsWith("DESCRIBE")) {
        String ip = WiFi.localIP().toString();
        String sdp = "v=0\r\n";
        sdp += "o=- 0 0 IN IP4 " + ip + "\r\n";
        sdp += "s=ESP32 RTSP Mic (" + String(currentSampleRate) + "Hz, 16-bit PCM)\r\n";
        sdp += "c=IN IP4 " + ip + "\r\n";
        sdp += "t=0 0\r\n";
        sdp += "m=audio 0 RTP/AVP 96\r\n";
        sdp += "a=rtpmap:96 L16/" + String(currentSampleRate) + "/1\r\n";
        sdp += "a=control:track1\r\n";
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n");
        session.client.print("Content-Type: application/sdp\r\n");
        session.client.print("Content-Base: rtsp://" + ip + ":8554/audio/\r\n");
        session.client.print("Content-Length: " + String(sdp.length()) + "\r\n\r\n");
        session.client.print(sdp);
    } else if (request.startsWith("SETUP")) {
        session.sessionId = String(random(100000000, 999999999));
        
        String transportResponse;
        parseTransportHeader(session, request, transportResponse, clientIdx);
        
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n");
        session.client.print("Session: " + session.sessionId + "\r\n");
        session.client.print("Transport: " + transportResponse + "\r\n\r\n");
    } else if (request.startsWith("PLAY")) {
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n");
        session.client.print("Session: " + session.sessionId + "\r\n");
        session.client.print("Range: npt=0.000-\r\n\r\n");
        session.streaming = true;
        session.rtpSequence = 0;
        session.rtpTimestamp = 0;
        session.packetsSent = 0;
        lastRtspPlayMs = millis();
        rtspPlayCount++;
        simplePrintln("Client " + String(clientIdx) + " STREAMING STARTED");
    } else if (request.startsWith("TEARDOWN")) {
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n");
        session.client.print("Session: " + session.sessionId + "\r\n\r\n");
        session.streaming = false;
        if (session.transport == TRANSPORT_UDP) {
            session.udpSocket.stop();
            session.clientRtpPort = 0;
        }
        simplePrintln("Client " + String(clientIdx) + " STREAMING STOPPED");
    } else if (request.startsWith("GET_PARAMETER")) {
        session.client.print("RTSP/1.0 200 OK\r\n");
        session.client.print("CSeq: " + cseq + "\r\n\r\n");
    }
}

void processRTSP(ClientSession &session, int clientIdx) {
    if (!session.client.connected()) return;
    if (session.client.available()) {
        int available = session.client.available();
        if (session.parseBufferPos + available >= (int)sizeof(session.parseBuffer)) {
            available = sizeof(session.parseBuffer) - session.parseBufferPos - 1;
            if (available <= 0) {
                simplePrintln("Client " + String(clientIdx) + " buffer overflow");
                session.parseBufferPos = 0;
                return;
            }
        }
        session.client.read(session.parseBuffer + session.parseBufferPos, available);
        session.parseBufferPos += available;
        char* endOfHeader = strstr((char*)session.parseBuffer, "\r\n\r\n");
        if (endOfHeader != nullptr) {
            *endOfHeader = '\0';
            String request = String((char*)session.parseBuffer);
            handleRTSPCommand(session, request, clientIdx);
            int headerLen = (endOfHeader - (char*)session.parseBuffer) + 4;
            memmove(session.parseBuffer, session.parseBuffer + headerLen, session.parseBufferPos - headerLen);
            session.parseBufferPos -= headerLen;
        }
    }
}

int findFreeSlot() {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].client.connected()) {
            return i;
        }
    }
    return -1;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    
    randomSeed((uint32_t)micros() ^ (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));
    bootTime = millis();
    rtpSSRC = (uint32_t)random(1, 0x7FFFFFFF);
    
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);
    Serial.println("RF switch control enabled (GPIO3 LOW)");
    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH);
    Serial.println("External antenna selected (GPIO14 HIGH)");
    
    loadAudioSettings();
    
    i2s_32bit_buffer = (int32_t*)malloc(currentBufferSize * sizeof(int32_t));
    i2s_16bit_buffer = (int16_t*)malloc(currentBufferSize * sizeof(int16_t));
    if (!i2s_32bit_buffer || !i2s_16bit_buffer) {
        simplePrintln("FATAL: Memory allocation failed!");
        ESP.restart();
    }
    
    WiFi.setSleep(false);
    WiFiManager wm;
    wm.setConnectTimeout(60);
    wm.setConfigPortalTimeout(180);
    if (!wm.autoConnect("ESP32-RTSP-Mic-AP")) {
        simplePrintln("WiFi failed, restarting...");
        ESP.restart();
    }
    simplePrintln("WiFi connected: " + WiFi.localIP().toString());
    
    applyWifiTxPower(true);
    setupOTA();
    setup_i2s_driver();
    updateHighpassCoeffs();
    
    if (!overheatLatched) {
        rtspServer.begin();
        rtspServer.setNoDelay(true);
        rtspServerEnabled = true;
    } else {
        rtspServerEnabled = false;
        rtspServer.stop();
    }
    
    webui_begin();
    
    lastStatsReset = millis();
    lastMemoryCheck = millis();
    lastPerformanceCheck = millis();
    lastWiFiCheck = millis();
    minFreeHeap = ESP.getFreeHeap();
    
    float initialTemp = temperatureRead();
    if (isTemperatureValid(initialTemp)) {
        maxTemperature = initialTemp;
        lastTemperatureC = initialTemp;
        lastTemperatureValid = true;
        overheatSensorFault = false;
    } else {
        maxTemperature = 0.0f;
        lastTemperatureC = 0.0f;
        lastTemperatureValid = false;
        overheatSensorFault = true;
        overheatLastReason = "Thermal protection disabled: temperature sensor unavailable.";
        overheatLastTimestamp = "";
        overheatTripTemp = 0.0f;
        overheatTriggeredAt = 0;
        persistOverheatNote();
        simplePrintln("WARNING: Temperature sensor unavailable at startup. Thermal protection paused.");
    }
    
    setCpuFrequencyMhz(cpuFrequencyMhz);
    simplePrintln("CPU frequency set to " + String(cpuFrequencyMhz) + " MHz");
    
    if (!overheatLatched) {
        simplePrintln("RTSP server ready on port 8554");
        simplePrintln("RTSP URL: rtsp://" + WiFi.localIP().toString() + ":8554/audio");
        simplePrintln("Max concurrent clients: " + String(MAX_CLIENTS));
    } else {
        simplePrintln("RTSP server paused due to thermal latch.");
    }
    simplePrintln("Web UI: http://" + WiFi.localIP().toString() + "/");
}

void loop() {
    ArduinoOTA.handle();
    webui_handleClient();
    
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 1000) {
        updateWebUIVars();
        lastUpdate = millis();
    }
    
    if (millis() - lastTempCheck > 60000) {
        checkTemperature();
        lastTempCheck = millis();
    }
    
    if (millis() - lastMemoryCheck > 30000) {
        uint32_t currentHeap = ESP.getFreeHeap();
        if (currentHeap < minFreeHeap) minFreeHeap = currentHeap;
        lastMemoryCheck = millis();
    }
    
    if (millis() - lastPerformanceCheck > (performanceCheckInterval * 60000UL)) {
        checkPerformance();
        lastPerformanceCheck = millis();
    }
    
    if (millis() - lastWiFiCheck > 30000) {
        checkWiFiHealth();
        lastWiFiCheck = millis();
    }
    
    checkScheduledReset();
    
    if (rtspServerEnabled) {
        // Handle disconnections
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].client && !clients[i].client.connected()) {
                if (clients[i].streaming || clients[i].sessionId != "") {
                    simplePrintln("Client " + String(i) + " disconnected");
                }
                clients[i].reset();
            }
            
            // Timeout inactive clients
            if (clients[i].client.connected() && !clients[i].streaming) {
                if (millis() - clients[i].lastActivity > 30000) {
                    simplePrintln("Client " + String(i) + " timeout");
                    clients[i].reset();
                }
            }
        }
        
        // Accept new connections
        WiFiClient newClient = rtspServer.available();
        if (newClient) {
            int slot = findFreeSlot();
            if (slot >= 0) {
                clients[slot].client = newClient;
                clients[slot].client.setNoDelay(true);
                clients[slot].parseBufferPos = 0;
                clients[slot].lastActivity = millis();
                lastRtspClientConnectMs = millis();
                rtspConnectCount++;
                simplePrintln("Client " + String(slot) + " connected from: " + newClient.remoteIP().toString());
            } else {
                simplePrintln("Max clients reached - rejecting: " + newClient.remoteIP().toString());
                newClient.stop();
            }
        }
        
        // Process all clients
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].client.connected()) {
                if (clients[i].client.available()) {
                    clients[i].lastActivity = millis();
                }
                processRTSP(clients[i], i);
            }
        }
        
        // Stream audio
        streamAudio();
    } else {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].client.connected()) {
                clients[i].reset();
            }
        }
    }
    
    if (scheduledRebootAt != 0 && millis() >= scheduledRebootAt) {
        if (scheduledFactoryReset) {
            resetToDefaultSettings();
        }
        delay(50);
        ESP.restart();
    }
}
