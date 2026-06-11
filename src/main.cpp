
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h> // Include the TFT_eSPI library
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <Wire.h>

#include "prototypes.h"
#include "display.h"
#include "version.h"

#define SCREEN_TIMEOUT 120 // seconds
#define PAIRED_MAC_MSG_TIMEOUT 5 // seconds
#define BUTTON_HOLD_ERASE_SECS 5 // seconds to hold button to erase paired MAC

#define TELEMETRY_MIN_INTERVAL_MS 5000  // Minimum 5 seconds between telemetry packets
#define TELEMETRY_MAX_INTERVAL_MS 60000 // Maximum 60 seconds between telemetry packets (periodic refresh)
#define HEARTBEAT_MISS_THRESHOLD 4  // Number of missed heartbeats before connection is considered lost

#define FUEL_SAMPLE_COUNT       8       // rolling buffer size (8 × 15s = 2-minute window)
#define FUEL_MIN_SAMPLES        3       // require at least 3 samples (~45 sec) before first report
#define FUEL_SAMPLE_INTERVAL_MS 15000   // ms between fuel ADC samples
#define FUEL_VARIANCE_THRESHOLD 25.0f   // population variance limit (~5% std-dev); tune as needed
#define FUEL_RESET_DELTA_THRESHOLD 25.0f // % jump that flushes the rolling buffer for fast large-change response
#define GPIO_EXP_CONFIRM_COUNT  5       // consecutive matching reads required before committing a new GPIO EXPANDER level (~500ms at 100ms loop)

// LiFePO4 16S pack voltage breakpoints
#define ELEC_EMPTY_V  40.0f   // 0%   — 2.5V/cell × 16, BMS cutoff floor
#define ELEC_FULL_V   53.20f  // 100% — measured resting full on this pack; above this clamps to 100%
// Factory default calibration: 2.690 V ADC → 53.20 V pack (100% SOC)
#define ELEC_BATT_DIVIDER        (53.20f / 2.690f)   // = 19.777
#define ELEC_SAMPLE_INTERVAL_MS  2000    // 2s EMA tick
#define ELEC_EMA_ALPHA           0.2f    // τ ≈ 5 samples (~10s); faster response while still rejecting single-sample ADC noise
#define ELEC_DEADBAND            1.0f    // min EMA deviation to update reported smoothedFuel; suppresses plateau ADC noise
#define ELEC_CAL_MAX             8       // max correction table entries
// Bump ELEC_CAL_VERSION when changing factory defaults; add a new "if (calVer < N)" migration block in setup().
#define ELEC_CAL_VERSION         1
#define MIN_ADC_SPACING_MV       30      // min ADC mV between adjacent cal entries (~0.6V pack); ~2× ADC noise floor
#define ADC_ELEC_TELEM_CHANGE_PCT 2.0f   // ADC_ELEC telemetry threshold; changes <2% are within ADC noise

struct ElecCalPt { float packV; float pct; };

#define STARTUP_GRACE_SECS 30    // Grace period to allow GCD connection before acting on SLEEP_PIN (matches GCD)
#define STANDALONE_SLEEP_GRACE_SECS 300  // If no GCD peer ever connects, wait 5 min before honoring SLEEP_PIN
#define DISPLAY_UPDATE_INTERVAL_MS 1000  // Refresh FUEL/BATT display every second regardless of telemetry state

#define SLEEP_PIN 35        // LOW = sleep (ignition OFF), HIGH = awake (ignition ON)
#define BUTTON_PIN 12       // GPIO34-39 do not have pullups
#define ONE_WIRE_PIN 13     // DS18B20 temperature sensor and any other One-Wire
#define ADC_FUEL_PIN 36     // 3.3v max
#define ADC_BATTERY_PIN 39  // 3.3v max

// Relay configuration
#define RELAY1_PIN 25 // used for headlights
#define RELAY2_PIN 26 // unused
#define RELAY3_PIN 27 // unused
#define RELAY4_PIN 14 // unused

#define DEBUG_MCP23008 0  // 1 = log every pin read; 0 = log GP changes only
#define DEBUG_ADC_ELEC 0  // 1 = log adcMv/battV/pct/ema every 2s; 0 = silent (telemetry line shows sent value)

// i2c configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQUENCY 100000    // purposely keep at 100kHz to allow longer bus
#define I2C_ADDR_MCP23008 0x27  // changed from 0x20 default to avoid other potential MCP23008 chips
#define I2C_ADDR_BH1750 0x23    // BH1750 default address (ADDR pin grounded); can be changed to 0x5C by wiring ADDR pin to VCC

#define ESPNOW_CHANNEL 1
#define ESPNOW_MAX_PAYLOAD 240  // Max payload after wrapper overhead subtracted (ESP-NOW limit: 250 bytes, wrapper: 9 bytes, payload: 241 bytes)

#define DISPLAY_ORIENTATION 0       // 0 = Normal, 1 = Flipped (180 degrees)

// Fuel sensor type constants (must match GCD src/config.h)
#define FUEL_SENSOR_NONE     0
#define FUEL_SENSOR_ADC_GAS  1
#define FUEL_SENSOR_GPIO_EXP 2
#define FUEL_SENSOR_ADC_ELEC 3

// ESP-NOW message types (must match GCD)
typedef enum {
    ESPNOW_MSG_TEXT = 0,
    ESPNOW_MSG_GPS_DATA = 1,
    ESPNOW_MSG_TELEMETRY = 2,
    ESPNOW_MSG_COMMAND = 3,
    ESPNOW_MSG_ACK = 4,
    ESPNOW_MSG_HEARTBEAT = 5,
    ESPNOW_MSG_IS_HOME = 6,
    ESPNOW_MSG_IS_DAYTIME = 7,
    ESPNOW_MSG_CONFIG = 8,        // GCD → GCI configuration (must match GCD)
    ESPNOW_MSG_GCI_VERSION = 9   // GCI → GCD version string
} espnow_msg_type_t;

// Config message payload (must match GCD src/types.h)
typedef struct __attribute__((packed)) {
    int32_t fuelSensorType;
    int32_t luxLightsOn;   // lux threshold to turn headlights ON
    int32_t luxLightsOff;  // lux threshold to turn headlights OFF
} structMsgConfig;

// ESP-NOW message structure (must match GCD)
typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t timestamp;
    uint16_t msg_seq_num;
    uint16_t data_len;
    uint8_t data[ESPNOW_MAX_PAYLOAD];
} espnow_message_t;

// Calculate actual packet size for sending
#define ESPNOW_PACKET_HEADER_SIZE 9
#define ESPNOW_PACKET_SIZE(data_len) (ESPNOW_PACKET_HEADER_SIZE + (data_len))

TFT_eSPI tft = TFT_eSPI(); // Create a TFT_eSPI object

OneWire oneWire(ONE_WIRE_PIN);        // Setup a oneWire instance to communicate with any OneWire device
DallasTemperature sensors(&oneWire);  // Pass oneWire reference to DallasTemperature library

Preferences preferences;

unsigned long screenStartTime = 0;
bool screenOn = true;
unsigned long lastGcdSendTime = 0;
uint16_t next_msg_seq_num = 0;
bool refreshTelemetry = false;      // Flag to refresh telemetry after connection/reconnection
bool heartbeatMissed = false;       // Flag set on first missed heartbeat

unsigned long buttonPressStartTime = 0;
bool buttonWasPressed = false;
bool enteringSleep = false;  // Flag to prevent multiple sleep attempts

// SLEEP_PIN debounce configuration
#define SLEEP_PIN_DEBOUNCE_MS 1000
int last_sleep_pin_state = HIGH;
int current_sleep_raw_state = HIGH;
unsigned long sleep_state_change_time_ms = 0;
bool debounced_sleep_state = false;

char thisDeviceMacStr[18];
char pairedMacStr[18] = "";  // Store the paired MAC address string for display updates
int consecutiveHeartbeatsMissed = HEARTBEAT_MISS_THRESHOLD;  // Start disconnected until first heartbeat
unsigned long lastHeartbeatCheckTime = 0;

// -99 indicates invalid/not installed
// variables for outbound data
int modeHeadLights = -99;
int outdoorLux = -99;
float airTemperature = -99;
float battVoltage = -99;

// Fuel sensor type (loaded from NVS, updated via ESPNOW_MSG_CONFIG from GCD)
int gciFuelSenseType = FUEL_SENSOR_ADC_GAS;
uint8_t lastMcpPins = 0xFF;  // last raw GPIO register read from MCP23008; displayed on status line

// Fuel level smoothing state
float smoothedFuel = -99;                 // value transmitted to GCD
float fuelSampleBuf[FUEL_SAMPLE_COUNT];   // rolling sample buffer
int   fuelSampleIdx = 0;
bool  fuelSampleFull = false;
unsigned long lastFuelSampleTime = 0;
float elecEma = -1.0f;  // internal EMA state for ADC_ELEC; separate from smoothedFuel to allow dead-band gating
ElecCalPt elecCal[ELEC_CAL_MAX];  // mutable NVS-backed (packV, pct) table
int       elecCalN = 0;
float     elecDivider = ELEC_BATT_DIVIDER;  // runtime divider; overridden by NVS "elec_divider"

// CLI state for ADC ELEC calibration
bool cliActive      = false;
bool cliPendingExit = false;  // true while waiting for Y/N confirmation on exit
bool cliDirty       = false;  // true when in-memory table has unsaved changes
char cliLineBuf[64];
int  cliLineLen  = 0;
bool cliLastWasCR = false;  // swallow LF that follows CR (CRLF line ending)
unsigned long sleepGraceStartMs = 0;  // reset on boot and on CLI exit to restart the sleep grace window

// Previous telemetry values for change detection
int prevModeHeadLights = -99;
float prevAirTemperature = -999;
float prevBattVoltage = -99;
float prevFuelLevel = 100.0f;

// Status variables received from GCD
bool is_home = false;      // True when GCD is within home geo-fence
bool is_daylight = true;   // True during daytime (between sunrise/sunset)

// BH1750 lux sensor and headlight relay state
bool  bhSensorPresent   = false;
float luxEma            = -99.0f;  // EMA-filtered lux; -99 = not yet valid
bool  headlightsOn      = false;
int   luxLightsOn       = 200;     // lux threshold to turn headlights ON  (loaded from NVS)
int   luxLightsOff      = 400;     // lux threshold to turn headlights OFF (loaded from NVS)
int   lastLuxSent       = -999;    // last outdoorLux value transmitted; for change-detection
unsigned long lastLuxSampleTime = 0;
volatile bool luxThresholdChanged = false;  // set by ESP-NOW callback to force immediate relay re-evaluation

// Golf cart command codes (must match GCD)
typedef enum {
    GCI_CMD_NONE = 0,
    GCI_CMD_ADD_PEER = 1,     // Add GCD MAC to GCI peer list
    GCI_CMD_REMOVE_PEER = 2,  // Future: remove peer
    GCI_CMD_REBOOT = 3,       // Future: reboot GCI
    // Add more commands as needed
} gci_command_t;

typedef struct struct_msg_to_gcd {
  int modeLights;
  int outdoorLux;
  float airTemp;
  float battVolts;
  float fuelPct;
} structMsgToGcd;

structMsgToGcd dataToGcd;

// variables for inbound data
int cmdFromGcd;

typedef struct struct_msg_from_gcd {
  int cmdNumber;
  uint8_t macAddr[6];  // For pairing command and future use
} structMsgFromGcd;

structMsgFromGcd dataFromGcd;

esp_now_peer_info_t peerInfo;

String tx_success;   // Variable to store if sending data was successful


// Bit-bang 9 SCL pulses + STOP before Wire.begin() to release any I2C slave
// stuck mid-transaction from a prior ESP32 reset or deep-sleep wake.
// Safe to call unconditionally — harmless when the bus is already clean.
void i2cBusRecover() {
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  bool sdaStuck = false;
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
    if (digitalRead(I2C_SDA_PIN) == LOW) sdaStuck = true;
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  // STOP condition: SDA rises while SCL is HIGH
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(5);
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  if (sdaStuck) Serial.println("I2C: SDA was stuck — bus recovery applied");
  else          Serial.println("I2C: bus clean");
}

void initMCP23008() {
  // Set all pins as inputs
  Wire.beginTransmission(I2C_ADDR_MCP23008);
  Wire.write(0x00);  // IODIR register
  Wire.write(0xFF);  // all inputs
  uint8_t err = Wire.endTransmission();
  if (err != 0) { Serial.printf("MCP23008 IODIR write failed (I2C err=%d)\n", err); return; }

  // Enable internal pull-ups on all pins so undriven pins read 1 (dry/fault), not 0
  Wire.beginTransmission(I2C_ADDR_MCP23008);
  Wire.write(0x06);  // GPPU register
  Wire.write(0xFF);  // pull-ups on all pins
  err = Wire.endTransmission();
  if (err != 0) { Serial.printf("MCP23008 GPPU write failed (I2C err=%d)\n", err); return; }

  // Read back IODIR to confirm writes are landing
  Wire.beginTransmission(I2C_ADDR_MCP23008);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.requestFrom((uint8_t)I2C_ADDR_MCP23008, (uint8_t)1);
  uint8_t iodir = Wire.available() ? Wire.read() : 0xFF;
  Serial.printf("MCP23008 initialized (IODIR readback=0x%02X)\n", iodir);
}

// Probe I2C address 0x23 for BH1750; if present, write Continuous High Resolution Mode 1.
// Sets bhSensorPresent. Safe to call even when sensor is absent.
void initBH1750() {
  Wire.beginTransmission(I2C_ADDR_BH1750);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("BH1750: not found (I2C err=%d)\n", err);
    return;
  }
  Wire.beginTransmission(I2C_ADDR_BH1750);
  Wire.write(0x10);  // Continuous High Resolution Mode 1: ~1 lux resolution, 120ms/sample
  err = Wire.endTransmission();
  bhSensorPresent = (err == 0);
  Serial.printf("BH1750: %s\n", bhSensorPresent ? "initialized" : "mode write failed");
}

// Read one 2-byte sample from BH1750 and return lux (raw / 1.2 per datasheet).
// Returns -1.0 on I2C failure; caller should discard and retain previous EMA.
float readBH1750Lux() {
  if (Wire.requestFrom((uint8_t)I2C_ADDR_BH1750, (uint8_t)2) != 2) return -1.0f;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  return raw / 1.2f;
}

// Full runtime recovery: release Wire, bit-bang bus clear, restart Wire, re-init chip.
// Wire.end() is required before bit-banging so the I2C peripheral releases SDA/SCL.
void mcpRuntimeRecover() {
  Serial.println("MCP23008: runtime bus recovery");
  Wire.end();
  i2cBusRecover();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  initMCP23008();
}

// Returns mapped SOC % (25/50/75/100) or -99 on I2C failure or invalid pin state.
// GP0=bit0 (25% sensor), GP1=bit1 (50% sensor), GP2=bit2 (75% sensor). Logic 1 = liquid present/at or above that level.
int readMCP23008Fuel() {
  static uint8_t failStreak = 0;
  static uint8_t lastPins   = 0xFF;

  Wire.beginTransmission(I2C_ADDR_MCP23008);
  Wire.write(0x09);  // GPIO register
  bool txFail = Wire.endTransmission(true) != 0;
  bool rxFail = !txFail && (Wire.requestFrom((uint8_t)I2C_ADDR_MCP23008, (uint8_t)1) != 1);

  if (txFail || rxFail) {
    failStreak++;
    Serial.printf("MCP23008: I2C %s fail (streak=%d)\n", txFail ? "tx" : "rx", failStreak);
    if (failStreak == 3 || (failStreak > 3 && (failStreak - 3) % 30 == 0))
      mcpRuntimeRecover();  // full Wire.end+recover+Wire.begin+init; retries every 30 failures
    return -99;
  }
  failStreak = 0;

  uint8_t pins = Wire.read();
  lastMcpPins = pins;
  bool gp0 = (pins >> 0) & 1;
  bool gp1 = (pins >> 1) & 1;
  bool gp2 = (pins >> 2) & 1;

#if DEBUG_MCP23008
  Serial.printf("MCP23008: pins=0x%02X [%d%d%d%d%d%d%d%d] GP0=%d GP1=%d GP2=%d\n",
                pins,
                (pins>>7)&1, (pins>>6)&1, (pins>>5)&1, (pins>>4)&1,
                (pins>>3)&1, (pins>>2)&1, (pins>>1)&1, (pins>>0)&1,
                gp0, gp1, gp2);
#else
  if (pins != lastPins)
    Serial.printf("MCP23008: GP0=%d GP1=%d GP2=%d (0x%02X)\n", gp0, gp1, gp2, pins);
#endif
  lastPins = pins;

  int result = -99;
  if ( gp0 &&  gp1 &&  gp2) result = 100;
  else if ( gp0 &&  gp1 && !gp2) result = 75;
  else if ( gp0 && !gp1 && !gp2) result = 50;
  else if (!gp0 && !gp1 && !gp2) result = 25;

  if (result == -99) return -99;
  return result;
}


/**************
 *    SETUP   *
 **************/

static void sortElecCal();  // defined after setup(); forward-declared so migration in setup() can call it

void setup(void) {
  Serial.begin(9600);
  Serial.println("\n=== GCI BOOT " VERSION " ===");
  sensors.begin();

  // set relay pinModes
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  // set relays to LOW (off)
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SLEEP_PIN, INPUT);  // External pull-down: LOW = sleep (ignition OFF), HIGH = awake (ignition ON)

  // Check for spurious wake before any display init.
  // Require SLEEP_PIN to remain HIGH for the full settle window — if it dips LOW
  // at any point, treat it as a glitch and go back to sleep. This handles
  // oscillating or multi-pulse glitch patterns.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    unsigned long settle_start = millis();
    while (millis() - settle_start < 1000) {
      if (digitalRead(SLEEP_PIN) == LOW) {
        Serial.println("Spurious wake detected - returning to deep sleep");
        Serial.flush();
        esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_PIN, 1);
        esp_deep_sleep_start();
      }
      delay(10);
    }
  }

  gpio_hold_dis((gpio_num_t)TFT_BL);  // Release hold from deep sleep (if any)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // Turn on backlight initially

  // Initialize WiFi directly via IDF — do NOT call WiFi.mode().
  // WiFi.mode() calls esp_wifi_init() with WIFI_INIT_CONFIG_DEFAULT, which
  // requests 4 static rx buffers. This device can only allocate 3, so the
  // call fails and leaks those 3 partial allocations, leaving insufficient
  // heap for a subsequent re-init. arduino-esp32 already calls
  // esp_netif_init() and esp_event_loop_create_default() in initArduino()
  // before setup() runs, so we can go straight to esp_wifi_init() here.
  {
    wifi_init_config_t wCfg = WIFI_INIT_CONFIG_DEFAULT();
    wCfg.static_rx_buf_num = 3;
    if (esp_wifi_init(&wCfg) != ESP_OK) {
      Serial.println("ESP-NOW: WiFi init failed");
      return;
    }
  }
  esp_wifi_set_mode(WIFI_MODE_STA);
  if (esp_wifi_start() != ESP_OK) {
    Serial.println("ESP-NOW: WiFi start failed");
    return;
  }

  tft.init(); // Initialize the display
  tft.setRotation(DISPLAY_ORIENTATION == 0 ? 1 : 3); // 1 = landscape normal, 3 = landscape flipped

  // Show splash screen for 2 seconds
  displaySplashScreen(tft, VERSION);

  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color to white

  // Display this device's MAC address
  createMacAddressStr(thisDeviceMacStr);
  displayMacLine(tft, thisDeviceMacStr);
  displayPrLine(tft, WAITING_FOR_PAIRING, "");
  Serial.println(thisDeviceMacStr);     // Display on Serial

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("ESP-NOW initialized");

  // Lock the radio onto ESPNOW_CHANNEL. Must be called AFTER esp_now_init()
  // because WiFi.mode() starts the driver asynchronously — calling
  // esp_wifi_set_channel() immediately after WiFi.mode() returns
  // ESP_ERR_WIFI_NOT_INIT. By the time esp_now_init() returns, the driver
  // is guaranteed started.
  esp_err_t chErr = ESP_FAIL;
  for (int i = 0; i < 5; i++) {
    chErr = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (chErr == ESP_OK) break;
    delay(20);
  }
  uint8_t home_ch = 0;
  wifi_second_chan_t home_sec = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&home_ch, &home_sec);
  if (chErr != ESP_OK || home_ch != ESPNOW_CHANNEL) {
    Serial.printf("ESP-NOW: channel set FAILED (err=%d, home=%u, want=%u)\n",
                  (int)chErr, (unsigned)home_ch, (unsigned)ESPNOW_CHANNEL);
  } else {
    Serial.printf("ESP-NOW: home channel = %u\n", (unsigned)home_ch);
  }

  // Open preferences and check for saved peer MAC
  preferences.begin("gci", false);

  // Load fuel sensor type; default NONE — GCD sends the configured type on connect
  gciFuelSenseType = preferences.getInt("fuel_sense_type", FUEL_SENSOR_NONE);
  Serial.printf("Fuel sense type: %d\n", gciFuelSenseType);

  // Load BH1750 lux thresholds (GCD pushes updates via ESPNOW_MSG_CONFIG)
  luxLightsOn  = preferences.getInt("lux_on",  200);
  luxLightsOff = preferences.getInt("lux_off", 400);
  Serial.printf("Lux thresholds: on=%d off=%d\n", luxLightsOn, luxLightsOff);

  // Load ELEC_CAL table from NVS; fall back to LiFePO4 16S factory curve if not present
  {
    int n = preferences.getInt("elec_cal_n", 0);
    if (n > 0 && n <= ELEC_CAL_MAX) {
      preferences.getBytes("elec_cal_pts", elecCal, n * sizeof(ElecCalPt));
      elecCalN = n;
    } else {
      // Fresh NVS: write current factory defaults (4-point LiFePO4 knee curve).
      elecCalN   = 4;
      elecCal[0] = {40.00f,   0.0f};   // 0%  — BMS cutoff floor
      elecCal[1] = {48.00f,  10.0f};   // 10% — lower knee (LiFePO4 steep rise from cutoff)
      elecCal[2] = {51.20f,  20.0f};   // 20% — knee exit, entering flat plateau
      elecCal[3] = {53.20f, 100.0f};   // 100% — measured anchor (2690 mV ADC)
      preferences.putInt("elec_cal_ver", ELEC_CAL_VERSION);
      preferences.putInt("elec_cal_n", elecCalN);
      preferences.putBytes("elec_cal_pts", elecCal, elecCalN * sizeof(ElecCalPt));
      preferences.putFloat("elec_divider", elecDivider);  // save compile-time default so CLI adcMv column is correct
    }
    float nvsDivider = preferences.getFloat("elec_divider", 0.0f);
    if (nvsDivider > 5.0f) elecDivider = nvsDivider;

    // Version migration: upgrade old factory defaults while preserving user-entered entries.
    int calVer = preferences.getInt("elec_cal_ver", 0);
    if (calVer < ELEC_CAL_VERSION) {
      // V0→V1: remove old factory 25% = 52.00V knee entry (not real user data).
      for (int i = 0; i < elecCalN; i++) {
        if (fabsf(elecCal[i].pct - 25.0f) < 1.0f && fabsf(elecCal[i].packV - 52.00f) < 0.1f) {
          for (int j = i; j < elecCalN - 1; j++) elecCal[j] = elecCal[j+1];
          elecCalN--;
          break;
        }
      }
      // Add improved LiFePO4 knee points if not already present and space allows.
      bool has10 = false, has20 = false;
      for (int i = 0; i < elecCalN; i++) {
        if (fabsf(elecCal[i].pct - 10.0f) < 1.0f) has10 = true;
        if (fabsf(elecCal[i].pct - 20.0f) < 1.0f) has20 = true;
      }
      if (!has10 && elecCalN < ELEC_CAL_MAX) elecCal[elecCalN++] = {48.00f, 10.0f};
      if (!has20 && elecCalN < ELEC_CAL_MAX) elecCal[elecCalN++] = {51.20f, 20.0f};
      sortElecCal();
      // Reset divider if it still matches the old factory default (53.20/2.684 = 19.821).
      if (fabsf(elecDivider - (53.20f / 2.684f)) < 0.01f) {
        elecDivider = ELEC_BATT_DIVIDER;
        preferences.putFloat("elec_divider", elecDivider);
      }
      preferences.putInt("elec_cal_ver", ELEC_CAL_VERSION);
      preferences.putInt("elec_cal_n", elecCalN);
      preferences.putBytes("elec_cal_pts", elecCal, elecCalN * sizeof(ElecCalPt));
    }
  }

  i2cBusRecover();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  {
    Serial.print("I2C scan: ");
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("0x%02X ", addr);
        found = true;
      }
    }
    Serial.println(found ? "" : "no devices found");
  }
  if (gciFuelSenseType == FUEL_SENSOR_GPIO_EXP)
    initMCP23008();
  initBH1750();

  // Try to load saved peer MAC address
  uint8_t savedMac[6] = {0};
  size_t macLen = preferences.getBytes("peer_mac", savedMac, 6);

  bool hasSavedPeer = false;
  if (macLen == 6) {
    // Check if MAC is valid (not all zeros)
    bool isValid = false;
    for (int i = 0; i < 6; i++) {
      if (savedMac[i] != 0) {
        isValid = true;
        break;
      }
    }

    if (isValid) {
      // Add saved peer
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, savedMac, 6);
      // channel = 0 means "use whatever the local home channel is" —
      // sidesteps IDF's "Peer channel != home channel" send-fail check.
      peerInfo.channel = 0;
      peerInfo.ifidx = WIFI_IF_STA;
      peerInfo.encrypt = false;

      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        sprintf(pairedMacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                savedMac[0], savedMac[1], savedMac[2],
                savedMac[3], savedMac[4], savedMac[5]);
        Serial.printf("Loaded saved peer: %s\n", pairedMacStr);

        // Display paired status (starts RED until heartbeat received)
        displayMacLine(tft, thisDeviceMacStr);
        displayPrLine(tft, PAIRED_DISCONNECTED, pairedMacStr);

        hasSavedPeer = true;
      }
    }
  }

  if (!hasSavedPeer) {
    Serial.println("No saved peer - waiting for pairing commands from GCD");

    // Display pairing status
    displayMacLine(tft, thisDeviceMacStr);
    displayPrLine(tft, WAITING_FOR_PAIRING, "");
  }
  screenStartTime = millis();
  sleepGraceStartMs = millis();  // start sleep grace window from end of setup

} /* END SETUP */


// ---- ELEC_CAL CLI helpers ----

static void sortElecCal() {
  for (int i = 1; i < elecCalN; i++) {
    ElecCalPt key = elecCal[i];
    int j = i - 1;
    while (j >= 0 && elecCal[j].packV > key.packV) { elecCal[j+1] = elecCal[j]; j--; }
    elecCal[j+1] = key;
  }
}

// Returns internal index of entry whose packV is within MIN_ADC_SPACING_MV of newPackV, or -1 if clear.
// skipInternalIdx: internal index to exclude (the entry being replaced); pass -1 to check all.
static int adcConflictIdx(float newPackV, int skipInternalIdx) {
  float newAdcMv = newPackV / elecDivider * 1000.0f;
  int ins = elecCalN;
  for (int i = 0; i < elecCalN; i++) {
    if (newPackV < elecCal[i].packV) { ins = i; break; }
  }
  if (ins > 0 && (ins - 1) != skipInternalIdx) {
    if (fabsf(newAdcMv - elecCal[ins-1].packV / elecDivider * 1000.0f) < MIN_ADC_SPACING_MV)
      return ins - 1;
  }
  if (ins < elecCalN && ins != skipInternalIdx) {
    if (fabsf(newAdcMv - elecCal[ins].packV / elecDivider * 1000.0f) < MIN_ADC_SPACING_MV)
      return ins;
  }
  return -1;
}

// Build display (pct-descending) → array index mapping; same sort as printElecCalTable.
static void buildDispMap(int out[]) {
  for (int i = 0; i < elecCalN; i++) out[i] = i;
  for (int i = 1; i < elecCalN; i++) {
    int key = out[i]; int j = i - 1;
    while (j >= 0 && elecCal[out[j]].pct < elecCal[key].pct) { out[j+1] = out[j]; j--; }
    out[j+1] = key;
  }
}
static int displayToArrayIdx(int n) {  // n = 1-based display index
  int m[ELEC_CAL_MAX]; buildDispMap(m); return m[n - 1];
}
static int arrayToDisplayIdx(int ai) {  // array index → 1-based display index
  int m[ELEC_CAL_MAX]; buildDispMap(m);
  for (int i = 0; i < elecCalN; i++) if (m[i] == ai) return i + 1;
  return -1;
}

static void printElecCalTable() {
  // Sort a display copy by pct ascending (storage order is by packV for interpolation)
  ElecCalPt disp[ELEC_CAL_MAX];
  memcpy(disp, elecCal, elecCalN * sizeof(ElecCalPt));
  for (int i = 1; i < elecCalN; i++) {
    ElecCalPt key = disp[i];
    int j = i - 1;
    while (j >= 0 && disp[j].pct < key.pct) { disp[j+1] = disp[j]; j--; }
    disp[j+1] = key;
  }
  Serial.printf("Table (%d of %d entries):\n", elecCalN, ELEC_CAL_MAX);
  Serial.println("  Idx   Charge   packV   adcMv");
  for (int i = 0; i < elecCalN; i++) {
    int adcMv = (int)(disp[i].packV / elecDivider * 1000.0f);
    Serial.printf("  [%d]   %3.0f%%   %6.2fV   %4dmV\n",
                  i+1, disp[i].pct, disp[i].packV, adcMv);
  }
  // Flag segments that are too narrow to measure accurately — suggest a midpoint reading
  for (int i = 1; i < elecCalN; i++) {
    float adcSpan = (elecCal[i].packV - elecCal[i-1].packV) / elecDivider * 1000.0f;
    if (adcSpan < 0.1f) continue;
    float slope = fabsf(elecCal[i].pct - elecCal[i-1].pct) / adcSpan;
    if (slope > 1.5f) {
      float midV = (elecCal[i-1].packV + elecCal[i].packV) * 0.5f;
      Serial.printf("Needs more data: %.0f%% @ %.2fV -> %.0f%% @ %.2fV spans only %.1fV\n",
                    elecCal[i-1].pct, elecCal[i-1].packV, elecCal[i].pct, elecCal[i].packV,
                    elecCal[i].packV - elecCal[i-1].packV);
      Serial.printf("  Charge/discharge to ~%.2fV, note BMS%%, then add '<pct> %.2f'\n", midV, midV);
    }
  }
}

static void processCliLine(const char* line) {
  while (*line == ' ' || *line == '\t') line++;

  if (!cliActive) {
    if (strcmp(line, "gci") == 0) {
      cliActive = true;
      cliDirty  = false;
      Serial.println("\n=== Calibration for ADC ELECTRIC ===");
      printElecCalTable();
      Serial.println("Commands: <pct> <V>  100 <V> <adcMv>  del <Idx>  edit <Idx> <pct> <V>  show  restore  save  exit  help");
      Serial.print("gci> ");
    }
    return;
  }

  // Pending exit Y/N confirmation
  if (cliPendingExit) {
    if (line[0] == 'Y' || line[0] == 'y') {
      Serial.println("Exiting. Changes discarded.");
      cliActive      = false;
      cliPendingExit = false;
      cliDirty       = false;
      sleepGraceStartMs = millis();
    } else if (line[0] == '\0') {
      Serial.print("Changes not saved, exit anyway (Y/N)? ");  // blank: re-ask
    } else {
      Serial.println("Exit cancelled.");
      cliPendingExit = false;
      Serial.print("gci> ");
    }
    return;
  }

  // blank line — ignore
  if (line[0] == '\0') {
    Serial.print("gci> ");
    return;
  }

  // "help"
  if (strcmp(line, "help") == 0) {
    Serial.println("--- Commands ---");
    Serial.println("  <pct> <V>             Add calibration point (charge 0-99, e.g. 85 52.50).");
    Serial.println("                        <pct> = charge % from cart display");
    Serial.println("                        <V>   = pack voltage from cart display");
    Serial.println("  100 <V> <adcMv>       100% calibration entry. Computes and saves divider ratio.");
    Serial.println("                        e.g. 100 53.20 2684");
    Serial.println("                        <V>     = pack voltage from cart display");
    Serial.println("                        <adcMv> = ADC reading shown on GCI display screen");
    Serial.println("                        Both values can be noted while cart is running, entered later.");
    Serial.println("  del <Idx>             Delete Idx (e.g. del 2).");
    Serial.println("  edit <Idx> <pct> <V>  Fix Idx (e.g. edit 2 80 52.40).");
    Serial.println("  show                  Re-display the calibration table.");
    Serial.println("  restore               Reset table and divider to factory defaults.");
    Serial.println("  save                  Save to NVS without exiting.");
    Serial.println("  exit                  Exit. Prompts if unsaved changes exist.");
    Serial.println("  help                  Show this help.");
    Serial.println("--- Table columns ---");
    Serial.println("  Charge  = charge % from cart display");
    Serial.println("  packV   = pack voltage from cart display");
    Serial.println("  adcMv   = expected ADC pin reading (verify against GCI screen)");
    Serial.print("gci> ");
    return;
  }

  // "show" — re-display the table
  if (strcmp(line, "show") == 0) {
    printElecCalTable();
    Serial.print("gci> ");
    return;
  }

  // "restore" — reset table and divider to factory defaults
  if (strcmp(line, "restore") == 0) {
    elecCalN   = 4;
    elecCal[0] = {40.00f,   0.0f};
    elecCal[1] = {48.00f,  10.0f};
    elecCal[2] = {51.20f,  20.0f};
    elecCal[3] = {53.20f, 100.0f};
    elecDivider = ELEC_BATT_DIVIDER;
    cliDirty = true;
    Serial.println("Table reset to factory defaults. Type 'save' to commit.");
    printElecCalTable();
    Serial.print("gci> ");
    return;
  }

  // "save" — commit to NVS, stay in CLI
  if (strcmp(line, "save") == 0) {
    preferences.putInt("elec_cal_n", elecCalN);
    preferences.putBytes("elec_cal_pts", elecCal, elecCalN * sizeof(ElecCalPt));
    cliDirty = false;
    Serial.println("Saved.");
    Serial.print("gci> ");
    return;
  }

  // "exit" — exit immediately if clean, prompt if dirty
  if (strcmp(line, "exit") == 0) {
    if (!cliDirty) {
      Serial.println("Exiting.");
      cliActive = false;
      sleepGraceStartMs = millis();
    } else {
      cliPendingExit = true;
      Serial.print("Changes not saved, exit anyway (Y/N)? ");
    }
    return;
  }

  // "del <n>"
  if (strncmp(line, "del ", 4) == 0) {
    int n = atoi(line + 4);
    int arrayIdx = displayToArrayIdx(n);
    if (n < 1 || n > elecCalN) {
      Serial.printf("? Idx %d out of range (1-%d)\n", n, elecCalN);
    } else if (fabsf(elecCal[arrayIdx].pct - 100.0f) < 1.0f) {
      Serial.println("? Cannot delete the 100% anchor. Use '100 <V> <adcMv>' to replace it.");
    } else if (fabsf(elecCal[arrayIdx].pct - 0.0f) < 1.0f) {
      Serial.println("? Cannot delete the 0% anchor. Use '0 <V>' to replace it.");
    } else {
      for (int i = arrayIdx; i < elecCalN-1; i++) elecCal[i] = elecCal[i+1];
      elecCalN--;
      cliDirty = true;
      Serial.printf("Deleted Idx %d.\n", n);
      printElecCalTable();
    }
    Serial.print("gci> ");
    return;
  }

  // "edit <n> <pct> <V>"
  if (strncmp(line, "edit ", 5) == 0) {
    int n; float pct, packV;
    if (sscanf(line+5, "%d %f %f", &n, &pct, &packV) == 3) {
      if (n < 1 || n > elecCalN) {
        Serial.printf("? Idx %d out of range (1-%d)\n", n, elecCalN);
      } else {
        int skipIdx = displayToArrayIdx(n);
        bool blocked = false;
        for (int i = 0; i < elecCalN; i++) {
          if (i == skipIdx) continue;
          if (fabsf(elecCal[i].pct - pct) < 1.0f) {
            Serial.printf("? Idx [%d] already at %.0f%%. Use 'del %d' to replace it.\n", arrayToDisplayIdx(i), elecCal[i].pct, arrayToDisplayIdx(i));
            blocked = true; break;
          }
        }
        if (!blocked) {
          int ci = adcConflictIdx(packV, skipIdx);
          if (ci >= 0) {
            Serial.printf("? %.2fV is %.2fV from existing %.0f%% @ %.2fV — entries need >=0.6V spacing.\n",
                          packV, fabsf(packV - elecCal[ci].packV), elecCal[ci].pct, elecCal[ci].packV);
            Serial.println("  Run 'show', then: edit <Idx> <pct> <V>  or  del <Idx> then re-add.");
          } else {
            elecCal[skipIdx] = {packV, pct};
            sortElecCal();
            cliDirty = true;
            Serial.printf("Edited Idx %d: %.0f%% = %.2fV\n", n, pct, packV);
            printElecCalTable();
          }
        }
      }
    } else {
      Serial.println("? Usage: edit <Idx> <pct> <V>");
    }
    Serial.print("gci> ");
    return;
  }

  // 3-token: "100 <V> <adcMv>" — calibration entry, sets divider
  {
    float t1, packV, adcMv;
    if (sscanf(line, "%f %f %f", &t1, &packV, &adcMv) == 3
            && fabsf(t1 - 100.0f) < 0.1f && packV > 10.0f && adcMv > 100.0f) {
      if (elecCalN >= ELEC_CAL_MAX) {
        Serial.printf("? Table full (%d max). Delete an Idx first.\n", ELEC_CAL_MAX);
        Serial.print("gci> ");
        return;
      }
      // Replace existing 100% row if present (remove it so the new entry takes its place)
      for (int i = 0; i < elecCalN; i++) {
        if (fabsf(elecCal[i].pct - 100.0f) < 1.0f) {
          for (int j = i; j < elecCalN-1; j++) elecCal[j] = elecCal[j+1];
          elecCalN--;
          break;
        }
      }
      float newDivider = packV / (adcMv / 1000.0f);
      preferences.putFloat("elec_divider", newDivider);
      elecDivider = newDivider;
      {
        int ci = adcConflictIdx(packV, -1);
        if (ci >= 0) {
          Serial.printf("? %.2fV is %.2fV from existing %.0f%% @ %.2fV — entries need >=0.6V spacing.\n",
                        packV, fabsf(packV - elecCal[ci].packV), elecCal[ci].pct, elecCal[ci].packV);
          Serial.println("  Run 'show', then: del <Idx> to remove the conflicting entry, then retry.");
          Serial.print("gci> ");
          return;
        }
      }
      elecCal[elecCalN++] = {packV, 100.0f};
      sortElecCal();
      cliDirty = true;
      Serial.printf("Divider set: %.3f (saved)\n", newDivider);
      Serial.printf("Added: 100%% = %.2fV\n", packV);
      printElecCalTable();
      Serial.print("gci> ");
      return;
    }
  }

  // 2-token: "<pct> <V>" — normal entry (pct 0–99)
  {
    float pct, packV;
    if (sscanf(line, "%f %f", &pct, &packV) == 2
            && pct >= 0.0f && pct <= 99.9f && packV > 10.0f) {
      if (elecCalN >= ELEC_CAL_MAX) {
        Serial.printf("? Table full (%d max). Delete an Idx first.\n", ELEC_CAL_MAX);
        Serial.print("gci> ");
        return;
      }
      // 0% anchor: replace existing row instead of blocking
      if (pct < 1.0f) {
        for (int i = 0; i < elecCalN; i++) {
          if (fabsf(elecCal[i].pct - 0.0f) < 1.0f) {
            for (int j = i; j < elecCalN-1; j++) elecCal[j] = elecCal[j+1];
            elecCalN--;
            break;
          }
        }
      } else {
        bool blocked = false;
        for (int i = 0; i < elecCalN; i++) {
          if (fabsf(elecCal[i].pct - pct) < 1.0f) {
            Serial.printf("? Idx [%d] already at %.0f%%. Use 'del %d' to replace it.\n", arrayToDisplayIdx(i), elecCal[i].pct, arrayToDisplayIdx(i));
            blocked = true; break;
          }
        }
        if (blocked) { Serial.print("> "); return; }
      }
      {
        int ci = adcConflictIdx(packV, -1);
        if (ci >= 0) {
          Serial.printf("? %.2fV is %.2fV from existing %.0f%% @ %.2fV — entries need >=0.6V spacing.\n",
                        packV, fabsf(packV - elecCal[ci].packV), elecCal[ci].pct, elecCal[ci].packV);
          Serial.println("  Run 'show', then: edit <Idx> <pct> <V>  or  del <Idx> then re-add.");
          Serial.print("gci> ");
          return;
        }
      }
      elecCal[elecCalN++] = {packV, pct};
      sortElecCal();
      cliDirty = true;
      Serial.printf("Added: %.0f%% = %.2fV\n", pct, packV);
      printElecCalTable();
    } else {
      Serial.println("? Commands: <pct> <V>  100 <V> <adcMv>  del <Idx>  edit <Idx> <pct> <V>  show  restore  save  exit  help");
    }
  }
  Serial.print("> ");
}

/**************
 *    LOOP    *
 **************/

void loop() {

  static float tempC_0, tempF_0;
  static float voltsFuelADC, voltsBattADC;
  static int percentFuel;
  static bool sendData = false;
  static unsigned long lastDisplayUpdateTime = 0;

  // Determine peer state once per loop — used by sleep logic and telemetry
  esp_now_peer_info_t peer;
  bool hasPeer = (esp_now_fetch_peer(true, &peer) == ESP_OK);

  // Check SLEEP_PIN with debounce
  // Requires pin to be stable for SLEEP_PIN_DEBOUNCE_MS before acting
  int sleep_raw_reading = digitalRead(SLEEP_PIN);
  unsigned long now = millis();

  if (sleep_raw_reading != current_sleep_raw_state) {
    current_sleep_raw_state = sleep_raw_reading;
    sleep_state_change_time_ms = now;
  }

  if (current_sleep_raw_state != last_sleep_pin_state) {
    if ((now - sleep_state_change_time_ms) >= SLEEP_PIN_DEBOUNCE_MS) {
      last_sleep_pin_state = current_sleep_raw_state;
      debounced_sleep_state = (current_sleep_raw_state == LOW);
    }
  }

  // Act on SLEEP_PIN when either:
  // 1. GCD is connected (early exit - GCD has received our heartbeat and will enter GCI_MODE)
  // 2. Grace period elapsed (safeguard - sleep even if GCD never connected)
  bool gcd_connected = hasPeer && (consecutiveHeartbeatsMissed < HEARTBEAT_MISS_THRESHOLD);
  unsigned long sleepGraceSecs = hasPeer ? STARTUP_GRACE_SECS : STANDALONE_SLEEP_GRACE_SECS;
  bool grace_elapsed = (millis() - sleepGraceStartMs) >= (sleepGraceSecs * 1000UL);
  if (!enteringSleep && !cliActive && debounced_sleep_state && (gcd_connected || grace_elapsed)) {
    enteringSleep = true;
    enterDeepSleep();
  }

  // Check for button state
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
  static bool splashDisplayed = false;  // Track if splash is currently shown
  static bool screenWasOff = false;     // Track if screen was off when button pressed

  if (buttonPressed && !buttonWasPressed) {
    // Button just pressed
    buttonPressStartTime = millis();
    buttonWasPressed = true;
    splashDisplayed = false;
    screenWasOff = !screenOn;

    // If screen is off, turn it on immediately and redraw
    if (screenWasOff) {
      tft.fillScreen(TFT_BLACK);  // Clear any artifacts
      digitalWrite(TFT_BL, HIGH);
      screenOn = true;
      redrawDisplayHeader();
      sendData = true;
      screenStartTime = millis();
    }
  } else if (buttonPressed && buttonWasPressed) {
    // Button is being held
    unsigned long pressDuration = millis() - buttonPressStartTime;

    if (pressDuration >= (BUTTON_HOLD_ERASE_SECS * 1000)) {
      // Long press threshold reached - ask for confirmation
      tft.fillScreen(TFT_BLACK);
      tft.setTextSize(1);  // Ensure normal text size
      tft.setTextDatum(TL_DATUM);  // Ensure normal datum
      tft.setCursor(10, 40);
      tft.setTextFont(4);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Reset GCD pairing?");
      tft.setCursor(10, 80);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Press to confirm");

      // Wait for button release first
      while (digitalRead(BUTTON_PIN) == LOW) {
        delay(50);
      }
      delay(200); // Debounce

      // Wait for confirmation press (5 second timeout)
      unsigned long confirmStartTime = millis();
      bool confirmed = false;

      while ((millis() - confirmStartTime) < 5000) {
        if (digitalRead(BUTTON_PIN) == LOW) {
          confirmed = true;
          // Wait for release
          while (digitalRead(BUTTON_PIN) == LOW) {
            delay(50);
          }
          break;
        }
        delay(50);
      }

      if (confirmed) {
        // Erase paired MAC
        preferences.remove("peer_mac");
        Serial.println("Paired MAC address erased from EEPROM");

        // Display confirmation message
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setCursor(10, 40);
        tft.setTextFont(4);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("Pairing reset!");

        delay(2000);

        // Reboot
        Serial.println("Rebooting...");
        ESP.restart();
      } else {
        // Cancelled - return to normal display
        Serial.println("Pairing reset cancelled");
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setCursor(10, 40);
        tft.setTextFont(4);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.println("Cancelled");

        delay(1000);

        // Redraw normal display
        tft.fillScreen(TFT_BLACK);
        redrawDisplayHeader();
        sendData = true;
        screenStartTime = millis();
      }

      buttonWasPressed = false;
      splashDisplayed = false;
    } else if (!splashDisplayed && pressDuration >= 500) {
      // Button held but not yet at reset threshold - show splash screen
      tft.fillScreen(TFT_BLACK);
      int16_t screenWidth = tft.width();
      tft.setTextDatum(TC_DATUM);
      tft.setTextFont(4);
      tft.setTextSize(2);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("GCI", screenWidth / 2, 20);
      tft.setTextSize(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      char versionLine[32];
      snprintf(versionLine, sizeof(versionLine), "Ver. %s", VERSION);
      tft.drawString(versionLine, screenWidth / 2, 85);
      tft.setTextDatum(TL_DATUM);
      tft.setTextSize(1);  // Reset text size
      splashDisplayed = true;
    }
  } else if (!buttonPressed && buttonWasPressed) {
    // Button just released
    buttonWasPressed = false;

    // If splash was displayed, redraw normal screen
    if (splashDisplayed) {
      tft.fillScreen(TFT_BLACK);
      redrawDisplayHeader();
      sendData = true;
    }
    splashDisplayed = false;
    screenWasOff = false;
    screenStartTime = millis();
  }

  // Check if screen timeout has been reached
  if (screenOn && (millis() - screenStartTime) >= (SCREEN_TIMEOUT * 1000)) {
    // Turn off the backlight only
    digitalWrite(TFT_BL, LOW);  // Turn off backlight
    screenOn = false;
  }

  // Check for heartbeat timeout and update connection status display
  static bool lastConnectionStatus = false;  // Track if we were connected last check
  bool isConnected = (consecutiveHeartbeatsMissed < HEARTBEAT_MISS_THRESHOLD);

  // Check if we need to increment missed heartbeat counter (every 10 seconds expected)
  if (hasPeer && (millis() - lastHeartbeatCheckTime) >= 10000) {
    consecutiveHeartbeatsMissed++;
    lastHeartbeatCheckTime = millis();

    // Set flag on first missed heartbeat for telemetry re-send
    if (consecutiveHeartbeatsMissed == 1) {
      heartbeatMissed = true;
      if (!cliActive) Serial.println("Heartbeat missed - telemetry will be sent when connection re-established");
    }

    if (consecutiveHeartbeatsMissed >= HEARTBEAT_MISS_THRESHOLD) {
      if (!cliActive) Serial.printf("Connection lost - missed %d heartbeats\n", consecutiveHeartbeatsMissed);
    }
  }

  // Update display if connection status changed
  if (!splashDisplayed && hasPeer && (isConnected != lastConnectionStatus) && strlen(pairedMacStr) > 0) {
    PairingStatus status = isConnected ? PAIRED_CONNECTED : PAIRED_DISCONNECTED;
    displayPrLine(tft, status, pairedMacStr);

    // On connection, send our version string to GCD
    if (isConnected) {
      espnow_message_t verMsg;
      verMsg.type = ESPNOW_MSG_GCI_VERSION;
      verMsg.timestamp = millis();
      verMsg.msg_seq_num = next_msg_seq_num++;
      char ver[24];
      snprintf(ver, sizeof(ver), "v%s", VERSION);
      verMsg.data_len = strlen(ver);
      memcpy(verMsg.data, ver, verMsg.data_len);
      esp_now_send(peer.peer_addr, (uint8_t*)&verMsg, ESPNOW_PACKET_SIZE(verMsg.data_len));
      if (!cliActive) Serial.printf("Sent GCI version to GCD: %s\n", ver);
      refreshTelemetry = true;
    }
  }

  lastConnectionStatus = isConnected;

  // Read sensors continuously (only if paired)
  if (hasPeer) {

    // Read temperature sensor
    sensors.requestTemperatures();
    tempC_0 = sensors.getTempCByIndex(0);
    tempF_0 = (tempC_0 == DEVICE_DISCONNECTED_C) ? -99.0f : (tempC_0 * 9.0 / 5.0 + 32.0);

    // Read ADC values — analogReadMilliVolts applies esp_adc_cal factory correction
    voltsFuelADC = analogReadMilliVolts(ADC_FUEL_PIN)   / 1000.0f;
    voltsBattADC = analogReadMilliVolts(ADC_BATTERY_PIN) / 1000.0f;

    // Select fuel sensing algorithm based on configured sensor type
    switch (gciFuelSenseType) {

      case FUEL_SENSOR_NONE:
        smoothedFuel = -99.0f;
        break;

      case FUEL_SENSOR_ADC_GAS: {
        float fuelCalc = (-90.21f * voltsFuelADC) + 105.55f;
        percentFuel = (int)fuelCalc;
        if (percentFuel > 100) percentFuel = 100;
        if (percentFuel < 0)   percentFuel = 0;

        if (millis() - lastFuelSampleTime >= FUEL_SAMPLE_INTERVAL_MS) {
          lastFuelSampleTime = millis();
          float sample = (float)percentFuel;
          if (fuelSampleFull && smoothedFuel != -99.0f && fabsf(sample - smoothedFuel) > FUEL_RESET_DELTA_THRESHOLD) {
            fuelSampleIdx = 0;
            fuelSampleFull = false;
          }
          fuelSampleBuf[fuelSampleIdx] = sample;
          fuelSampleIdx = (fuelSampleIdx + 1) % FUEL_SAMPLE_COUNT;
          if (fuelSampleIdx == 0) fuelSampleFull = true;
          int count = fuelSampleFull ? FUEL_SAMPLE_COUNT : fuelSampleIdx;
          if (count > 0) {
            float mean = 0.0f;
            for (int i = 0; i < count; i++) mean += fuelSampleBuf[i];
            mean /= count;
            float variance = 0.0f;
            for (int i = 0; i < count; i++) { float d = fuelSampleBuf[i] - mean; variance += d * d; }
            variance /= count;
            if (count >= FUEL_MIN_SAMPLES && (fuelSampleFull || variance < FUEL_VARIANCE_THRESHOLD))
              smoothedFuel = mean;
          }
        }
        break;
      }

      case FUEL_SENSOR_ADC_ELEC: {
        float raw = voltsBattADC * elecDivider;
        float pct;
        if (raw <= elecCal[0].packV) {
          pct = elecCal[0].pct;
        } else if (raw >= elecCal[elecCalN-1].packV) {
          pct = elecCal[elecCalN-1].pct;
        } else {
          pct = elecCal[elecCalN-1].pct;
          for (int i = 1; i < elecCalN; i++) {
            if (raw < elecCal[i].packV) {
              float t = (raw - elecCal[i-1].packV) / (elecCal[i].packV - elecCal[i-1].packV);
              pct = elecCal[i-1].pct + t * (elecCal[i].pct - elecCal[i-1].pct);
              break;
            }
          }
        }
        if (millis() - lastFuelSampleTime >= ELEC_SAMPLE_INTERVAL_MS) {
          lastFuelSampleTime = millis();
          if (elecEma < 0.0f) {
            elecEma = pct;
            smoothedFuel = pct;
          } else {
            elecEma = elecEma * (1.0f - ELEC_EMA_ALPHA) + pct * ELEC_EMA_ALPHA;
            if (fabsf(elecEma - smoothedFuel) >= ELEC_DEADBAND)
              smoothedFuel = elecEma;
          }
#if DEBUG_ADC_ELEC
          Serial.printf("ADC_ELEC: adcMv=%d packV=%.3fV pct=%.1f%% ema=%.1f%% reported=%.1f%%\n",
                        (int)(voltsBattADC * 1000.0f), raw, pct, elecEma, smoothedFuel);
#endif
        }
        break;
      }

      case FUEL_SENSOR_GPIO_EXP: {
        static int     pendingGpioFuel  = -99;
        static uint8_t gpioConfirmCount = 0;
        int gpioFuel = readMCP23008Fuel();
        if (gpioFuel == -99) {
          smoothedFuel    = -99.0f;
          pendingGpioFuel = -99;
          gpioConfirmCount = 0;
        } else if (gpioFuel == pendingGpioFuel) {
          if (++gpioConfirmCount >= GPIO_EXP_CONFIRM_COUNT) {
            if ((float)gpioFuel != smoothedFuel && !cliActive)
              Serial.printf("GPIO_EXP: confirmed fuel=%d%% (was %.0f%%)\n", gpioFuel, smoothedFuel);
            smoothedFuel    = (float)gpioFuel;
            gpioConfirmCount = GPIO_EXP_CONFIRM_COUNT; // cap to prevent overflow on long stable reads
          }
        } else {
          pendingGpioFuel  = gpioFuel;
          gpioConfirmCount = 1;
        }
        break;
      }

      default:
        smoothedFuel = -99.0f;
        break;
    }

    // Re-evaluate headlight relay immediately when thresholds change (no waiting for 1-second timer)
    if (luxThresholdChanged && bhSensorPresent && luxEma > 0.0f) {
      luxThresholdChanged = false;
      if (luxEma < (float)luxLightsOn && !headlightsOn) {
        headlightsOn = true;
        digitalWrite(RELAY1_PIN, HIGH);
        modeHeadLights = 1;
        if (!cliActive) Serial.printf("Headlights ON after threshold update (lux=%.0f < %d)\n", luxEma, luxLightsOn);
      } else if (luxEma >= (float)luxLightsOff && headlightsOn) {
        headlightsOn = false;
        digitalWrite(RELAY1_PIN, LOW);
        modeHeadLights = 0;
        if (!cliActive) Serial.printf("Headlights OFF after threshold update (lux=%.0f >= %d)\n", luxEma, luxLightsOff);
      }
    }

    // Sample BH1750 and control headlight relay every 1 s
    if (millis() - lastLuxSampleTime >= 1000) {
      lastLuxSampleTime = millis();
      if (bhSensorPresent) {
        float raw = readBH1750Lux();
        if (raw > 0.0f)
          luxEma = (luxEma < 0.0f) ? raw : (0.3f * raw + 0.7f * luxEma);
        // Hysteresis: turn ON below luxLightsOn, turn OFF above luxLightsOff
        if (luxEma > 0.0f && luxEma < (float)luxLightsOn && !headlightsOn) {
          headlightsOn = true;
          digitalWrite(RELAY1_PIN, HIGH);
          if (!cliActive) Serial.printf("Headlights ON (lux=%.0f < %d)\n", luxEma, luxLightsOn);
        } else if (luxEma >= (float)luxLightsOff && headlightsOn) {
          headlightsOn = false;
          digitalWrite(RELAY1_PIN, LOW);
          if (!cliActive) Serial.printf("Headlights OFF (lux=%.0f >= %d)\n", luxEma, luxLightsOff);
        }
        modeHeadLights = headlightsOn ? 1 : 0;
      } else {
        // No sensor: follow is_daylight broadcast from GCD
        bool wanted = !is_daylight;
        if (wanted != headlightsOn) {
          headlightsOn = wanted;
          digitalWrite(RELAY1_PIN, headlightsOn ? HIGH : LOW);
          modeHeadLights = headlightsOn ? 1 : 0;
          if (!cliActive) Serial.printf("Headlights %s (is_daylight=%d)\n", headlightsOn ? "ON" : "OFF", is_daylight);
        }
      }
      // Keep outdoorLux global in sync for telemetry packing
      outdoorLux = (luxEma > 0.0f) ? (int)luxEma : -99;
      if (screenOn && !splashDisplayed) displayBattLuxRow(tft, voltsBattADC, outdoorLux, bhSensorPresent);
    }

    // Check if telemetry should be sent
    // Send when: data changed, screen turned on, initial connection, or after missed heartbeat when reconnected
    bool luxChanged = bhSensorPresent && (abs(outdoorLux - lastLuxSent) > 50);
    bool dataChanged = sendData || refreshTelemetry || luxChanged || hasSignificantTelemetryChange(voltsBattADC, tempF_0, smoothedFuel, modeHeadLights);
    bool resendAfterMissedHeartbeat = heartbeatMissed && (consecutiveHeartbeatsMissed == 0);

    // Check if minimum interval has elapsed since last send
    bool intervalElapsed = (millis() - lastGcdSendTime) >= TELEMETRY_MIN_INTERVAL_MS;
    bool stale = (millis() - lastGcdSendTime) >= TELEMETRY_MAX_INTERVAL_MS;

    if ((dataChanged || resendAfterMissedHeartbeat || stale) && intervalElapsed) {

      // Update previous values
      prevBattVoltage = voltsBattADC;
      prevAirTemperature = tempF_0;
      prevFuelLevel = smoothedFuel;
      prevModeHeadLights = modeHeadLights;
      lastLuxSent = outdoorLux;

      // stuff the dataToGcd structure for transmission
      dataToGcd.modeLights = modeHeadLights;
      dataToGcd.outdoorLux = outdoorLux;
      dataToGcd.airTemp = tempF_0;
      dataToGcd.battVolts = voltsBattADC;
      dataToGcd.fuelPct = smoothedFuel;

      sendData = true;

      // Wrap telemetry data in espnow_message_t
      espnow_message_t msg;
      msg.type = ESPNOW_MSG_TELEMETRY;
      msg.timestamp = millis();
      msg.msg_seq_num = next_msg_seq_num++;
      msg.data_len = sizeof(dataToGcd);
      memcpy(msg.data, &dataToGcd, sizeof(dataToGcd));

      // Send header + actual telemetry payload
      esp_err_t result = esp_now_send(peer.peer_addr, (uint8_t *) &msg, ESPNOW_PACKET_SIZE(sizeof(dataToGcd)));
      lastGcdSendTime = millis();

      if (resendAfterMissedHeartbeat) {
        if (!cliActive) Serial.println("Telemetry sent after connection re-established");
        heartbeatMissed = false;  // Clear the missed heartbeat flag
      } else if (stale && !dataChanged) {
        if (!cliActive) Serial.println("Telemetry sent (periodic refresh)");
      } else {
        if (!cliActive) Serial.println("Telemetry sent due to significant change");
      }
      refreshTelemetry = false;  // Clear the refresh telemetry flag
    } else if ((dataChanged || resendAfterMissedHeartbeat || stale) && !intervalElapsed) {
      if (!cliActive) Serial.printf("Telemetry change detected but throttled (%.1fs until next send allowed)\n",
                    (TELEMETRY_MIN_INTERVAL_MS - (millis() - lastGcdSendTime)) / 1000.0);
    }
  }

  // Update display on 1-second timer OR immediately on sendData (button press / telemetry sent)
  // Skip while splash screen is shown so data rows don't overwrite it
  bool displayDue = !splashDisplayed && (sendData || (screenOn && (millis() - lastDisplayUpdateTime) >= DISPLAY_UPDATE_INTERVAL_MS));
  if (displayDue) {
    bool sensorConnected = (tempC_0 != DEVICE_DISCONNECTED_C);
    displayTempFuelRow(tft, tempF_0, sensorConnected, voltsFuelADC);
    displayBattLuxRow(tft, voltsBattADC, outdoorLux, bhSensorPresent);
    displayFuelSenseLine(tft, gciFuelSenseType, lastMcpPins, headlightsOn);
    lastDisplayUpdateTime = millis();
  }

  // Serial log only when telemetry was sent and CLI is not active
  if (sendData) {
    if (!cliActive) {
      char gcdMacStr[18];
      if (hasPeer) {
        sprintf(gcdMacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                peer.peer_addr[0], peer.peer_addr[1], peer.peer_addr[2],
                peer.peer_addr[3], peer.peer_addr[4], peer.peer_addr[5]);
      }
      Serial.printf("Telemetry to %s : Lights=%d, Lux=%d, Temp=%.1f, Batt=%.2f, Fuel=%.1f\n",
                    hasPeer ? gcdMacStr : "No Peer",
                    modeHeadLights, outdoorLux, tempF_0, voltsBattADC, smoothedFuel);
    }
    sendData = false;
  }

  // Poll serial input for CLI trigger ("gci") and CLI commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      cliLastWasCR = true;
      if (cliLineLen > 0 || cliActive) {
        cliLineBuf[cliLineLen] = '\0';
        processCliLine(cliLineBuf);
      }
      cliLineLen = 0;
    } else if (c == '\n') {
      if (cliLastWasCR) {
        cliLastWasCR = false;  // swallow the LF of a CRLF pair
      } else {
        if (cliLineLen > 0 || cliActive) {
          cliLineBuf[cliLineLen] = '\0';
          processCliLine(cliLineBuf);
        }
        cliLineLen = 0;
      }
    } else {
      cliLastWasCR = false;
      if (cliLineLen < (int)sizeof(cliLineBuf) - 1)
        cliLineBuf[cliLineLen++] = c;
    }
  }

  delay(100); // Check every 100ms for responsive button

}  /* END LOOP */


/*******************
 *    FUNCTIONS    *
 *******************/

// Check if telemetry data has changed significantly
bool hasSignificantTelemetryChange(float battVolts, float airTemp, float fuel, int modeLights) {
  bool changed = false;

  // Check battery voltage change (>0.1V)
  if (fabs(battVolts - prevBattVoltage) > 0.1) {
    Serial.printf("Battery voltage changed: %.2fV -> %.2fV (delta: %.2fV)\n",
                  prevBattVoltage, battVolts, fabs(battVolts - prevBattVoltage));
    changed = true;
  }

  // Check temperature change (>1°F)
  if (fabs(airTemp - prevAirTemperature) > 1.0) {
    Serial.printf("Temperature changed: %.1fF -> %.1fF (delta: %.1fF)\n",
                  prevAirTemperature, airTemp, fabs(airTemp - prevAirTemperature));
    changed = true;
  }

  // Check fuel level change; ADC_ELEC uses a higher threshold since sub-2% changes are within ADC noise
  float fuelThresh = (gciFuelSenseType == FUEL_SENSOR_ADC_ELEC) ? ADC_ELEC_TELEM_CHANGE_PCT : 1.0f;
  if (fabs(fuel - prevFuelLevel) > fuelThresh) {
    Serial.printf("Fuel level changed: %.1f%% -> %.1f%% (delta: %.1f%%)\n",
                  prevFuelLevel, fuel, fabs(fuel - prevFuelLevel));
    changed = true;
  }

  // Check headlight mode change (any change)
  if (modeLights != prevModeHeadLights) {
    Serial.printf("Headlight mode changed: %d -> %d\n", prevModeHeadLights, modeLights);
    changed = true;
  }

  return changed;
}

void redrawDisplayHeader() {
  PairingStatus status;
  if (strlen(pairedMacStr) > 0) {
    bool isConnected = (consecutiveHeartbeatsMissed < HEARTBEAT_MISS_THRESHOLD);
    status = isConnected ? PAIRED_CONNECTED : PAIRED_DISCONNECTED;
  } else {
    status = WAITING_FOR_PAIRING;
  }
  displayMacLine(tft, thisDeviceMacStr);
  displayPrLine(tft, status, pairedMacStr);
}

void BeforeSleeping() {
  // Turn on backlight to ensure message is visible
  digitalWrite(TFT_BL, HIGH);

  // Clear display and show sleep message
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(1, 40);
  tft.setTextFont(4);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println("ENTERING DEEP\nSLEEP MODE");

  // Display message for 2 seconds
  delay(2000);
}

void enterDeepSleep() {
  // Execute pre-sleep tasks
  BeforeSleeping();
  // Clear and turn off display
  tft.fillScreen(TFT_BLACK);
  digitalWrite(TFT_BL, LOW);  // Turn off backlight

  // Put display into sleep mode to release SPI bus
  tft.writecommand(0x10); // Sleep mode command for ST7789

  // Shutdown WiFi and ESP-NOW to save power
  esp_now_deinit();
  WiFi.disconnect(true);
  esp_wifi_stop();

  // Configure ext0 wake-up on SLEEP_PIN going HIGH (1 = wake on HIGH level)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_PIN, 1);

  // Hold backlight pin LOW during deep sleep (GPIO loses state otherwise)
  gpio_hold_en((gpio_num_t)TFT_BL);

  // Small delay to ensure operations complete
  delay(100);

  // Enter deep sleep (will wake and reboot when SLEEP_PIN goes HIGH)
  Serial.println("GCI entering deep sleep");
  Serial.flush();
  esp_deep_sleep_start();
}

void createMacAddressStr(char* MacStr){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {

    sprintf(MacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            baseMac[0], baseMac[1], baseMac[2],
            baseMac[3], baseMac[4], baseMac[5]);
  }
  else {
    Serial.println("Failed to read MAC address");
    tft.println("MAC read failed");
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    // Half-duplex collision: GCD may have been transmitting (e.g. GPS_DATA broadcast)
    // at the same moment GCI sent this packet; GCD's L2 ACK was not received.
    // Transient and expected — no retry needed.
    Serial.println("Send Status: Fail");
  }
  if (status ==0){
    tx_success = "Delivery Success :)";
  }
  else{
    tx_success = "Delivery Fail :(";
  }
}

// Callback when data is received (promiscuous mode for pairing)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Check for RAW pairing command (before any peers exist)
  if (len == sizeof(structMsgFromGcd)) {
    memcpy(&dataFromGcd, incomingData, sizeof(dataFromGcd));

    if (dataFromGcd.cmdNumber == GCI_CMD_ADD_PEER) {
      // Check if we already have a peer (only allow one peer at a time)
      esp_now_peer_info_t existingPeer;
      bool hasPeer = (esp_now_fetch_peer(true, &existingPeer) == ESP_OK);

      if (hasPeer) {
        Serial.println("Pairing rejected - already paired with another device");
        return;
      }

      // Extract MAC address from command payload
      uint8_t newPeerMac[6];
      memcpy(newPeerMac, dataFromGcd.macAddr, 6);

      sprintf(pairedMacStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              newPeerMac[0], newPeerMac[1], newPeerMac[2],
              newPeerMac[3], newPeerMac[4], newPeerMac[5]);

      // Check if peer already exists
      if (!esp_now_is_peer_exist(newPeerMac)) {
        // Add new peer
        esp_now_peer_info_t newPeer = {};  // Initialize all fields to zero
        memcpy(newPeer.peer_addr, newPeerMac, 6);
        newPeer.channel = 0;  // 0 = use local home channel (avoids IDF mismatch check)
        newPeer.ifidx = WIFI_IF_STA;  // WiFi Station interface
        newPeer.encrypt = false;

        if (esp_now_add_peer(&newPeer) == ESP_OK) {
          Serial.printf("*** PAIRED with GCD (RAW MODE): %s ***\n", pairedMacStr);

          // Save peer MAC to preferences
          preferences.putBytes("peer_mac", newPeerMac, 6);
          Serial.println("Saved peer MAC to EEPROM");

          // Update display to show paired status (GREEN since we just received a message)
          displayMacLine(tft, thisDeviceMacStr);
          displayPrLine(tft, PAIRED_CONNECTED, pairedMacStr);

          // Send ACK back to GCD in WRAPPED mode
          espnow_message_t ackMsg;
          ackMsg.type = ESPNOW_MSG_ACK;
          ackMsg.timestamp = millis();
          ackMsg.msg_seq_num = next_msg_seq_num++;
          ackMsg.data_len = 0;

          Serial.printf("Sending ACK (%d bytes) to GCD...\n", ESPNOW_PACKET_SIZE(0));
          esp_err_t result = esp_now_send(newPeerMac, (uint8_t*)&ackMsg, ESPNOW_PACKET_SIZE(0));
          if (result == ESP_OK) {
            Serial.println("Sent ACK to GCD - Switched to WRAPPED mode for communication");
          } else {
            Serial.printf("Failed to send ACK to GCD - Error code: %d\n", result);
          }
        } else {
          Serial.println("Failed to add peer");
        }
      } else {
        Serial.println("Peer already exists");
      }
    }
    return;
  }

  // Check if this is a wrapped message
  if (len >= ESPNOW_PACKET_HEADER_SIZE) {
    // Filter wrapped messages - only accept from registered peers
    if (!esp_now_is_peer_exist(mac)) {
      Serial.println("ESP-NOW: Ignoring message from unknown peer");
      return;
    }

    // Reset heartbeat counter - we received a message from GCD
    consecutiveHeartbeatsMissed = 0;
    lastHeartbeatCheckTime = millis();

    espnow_message_t* msg = (espnow_message_t*)incomingData;

    char mac_str[18];
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (!cliActive) Serial.printf("Received %s from %s\n",
                  msg->type == ESPNOW_MSG_COMMAND   ? "COMMAND" :
                  msg->type == ESPNOW_MSG_TEXT       ? "TEXT" :
                  msg->type == ESPNOW_MSG_HEARTBEAT  ? "HEARTBEAT" :
                  msg->type == ESPNOW_MSG_CONFIG     ? "CONFIG" : "MESSAGE",
                  mac_str);

    // Route by message type
    switch (msg->type) {
      case ESPNOW_MSG_COMMAND: {
        // Extract command from message data
        memcpy(&dataFromGcd, msg->data, sizeof(dataFromGcd));
        Serial.print("Command: ");
        Serial.println(dataFromGcd.cmdNumber);
        cmdFromGcd = dataFromGcd.cmdNumber;

        // Process commands
        switch (dataFromGcd.cmdNumber) {
          case GCI_CMD_ADD_PEER: {
            // Check if we already have a peer (only allow one peer at a time)
            esp_now_peer_info_t existingPeer;
            bool alreadyHasPeer = (esp_now_fetch_peer(true, &existingPeer) == ESP_OK);

            if (alreadyHasPeer) {
              Serial.println("Pairing rejected - already paired with another device");
              break;
            }

            // Extract MAC address from command payload
            uint8_t newPeerMac[6];
            memcpy(newPeerMac, dataFromGcd.macAddr, 6);

            // Check if peer already exists
            if (!esp_now_is_peer_exist(newPeerMac)) {
              // Add new peer
              esp_now_peer_info_t newPeer;
              memcpy(newPeer.peer_addr, newPeerMac, 6);
              newPeer.channel = 0;  // 0 = use local home channel (avoids IDF mismatch check)
              newPeer.encrypt = false;

              if (esp_now_add_peer(&newPeer) == ESP_OK) {
                Serial.printf("*** PAIRED with GCD: %02X:%02X:%02X:%02X:%02X:%02X ***\n",
                             newPeerMac[0], newPeerMac[1], newPeerMac[2],
                             newPeerMac[3], newPeerMac[4], newPeerMac[5]);

                // Update display to show paired status (connected since we just received message)
                displayMacLine(tft, thisDeviceMacStr);
                displayPrLine(tft, PAIRED_CONNECTED, mac_str);
              } else {
                Serial.println("Failed to add peer");
              }
            } else {
              Serial.println("Peer already exists");
            }
            break;
          }

          case GCI_CMD_REMOVE_PEER:
            Serial.println("Remove peer command received (not implemented)");
            break;

          case GCI_CMD_REBOOT:
            Serial.println("Reboot command received (not implemented)");
            break;

          default:
            Serial.println("Unknown command");
            break;
        }
        break;
      }

      case ESPNOW_MSG_GPS_DATA:
        // GCD broadcasts GPS position to all peers periodically — no action needed on GCI
        break;

      case ESPNOW_MSG_TEXT:
        if (!cliActive) { Serial.print("Text message: "); Serial.println((char*)msg->data); }
        break;

      case ESPNOW_MSG_HEARTBEAT:
        // Respond to heartbeat from known peer (closed-loop keepalive)
        if (esp_now_is_peer_exist(mac)) {
          if (!cliActive) Serial.printf("Heartbeat from %s - sending response\n", mac_str);

          // Send heartbeat response back to GCD
          espnow_message_t response;
          response.type = ESPNOW_MSG_HEARTBEAT;
          response.timestamp = millis();
          response.msg_seq_num = next_msg_seq_num++;
          response.data_len = 4;
          memcpy(response.data, &response.timestamp, 4);

          esp_now_send(mac, (uint8_t*)&response, ESPNOW_PACKET_SIZE(4));

        }
        break;

      case ESPNOW_MSG_IS_HOME:
        if (msg->data_len >= 1) {
          is_home = (msg->data[0] != 0);
          if (!cliActive) Serial.printf("Received is_home status: %s\n", is_home ? "HOME" : "AWAY");
        }
        break;

      case ESPNOW_MSG_IS_DAYTIME:
        if (msg->data_len >= 1) {
          is_daylight = (msg->data[0] != 0);
          if (!cliActive) Serial.printf("Received is_daylight status: %s\n", is_daylight ? "DAYTIME" : "NIGHTTIME");
        }
        break;

      case ESPNOW_MSG_CONFIG:
        if (msg->data_len >= (int)sizeof(int32_t)) {  // at minimum fuelSensorType
          structMsgConfig cfg = {};
          memcpy(&cfg, msg->data, min((int)msg->data_len, (int)sizeof(cfg)));
          gciFuelSenseType = cfg.fuelSensorType;
          preferences.putInt("fuel_sense_type", gciFuelSenseType);
          if (!cliActive) Serial.printf("Received fuel_sense_type=%d, saved to NVS\n", gciFuelSenseType);
          if (gciFuelSenseType == FUEL_SENSOR_GPIO_EXP)
            initMCP23008();
          if (msg->data_len >= (int)sizeof(structMsgConfig)) {
            luxLightsOn  = (int)cfg.luxLightsOn;
            luxLightsOff = (int)cfg.luxLightsOff;
            preferences.putInt("lux_on",  luxLightsOn);
            preferences.putInt("lux_off", luxLightsOff);
            if (!cliActive) Serial.printf("Lux thresholds updated: on=%d off=%d\n", luxLightsOn, luxLightsOff);
            luxThresholdChanged = true;
          }
        }
        break;

      default:
        Serial.print("Unknown message type: ");
        Serial.println(msg->type);
        break;
    }
  } else {
    Serial.print("Received unexpected message size: ");
    Serial.println(len);
  }
}