
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

#include "prototypes.h"
#include "display.h"
#include "version.h"

#define SCREEN_TIMEOUT 60 // seconds
#define PAIRED_MAC_MSG_TIMEOUT 5 // seconds
#define BUTTON_HOLD_ERASE_SECS 5 // seconds to hold button to erase paired MAC

#define TELEMETRY_MIN_INTERVAL_MS 5000  // Minimum 5 seconds between telemetry packets
#define HEARTBEAT_MISS_THRESHOLD 4  // Number of missed heartbeats before connection is considered lost

#define FUEL_SAMPLE_COUNT       8       // rolling buffer size (8 × 15s = 2-minute window)
#define FUEL_MIN_SAMPLES        3       // require at least 3 samples (~45 sec) before first report
#define FUEL_SAMPLE_INTERVAL_MS 15000   // ms between fuel ADC samples
#define FUEL_VARIANCE_THRESHOLD 25.0f   // population variance limit (~5% std-dev); tune as needed
#define FUEL_RESET_DELTA_THRESHOLD 25.0f // % jump that flushes the rolling buffer for fast large-change response
#define STARTUP_GRACE_SECS 30  // Grace period to allow GCD connection before acting on SLEEP_PIN (matches GCD)
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

// i2c configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQUENCY 100000    // purposely keep at 100kHz to allow longer bus
#define I2C_ADDR_MCP23008 0x20  // default address with all address pins grounded; can be changed by wiring address pins to VCC
#define I2C_ADDR_BH1750 0x23    // BH1750 default address (ADDR pin grounded); can be changed to 0x5C by wiring ADDR pin to VCC

#define ESPNOW_CHANNEL 1
#define ESPNOW_MAX_PAYLOAD 240  // Max payload after wrapper overhead subtracted (ESP-NOW limit: 250 bytes, wrapper: 9 bytes, payload: 241 bytes)

#define DISPLAY_ORIENTATION 0       // 0 = Normal, 1 = Flipped (180 degrees)

// ESP-NOW message types (must match GCD)
typedef enum {
    ESPNOW_MSG_TEXT = 0,
    ESPNOW_MSG_GPS_DATA = 1,
    ESPNOW_MSG_TELEMETRY = 2,
    ESPNOW_MSG_COMMAND = 3,
    ESPNOW_MSG_ACK = 4,
    ESPNOW_MSG_HEARTBEAT = 5,
    ESPNOW_MSG_IS_HOME = 6,
    ESPNOW_MSG_IS_DAYTIME = 7
} espnow_msg_type_t;

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
int outdoorLuminosity = -99;
float airTemperature = -99;
float battVoltage = -99;

// Fuel level smoothing state
float smoothedFuel = -99;                 // value transmitted to GCD
float fuelSampleBuf[FUEL_SAMPLE_COUNT];   // rolling sample buffer
int   fuelSampleIdx = 0;
bool  fuelSampleFull = false;
unsigned long lastFuelSampleTime = 0;

// Previous telemetry values for change detection
int prevModeHeadLights = -99;
float prevAirTemperature = -999;
float prevBattVoltage = -99;
float prevFuelLevel = 100.0f;

// Status variables received from GCD
bool is_home = false;      // True when GCD is within home geo-fence
bool is_daylight = true;   // True during daytime (between sunrise/sunset)

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
  int outdoorLum;
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


/**************
 *    SETUP   *
 **************/

void setup(void) {
  Serial.begin(9600);
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
        sprintf(pairedMacStr, "%02x:%02x:%02x:%02x:%02x:%02x",
                savedMac[0], savedMac[1], savedMac[2],
                savedMac[3], savedMac[4], savedMac[5]);
        Serial.printf("Loaded saved peer: %s\n", pairedMacStr);

        // Display paired status (starts RED until heartbeat received)
        displayPairingLine(tft, PAIRED_DISCONNECTED, pairedMacStr);

        hasSavedPeer = true;
      }
    }
  }

  if (!hasSavedPeer) {
    Serial.println("No saved peer - waiting for pairing commands from GCD");

    // Display pairing status
    displayPairingLine(tft, WAITING_FOR_PAIRING, "");
  }
  screenStartTime = millis();   // Record screen start time

} /* END SETUP */


/**************
 *    LOOP    *
 **************/

void loop() {

  static float tempC_0, tempF_0;
  static int rawFuelADC, rawBattADC;
  static float voltsFuelADC, voltsBattADC;
  static int percentFuel;
  static bool sendData = false;
  static unsigned long lastDisplayUpdateTime = 0;

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
  bool gcd_connected = (consecutiveHeartbeatsMissed < HEARTBEAT_MISS_THRESHOLD);
  bool grace_elapsed = millis() >= (STARTUP_GRACE_SECS * 1000UL);
  if (!enteringSleep && debounced_sleep_state && (gcd_connected || grace_elapsed)) {
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

  // Check if we have a paired device before attempting to send
  esp_now_peer_info_t peer;
  bool hasPeer = (esp_now_fetch_peer(true, &peer) == ESP_OK);

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
      Serial.println("Heartbeat missed - telemetry will be sent when connection re-established");
    }

    if (consecutiveHeartbeatsMissed >= HEARTBEAT_MISS_THRESHOLD) {
      Serial.printf("Connection lost - missed %d heartbeats\n", consecutiveHeartbeatsMissed);
    }
  }

  // Update display if connection status changed
  if (hasPeer && (isConnected != lastConnectionStatus) && strlen(pairedMacStr) > 0) {
    PairingStatus status = isConnected ? PAIRED_CONNECTED : PAIRED_DISCONNECTED;
    displayPairingLine(tft, status, pairedMacStr);
  }

  lastConnectionStatus = isConnected;

  // Read sensors continuously (only if paired)
  if (hasPeer) {

    // Read temperature sensor
    sensors.requestTemperatures();
    tempC_0 = sensors.getTempCByIndex(0);
    tempF_0 = tempC_0 * 9.0 / 5.0 + 32.0;

    // Read ADC values
    rawFuelADC = analogRead(ADC_FUEL_PIN);
    rawBattADC = analogRead(ADC_BATTERY_PIN);

    // Convert ADC values to volts (0-4095 -> 0-3.3V)
    voltsFuelADC = (rawFuelADC / 4095.0) * 3.3;
    voltsBattADC = (rawBattADC / 4095.0) * 3.3;

    // Calculate fuel percentage from voltage
    float fuelCalc = (-90.21 * voltsFuelADC) + 105.55;
    percentFuel = (int)fuelCalc;

    // Limit percentFuel to 0-100 range
    if (percentFuel > 100) percentFuel = 100;
    if (percentFuel < 0) percentFuel = 0;

    // Collect timed fuel sample into rolling buffer for variance-based filtering.
    // Accepts a reading only when >= FUEL_MIN_SAMPLES have been gathered and variance
    // is low (stable sensor).  High variance or too few samples → keep smoothedFuel unchanged (stays -99 until first valid reading).
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
        for (int i = 0; i < count; i++) {
          float d = fuelSampleBuf[i] - mean;
          variance += d * d;
        }
        variance /= count;

        if (count >= FUEL_MIN_SAMPLES && variance < FUEL_VARIANCE_THRESHOLD)
          smoothedFuel = mean;
        // else: keep current value (stays -99 until first valid reading; holds last accepted reading if temporarily noisy)
      }
    }

    // Check if telemetry should be sent
    // Send when: data changed, screen turned on, initial connection, or after missed heartbeat when reconnected
    bool dataChanged = sendData || refreshTelemetry || hasSignificantTelemetryChange(voltsBattADC, tempF_0, smoothedFuel, modeHeadLights);
    bool resendAfterMissedHeartbeat = heartbeatMissed && (consecutiveHeartbeatsMissed == 0);

    // Check if minimum interval has elapsed since last send
    bool intervalElapsed = (millis() - lastGcdSendTime) >= TELEMETRY_MIN_INTERVAL_MS;

    if ((dataChanged || resendAfterMissedHeartbeat) && intervalElapsed) {

      // Update previous values
      prevBattVoltage = voltsBattADC;
      prevAirTemperature = tempF_0;
      prevFuelLevel = smoothedFuel;
      prevModeHeadLights = modeHeadLights;

      // stuff the dataToGcd structure for transmission
      dataToGcd.modeLights = modeHeadLights;
      dataToGcd.outdoorLum = outdoorLuminosity;
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
        Serial.println("Telemetry sent after connection re-established");
        heartbeatMissed = false;  // Clear the missed heartbeat flag
      } else {
        Serial.println("Telemetry sent due to significant change");
      }
      refreshTelemetry = false;  // Clear the refresh telemetry flag
    } else if ((dataChanged || resendAfterMissedHeartbeat) && !intervalElapsed) {
      Serial.printf("Telemetry change detected but throttled (%.1fs until next send allowed)\n",
                    (TELEMETRY_MIN_INTERVAL_MS - (millis() - lastGcdSendTime)) / 1000.0);
    }
  }

  // Update display on 1-second timer OR immediately on sendData (button press / telemetry sent)
  bool displayDue = sendData || (screenOn && (millis() - lastDisplayUpdateTime) >= DISPLAY_UPDATE_INTERVAL_MS);
  if (displayDue) {
    bool sensorConnected = (tempC_0 != DEVICE_DISCONNECTED_C);
    displayTempLine(tft, tempF_0, sensorConnected);
    displayFuelBattLine(tft, voltsFuelADC, voltsBattADC);
    lastDisplayUpdateTime = millis();
  }

  // Serial log only when telemetry was sent
  if (sendData) {
    char gcdMacStr[18];
    if (hasPeer) {
      sprintf(gcdMacStr, "%02x:%02x:%02x:%02x:%02x:%02x",
              peer.peer_addr[0], peer.peer_addr[1], peer.peer_addr[2],
              peer.peer_addr[3], peer.peer_addr[4], peer.peer_addr[5]);
    }
    Serial.printf("Telemetry to %s : Lights=%d, Lum=%d, Temp=%.1f, Batt=%.2f, Fuel=%.1f\n",
                  hasPeer ? gcdMacStr : "No Peer",
                  modeHeadLights, outdoorLuminosity, tempF_0, voltsBattADC, (float)percentFuel);
    sendData = false;
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

  // Check fuel level change (>1%)
  if (fabs(fuel - prevFuelLevel) > 1.0) {
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
  // Redraw MAC address line
  displayMacLine(tft, thisDeviceMacStr);

  // Redraw paired status line if we have a peer
  if (strlen(pairedMacStr) > 0) {
    bool isConnected = (consecutiveHeartbeatsMissed < HEARTBEAT_MISS_THRESHOLD);
    PairingStatus status = isConnected ? PAIRED_CONNECTED : PAIRED_DISCONNECTED;
    displayPairingLine(tft, status, pairedMacStr);
  }
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
  esp_deep_sleep_start();
}

void createMacAddressStr(char* MacStr){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {

    sprintf(MacStr, "%02x:%02x:%02x:%02x:%02x:%02x",
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

      sprintf(pairedMacStr, "%02x:%02x:%02x:%02x:%02x:%02x",
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
          displayPairingLine(tft, PAIRED_CONNECTED, pairedMacStr);

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
    sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Serial.printf("Received %s from %s\n",
                  msg->type == ESPNOW_MSG_COMMAND ? "COMMAND" :
                  msg->type == ESPNOW_MSG_TEXT ? "TEXT" :
                  msg->type == ESPNOW_MSG_HEARTBEAT ? "HEARTBEAT" : "MESSAGE",
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
                displayPairingLine(tft, PAIRED_CONNECTED, mac_str);
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
        Serial.print("Text message: ");
        Serial.println((char*)msg->data);
        break;

      case ESPNOW_MSG_HEARTBEAT:
        // Respond to heartbeat from known peer (closed-loop keepalive)
        if (esp_now_is_peer_exist(mac)) {
          Serial.printf("Heartbeat from %s - sending response\n", mac_str);

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
          Serial.printf("Received is_home status: %s\n", is_home ? "HOME" : "AWAY");
        }
        break;

      case ESPNOW_MSG_IS_DAYTIME:
        if (msg->data_len >= 1) {
          is_daylight = (msg->data[0] != 0);
          Serial.printf("Received is_daylight status: %s\n", is_daylight ? "DAYTIME" : "NIGHTTIME");
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