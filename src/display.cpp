/*
 * DISPLAY.CPP - Implementation of simple display functions
 *
 * Each function handles ONE row of the 240x135px display.
 * All rows use font 2 (16px) for both labels and values.
 *
 * Layout (5 rows at 26px pitch):
 *   Y=  4  MAC 90-38-0c-da-ce-90
 *   Y= 30  PR  ec-e3-34-20-4b-b8  (or WAITING FOR PAIRING)
 *   Y= 56  TEMP 78.5F   FUEL 0.728V
 *   Y= 82  BATT 2.784V   LUX 121 LX
 *   Y=108  GPIO EXPANDER 1-0-0   HL ON
 */

#include "display.h"

// ============================================================================
// TEXT STYLE HELPERS
// ============================================================================

void setLabelStyle(TFT_eSPI &tft) {
    tft.setTextFont(2);
    tft.setTextColor(COLOR_LABEL, TFT_BLACK);
}

// Print MAC string replacing ':' separators with '-'
static void printMacDashes(TFT_eSPI &tft, const char* mac) {
    char buf[18];
    int len = strlen(mac);
    if (len > 17) len = 17;
    for (int i = 0; i < len; i++)
        buf[i] = (mac[i] == ':') ? '-' : mac[i];
    buf[len] = '\0';
    tft.print(buf);
}

// ============================================================================
// ROW FUNCTIONS
// ============================================================================

// Row 1: own device MAC address with dash separators
void displayMacLine(TFT_eSPI &tft, const char* thisMac) {
    tft.fillRect(0, MAC_LINE_Y, SCREEN_WIDTH, 20, TFT_BLACK);
    tft.setCursor(1, MAC_LINE_Y);
    setLabelStyle(tft);
    tft.print("MAC ");
    tft.setTextColor(COLOR_VALUE, TFT_BLACK);
    printMacDashes(tft, thisMac);
}

// Row 2: paired MAC (green=connected, red=disconnected) or yellow WAITING message
void displayPrLine(TFT_eSPI &tft, PairingStatus status, const char* pairedMac) {
    tft.fillRect(0, PR_LINE_Y, SCREEN_WIDTH, 20, TFT_BLACK);
    tft.setCursor(1, PR_LINE_Y);
    if (status == WAITING_FOR_PAIRING) {
        tft.setTextFont(2);
        tft.setTextColor(COLOR_WAITING, TFT_BLACK);
        tft.print("WAITING FOR PAIRING");
    } else {
        setLabelStyle(tft);
        tft.print("PR  ");
        tft.setTextColor(
            (status == PAIRED_CONNECTED) ? COLOR_CONNECTED : COLOR_DISCONNECTED,
            TFT_BLACK);
        printMacDashes(tft, pairedMac);
    }
}

// Row 3: temperature and fuel voltage inline
void displayTempFuelRow(TFT_eSPI &tft, float tempF, bool tempOk, float fuelV) {
    tft.fillRect(0, SENSOR_LINE_Y, SCREEN_WIDTH, 20, TFT_BLACK);

    // TEMP (x=1)
    tft.setCursor(1, SENSOR_LINE_Y);
    setLabelStyle(tft);
    tft.print("TEMP ");
    tft.setTextColor(COLOR_VALUE, TFT_BLACK);
    if (tempOk) {
        tft.print(tempF, 1);
        tft.print(" F");
    } else {
        tft.print("--");
    }

    // FUEL (x=95)
    tft.setCursor(95, SENSOR_LINE_Y);
    setLabelStyle(tft);
    tft.print("FUEL ");
    tft.setTextColor(COLOR_VALUE, TFT_BLACK);
    tft.print(fuelV, 3);
    tft.print(" V");
}

// Row 4: battery voltage and lux reading inline
void displayBattLuxRow(TFT_eSPI &tft, float battV, int lux, bool sensorPresent) {
    tft.fillRect(0, LUX_LINE_Y, SCREEN_WIDTH, 20, TFT_BLACK);

    // BATT (x=1)
    tft.setCursor(1, LUX_LINE_Y);
    setLabelStyle(tft);
    tft.print("BATT ");
    tft.setTextColor(COLOR_VALUE, TFT_BLACK);
    tft.print(battV, 3);
    tft.print(" V");

    // LUX (x=100) — "---" in dark grey when no BH1750
    tft.setCursor(100, LUX_LINE_Y);
    setLabelStyle(tft);
    tft.print("LUX ");
    if (sensorPresent && lux >= 0) {
        tft.setTextColor(COLOR_VALUE, TFT_BLACK);
        tft.print(lux);
        tft.print(" LX");
    } else {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.print("---");
    }
}

// Row 5: fuel sensor type with dashes between GPIO bits, HL relay state at right
void displayFuelSenseLine(TFT_eSPI &tft, int sensorType, uint8_t gpPins, bool hlOn) {
    tft.fillRect(0, FUEL_SENSE_LINE_Y, SCREEN_WIDTH, SCREEN_HEIGHT - FUEL_SENSE_LINE_Y, TFT_BLACK);
    tft.setCursor(1, FUEL_SENSE_LINE_Y);
    tft.setTextFont(2);
    tft.setTextColor(COLOR_LABEL, TFT_BLACK);
    switch (sensorType) {
        case 0: tft.print("NO FUEL SENSOR"); break;
        case 1: tft.print("ADC GAS");        break;
        case 3: tft.print("ADC ELECTRIC");   break;
        case 2: {
            tft.print("GPIO EXPANDER ");
            tft.setTextColor(COLOR_VALUE, TFT_BLACK);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d-%d-%d",
                     (gpPins >> 0) & 1, (gpPins >> 1) & 1, (gpPins >> 2) & 1);
            tft.print(buf);
            break;
        }
        default: tft.print("UNKNOWN"); break;
    }

    // HL relay state at x=175
    tft.setCursor(175, FUEL_SENSE_LINE_Y);
    setLabelStyle(tft);
    tft.print("HL ");
    tft.setTextColor(hlOn ? TFT_GREEN : COLOR_VALUE, TFT_BLACK);
    tft.print(hlOn ? "ON" : "OFF");
}

// ============================================================================
// FULL SCREEN OPERATIONS
// ============================================================================

void clearScreen(TFT_eSPI &tft) {
    tft.fillScreen(TFT_BLACK);
}

void displaySplashScreen(TFT_eSPI &tft, const char* version) {
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
    snprintf(versionLine, sizeof(versionLine), "Ver. %s", version);
    tft.drawString(versionLine, screenWidth / 2, 85);

    tft.setTextDatum(TL_DATUM);
    delay(2500);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
}
