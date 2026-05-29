/*
 * DISPLAY.CPP - Implementation of simple display functions
 *
 * Each function here handles ONE specific part of the display.
 * This makes it easy to update the display without duplicating code.
 */

#include "display.h"

// ============================================================================
// TEXT STYLE HELPERS
// ============================================================================

void setLabelStyle(TFT_eSPI &tft) {
    tft.setTextFont(2);                    // Small font
    tft.setTextColor(COLOR_LABEL, TFT_BLACK);  // Cyan text
}

void setValueStyle(TFT_eSPI &tft) {
    tft.setTextFont(4);                    // Medium font
    tft.setTextColor(COLOR_VALUE, TFT_BLACK);  // White text
}

// ============================================================================
// LINE-BY-LINE DISPLAY FUNCTIONS
// ============================================================================

void displayMacLine(TFT_eSPI &tft, const char* macAddress) {
    // Clear the line
    tft.fillRect(0, MAC_LINE_Y, SCREEN_WIDTH, 26, TFT_BLACK);

    // Draw label
    tft.setCursor(1, MAC_LINE_Y);
    setLabelStyle(tft);
    tft.print("MAC");

    // Draw value
    setValueStyle(tft);
    tft.println(macAddress);
}

void displayPairingLine(TFT_eSPI &tft, PairingStatus status, const char* pairedMac) {
    // Clear the line
    tft.fillRect(0, PAIRED_LINE_Y, SCREEN_WIDTH, 16, TFT_BLACK);
    tft.setCursor(1, PAIRED_LINE_Y);

    if (status == WAITING_FOR_PAIRING) {
        // Show yellow "waiting" message
        tft.setTextFont(2);
        tft.setTextColor(COLOR_WAITING, TFT_BLACK);
        tft.print("WAITING FOR PAIRING...");
    } else {
        // Show "PR" label
        setLabelStyle(tft);
        tft.print("PR");

        // Show MAC address in green (connected) or red (disconnected)
        tft.setTextFont(4);
        uint16_t color = (status == PAIRED_CONNECTED) ? COLOR_CONNECTED : COLOR_DISCONNECTED;
        tft.setTextColor(color, TFT_BLACK);
        tft.print(pairedMac);
    }
}

void displayTempLine(TFT_eSPI &tft, float tempF, bool sensorConnected) {
    // Clear the value area (keep label area)
    tft.fillRect(60, TEMP_LINE_Y, 260, 18, TFT_BLACK);

    // Draw label
    tft.setCursor(1, TEMP_LINE_Y);
    setLabelStyle(tft);
    tft.print("TEMP ");

    // Draw value
    setValueStyle(tft);
    if (sensorConnected) {
        tft.print(tempF);
        tft.println(" F");  // Note: degree symbol removed for simplicity
    } else {
        tft.println("No sensor");
    }
}

void displayFuelBattLine(TFT_eSPI &tft, float fuelVolts, float battVolts) {
    // Clear only the fuel/batt area; leave FUEL_SENSE_LINE_Y and below for displayFuelSenseLine
    tft.fillRect(0, FUEL_LINE_Y, SCREEN_WIDTH, FUEL_SENSE_LINE_Y - FUEL_LINE_Y, TFT_BLACK);

    // FUEL label stacked: "FUEL" over "VOLT"
    setLabelStyle(tft);
    tft.setCursor(1, FUEL_LINE_Y);
    tft.print("FUEL");
    tft.setCursor(1, FUEL_LINE_Y + 16);
    tft.print("VOLT");

    // Fuel value — font 4 (26px tall) centered against the 32px stacked label
    setValueStyle(tft);
    tft.setCursor(40, FUEL_LINE_Y + 3);
    tft.print(fuelVolts, 3);

    // BATT label stacked: "BATT" over "VOLT"
    setLabelStyle(tft);
    tft.setCursor(130, FUEL_LINE_Y);
    tft.print("BATT");
    tft.setCursor(130, FUEL_LINE_Y + 16);
    tft.print("VOLT");

    // Batt value
    setValueStyle(tft);
    tft.setCursor(168, FUEL_LINE_Y + 3);
    tft.print(battVolts, 3);
}

void displayFuelSenseLine(TFT_eSPI &tft, int sensorType, uint8_t gpPins) {
    tft.fillRect(0, FUEL_SENSE_LINE_Y, SCREEN_WIDTH, SCREEN_HEIGHT - FUEL_SENSE_LINE_Y, TFT_BLACK);
    tft.setCursor(1, FUEL_SENSE_LINE_Y);
    tft.setTextFont(2);
    tft.setTextColor(COLOR_LABEL, TFT_BLACK);
    switch (sensorType) {
        case 0: tft.print("NO SENSOR");  break;
        case 1: tft.print("ADC GAS");    break;
        case 3: tft.print("ADC ELEC");   break;
        case 2: {
            char buf[24];
            snprintf(buf, sizeof(buf), "GPIO EXPANDER %d%d%d",
                     (gpPins >> 0) & 1, (gpPins >> 1) & 1, (gpPins >> 2) & 1);
            tft.print(buf);
            break;
        }
        default: tft.print("UNKNOWN"); break;
    }
}

// ============================================================================
// FULL SCREEN OPERATIONS
// ============================================================================

void clearScreen(TFT_eSPI &tft) {
    tft.fillScreen(TFT_BLACK);
}

void redrawAllLines(TFT_eSPI &tft, const char* thisMac, PairingStatus status,
                    const char* pairedMac, float tempF, bool sensorConnected,
                    float fuelVolts, float battVolts) {
    // Redraw each line of the display
    displayMacLine(tft, thisMac);
    displayPairingLine(tft, status, pairedMac);
    displayTempLine(tft, tempF, sensorConnected);
    displayFuelBattLine(tft, fuelVolts, battVolts);
}

void displaySplashScreen(TFT_eSPI &tft, const char* version) {
    // Clear screen
    tft.fillScreen(TFT_BLACK);

    // Get actual screen width after rotation
    int16_t screenWidth = tft.width();

    // Use top-center datum for centered text
    tft.setTextDatum(TC_DATUM);

    // Draw "GCI" in large yellow font, centered
    tft.setTextFont(4);
    tft.setTextSize(2);  // Double size for large text
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("GCI", screenWidth / 2, 20);

    // Draw "Ver. x.x.x" in white, centered below
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    char versionLine[32];
    snprintf(versionLine, sizeof(versionLine), "Ver. %s", version);
    tft.drawString(versionLine, screenWidth / 2, 85);

    // Reset datum to top-left for other functions
    tft.setTextDatum(TL_DATUM);

    // Display for 2.5 seconds
    delay(2500);

    // Clear screen before returning
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);  // Reset to default size
}
