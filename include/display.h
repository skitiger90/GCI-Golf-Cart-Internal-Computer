/*
 * DISPLAY.H - Simple display layout and helper functions
 *
 * Layout: 5 rows at 26px pitch, 240×135px (landscape), font 2 (16px)
 *   Y=  4  MAC 90-38-0c-da-ce-90
 *   Y= 30  PR  ec-e3-34-20-4b-b8  (or WAITING FOR PAIRING)
 *   Y= 56  TEMP 78.5F   FUEL 0.728V
 *   Y= 82  BATT 2.784V   LUX 121 LX
 *   Y=108  GPIO EXPANDER 1-0-0   HL:ON
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <TFT_eSPI.h>

// ============================================================================
// SCREEN LAYOUT - ST7789 in landscape: TFT_HEIGHT=240 x TFT_WIDTH=135
// ============================================================================
#define SCREEN_WIDTH      240
#define SCREEN_HEIGHT     135

#define MAC_LINE_Y          4  // "MAC xx-xx-xx-xx-xx-xx"
#define PR_LINE_Y          30  // "PR  xx-xx-xx-xx-xx-xx" or "WAITING FOR PAIRING"
#define SENSOR_LINE_Y      56  // "TEMP 78.5F   FUEL 0.728V"
#define LUX_LINE_Y         82  // "BATT 2.784V   LUX 121 LX"
#define FUEL_SENSE_LINE_Y 108  // "GPIO EXPANDER 1-0-0   HL:ON"

// ============================================================================
// COLORS - Named constants instead of hex codes
// ============================================================================
#define COLOR_LABEL        TFT_CYAN    // Labels: "MAC", "TEMP", etc.
#define COLOR_VALUE        TFT_WHITE   // Normal values
#define COLOR_CONNECTED    TFT_GREEN   // Paired and connected
#define COLOR_DISCONNECTED TFT_RED     // Paired but disconnected
#define COLOR_WAITING      TFT_YELLOW  // Waiting for pairing

// ============================================================================
// PAIRING STATUS
// ============================================================================
enum PairingStatus {
    WAITING_FOR_PAIRING,
    PAIRED_CONNECTED,
    PAIRED_DISCONNECTED
};

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void setLabelStyle(TFT_eSPI &tft);

// Row functions — each redraws exactly its row (20px clear region)
void displayMacLine(TFT_eSPI &tft, const char* thisMac);
void displayPrLine(TFT_eSPI &tft, PairingStatus status, const char* pairedMac);
void displayTempFuelRow(TFT_eSPI &tft, float tempF, bool tempOk, float fuelV);
void displayBattLuxRow(TFT_eSPI &tft, float battV, int lux, bool sensorPresent);
void displayFuelSenseLine(TFT_eSPI &tft, int sensorType, uint8_t gpPins, bool hlOn);

// Full screen operations
void clearScreen(TFT_eSPI &tft);
void displaySplashScreen(TFT_eSPI &tft, const char* version);

#endif // DISPLAY_H
