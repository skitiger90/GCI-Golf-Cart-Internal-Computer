/*
 * DISPLAY.H - Simple display layout and helper functions
 *
 * This file centralizes all display-related positioning, colors, and styling
 * to make the code easier to read and maintain.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <TFT_eSPI.h>

// ============================================================================
// SCREEN LAYOUT - All Y-coordinates in one place
// ============================================================================
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 135

#define MAC_LINE_Y       2     // "MAC: xx:xx:xx:xx:xx:xx"
#define PAIRED_LINE_Y    28    // "PR xx:xx:xx:xx:xx:xx" or "WAITING FOR PAIRING..."
#define TEMP_LINE_Y      54    // "TEMP 72.5°F"
#define FUEL_LINE_Y      80    // "FUEL 1234    BATT 5678"
#define FUEL_SENSE_LINE_Y 112  // "NO SENSOR" / "ADC GAS" / "GPIO EXPANDER 011"

// ============================================================================
// COLORS - Named constants instead of hex codes
// ============================================================================
#define COLOR_LABEL      TFT_CYAN    // For labels like "MAC", "TEMP", etc.
#define COLOR_VALUE      TFT_WHITE   // For normal values
#define COLOR_CONNECTED  TFT_GREEN   // When paired and connected
#define COLOR_DISCONNECTED TFT_RED   // When paired but disconnected
#define COLOR_WAITING    TFT_YELLOW  // When waiting for pairing

// ============================================================================
// PAIRING STATUS - Simple enum for connection states
// ============================================================================
enum PairingStatus {
    WAITING_FOR_PAIRING,    // Not paired yet
    PAIRED_CONNECTED,       // Paired and receiving heartbeats
    PAIRED_DISCONNECTED     // Paired but no heartbeats
};

// ============================================================================
// DISPLAY FUNCTIONS - Declared here, implemented in display.cpp
// ============================================================================

// Set text styles (reduces repetitive code)
void setLabelStyle(TFT_eSPI &tft);     // Small cyan text for labels
void setValueStyle(TFT_eSPI &tft);     // Large white text for values

// Display specific lines (each function handles one line of the display)
void displayMacLine(TFT_eSPI &tft, const char* macAddress);
void displayPairingLine(TFT_eSPI &tft, PairingStatus status, const char* pairedMac);
void displayTempLine(TFT_eSPI &tft, float tempF, bool sensorConnected);
void displayFuelBattLine(TFT_eSPI &tft, float fuelVolts, float battVolts);
void displayFuelSenseLine(TFT_eSPI &tft, int sensorType, uint8_t gpPins);

// Full screen operations
void clearScreen(TFT_eSPI &tft);
void redrawAllLines(TFT_eSPI &tft, const char* thisMac, PairingStatus status,
                    const char* pairedMac, float tempF, bool sensorConnected,
                    float fuelVolts, float battVolts);

// Splash screen - displays "GCI" and version for 2 seconds
void displaySplashScreen(TFT_eSPI &tft, const char* version);

#endif // DISPLAY_H
