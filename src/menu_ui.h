#ifndef MENU_UI_H
#define MENU_UI_H

#include <M5Core2.h>
#include "imu_ui.h"

// Forward declarations of applications
void runSdTester();
void runWifiRadar();
void runImuDashboard();
void runSysDebugger();

// Helper to draw the main menu screen with a premium visual design
void drawMainMenu() {
    // 1. Premium background: Sleek dark charcoal/blue
    // Color: 0x0842 (RGB: R=1, G=2, B=2 => very dark slate blue)
    M5.Lcd.fillScreen(0x0842); 

    // 2. Top Header Bar
    // Color: 0x10A2 (RGB: R=2, G=5, B=2 => dark sapphire blue)
    M5.Lcd.fillRect(0, 0, 320, 45, 0x10A2);
    // Neon Cyan horizontal accent divider line under header (2px thickness)
    M5.Lcd.fillRect(0, 45, 320, 2, 0x07FF);

    // 3. Header Title text
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("SYSTEM CONTROLLER", 160, 22, 4); // Clean size 4 font
    
    // 3b. Battery Indicator
    float batVolts = M5.Axp.GetBatVoltage();
    bool isCharging = M5.Axp.isCharging();
    int batPct = (int)((batVolts < 3.2f) ? 0 : (batVolts - 3.2f) * 100.0f);
    if (batPct > 100) batPct = 100;
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.setTextColor(isCharging ? 0x07E0 : TFT_WHITE); // Green if charging
    char batBuf[16];
    sprintf(batBuf, "%d%%", batPct);
    M5.Lcd.drawString(batBuf, 312, 22, 2);

    // 4. Button 1: SD Tester (Cyan theme)
    M5.Lcd.fillRoundRect(15, 65, 140, 70, 8, 0x10A2); 
    M5.Lcd.drawRoundRect(15, 65, 140, 70, 8, 0x07FF); 
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString("SD TESTER", 85, 100, 2);

    // 5. Button 2: WiFi Radar (Orange theme)
    M5.Lcd.fillRoundRect(165, 65, 140, 70, 8, 0x10A2); 
    M5.Lcd.drawRoundRect(165, 65, 140, 70, 8, 0xFDA0); 
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString("WIFI RADAR", 235, 100, 2);

    // 6. Button 3: IMU Dashboard (Green theme)
    M5.Lcd.fillRoundRect(15, 145, 140, 70, 8, 0x10A2); 
    M5.Lcd.drawRoundRect(15, 145, 140, 70, 8, 0x07E0); 
    M5.Lcd.setTextColor(0x07E0); // Vivid green text
    M5.Lcd.drawString("IMU DASH", 85, 180, 2);

    // 7. Button 4: Sys Debugger (Orange theme)
    M5.Lcd.fillRoundRect(165, 145, 140, 70, 8, 0x10A2); 
    M5.Lcd.drawRoundRect(165, 145, 140, 70, 8, 0xFDA0); 
    M5.Lcd.setTextColor(0xFDA0); // Vivid orange text
    M5.Lcd.drawString("SYS DEBUG", 235, 180, 2);

    // 7. Bottom Navigation Indicator (above capacitive Button B)
    M5.Lcd.setTextColor(0x7BEF); // Subtle gray color
    M5.Lcd.drawString("Press Center Button [B] to Exit Apps", 160, 230, 2);
}

// Helper to check if capacitive Button B (Home/Back) was pressed
bool isHomeButtonPressed() {
    M5.update();
    if (M5.BtnB.wasPressed()) {
        return true;
    }
    
    // Double safety check: Touch coordinates in physical Button B area
    // Button B: X range [110, 210], Y range >= 240 (extending below active display)
    TouchPoint_t pos = M5.Touch.getPressPoint();
    if (pos.x >= 110 && pos.x <= 210 && pos.y >= 240) {
        return true;
    }
    return false;
}

// Helper to wait until screen touch is fully released
void waitForTouchRelease() {
    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
        
        // Check for physical Button B (Home/Exit)
        if (isHomeButtonPressed()) {
            break;
        }
        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x == -1) {
            break;
        }
        delay(10);
    }
}

// Main menu driver function
void runMainMenu() {
    drawMainMenu();
    waitForTouchRelease();

    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
        
        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x != -1) {
            // Check if Button 1 (SD Tester) was pressed: X: 15..155, Y: 65..135
            if (pos.x >= 15 && pos.x <= 155 && pos.y >= 65 && pos.y <= 135) {
                // Visual feedback: briefly highlight Button 1
                M5.Lcd.fillRoundRect(15, 65, 140, 70, 8, 0x1CE7); // Bright grey-blue
                M5.Lcd.drawRoundRect(15, 65, 140, 70, 8, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("SD TESTER", 85, 100, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to SD Tester app loop
                runSdTester();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
            // Check if Button 2 (WiFi Radar) was pressed: X: 165..305, Y: 65..135
            else if (pos.x >= 165 && pos.x <= 305 && pos.y >= 65 && pos.y <= 135) {
                // Visual feedback: briefly highlight Button 2
                M5.Lcd.fillRoundRect(165, 65, 140, 70, 8, 0x1CE7); // Bright grey-blue
                M5.Lcd.drawRoundRect(165, 65, 140, 70, 8, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("WIFI RADAR", 235, 100, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to WiFi Radar app loop
                runWifiRadar();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
            // Check if Button 3 (IMU Dashboard) was pressed: X: 15..155, Y: 145..215
            else if (pos.x >= 15 && pos.x <= 155 && pos.y >= 145 && pos.y <= 215) {
                // Visual feedback: briefly highlight Button 3 using green highlight color 0x03E0
                M5.Lcd.fillRoundRect(15, 145, 140, 70, 8, 0x03E0); // Green highlight
                M5.Lcd.drawRoundRect(15, 145, 140, 70, 8, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("IMU DASH", 85, 180, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to IMU dashboard
                runImuDashboard();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
            // Check if Button 4 (Sys Debugger) was pressed: X: 165..305, Y: 145..215
            else if (pos.x >= 165 && pos.x <= 305 && pos.y >= 145 && pos.y <= 215) {
                // Visual feedback: briefly highlight Button 4 using orange/yellow highlight color 0xE79E
                M5.Lcd.fillRoundRect(165, 145, 140, 70, 8, 0xE79E); // Orange/yellow highlight
                M5.Lcd.drawRoundRect(165, 145, 140, 70, 8, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("SYS DEBUG", 235, 180, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to Sys Debugger
                runSysDebugger();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
        }
        delay(10);
    }
}



#endif // MENU_UI_H
