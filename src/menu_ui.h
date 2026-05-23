#ifndef MENU_UI_H
#define MENU_UI_H

#include <M5Core2.h>

// Forward declarations of applications
void runSdTester();
void runWifiRadar();

// Helper to draw the main menu screen with a premium visual design
void drawMainMenu() {
    // 1. Premium background: Sleek dark charcoal/blue
    // Color: 0x0842 (RGB: R=1, G=2, B=2 => very dark slate blue)
    M5.Lcd.fillScreen(0x0842); 

    // 2. Top Header Bar
    // Color: 0x10A2 (RGB: R=2, G=5, B=2 => dark sapphire blue)
    M5.Lcd.fillRect(0, 0, 320, 50, 0x10A2);
    // Neon Cyan horizontal accent divider line under header (2px thickness)
    M5.Lcd.fillRect(0, 50, 320, 2, 0x07FF);

    // 3. Header Title text
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("SYSTEM CONTROLLER", 160, 25, 4); // Clean size 4 font

    // 4. Grid Button 1: SD Tester (Cyan theme)
    // Position: X=20, Y=75, W=130, H=125. Corner radius = 10
    M5.Lcd.fillRoundRect(20, 75, 130, 125, 10, 0x10A2); // Button BG
    M5.Lcd.drawRoundRect(20, 75, 130, 125, 10, 0x07FF); // Cyan border outline
    
    // Label texts for Button 1
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString("SD", 85, 110, 4); // Font 4 (large)
    M5.Lcd.setTextColor(0x07FF);
    M5.Lcd.drawString("TESTER", 85, 150, 2); // Font 2 (medium)

    // 5. Grid Button 2: WiFi Radar (Orange theme)
    // Position: X=170, Y=75, W=130, H=125. Corner radius = 10
    M5.Lcd.fillRoundRect(170, 75, 130, 125, 10, 0x10A2); // Button BG
    M5.Lcd.drawRoundRect(170, 75, 130, 125, 10, 0xFDA0); // Neon Orange border outline
    
    // Label texts for Button 2
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString("WIFI", 235, 110, 4); // Font 4 (large)
    M5.Lcd.setTextColor(0xFDA0);
    M5.Lcd.drawString("RADAR", 235, 150, 2); // Font 2 (medium)

    // 6. Bottom Navigation Indicator (above capacitive Button B)
    M5.Lcd.setTextColor(0x7BEF); // Subtle gray color
    M5.Lcd.drawString("Press Center Button [B] to Exit Apps", 160, 220, 2);
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
        M5.update();
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
        M5.update();
        
        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x != -1) {
            // Check if Button 1 (SD Tester) was pressed: X: 20..150, Y: 75..200
            if (pos.x >= 20 && pos.x <= 150 && pos.y >= 75 && pos.y <= 200) {
                // Visual feedback: briefly highlight Button 1
                M5.Lcd.fillRoundRect(20, 75, 130, 125, 10, 0x1CE7); // Bright grey-blue
                M5.Lcd.drawRoundRect(20, 75, 130, 125, 10, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("SD", 85, 110, 4);
                M5.Lcd.setTextColor(0x07FF);
                M5.Lcd.drawString("TESTER", 85, 150, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to SD Tester app loop
                runSdTester();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
            // Check if Button 2 (WiFi Radar) was pressed: X: 170..300, Y: 75..200
            else if (pos.x >= 170 && pos.x <= 300 && pos.y >= 75 && pos.y <= 200) {
                // Visual feedback: briefly highlight Button 2
                M5.Lcd.fillRoundRect(170, 75, 130, 125, 10, 0x1CE7); // Bright grey-blue
                M5.Lcd.drawRoundRect(170, 75, 130, 125, 10, TFT_WHITE); // White border
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.drawString("WIFI", 235, 110, 4);
                M5.Lcd.setTextColor(0xFDA0);
                M5.Lcd.drawString("RADAR", 235, 150, 2);
                
                delay(150); // Keep highlighted briefly for responsiveness
                waitForTouchRelease();
                
                // Route to WiFi Radar app loop
                runWifiRadar();
                
                // When we exit, redraw main menu and continue loop
                drawMainMenu();
                waitForTouchRelease();
            }
        }
        delay(10);
    }
}



#endif // MENU_UI_H
