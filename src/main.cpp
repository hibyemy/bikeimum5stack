/**
 * @file main.cpp
 * @brief Monolithic App Launcher firmware for M5Stack Core2.
 * 
 * Orchestrates menu routing, capacitive touch, custom SPI sharing, and SdFat.
 */

#include <M5Core2.h>
#include "hw_spi.h"
#include "sd_diagnostics.h"
#include "menu_ui.h"
#include "wifi_radar.h"
#include "imu_ui.h"

// Track SD initialization state globally
bool isSdInitialized = false;

// Custom progress print class for Formatter progress bar
class LcdPrintProgress : public Print {
public:
    size_t write(uint8_t c) override {
        Serial.write(c); // Always duplicate to Serial monitor
        if (c == '.') {
            dotCount++;
            drawProgressBar();
        }
        return 1;
    }
    void reset() {
        dotCount = 0;
        // Draw progress bar outline frame
        M5.Lcd.drawRect(20, 140, 280, 18, TFT_WHITE);
        M5.Lcd.fillRect(21, 141, 278, 16, TFT_BLACK);
    }
private:
    int dotCount = 0;
    void drawProgressBar() {
        int width = (dotCount * 278) / 32;
        if (width > 278) width = 278;
        M5.Lcd.fillRect(21, 141, width, 16, 0x07E0); // Green color progress fill
    }
};

LcdPrintProgress progressPrinter;

// Screen logging variables and helpers
int logCursorY = 55;

void initLogScreen(const String& title, uint16_t headerColor, uint16_t dividerColor) {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.fillRect(0, 0, 320, 45, headerColor);
    M5.Lcd.drawFastHLine(0, 45, 320, dividerColor);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString(title, 160, 22, 4);
    M5.Lcd.setTextDatum(TL_DATUM);
    logCursorY = 55;
}

void logToScreen(const String& msg, uint16_t color) {
    Serial.println(msg);
    M5.Lcd.setTextColor(color);
    if (logCursorY > 200) {
        // Clear log area
        M5.Lcd.fillRect(0, 50, 320, 155, TFT_BLACK);
        logCursorY = 55;
    }
    M5.Lcd.drawString(msg, 10, logCursorY, 2);
    logCursorY += 18;
}

void logExitHint() {
    M5.Lcd.setTextColor(0x7BEF); // Subtle gray
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("Press Center Button [B] to return", 160, 220, 2);
    M5.Lcd.setTextDatum(TL_DATUM);
}

void drawSdTesterMenu() {
    M5.Lcd.fillScreen(0x0842); // Sleek charcoal/blue background

    // Top Header Bar
    M5.Lcd.fillRect(0, 0, 320, 45, 0x10A2); // Dark sapphire header
    M5.Lcd.drawFastHLine(0, 45, 320, 0x07FF); // Neon Cyan divider line
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("SD DIAGNOSTICS SUITE", 160, 22, 4);

    // Button 1: Speed Test (Cyan)
    M5.Lcd.fillRoundRect(15, 55, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(15, 55, 135, 40, 6, 0x07FF);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.drawString("Speed Test", 82, 75, 2);

    // Button 2: LBA Test (Orange)
    M5.Lcd.fillRoundRect(170, 55, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(170, 55, 135, 40, 6, 0xFDA0);
    M5.Lcd.drawString("LBA Check", 237, 75, 2);

    // Button 3: Format Card (Red)
    M5.Lcd.fillRoundRect(15, 105, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(15, 105, 135, 40, 6, 0xF800);
    M5.Lcd.drawString("Format Card", 82, 125, 2);

    // Button 4: Decode CID (Green)
    M5.Lcd.fillRoundRect(170, 105, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(170, 105, 135, 40, 6, 0x07E0);
    M5.Lcd.drawString("Decode CID", 237, 125, 2);

    // Button 5: Latency Test (Yellow)
    M5.Lcd.fillRoundRect(15, 155, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(15, 155, 135, 40, 6, 0xEFE0);
    M5.Lcd.drawString("Latency Test", 82, 175, 2);

    // Button 6: Hex Dump (Grey)
    M5.Lcd.fillRoundRect(170, 155, 135, 40, 6, 0x10A2);
    M5.Lcd.drawRoundRect(170, 155, 135, 40, 6, 0x7BEF);
    M5.Lcd.drawString("Raw Hex Dump", 237, 175, 2);

    // Bottom bezel hint
    M5.Lcd.setTextColor(0x7BEF);
    M5.Lcd.drawString("Press Center Button [B] to Exit to Main Menu", 160, 220, 2);
}

void runSdTester() {
    drawSdTesterMenu();
    waitForTouchRelease();

    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);

        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x != -1) {
            // Button 1: Speed Test: X: 15..150, Y: 55..95
            if (pos.x >= 15 && pos.x <= 150 && pos.y >= 55 && pos.y <= 95) {
                M5.Lcd.fillRoundRect(15, 55, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(15, 55, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("Speed Test", 82, 75, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("SPEED TEST", 0x10A2, 0x07FF);
                logToScreen("WARNING: Test writes a 4MB temp file", TFT_YELLOW);
                logToScreen("to measure throughput. Old data", TFT_YELLOW);
                logToScreen("on this file path will be lost.", TFT_YELLOW);
                logToScreen("Double tap screen to confirm...", TFT_YELLOW);

                bool confirmed = false;
                uint32_t firstTapTime = 0;
                while (true) {
                    if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
                    M5.update();
                    if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
                    if (isHomeButtonPressed()) {
                        break;
                    }
                    TouchPoint_t tPos = M5.Touch.getPressPoint();
                    if (tPos.x != -1) {
                        if (firstTapTime == 0) {
                            firstTapTime = millis();
                            logToScreen("First tap. Tap again to start...", TFT_YELLOW);
                            delay(300);
                            waitForTouchRelease();
                        } else {
                            if (millis() - firstTapTime < 2000) {
                                confirmed = true;
                                break;
                            } else {
                                firstTapTime = millis();
                                logToScreen("Tap timeout. Tap again...", TFT_YELLOW);
                                delay(300);
                                waitForTouchRelease();
                            }
                        }
                    }
                    delay(10);
                }

                if (confirmed) {
                    M5.Lcd.fillRect(0, 50, 320, 155, TFT_BLACK);
                    logCursorY = 55;
                    logToScreen("Initializing write speed test...");
                    float wSpeed = writeSpeedTest();
                    if (wSpeed > 0) {
                        logToScreen("Write Speed: " + String(wSpeed, 2) + " MB/s", TFT_GREEN);
                    } else {
                        logToScreen("Write Speed Test: FAILED", TFT_RED);
                    }

                    logToScreen("Initializing read speed test...");
                    float rSpeed = readSpeedTest();
                    if (rSpeed > 0) {
                        logToScreen("Read Speed: " + String(rSpeed, 2) + " MB/s", TFT_GREEN);
                    } else {
                        logToScreen("Read Speed Test: FAILED", TFT_RED);
                    }
                } else {
                    logToScreen("Speed test canceled.", TFT_WHITE);
                }

                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }
            
            // Button 2: LBA Check: X: 170..305, Y: 55..95
            else if (pos.x >= 170 && pos.x <= 305 && pos.y >= 55 && pos.y <= 95) {
                M5.Lcd.fillRoundRect(170, 55, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(170, 55, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("LBA Check", 237, 75, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("LBA STRIP TEST", 0x10A2, 0xFDA0);
                logToScreen("WARNING: Low-level raw sector writes!", TFT_RED);
                logToScreen("Data loss will occur if interrupted", TFT_RED);
                logToScreen("or if the SD memory card fails!", TFT_RED);
                logToScreen("Double tap screen to confirm...", TFT_YELLOW);
                
                bool confirmed = false;
                uint32_t firstTapTime = 0;
                while (true) {
                    if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
                    M5.update();
                    if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
                    if (isHomeButtonPressed()) {
                        break;
                    }
                    TouchPoint_t tPos = M5.Touch.getPressPoint();
                    if (tPos.x != -1) {
                        if (firstTapTime == 0) {
                            firstTapTime = millis();
                            logToScreen("First tap. Tap again to start...", TFT_YELLOW);
                            delay(300);
                            waitForTouchRelease();
                        } else {
                            if (millis() - firstTapTime < 2000) {
                                confirmed = true;
                                break;
                            } else {
                                firstTapTime = millis();
                                logToScreen("Tap timeout. Tap again...", TFT_YELLOW);
                                delay(300);
                                waitForTouchRelease();
                            }
                        }
                    }
                    delay(10);
                }

                if (confirmed) {
                    M5.Lcd.fillRect(0, 50, 320, 155, TFT_BLACK);
                    logCursorY = 55;
                    logToScreen("Starting Direct LBA diagnostics...");
                    logToScreen("Writing & verifying 1GB jumps...");
                    bool lbaOk = lbaStripTest();
                    if (lbaOk) {
                        logToScreen("LBA Strip Test: PASS", TFT_GREEN);
                        logToScreen("No memory wrapping detected.", TFT_GREEN);
                    } else {
                        logToScreen("LBA Strip Test: FAIL!", TFT_RED);
                        logToScreen("Wrap-around/counterfeit detected!", TFT_RED);
                    }
                } else {
                    logToScreen("LBA check canceled.", TFT_WHITE);
                }

                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }

            // Button 3: Format Card: X: 15..150, Y: 105..145
            else if (pos.x >= 15 && pos.x <= 150 && pos.y >= 105 && pos.y <= 145) {
                M5.Lcd.fillRoundRect(15, 105, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(15, 105, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("Format Card", 82, 125, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("LOW-LEVEL FORMAT", 0x10A2, 0xF800);
                logToScreen("WARNING: Destructive Operation!", TFT_RED);
                logToScreen("All files on card will be erased.", TFT_RED);
                logToScreen("Double tap screen to confirm...", TFT_YELLOW);
                
                bool confirmed = false;
                uint32_t firstTapTime = 0;
                while (true) {
                    if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
                    M5.update();
                    if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
                    if (isHomeButtonPressed()) {
                        break;
                    }
                    TouchPoint_t tPos = M5.Touch.getPressPoint();
                    if (tPos.x != -1) {
                        if (firstTapTime == 0) {
                            firstTapTime = millis();
                            logToScreen("First tap detected. Tap again...", TFT_YELLOW);
                            delay(300);
                            waitForTouchRelease();
                        } else {
                            if (millis() - firstTapTime < 2000) {
                                confirmed = true;
                                break;
                            } else {
                                firstTapTime = millis();
                                logToScreen("Tap timeout. Tap again...", TFT_YELLOW);
                                delay(300);
                                waitForTouchRelease();
                            }
                        }
                    }
                    delay(10);
                }

                if (confirmed) {
                    M5.Lcd.fillRect(0, 50, 320, 155, TFT_BLACK);
                    logCursorY = 55;
                    logToScreen("Eraser & Formatter active...");
                    progressPrinter.reset();
                    bool fmtOk = formatCard(&progressPrinter);
                    if (fmtOk) {
                        isSdInitialized = true;
                        logToScreen("Format: SUCCESS", TFT_GREEN);
                        logToScreen("FAT partition reconstructed.", TFT_GREEN);
                    } else {
                        isSdInitialized = false;
                        logToScreen("Format: FAILED!", TFT_RED);
                    }
                } else {
                    logToScreen("Format canceled by user.", TFT_WHITE);
                }

                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }

            // Button 4: Decode CID: X: 170..305, Y: 105..145
            else if (pos.x >= 170 && pos.x <= 305 && pos.y >= 105 && pos.y <= 145) {
                M5.Lcd.fillRoundRect(170, 105, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(170, 105, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("Decode CID", 237, 125, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("CID & CSD DECODER", 0x10A2, 0x07E0);
                
                if (!isSdInitialized && !initSPIAndSD(20)) {
                    logToScreen("SD initialization failed!", TFT_RED);
                } else {
                    isSdInitialized = true;
                    cid_t cid;
                    csd_t csd;
                    if (getSdFat().card()->readCID(&cid) && getSdFat().card()->readCSD(&csd)) {
                        logToScreen("MID (Manufacturer): 0x" + String(cid.mid, HEX));
                        char oidStr[3] = { (char)cid.oid[0], (char)cid.oid[1], '\0' };
                        logToScreen("OID (OEM ID):       " + String(oidStr));
                        char pnmStr[6] = { (char)cid.pnm[0], (char)cid.pnm[1], (char)cid.pnm[2], (char)cid.pnm[3], (char)cid.pnm[4], '\0' };
                        logToScreen("PNM (Product Name): " + String(pnmStr));
                        logToScreen("PRV (Revision):     " + String(cid.prvN()) + "." + String(cid.prvM()));
                        logToScreen("PSN (Serial):       0x" + String(cid.psn(), HEX));
                        logToScreen("MDT (Mfg Date):     " + String(cid.mdtMonth()) + "/" + String(cid.mdtYear()));
                        
                        uint32_t secSize = csd.capacity();
                        if (secSize == 0) secSize = getSdFat().card()->sectorCount();
                        logToScreen("CSD Size:           " + String(secSize * 512.0 / 1e9, 2) + " GB");
                    } else {
                        logToScreen("Failed to read CID/CSD registers.", TFT_RED);
                    }
                }

                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }

            // Button 5: Latency Test: X: 15..150, Y: 155..195
            else if (pos.x >= 15 && pos.x <= 150 && pos.y >= 155 && pos.y <= 195) {
                M5.Lcd.fillRoundRect(15, 155, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(15, 155, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("Latency Test", 82, 175, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("LATENCY DIAGNOSTICS", 0x10A2, 0xEFE0);
                logToScreen("Writing 2000 small blocks...");
                logToScreen("Measuring write latency...");
                
                bool latOk = false;
                if (!isSdInitialized && !initSPIAndSD(20)) {
                    logToScreen("SD Card failed to init!", TFT_RED);
                } else {
                    isSdInitialized = true;
                    const char* latPath = "/latency_test.tmp";
                    if (getSdFat().exists(latPath)) getSdFat().remove(latPath);
                    FsFile file = getSdFat().open(latPath, O_WRONLY | O_CREAT | O_TRUNC);
                    if (!file) {
                        logToScreen("Failed to open latency file!", TFT_RED);
                    } else {
                        uint8_t dummyBuf[512];
                        memset(dummyBuf, 0, 512);
                        uint32_t maxLat = 0;
                        uint32_t minLat = 9999999;
                        uint64_t sumLat = 0;
                        const uint32_t totalWrites = 2000;
                        
                        for (uint32_t w = 0; w < totalWrites; w++) {
                            uint32_t st = micros();
                            if (file.write(dummyBuf, 512) != 512) {
                                break;
                            }
                            uint32_t elapsedLat = micros() - st;
                            sumLat += elapsedLat;
                            if (elapsedLat > maxLat) maxLat = elapsedLat;
                            if (elapsedLat < minLat) minLat = elapsedLat;
                            
                            if (w % 500 == 0) {
                                logToScreen("Written " + String(w) + " blocks...");
                            }
                            yield();
                        }
                        uint32_t syncSt = micros();
                        file.sync();
                        uint32_t syncLat = micros() - syncSt;
                        file.close();
                        getSdFat().remove(latPath);
                        
                        logToScreen("Written: 2000 blocks");
                        logToScreen("Min Latency:  " + String(minLat) + " us", TFT_GREEN);
                        logToScreen("Max Latency:  " + String(maxLat) + " us", maxLat > 50000 ? TFT_RED : TFT_GREEN);
                        logToScreen("Avg Latency:  " + String((float)sumLat/totalWrites, 1) + " us");
                        logToScreen("Sync Latency: " + String(syncLat) + " us");
                        latOk = true;
                    }
                }
                
                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }

            // Button 6: Hex Dump: X: 170..305, Y: 155..195
            else if (pos.x >= 170 && pos.x <= 305 && pos.y >= 155 && pos.y <= 195) {
                M5.Lcd.fillRoundRect(170, 155, 135, 40, 6, 0x1CE7);
                M5.Lcd.drawRoundRect(170, 155, 135, 40, 6, TFT_WHITE);
                M5.Lcd.setTextColor(TFT_WHITE);
                M5.Lcd.setTextDatum(MC_DATUM);
                M5.Lcd.drawString("Raw Hex Dump", 237, 175, 2);
                delay(150);
                waitForTouchRelease();

                initLogScreen("RAW HEX DUMPER", 0x10A2, 0x7BEF);
                logToScreen("Reading Block 0 (MBR/Boot)...");
                
                if (!isSdInitialized && !initSPIAndSD(20)) {
                    logToScreen("SD Card failed to init!", TFT_RED);
                } else {
                    isSdInitialized = true;
                    uint8_t dumpBuf[512];
                    if (!getSdFat().card()->readSector(0, dumpBuf)) {
                        logToScreen("Error reading Block 0!", TFT_RED);
                    } else {
                        hexDumpBlock(0);
                        
                        logToScreen("First 64 Bytes of sector 0:", TFT_YELLOW);
                        for (int line = 0; line < 4; line++) {
                            String hexStr = "";
                            String ascStr = "";
                            int startIdx = line * 16;
                            for (int cell = 0; cell < 16; cell++) {
                                uint8_t v = dumpBuf[startIdx + cell];
                                char hexCell[4];
                                sprintf(hexCell, "%02X ", v);
                                hexStr += String(hexCell);
                                if (v >= 32 && v <= 126) ascStr += (char)v;
                                else ascStr += '.';
                            }
                            logToScreen(hexStr + " | " + ascStr);
                        }
                        logToScreen("Full 512-byte dump sent to Serial!", TFT_GREEN);
                    }
                }

                logExitHint();
                while (!isHomeButtonPressed()) {
                    delay(20);
                }
                drawSdTesterMenu();
                waitForTouchRelease();
            }
        }

        // Return check
        if (isHomeButtonPressed()) {
            break;
        }
        delay(20);
    }
}

// Actual WiFi CSI Human Radar application with Vital Signs DSP extraction
void runWifiRadar() {
    M5.Lcd.fillScreen(0x0842); // Charcoal background
    
    // Header
    M5.Lcd.fillRect(0, 0, 320, 45, 0x2800); // Dark red/orange header
    M5.Lcd.drawFastHLine(0, 45, 320, 0xFDA0); // Orange separator
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("WIFI CSI RADAR", 160, 22, 4);

    M5.Lcd.drawString("Initializing passive CSI...", 160, 110, 2);
    M5.Lcd.drawString("Scanning Channel 6...", 160, 135, 2);
    
    delay(200);
    waitForTouchRelease();

    if (!initWifiRadar(6)) {
        M5.Lcd.fillScreen(0x0842);
        M5.Lcd.fillRect(0, 0, 320, 45, 0x2800);
        M5.Lcd.drawFastHLine(0, 45, 320, 0xFDA0);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.drawString("WIFI CSI RADAR", 160, 22, 4);
        
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.drawString("CSI Initialization Failed!", 160, 110, 2);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.drawString("Press Center Button [B] to Exit", 160, 180, 2);
        while (!isHomeButtonPressed()) {
            delay(20);
        }
        return;
    }

    // Successfully initialized WiFi CSI sniffer
    M5.Lcd.fillScreen(0x0842);
    
    // Draw Header
    M5.Lcd.fillRect(0, 0, 320, 45, 0x2800); 
    M5.Lcd.drawFastHLine(0, 45, 320, 0xFDA0); 
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("WIFI CSI RADAR", 160, 22, 4);

    // Bezel hint at the bottom
    M5.Lcd.setTextColor(0x7BEF); // Subtle gray
    M5.Lcd.drawString("Press Center Button [B] to return to Menu", 160, 232, 1);

    // Bounding Box Layout configuration
    #define GRAPH_WIDTH 260
    #define GRAPH_X 30
    
    // Breathing Graph (Cyan)
    #define B_GRAPH_Y 50
    #define B_GRAPH_HEIGHT 55
    #define B_GRAPH_CENTER (B_GRAPH_Y + B_GRAPH_HEIGHT / 2)
    
    // Heart Rate Graph (Red)
    #define H_GRAPH_Y 115
    #define H_GRAPH_HEIGHT 55
    #define H_GRAPH_CENTER (H_GRAPH_Y + H_GRAPH_HEIGHT / 2)

    int b_history[GRAPH_WIDTH] = {0};
    int h_history[GRAPH_WIDTH] = {0};
    
    uint32_t last_filter_update = 0;
    uint32_t last_ui_update = 0;

    // Draw static graph frames
    M5.Lcd.drawRect(GRAPH_X - 1, B_GRAPH_Y - 1, GRAPH_WIDTH + 2, B_GRAPH_HEIGHT + 2, 0x10E7); // Dark cyan frame
    M5.Lcd.drawRect(GRAPH_X - 1, H_GRAPH_Y - 1, GRAPH_WIDTH + 2, H_GRAPH_HEIGHT + 2, 0x8000); // Dark red frame

    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);

        // Check if Center button pressed to exit
        if (isHomeButtonPressed()) {
            break;
        }

        uint32_t now = millis();

        // 1. Resampling and filtering at 10 Hz (every 100 ms)
        if (now - last_filter_update >= 100) {
            last_filter_update = now;

            float score = latest_motion_score;

            // Process filters
            float b_val = breathingFilter.process(score);
            float b_env = breathingEnv.process(b_val);
            float b_bpm = breathingBpmEst.process(b_val, b_env, 0.015f, now); // Squelch threshold 0.015

            float h_val = heartRateFilter.process(score);
            float h_env = heartRateEnv.process(h_val);
            float h_bpm = heartRateBpmEst.process(h_val, h_env, 0.005f, now); // Squelch threshold 0.005

            // Update exposed states
            latest_breathing_val = b_val;
            latest_heart_val = h_val;
            latest_breathing_bpm = b_bpm;
            latest_heart_bpm = h_bpm;
            breathing_envelope = b_env;
            heart_envelope = h_env;

            // Shift graph histories
            for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
                b_history[i] = b_history[i + 1];
                h_history[i] = h_history[i + 1];
            }

            // Scale values to fit inside B_GRAPH_HEIGHT/2 (dynamic scaling based on envelope)
            float b_scale = max(b_env * 1.5f, 0.05f);
            int b_scaled = (int)((b_val / b_scale) * (B_GRAPH_HEIGHT / 2));
            if (b_scaled > (B_GRAPH_HEIGHT / 2)) b_scaled = (B_GRAPH_HEIGHT / 2);
            if (b_scaled < -(B_GRAPH_HEIGHT / 2)) b_scaled = -(B_GRAPH_HEIGHT / 2);
            b_history[GRAPH_WIDTH - 1] = b_scaled;

            float h_scale = max(h_env * 1.5f, 0.02f);
            int h_scaled = (int)((h_val / h_scale) * (H_GRAPH_HEIGHT / 2));
            if (h_scaled > (H_GRAPH_HEIGHT / 2)) h_scaled = (H_GRAPH_HEIGHT / 2);
            if (h_scaled < -(H_GRAPH_HEIGHT / 2)) h_scaled = -(H_GRAPH_HEIGHT / 2);
            h_history[GRAPH_WIDTH - 1] = h_scaled;
        }

        // 2. UI Graph and HUD redraw task (running at 10 Hz)
        if (now - last_ui_update >= 100) {
            last_ui_update = now;

            float score = latest_motion_score;
            float b_env = breathing_envelope;
            float b_bpm = latest_breathing_bpm;
            float h_bpm = latest_heart_bpm;

            // Redraw Breathing Waveform Area
            M5.Lcd.fillRect(GRAPH_X, B_GRAPH_Y, GRAPH_WIDTH, B_GRAPH_HEIGHT, 0x0842);
            M5.Lcd.drawFastHLine(GRAPH_X, B_GRAPH_CENTER, GRAPH_WIDTH, 0x10A2); // Cyan midline
            for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
                int y1 = B_GRAPH_CENTER - b_history[i];
                int y2 = B_GRAPH_CENTER - b_history[i + 1];
                M5.Lcd.drawLine(GRAPH_X + i, y1, GRAPH_X + i + 1, y2, 0x07FF); // Cyan breathing line
            }

            // Redraw Heart Rate Waveform Area
            M5.Lcd.fillRect(GRAPH_X, H_GRAPH_Y, GRAPH_WIDTH, H_GRAPH_HEIGHT, 0x0842);
            M5.Lcd.drawFastHLine(GRAPH_X, H_GRAPH_CENTER, GRAPH_WIDTH, 0x2800); // Red midline
            for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
                int y1 = H_GRAPH_CENTER - h_history[i];
                int y2 = H_GRAPH_CENTER - h_history[i + 1];
                M5.Lcd.drawLine(GRAPH_X + i, y1, GRAPH_X + i + 1, y2, TFT_RED); // Red heart line
            }

            // Presence and Proximity Classifier Logic
            String presence = "Vacant";
            uint16_t presence_color = 0x7BEF; // Gray
            
            if (score > 1.2f) {
                presence = "Active";
                presence_color = TFT_YELLOW;
            } else if (b_env >= 0.015f) {
                presence = "Resting";
                presence_color = 0x07E0; // Green
            }

            String proximity = "N/A";
            if (presence != "Vacant") {
                if (score > 3.0f || b_env > 0.08f) {
                    proximity = "Near";
                } else if (score > 1.5f || b_env > 0.03f) {
                    proximity = "Mid";
                } else {
                    proximity = "Far";
                }
            }

            // Draw HUD text area
            M5.Lcd.fillRect(10, 175, 300, 52, 0x0842); // Clear HUD area
            M5.Lcd.setTextDatum(TL_DATUM);

            // Col 1: Presence & Proximity
            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.drawString("Presence:", 15, 178, 2);
            M5.Lcd.setTextColor(presence_color);
            M5.Lcd.drawString(presence, 90, 178, 2);

            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.drawString("Proximity:", 15, 198, 2);
            if (proximity == "Near") M5.Lcd.setTextColor(TFT_RED);
            else if (proximity == "Mid") M5.Lcd.setTextColor(TFT_YELLOW);
            else if (proximity == "Far") M5.Lcd.setTextColor(0x07E0); // Green
            else M5.Lcd.setTextColor(0x7BEF); // Gray
            M5.Lcd.drawString(proximity, 90, 198, 2);

            // Col 2: Vital Rates (BPMs)
            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.drawString("Breathing:", 155, 178, 2);
            if (b_bpm > 0.0f) {
                M5.Lcd.setTextColor(0x07FF); // Cyan
                M5.Lcd.drawString(String(b_bpm, 1) + " BPM", 240, 178, 2);
            } else {
                M5.Lcd.setTextColor(0x7BEF);
                M5.Lcd.drawString("-- BPM", 240, 178, 2);
            }

            M5.Lcd.setTextColor(TFT_WHITE);
            M5.Lcd.drawString("Heart Rate:", 155, 198, 2);
            if (h_bpm > 0.0f) {
                M5.Lcd.setTextColor(TFT_RED); // Red
                M5.Lcd.drawString(String(h_bpm, 1) + " BPM", 245, 198, 2);
            } else {
                M5.Lcd.setTextColor(0x7BEF);
                M5.Lcd.drawString("-- BPM", 245, 198, 2);
            }
        }

        delay(10);
    }

    // Clean teardown when exiting
    M5.Lcd.fillScreen(0x0842);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("Stopping WiFi CSI sniffer...", 160, 120, 2);
    deinitWifiRadar();
    delay(200);
}

void setup() {
    // 1. Initialize M5Stack Core2 hardware
    // This turns on the AXP192 power manager, enabling power (LDO2) to LCD and SD Card
    M5.begin();
    
    // Limit AXP192 Battery Charge Voltage to 4.10V (approx 80-90%) to preserve lifespan
    Wire1.beginTransmission(0x34); // AXP192 I2C address
    Wire1.write(0x33);             // Charge control register 1
    Wire1.endTransmission();
    Wire1.requestFrom((uint8_t)0x34, (uint8_t)1);
    if (Wire1.available()) {
        uint8_t reg33 = Wire1.read();
        // Bits [6:5] control charge voltage: 00 = 4.10V, 01 = 4.15V, 10 = 4.20V (default)
        reg33 &= ~(0b01100000); // Clear bits 5 and 6 to set 4.10V
        Wire1.beginTransmission(0x34);
        Wire1.write(0x33);
        Wire1.write(reg33);
        Wire1.endTransmission();
    }
    
    Serial.println("[SYSTEM] Booting M5Stack Core2 App Launcher...");

    // 2. Initialize the shared SPI bus & SD Card (20MHz speed)
    isSdInitialized = initSPIAndSD(20);

    if (isSdInitialized) {
        printSDCardInfo();
    } else {
        Serial.println("[SYSTEM] Warning: SD card failed to initialize on startup.");
    }

    // 3. Launch UI Loop
    runMainMenu();
}

void loop() {
    // Execution stays within runMainMenu() event loop.
    // If it ever exits, keep calling it.
    runMainMenu();
}
