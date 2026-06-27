#ifndef IMU_UI_H
#define IMU_UI_H

#include <M5Core2.h>
#include "imu_engine.h"

// Forward declarations of local helpers
void localWaitForTouchRelease();
void drawStaticPanels(TFT_eSprite& sprite);
void drawLeftPanel(TFT_eSprite& sprite);
void drawCenterPanel(TFT_eSprite& sprite);
void drawRightPanel(TFT_eSprite& sprite);

inline void localWaitForTouchRelease() {
    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
        
        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x == -1) {
            break;
        }
        delay(10);
    }
}

inline void drawStaticPanels(TFT_eSprite& sprite) {
    // 1. Header Bar
    sprite.fillRect(0, 0, 320, 35, 0x10A2); // Sapphire header
    sprite.drawFastHLine(0, 35, 320, 0x07FF); // Cyan divider line
    
    // Left Battery and IMU Source
    sprite.setTextDatum(ML_DATUM);
    
    // 1. Battery Indicator
    float batVolts = 0.0f;
    bool isCharging = false;
    if (IMUEngine::imuMutex != NULL && xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY) == pdTRUE) {
        batVolts = M5.Axp.GetBatVoltage();
        isCharging = M5.Axp.isCharging();
        xSemaphoreGive(IMUEngine::imuMutex);
    }
    int batPct = (int)((batVolts < 3.2f) ? 0 : (batVolts - 3.2f) * 100.0f);
    if (batPct > 100) batPct = 100;
    sprite.setTextColor(isCharging ? 0x07E0 : TFT_WHITE);
    char batBuf[16];
    sprintf(batBuf, "BAT: %d%%", batPct);
    sprite.drawString(batBuf, 8, 12, 2); // Larger Font 2
    
    // 2. IMU Source
    bool extIMU = IMUEngine::getBmi160Detected();
    bool onboardIMU = IMUEngine::getOnboardImuDetected();
    if (!extIMU && !onboardIMU) {
        sprite.setTextColor(TFT_RED);
        sprite.drawString("NO IMU DETECTED!", 8, 26, 1);
    } else {
        sprite.setTextColor(0x07FF); // Cyan
        sprite.drawString(extIMU ? "IMU: BMI160" : "IMU: MPU6886", 8, 26, 1);
    }
    
    // Title
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("9DOF RACING TELEMETRY", 160, 17, 2); // Font 2 for title
    
    // Right GPS Status
    uint32_t sats = IMUEngine::getGpsSats();
    sprite.setTextDatum(MR_DATUM);
    if (sats == 0) {
        sprite.setTextColor(TFT_RED);
        sprite.drawString("GPS: NO LOCK", 312, 17, 2); // Increased to Font 2
    } else {
        sprite.setTextColor(0x07E0); // Green
        char gpsBuf[32];
        sprintf(gpsBuf, "GPS: OK (%d)", sats);
        sprite.drawString(gpsBuf, 312, 17, 2); // Increased to Font 2
    }

    // 2. Bottom Button Zone (Y: 212 to 238)
    sprite.setTextDatum(MC_DATUM);
    
    // Button A: CALIBRATE
    sprite.fillRoundRect(2, 212, 102, 26, 4, 0x10A2);
    sprite.drawRoundRect(2, 212, 102, 26, 4, 0x07FF); // Cyan border
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("CALIBRATE", 53, 225, 2);

    // Button B: EXIT
    sprite.fillRoundRect(109, 212, 102, 26, 4, 0x10A2);
    sprite.drawRoundRect(109, 212, 102, 26, 4, TFT_RED); // Red border
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("EXIT", 160, 225, 2);

    // Button C: RESET
    sprite.fillRoundRect(216, 212, 102, 26, 4, 0x10A2);
    sprite.drawRoundRect(216, 212, 102, 26, 4, 0xFDA0); // Orange border
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("RESET", 267, 225, 2);
}

inline void drawLeftPanel(TFT_eSprite& sprite) {
    // Left Panel (Lean Angle): X=6, Y=40, W=102, H=168
    int x = 6;
    int y = 40;
    int w = 102;
    int h = 168;
    
    // Draw Cyan panel border
    sprite.drawRoundRect(x, y, w, h, 4, 0x07FF);
    
    // Title
    int cx = x + w / 2; // 57
    sprite.setTextColor(0x07FF);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("LEAN ANGLE", cx, 52, 2); // Font 2 for panel headers
    
    int cy = 110;
    int r = 38;
    
    // Semi-circular inclinometer arc showing roll (left/right tilt) from -60 to +60 degrees
    for (float a = -60.0f; a < 60.0f; a += 1.0f) {
        float rad1 = (a - 90.0f) * DEG_TO_RAD;
        float rad2 = (a + 1.0f - 90.0f) * DEG_TO_RAD;
        float x1 = cx + r * cosf(rad1);
        float y1 = cy + r * sinf(rad1);
        float x2 = cx + r * cosf(rad2);
        float y2 = cy + r * sinf(rad2);
        
        uint16_t color = 0x4208; // Subtle gray
        float abs_a = fabsf(a);
        if (abs_a <= 20.0f) {
            color = 0x07E0; // Green
        } else if (abs_a <= 45.0f) {
            color = 0xFDA0; // Orange
        } else {
            color = TFT_RED; // Red
        }
        sprite.drawLine(x1, y1, x2, y2, color);
    }
    
    // Major tick marks
    for (int a = -60; a <= 60; a += 15) {
        float rad = (a - 90.0f) * DEG_TO_RAD;
        float cos_r = cosf(rad);
        float sin_r = sinf(rad);
        
        float x_in = cx + (r - 3) * cos_r;
        float y_in = cy + (r - 3) * sin_r;
        float x_out = cx + (r + 2) * cos_r;
        float y_out = cy + (r + 2) * sin_r;
        
        sprite.drawLine(x_in, y_in, x_out, y_out, TFT_WHITE);
    }
    
    // Dynamic roll angle
    float current_roll = IMUEngine::getRoll();
    if (current_roll > 60.0f) current_roll = 60.0f;
    if (current_roll < -60.0f) current_roll = -60.0f;
    
    float roll_rad = (current_roll - 90.0f) * DEG_TO_RAD;
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    
    // Pivot
    sprite.fillCircle(cx, cy, 3, 0x07FF);
    sprite.drawCircle(cx, cy, 5, TFT_WHITE);
    
    // Dynamic rotating line/needle
    float needle_x = cx + (r - 4) * cos_roll;
    float needle_y = cy + (r - 4) * sin_roll;
    sprite.drawLine(cx, cy, needle_x, needle_y, 0x07FF);
    
    // Peak markers (orange dots) representing max left and right lean
    float peak_l = IMUEngine::getPeakLeftLean();
    float peak_r = IMUEngine::getPeakRightLean();
    if (peak_l > 0.01f) {
        float p_left = -peak_l;
        if (p_left < -60.0f) p_left = -60.0f;
        float rad_p_left = (p_left - 90.0f) * DEG_TO_RAD;
        float px = cx + (r + 1) * cosf(rad_p_left);
        float py = cy + (r + 1) * sinf(rad_p_left);
        sprite.fillCircle(px, py, 2.5, 0xFDA0); // Orange dot
    }
    if (peak_r > 0.01f) {
        float p_right = peak_r;
        if (p_right > 60.0f) p_right = 60.0f;
        float rad_p_right = (p_right - 90.0f) * DEG_TO_RAD;
        float px = cx + (r + 1) * cosf(rad_p_right);
        float py = cy + (r + 1) * sinf(rad_p_right);
        sprite.fillCircle(px, py, 2.5, 0xFDA0); // Orange dot
    }
    
    // Large digital roll readout (e.g. "12.3 L" or "4.5 R")
    String leanStr;
    if (current_roll < -0.05f) {
        leanStr = String(-current_roll, 1) + " L";
    } else if (current_roll > 0.05f) {
        leanStr = String(current_roll, 1) + " R";
    } else {
        leanStr = "0.0";
    }
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString(leanStr, cx, 155, 4); // Y=155, Font 4
    
    // Peak label summary (symmetrical)
    char peakStr[32];
    sprintf(peakStr, "PK: %.0fL / %.0fR", peak_l, peak_r);
    sprite.setTextColor(0x7BEF);
    sprite.drawString(peakStr, cx, 185, 2);
}

inline void drawCenterPanel(TFT_eSprite& sprite) {
    // Center Panel (Wheelie / Dive): X=112, Y=40, W=102, H=168
    int x = 112;
    int y = 40;
    int w = 102;
    int h = 168;
    
    // Draw Green panel border
    sprite.drawRoundRect(x, y, w, h, 4, 0x07E0);
    
    // Title
    int cx = x + w / 2; // 163
    sprite.setTextColor(0x07E0);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("PITCH STUNT", cx, 52, 2); // Font 2 for panel headers
    
    int cy = 110;
    
    // Dynamic vector-drawn motorcycle side profile (wheels as circles, frame/forks as lines)
    // that rotates/pitches dynamically based on pitch angle.
    float pitch_val = IMUEngine::getPitch();
    // Constrain pitch to reasonable visual limits (e.g., -45 to +45)
    float pitch_disp = pitch_val;
    if (pitch_disp > 45.0f) pitch_disp = 45.0f;
    if (pitch_disp < -45.0f) pitch_disp = -45.0f;
    
    float pitch_rad = pitch_disp * DEG_TO_RAD;
    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);
    
    // Local coordinates (relative to CX, CY)
    float rx_hub_l = -22.0f, ry_hub_l = 8.0f;   // Rear wheel hub
    float fx_hub_l = 22.0f,  ry_fhub_l = 8.0f;  // Front wheel hub
    float bb_x_l = -2.0f,    bb_y_l = 4.0f;     // Bottom bracket / Engine base
    float seat_x_l = -12.0f, seat_y_l = -10.0f; // Seat / Saddle
    float head_x_l = 10.0f,  head_y_l = -10.0f; // Head tube
    float bars_x_l = 8.0f,   bars_y_l = -18.0f; // Handlebars top
    
    // Rotate coordinates around pivot (CX, CY)
    // Formula:
    // x_rot = x * cos(pitch) + y * sin(pitch)
    // y_rot = -x * sin(pitch) + y * cos(pitch)
    float rx_hub = cx + (rx_hub_l * cos_p + ry_hub_l * sin_p);
    float ry_hub = cy + (-rx_hub_l * sin_p + ry_hub_l * cos_p);
    
    float fx_hub = cx + (fx_hub_l * cos_p + ry_fhub_l * sin_p);
    float fy_hub = cy + (-fx_hub_l * sin_p + ry_fhub_l * cos_p);
    
    float bb_x = cx + (bb_x_l * cos_p + bb_y_l * sin_p);
    float bb_y = cy + (-bb_x_l * sin_p + bb_y_l * cos_p);
    
    float seat_x = cx + (seat_x_l * cos_p + seat_y_l * sin_p);
    float seat_y = cy + (-seat_x_l * sin_p + seat_y_l * cos_p);
    
    float head_x = cx + (head_x_l * cos_p + head_y_l * sin_p);
    float head_y = cy + (-head_x_l * sin_p + head_y_l * cos_p);
    
    float bars_x = cx + (bars_x_l * cos_p + bars_y_l * sin_p);
    float bars_y = cy + (-bars_x_l * sin_p + bars_y_l * cos_p);
    
    // Draw static ground reference line (faint gray)
    sprite.drawFastHLine(cx - 30, cy + 16, 60, 0x1843);
    
    // Draw rear wheel (black tire outline, grey rim, green hub)
    sprite.drawCircle(rx_hub, ry_hub, 8, TFT_WHITE);
    sprite.drawCircle(rx_hub, ry_hub, 6, 0x7BEF);
    sprite.fillCircle(rx_hub, ry_hub, 2, 0x07E0);
    
    // Draw front wheel
    sprite.drawCircle(fx_hub, fy_hub, 8, TFT_WHITE);
    sprite.drawCircle(fx_hub, fy_hub, 6, 0x7BEF);
    sprite.fillCircle(fx_hub, fy_hub, 2, 0x07E0);
    
    // Draw frame parts
    sprite.drawLine(rx_hub, ry_hub, bb_x, bb_y, 0x7BEF);     // Swingarm (grey)
    sprite.drawLine(bb_x, bb_y, head_x, head_y, 0x07E0);     // Main frame tube (green)
    sprite.drawLine(bb_x, bb_y, seat_x, seat_y, 0x07E0);     // Seat tube (green)
    sprite.drawLine(rx_hub, ry_hub, seat_x, seat_y, 0x7BEF); // Rear subframe (grey)
    sprite.drawLine(head_x, head_y, fx_hub, fy_hub, 0xFDA0); // Front fork (orange)
    sprite.drawLine(head_x, head_y, bars_x, bars_y, TFT_WHITE); // Handlebars neck (white)
    
    // Handlebars crossbar
    sprite.drawLine(bars_x - 3, bars_y - 2, bars_x + 3, bars_y + 2, TFT_WHITE);
    
    // Pitch digital readout (e.g. "+12.4°" or "-4.2°")
    String pitchStr = (pitch_val >= 0.0f ? "+" : "") + String(pitch_val, 1) + "\xDF";
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString(pitchStr, cx, 155, 4); // Y=155, Font 4
    
    // Label positive pitch as "WHEELIE", negative pitch as "DIVE", neutral as "LEVEL"
    String stuntLabel = "LEVEL";
    uint16_t stuntColor = 0x7BEF;
    if (pitch_val > 1.5f) {
        stuntLabel = "WHEELIE";
        stuntColor = 0x07E0; // Green
    } else if (pitch_val < -1.5f) {
        stuntLabel = "DIVE";
        stuntColor = TFT_RED; // Red
    }
    sprite.setTextColor(stuntColor);
    sprite.drawString(stuntLabel, cx, 172, 2);
    
    // Session peak wheelie and peak dive
    float peak_w = IMUEngine::getPeakWheelie();
    float peak_d = IMUEngine::getPeakDive();
    char peakPStr[32];
    sprintf(peakPStr, "PK: %.0fW / %.0fD", peak_w, peak_d);
    sprite.setTextColor(0x7BEF);
    sprite.drawString(peakPStr, cx, 185, 2);
}

inline void drawRightPanel(TFT_eSprite& sprite) {
    // Right Panel (Speed & G-Force): X=218, Y=40, W=96, H=168
    int x = 218;
    int y = 40;
    int w = 96;
    int h = 168;
    
    // Draw Orange panel border
    sprite.drawRoundRect(x, y, w, h, 4, 0xFDA0);
    
    // Title
    int cx = x + (w - 30) / 2; // Center of left portion
    int tx = 251; // Target X for centered elements
    sprite.setTextColor(0xFDA0);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("SPEED & G", tx, 52, 2); // Font 2 for panel headers
    
    // GPS Speed
    float speed = IMUEngine::getGpsSpeedKmh();
    uint32_t sats = IMUEngine::getGpsSats();
    String speedStr;
    if (sats == 0 || speed <= 0.05f) {
        speedStr = "--";
    } else {
        speedStr = String((int)speed);
    }
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString(speedStr, tx, 75, 4); // Large speed Font 4
    sprite.setTextColor(0x7BEF);
    sprite.drawString("km/h", tx, 95, 1);
    
    // Heading/Compass (e.g., "NE 45°") using IMUEngine::getYaw()
    float yawVal = IMUEngine::getYaw();
    const char* dir = "N";
    if (yawVal >= 22.5f && yawVal < 67.5f) dir = "NE";
    else if (yawVal >= 67.5f && yawVal < 112.5f) dir = "E";
    else if (yawVal >= 112.5f && yawVal < 157.5f) dir = "SE";
    else if (yawVal >= 157.5f && yawVal < 202.5f) dir = "S";
    else if (yawVal >= 202.5f && yawVal < 247.5f) dir = "SW";
    else if (yawVal >= 247.5f && yawVal < 292.5f) dir = "W";
    else if (yawVal >= 292.5f && yawVal < 337.5f) dir = "NW";
    
    char compassBuf[32];
    sprintf(compassBuf, "%s %.0f\xDF", dir, yawVal);
    sprite.setTextColor(0x07FF); // Cyan
    sprite.drawString(compassBuf, tx, 115, 2);
    
    // Numerical G-Force
    float g_val = IMUEngine::getLinearAccel();
    sprite.setTextColor(0x7BEF);
    sprite.drawString("G-FORCE", tx, 142, 1);
    
    String gStr = (g_val >= 0.0f ? "+" : "") + String(g_val, 2) + "G";
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString(gStr, tx, 158, 2);
    
    // Peak G summary
    float peak_a = IMUEngine::getPeakAccel();
    float peak_d = IMUEngine::getPeakDecel();
    char peakGStr[32];
    sprintf(peakGStr, "+%.2f / -%.2f", peak_a, peak_d);
    sprite.setTextColor(0x7BEF);
    sprite.drawString(peakGStr, tx, 185, 2);
    
    // Vertical G-Force Bar (on the right portion of Right Panel)
    int bx = 296; // center of right portion (X=284 to 308)
    int by_center = 110;
    float scale_y = 40.0f; // pixels per G
    
    // Draw background bar frame (height 80, Y = [70, 150])
    sprite.fillRect(bx - 4, by_center - 40, 8, 80, 0x1843);
    sprite.drawRect(bx - 4, by_center - 40, 8, 80, 0x7BEF);
    
    // Center line (0G)
    sprite.drawFastHLine(bx - 7, by_center, 14, TFT_WHITE);
    
    // Fill bar (green for accel, red for decel)
    if (g_val > 0.0f) {
        int h_fill = (int)(g_val * scale_y);
        if (h_fill > 40) h_fill = 40;
        sprite.fillRect(bx - 3, by_center - h_fill, 6, h_fill, 0x07E0); // Green
    } else if (g_val < 0.0f) {
        int h_fill = (int)(-g_val * scale_y);
        if (h_fill > 40) h_fill = 40;
        sprite.fillRect(bx - 3, by_center, 6, h_fill, TFT_RED); // Red
    }
    
    // Ticks on left side of bar (+1.0, +0.5, -0.5, -1.0)
    float ticks[] = {1.0f, 0.5f, -0.5f, -1.0f};
    for (int i = 0; i < 4; i++) {
        int ty = by_center - (int)(ticks[i] * scale_y);
        sprite.drawFastHLine(bx - 7, ty, 3, 0x7BEF);
    }
    
    // Session peak G markers
    if (peak_a > 0.01f) {
        int py = by_center - (int)(peak_a * scale_y);
        if (py < by_center - 40) py = by_center - 40;
        sprite.drawFastHLine(bx - 6, py, 12, 0x07FF); // Cyan tick for Peak Accel
    }
    if (peak_d > 0.01f) {
        int py = by_center + (int)(peak_d * scale_y);
        if (py > by_center + 40) py = by_center + 40;
        sprite.drawFastHLine(bx - 6, py, 12, 0xFDA0); // Orange tick for Peak Decel
    }
}

// Main run entry function
inline void runImuDashboard() {
    // Ensure IMU is initialized
    IMUEngine::init();

    // Create double-buffered sprite
    TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);
    sprite.createSprite(320, 240);

    // Dynamic Self-Test Splash Screen Loop (1.8 seconds maximum)
    uint32_t splashStart = millis();
    while (millis() - splashStart < 1800) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
        
        bool ext_imu = IMUEngine::getBmi160Detected();
        bool onboard_imu = IMUEngine::getOnboardImuDetected();
        bool imu_ok = ext_imu || onboard_imu;
        bool mag_ok = IMUEngine::getLsm303Detected();
        bool gps_ok = IMUEngine::getGpsDetected();
        
        sprite.fillSprite(0x0842); // slate charcoal
        sprite.drawRoundRect(15, 15, 290, 210, 8, 0x07FF); // Cyan border frame
        
        sprite.setTextColor(TFT_WHITE);
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("SYSTEM SELF-TEST", 160, 45, 4); // Font 4
        sprite.drawFastHLine(30, 65, 260, 0x1843);
        
        // 1. IMU Status
        sprite.setTextDatum(ML_DATUM);
        sprite.setTextColor(TFT_WHITE);
        if (ext_imu) {
            sprite.drawString("IMU (BMI160 Click):", 35, 95, 2);
        } else if (onboard_imu) {
            sprite.drawString("IMU (MPU6886 Onboard):", 35, 95, 2);
        } else {
            sprite.drawString("IMU Sensor:", 35, 95, 2);
        }
        sprite.setTextDatum(MR_DATUM);
        if (imu_ok) {
            sprite.setTextColor(0x07E0); // Green
            sprite.drawString("OK", 285, 95, 2);
        } else {
            sprite.setTextColor(TFT_RED); // Red
            sprite.drawString("FAIL (NO IMU)", 285, 95, 2);
        }
        
        // 2. MAG Status
        sprite.setTextDatum(ML_DATUM);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString("MAG (LSM303 Compass):", 35, 130, 2);
        sprite.setTextDatum(MR_DATUM);
        if (mag_ok) {
            sprite.setTextColor(0x07E0); // Green
            sprite.drawString("OK", 285, 130, 2);
        } else {
            sprite.setTextColor(TFT_RED); // Red
            sprite.drawString("FAIL (NO MAG)", 285, 130, 2);
        }
        
        // 3. GPS Status
        sprite.setTextDatum(ML_DATUM);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString("GPS (Adafruit GPS):", 35, 165, 2);
        sprite.setTextDatum(MR_DATUM);
        if (gps_ok) {
            sprite.setTextColor(0x07E0); // Green
            sprite.drawString("OK", 285, 165, 2);
        } else {
            uint32_t elapsed = millis() - splashStart;
            if (elapsed < 1200) {
                sprite.setTextColor(TFT_YELLOW); // Yellow
                sprite.drawString("CHECKING...", 285, 165, 2);
            } else {
                sprite.setTextColor(TFT_RED); // Red
                sprite.drawString("FAIL (NO RX)", 285, 165, 2);
            }
        }
        
        sprite.setTextDatum(MC_DATUM);
        sprite.setTextColor(0x7BEF); // Gray
        sprite.drawString("Tap screen to skip", 160, 205, 1);
        
        sprite.pushSprite(0, 0);
        
        // Check for skip tap
        TouchPoint_t t_pos = M5.Touch.getPressPoint();
        if (t_pos.x != -1) {
            break;
        }
        delay(30);
    }

    localWaitForTouchRelease();

    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);

        // 1. Exit check (physical Button B)
        if (M5.BtnB.wasPressed()) {
            break;
        }

        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x != -1) {
            // Button B (Exit) region on screen: [109, 211], Y >= 212
            if (pos.x >= 109 && pos.x < 211 && pos.y >= 212) {
                break;
            }

            // Button A (Calibrate) region on screen: [0, 108], Y >= 212
            if (pos.x >= 0 && pos.x < 108 && pos.y >= 212) {
                // Show calibrating overlay popup on screen
                sprite.fillSprite(0x0842);
                drawStaticPanels(sprite);
                
                // Draw pop-up modal: "CALIBRATING... HOLD STILL"
                sprite.fillRoundRect(30, 70, 260, 100, 8, 0x10A2);
                sprite.drawRoundRect(30, 70, 260, 100, 8, 0xFDA0); // Orange
                sprite.setTextColor(TFT_WHITE);
                sprite.setTextDatum(MC_DATUM);
                sprite.drawString("CALIBRATING...", 160, 105, 4);
                sprite.drawString("HOLD STILL", 160, 140, 2);
                sprite.pushSprite(0, 0);
                
                // Trigger engine calibration (takes 2.0s)
                bool success = IMUEngine::calibrate();
                
                // Show calibration completion/failure popup
                sprite.fillSprite(0x0842);
                drawStaticPanels(sprite);
                sprite.fillRoundRect(30, 70, 260, 100, 8, 0x10A2);
                
                if (success) {
                    sprite.drawRoundRect(30, 70, 260, 100, 8, 0x07E0); // Green
                    sprite.setTextColor(0x07E0);
                    sprite.drawString("CALIBRATED!", 160, 120, 4);
                } else {
                    sprite.drawRoundRect(30, 70, 260, 100, 8, TFT_RED); // Red
                    sprite.setTextColor(TFT_RED);
                    sprite.drawString("CALIB FAILED!", 160, 105, 4);
                    sprite.drawString("KEEP STILL & RETRY", 160, 140, 2);
                }
                
                sprite.pushSprite(0, 0);
                delay(1500);
                localWaitForTouchRelease();
                continue;
            }

            // Button C (Reset Peaks) region on screen: [212, 320], Y >= 212
            if (pos.x >= 212 && pos.x <= 320 && pos.y >= 212) {
                // Show reset popup modal: "PEAKS RESET"
                sprite.fillSprite(0x0842);
                drawStaticPanels(sprite);
                
                sprite.fillRoundRect(30, 70, 260, 100, 8, 0x10A2);
                sprite.drawRoundRect(30, 70, 260, 100, 8, 0xFDA0);
                sprite.setTextColor(TFT_WHITE);
                sprite.setTextDatum(MC_DATUM);
                sprite.drawString("PEAKS RESET", 160, 120, 4);
                sprite.pushSprite(0, 0);
                
                IMUEngine::resetPeaks();
                
                delay(1000);
                localWaitForTouchRelease();
                continue;
            }
        }

        // 2. Render dashboard content (reading thread-safe values from FreeRTOS task)
        sprite.fillSprite(0x0842); // slate charcoal background
        drawStaticPanels(sprite);
        drawLeftPanel(sprite);
        drawCenterPanel(sprite);
        drawRightPanel(sprite);

        // 3. Push buffer to screen
        sprite.pushSprite(0, 0);

        delay(33); // ~30Hz UI refresh
    }

    // Clean up
    sprite.deleteSprite();
    localWaitForTouchRelease();
}

inline void runSysDebugger() {
    IMUEngine::init(); // Ensure initialized
    TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);
    sprite.createSprite(320, 240);
    
    localWaitForTouchRelease();
    
    bool runScanner = true;
    String i2c_scan_results = "";
    String wire0_scan_results = "";
    
    while (true) {
        if (IMUEngine::imuMutex != NULL) xSemaphoreTake(IMUEngine::imuMutex, portMAX_DELAY);
        M5.update();
        if (IMUEngine::imuMutex != NULL) xSemaphoreGive(IMUEngine::imuMutex);
        
        if (M5.BtnB.wasPressed()) break;
        
        TouchPoint_t pos = M5.Touch.getPressPoint();
        if (pos.x != -1) {
            // Button B (Exit): [109, 211], Y >= 212
            if (pos.x >= 109 && pos.x < 211 && pos.y >= 212) {
                break;
            }
            // Button A (Scan I2C): [0, 108], Y >= 212
            if (pos.x >= 0 && pos.x < 108 && pos.y >= 212) {
                runScanner = true;
                localWaitForTouchRelease();
            }
            // Button C (Reset peaks/GPS debug): [212, 320], Y >= 212
            if (pos.x >= 212 && pos.x <= 320 && pos.y >= 212) {
                IMUEngine::resetPeaks();
                localWaitForTouchRelease();
            }
        }
        
        // Run I2C scan if triggered
        if (runScanner) {
            // Suspend background task to prevent I2C bus contention.
            if (IMUEngine::imuTaskHandle != NULL) {
                IMUEngine::pause_background_task = true; delay(20);
            }
            delay(10);

            // Scan Wire (GPIO 21/22 - M-BUS)
            // Targeted scan to prevent locking up the AXP192 power chip
            i2c_scan_results = "";
            int scan_count = 0;
            const uint8_t scan_addrs[] = {0x19, 0x1E, 0x34, 0x38, 0x51, 0x68, 0x69};
            for (int i = 0; i < 7; i++) {
                uint8_t address = scan_addrs[i];
                Wire1.beginTransmission(address);
                if (Wire1.endTransmission() == 0) {
                    char addrHex[16];
                    sprintf(addrHex, "0x%02X", address);
                    i2c_scan_results += String(addrHex);
                    if (address == 0x1E) i2c_scan_results += "(MAG) ";
                    else if (address == 0x19) i2c_scan_results += "(ACC) ";
                    else if (address == 0x69 || address == 0x68) i2c_scan_results += "(IMU) ";
                    else if (address == 0x34) i2c_scan_results += "(PWR) ";
                    else if (address == 0x38) i2c_scan_results += "(TCH) ";
                    else if (address == 0x51) i2c_scan_results += "(RTC) ";
                    else i2c_scan_results += " ";
                    scan_count++;
                }
            }
            if (scan_count == 0) i2c_scan_results = "No devices!";

            // Resume the background task
            if (IMUEngine::imuTaskHandle != NULL) {
                IMUEngine::pause_background_task = false;
            }
            runScanner = false;
        }
        
        sprite.fillSprite(0x0842); // slate charcoal
        
        // 1. Header
        sprite.fillRect(0, 0, 320, 35, 0x10A2); // Sapphire header
        sprite.drawFastHLine(0, 35, 320, 0x07FF); // Cyan line
        sprite.setTextColor(TFT_WHITE);
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("SYSTEM HARDWARE DEBUGGER", 160, 17, 2);
        
        // 2. I2C Bus Scan Results
        sprite.setTextDatum(ML_DATUM);
        sprite.setTextColor(0xFDA0); // Orange
        sprite.drawString("I2C Scanner (Wire1 M-BUS 21/22):", 10, 42, 1);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(i2c_scan_results, 10, 56, 2);
        
        // Detection Status Summary
        char det_buf[80];
        sprintf(det_buf, "BMI160:%s  LSM303:%s  GPS:%s",
            IMUEngine::getBmi160Detected() ? "OK" : "FAIL",
            IMUEngine::getLsm303Detected() ? "OK" : "FAIL",
            IMUEngine::getGpsDetected() ? "OK" : "FAIL");
        sprite.setTextColor(IMUEngine::getBmi160Detected() && IMUEngine::getLsm303Detected() ? (uint16_t)0x07E0 : TFT_YELLOW);
        sprite.drawString(det_buf, 10, 78, 1);
        
        // 3. Raw GPS Terminal Stream
        sprite.setTextColor(0xFDA0);
        sprite.drawString("GPS Serial2 (RX:13 TX:14):", 10, 108, 1);
        
        char gps_buf[64];
        IMUEngine::getGpsRawBuffer(gps_buf);
        sprite.setTextColor(0x07E0); // Green terminal font
        sprite.drawString(gps_buf, 10, 120, 1);
        
        // GPS Stats
        char gps_stats[64];
        sprintf(gps_stats, "Chars: %u  Errors: %u", 
                (uint32_t)IMUEngine::gps.charsProcessed(), (uint32_t)IMUEngine::gps.failedChecksum());
        sprite.setTextColor(0x7BEF); // Subtle gray
        sprite.drawString(gps_stats, 10, 132, 1);
        
        // 4. Raw Sensor Values
        sprite.setTextColor(0xFDA0);
        sprite.drawString("Raw Telemetry & Alignment Check:", 10, 148, 1);
        
        char raw_buf1[64];
        sprintf(raw_buf1, "EXT ACC: [%.2f, %.2f, %.2f]G", IMUEngine::raw_accel.x, IMUEngine::raw_accel.y, IMUEngine::raw_accel.z);
        char raw_buf2[64];
        sprintf(raw_buf2, "EXT GYR: [%.1f, %.1f, %.1f]dps", IMUEngine::raw_gyro.x, IMUEngine::raw_gyro.y, IMUEngine::raw_gyro.z);
        char raw_buf3[64];
        sprintf(raw_buf3, "EXT MAG: [%.1f, %.1f, %.1f]raw", IMUEngine::raw_mag.x, IMUEngine::raw_mag.y, IMUEngine::raw_mag.z);
        
        float int_ax = 0.0f, int_ay = 0.0f, int_az = 0.0f;
        if (IMUEngine::getOnboardImuDetected()) {
            M5.IMU.getAccelData(&int_ax, &int_ay, &int_az);
        }
        char raw_buf4[64];
        sprintf(raw_buf4, "INT MPU: [%.2f, %.2f, %.2f]G", int_ax, int_ay, int_az);
        
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(raw_buf1, 10, 162, 1);
        sprite.drawString(raw_buf4, 10, 174, 1);
        sprite.drawString(raw_buf2, 10, 186, 1);
        sprite.drawString(raw_buf3, 10, 198, 1);
        
        // 5. Buttons footer
        sprite.setTextDatum(MC_DATUM);
        sprite.fillRoundRect(2, 212, 102, 26, 4, 0x10A2);
        sprite.drawRoundRect(2, 212, 102, 26, 4, 0x07FF);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString("RE-SCAN", 53, 225, 2);
        
        sprite.fillRoundRect(109, 212, 102, 26, 4, 0x10A2);
        sprite.drawRoundRect(109, 212, 102, 26, 4, TFT_RED);
        sprite.drawString("EXIT", 160, 225, 2);
        
        sprite.fillRoundRect(216, 212, 102, 26, 4, 0x10A2);
        sprite.drawRoundRect(216, 212, 102, 26, 4, 0xFDA0);
        sprite.drawString("RESET", 267, 225, 2);
        
        sprite.pushSprite(0, 0);
        delay(100); // 10Hz screen refresh is fine for debugger
    }
    
    sprite.deleteSprite();
    localWaitForTouchRelease();
}

#endif // IMU_UI_H
