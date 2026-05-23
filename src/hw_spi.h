/**
 * @file hw_spi.h
 * @brief SPI and SdFat Initialization Logic for M5Stack Core2.
 * 
 * Target Board: M5Stack Core2
 * SPI Pin Definitions:
 *   - SCK:  18
 *   - MISO: 38
 *   - MOSI: 23
 *   - CS:   4 (SD Card Chip Select)
 *   - LCD CS: 5 (For disabling display during initialization / ensuring clean state)
 * 
 * SPI Sharing Design:
 *   - Configures the SdFat library using the `SdSpiConfig` helper.
 *   - Uses `SHARED_SPI` mode to enable SPI transactions. This ensures that when SdFat 
 *     accesses the SD card, it locks the SPI bus and adjusts settings using SPI transactions.
 *     Similarly, when the LCD or other SPI peripherals use the bus, they lock and unlock 
 *     via transactions, preventing bus conflicts.
 *   - Forces both the SD CS (GPIO 4) and LCD CS (GPIO 5) to HIGH (disabled state) before 
 *     bus initialization. This ensures the SPI bus starts in a well-defined state where 
 *     neither peripheral is listening to early clock cycles.
 * 
 * Hardware Power:
 *   - The SD Card slot on M5Stack Core2 is powered via the AXP192 Power Management IC.
 *   - By default, LDO2 (which powers the LCD logic and SD card) must be enabled (usually 3.3V).
 *   - This is normally handled by M5.begin() or M5Unified.begin(). It is recommended to 
 *     call M5.begin() before calling the initialization function in this header.
 */

#ifndef HW_SPI_H
#define HW_SPI_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

// Custom pin definitions for M5Stack Core2 SD Card SPI
#define SD_SCK_PIN    18
#define SD_MISO_PIN   38
#define SD_MOSI_PIN   23
#define SD_CS_PIN     4
#define LCD_CS_PIN    5   // Shared SPI LCD CS Pin

// Helper function to access the single global SdFat instance
inline SdFat& getSdFat() {
    static SdFat sdInstance;
    return sdInstance;
}

/**
 * @brief Initializes the SPI bus and the SdFat filesystem.
 * @param speedMHz SPI bus frequency for the SD card (defaults to 20MHz).
 *                 If sharing with LCD, 20MHz is a stable and high-performance choice.
 *                 If communication issues occur, try lowering to 10MHz or 5MHz.
 * @return true if both SPI and SdFat initialized successfully, false otherwise.
 */
inline bool initSPIAndSD(uint8_t speedMHz = 20) {
    Serial.println("[SD_INIT] Starting SPI & SdFat initialization for M5Stack Core2...");

    // 1. Configure Chip Select pins immediately to prevent bus contention.
    // Setting both CS pins to HIGH ensures both peripherals start in an inactive state.
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    
    pinMode(LCD_CS_PIN, OUTPUT);
    digitalWrite(LCD_CS_PIN, HIGH);

    // 2. Initialize the shared SPI bus with custom pins.
    // The LCD also uses SCK=18, MISO=38, MOSI=23 on M5Stack Core2.
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    Serial.println("[SD_INIT] SPI bus initialized (SCK: 18, MISO: 38, MOSI: 23, CS: 4).");

    // 3. Configure SdFat to use the shared SPI bus.
    // SdSpiConfig parameters:
    //   - SD_CS_PIN: The Chip Select pin.
    //   - SHARED_SPI: Configures SdFat to yield the bus and use SPI transactions,
    //     allowing co-existence with the LCD display.
    //   - SD_SCK_MHZ(speedMHz): Sets SPI clock frequency.
    //   - &SPI: Pointer to the standard SPIClass instance.
    SdSpiConfig config(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(speedMHz), &SPI);

    // 4. Initialize SdFat
    if (!getSdFat().begin(config)) {
        Serial.println("[SD_INIT] ERROR: SdFat initialization failed!");
        // Print detailed initialization error if available
        if (getSdFat().sdErrorCode()) {
            Serial.print("[SD_INIT] SdFat Error Code: 0x");
            Serial.println(getSdFat().sdErrorCode(), HEX);
            Serial.print("[SD_INIT] SdFat Error Data: 0x");
            Serial.println(getSdFat().sdErrorData(), HEX);
        }
        return false;
    }

    Serial.println("[SD_INIT] SdFat successfully initialized!");
    return true;
}

/**
 * @brief Reads and prints SD card metadata (Card type, capacity, partition structure).
 */
inline void printSDCardInfo() {
    if (!getSdFat().card()) {
        Serial.println("[SD_INFO] Error: No card initialized.");
        return;
    }

    Serial.println("================ SD CARD INFORMATION ================");
    
    // 1. Card Type
    uint8_t cardType = getSdFat().card()->type();
    Serial.print("Card Type:       ");
    switch (cardType) {
        case SD_CARD_TYPE_SD1:
            Serial.println("SD1 (Standard capacity V1)");
            break;
        case SD_CARD_TYPE_SD2:
            Serial.println("SD2 (Standard capacity V2)");
            break;
        case SD_CARD_TYPE_SDHC:
            Serial.println("SDHC/SDXC (High capacity)");
            break;
        default:
            Serial.println("Unknown card type");
            break;
    }

    // 2. Card Size
    uint32_t sectors = getSdFat().card()->sectorCount();
    if (sectors == 0) {
        Serial.println("Error: Failed to read card size.");
    } else {
        double sizeGB = (double)sectors * 512.0 / (1024.0 * 1024.0 * 1024.0);
        Serial.printf("Capacity:        %.2f GB (%u sectors)\n", sizeGB, sectors);
    }

    // 3. Filesystem Type
    uint8_t fatType = getSdFat().vol()->fatType();
    Serial.print("File System:     ");
    switch (fatType) {
        case 12:
            Serial.println("FAT12");
            break;
        case 16:
            Serial.println("FAT16");
            break;
        case 32:
            Serial.println("FAT32");
            break;
        case 64:
            Serial.println("exFAT");
            break;
        default:
            Serial.printf("Unknown FAT type (%u)\n", fatType);
            break;
    }
    Serial.println("=====================================================");
}

/**
 * @brief Validates SPI sharing and file operation integrity by performing
 *        a quick write-then-read validation test on the card.
 * @return true if validation passes successfully, false otherwise.
 */
inline bool performSDCardTest() {
    Serial.println("[SD_TEST] Running file write and read validation test...");
    const char* filePath = "/spi_test.tmp";
    const char* testMsg = "M5Stack Core2 SPI Sharing & SdFat Verification Token: 0xDEADBEEF";

    // 1. Open file for writing (creates file if it doesn't exist, overwrites/truncates if it does)
    FsFile file = getSdFat().open(filePath, O_RDWR | O_CREAT | O_TRUNC);
    if (!file) {
        Serial.println("[SD_TEST] ERROR: Failed to open file for writing.");
        return false;
    }

    // 2. Write verification message to the file
    int writtenBytes = file.println(testMsg);
    file.close(); // Save and close
    
    if (writtenBytes <= 0) {
        Serial.println("[SD_TEST] ERROR: Failed to write to file.");
        return false;
    }
    Serial.printf("[SD_TEST] Successfully wrote %d bytes to %s\n", writtenBytes, filePath);

    // 3. Re-open the file for reading
    file = getSdFat().open(filePath, O_READ);
    if (!file) {
        Serial.println("[SD_TEST] ERROR: Failed to open file for reading.");
        return false;
    }

    // 4. Read back and verify the file content
    String readLine = file.readStringUntil('\n');
    readLine.trim(); // Remove whitespace / newline
    file.close();

    Serial.print("[SD_TEST] Content read back: \"");
    Serial.print(readLine);
    Serial.println("\"");

    if (readLine == testMsg) {
        Serial.println("[SD_TEST] SUCCESS: Content matches exactly. SPI transaction validation passed!");
        
        // 5. Clean up by deleting the temporary file
        if (getSdFat().remove(filePath)) {
            Serial.println("[SD_TEST] Cleaned up temporary test file.");
        } else {
            Serial.println("[SD_TEST] Warning: Failed to delete temporary test file.");
        }
        return true;
    } else {
        Serial.println("[SD_TEST] ERROR: Read content mismatch!");
        return false;
    }
}

#endif // HW_SPI_H
