#ifndef SD_DIAGNOSTICS_H
#define SD_DIAGNOSTICS_H

#include <Arduino.h>
#include <SdFat.h>
#include "hw_spi.h"

// Forward declaration of display log helper defined in main.cpp
void logToScreen(const String& msg, uint16_t color = 0xFFFF);

/**
 * @brief Measures sequential write speed of the SD card using a 32 KB buffer.
 * @param filepath The path of the temporary file to write.
 * @return Write speed in MB/s.
 */
inline float writeSpeedTest(const char* filepath = "/speed_test.bin") {
    if (!getSdFat().card()) {
        logToScreen("[SPEED_TEST] Error: SD card not initialized.", 0xF800);
        return 0.0f;
    }
    
    // Remove if file already exists
    if (getSdFat().exists(filepath)) {
        getSdFat().remove(filepath);
    }
    
    FsFile file = getSdFat().open(filepath, O_WRONLY | O_CREAT | O_TRUNC);
    if (!file) {
        logToScreen("[SPEED_TEST] Error: Failed to open file.", 0xF800);
        return 0.0f;
    }
    
    const size_t bufSize = 32768; // 32 KB
    uint8_t* buf = (uint8_t*)malloc(bufSize);
    if (!buf) {
        logToScreen("[SPEED_TEST] Error: Memory allocation failed.", 0xF800);
        file.close();
        return 0.0f;
    }
    
    // Fill buffer with dummy data
    for (size_t i = 0; i < bufSize; i++) {
        buf[i] = (uint8_t)(i % 256);
    }
    
    const uint32_t totalSize = 4194304; // 4 MB total size
    const uint32_t numWrites = totalSize / bufSize; // 128 blocks
    
    logToScreen("[SPEED_TEST] Writing 4 MB (128 x 32 KB blocks)...");
    uint32_t startTime = millis();
    for (uint32_t i = 0; i < numWrites; i++) {
        if (file.write(buf, bufSize) != bufSize) {
            logToScreen("[SPEED_TEST] Error: Write failed at block " + String(i), 0xF800);
            free(buf);
            file.close();
            return 0.0f;
        }
        if (i % 32 == 0) {
            yield();
        }
    }
    file.sync();
    uint32_t elapsed = millis() - startTime;
    file.close();
    free(buf);
    
    float speed = 0.0f;
    if (elapsed > 0) {
        speed = 4000.0f / (float)elapsed; // 4 MB / (elapsed / 1000 s)
    }
    Serial.printf("[SPEED_TEST] Write speed: %.2f MB/s (%u ms elapsed)\n", speed, elapsed);
    return speed;
}

/**
 * @brief Measures sequential read speed of the SD card using a 32 KB buffer.
 * @param filepath The path of the file to read.
 * @return Read speed in MB/s.
 */
inline float readSpeedTest(const char* filepath = "/speed_test.bin") {
    if (!getSdFat().card()) {
        logToScreen("[SPEED_TEST] Error: SD card not initialized.", 0xF800);
        return 0.0f;
    }
    
    FsFile file = getSdFat().open(filepath, O_READ);
    if (!file) {
        logToScreen("[SPEED_TEST] Error: Run write test first.", 0xF800);
        return 0.0f;
    }
    
    const size_t bufSize = 32768; // 32 KB
    uint8_t* buf = (uint8_t*)malloc(bufSize);
    if (!buf) {
        logToScreen("[SPEED_TEST] Error: Memory allocation failed.", 0xF800);
        file.close();
        return 0.0f;
    }
    
    const uint32_t totalSize = 4194304; // 4 MB
    const uint32_t numReads = totalSize / bufSize;
    
    logToScreen("[SPEED_TEST] Reading 4 MB (128 x 32 KB blocks)...");
    uint32_t startTime = millis();
    for (uint32_t i = 0; i < numReads; i++) {
        if (file.read(buf, bufSize) != (int32_t)bufSize) {
            logToScreen("[SPEED_TEST] Error: Read failed at block " + String(i), 0xF800);
            free(buf);
            file.close();
            return 0.0f;
        }
        if (i % 32 == 0) {
            yield();
        }
    }
    uint32_t elapsed = millis() - startTime;
    file.close();
    free(buf);
    
    // Clean up temporary file
    getSdFat().remove(filepath);
    
    float speed = 0.0f;
    if (elapsed > 0) {
        speed = 4000.0f / (float)elapsed; // 4 MB / (elapsed / 1000 s)
    }
    Serial.printf("[SPEED_TEST] Read speed: %.2f MB/s (%u ms elapsed)\n", speed, elapsed);
    return speed;
}

/**
 * @brief Verifies card capacity and checks for counterfeiting by writing
 *        signatures in 1 GB increments and verifying no wrap-around occurs.
 *        Original sector contents are backed up and restored.
 * @return true if card passes the check, false if counterfeit/error.
 */
inline bool lbaStripTest() {
    if (!getSdFat().card()) {
        logToScreen("[LBA_TEST] Error: SD card not initialized.", 0xF800);
        return false;
    }
    
    uint32_t sectorCount = getSdFat().card()->sectorCount();
    if (sectorCount == 0) {
        logToScreen("[LBA_TEST] Error: Failed to read sector count.", 0xF800);
        return false;
    }
    
    const uint32_t sectorStep = 2097152; // 1 GB in 512-byte sectors
    uint32_t numSectors = 0;
    for (uint32_t sec = 0; sec < sectorCount; sec += sectorStep) {
        numSectors++;
    }
    
    // Check available heap memory to avoid allocation failure
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t requiredHeap = numSectors * 512;
    if (requiredHeap > freeHeap - 32768) {
        uint32_t maxAllowedSectors = (freeHeap > 32768) ? (freeHeap - 32768) / 512 : 0;
        if (maxAllowedSectors < numSectors) {
            numSectors = maxAllowedSectors;
            logToScreen("[LBA_TEST] Warning: Low memory! Testing first " + String(numSectors) + " GB.", 0xFDA0);
        }
    }
    
    if (numSectors == 0) {
        logToScreen("[LBA_TEST] Error: Insufficient memory.", 0xF800);
        return false;
    }
    
    logToScreen("[LBA_TEST] Running LBA Strip Test on " + String(numSectors) + " sectors...");
    
    uint8_t* backupBuffer = (uint8_t*)malloc(numSectors * 512);
    if (!backupBuffer) {
        logToScreen("[LBA_TEST] Error: Backup allocation failed.", 0xF800);
        return false;
    }
    
    // 1. Read and backup original sector contents
    logToScreen("Backup: Reading sector contents...");
    for (uint32_t i = 0; i < numSectors; i++) {
        uint32_t sector = i * sectorStep;
        if (!getSdFat().card()->readSector(sector, backupBuffer + (i * 512))) {
            logToScreen("[LBA_TEST] Error reading sector " + String(sector), 0xF800);
            free(backupBuffer);
            return false;
        }
        if (i % 2 == 0) {
            logToScreen("Backed up: " + String(i) + " GB sector");
            yield();
        }
    }
    
    // 2. Write unique signatures
    logToScreen("Writing signature test blocks...");
    uint8_t signatureBlock[512];
    for (uint32_t i = 0; i < numSectors; i++) {
        uint32_t sector = i * sectorStep;
        memset(signatureBlock, 0, 512);
        
        snprintf((char*)signatureBlock, 64, "LBA_SIG_SEC_%u_IDX_%u", sector, i);
        for (size_t j = 64; j < 512; j++) {
            signatureBlock[j] = (uint8_t)((sector + j + i) & 0xFF);
        }
        
        if (!getSdFat().card()->writeSector(sector, signatureBlock)) {
            logToScreen("[LBA_TEST] Error writing sector " + String(sector), 0xF800);
            // Restore sectors already written before exiting
            for (uint32_t r = 0; r < i; r++) {
                getSdFat().card()->writeSector(r * sectorStep, backupBuffer + (r * 512));
            }
            free(backupBuffer);
            return false;
        }
        if (i % 2 == 0) {
            logToScreen("Written signature at: " + String(i) + " GB");
            yield();
        }
    }
    
    // 3. Read back and verify signatures
    logToScreen("Verifying signatures (detecting spoof)...");
    bool counterfeit = false;
    for (uint32_t i = 0; i < numSectors; i++) {
        uint32_t sector = i * sectorStep;
        uint8_t readBlock[512];
        
        if (!getSdFat().card()->readSector(sector, readBlock)) {
            logToScreen("[LBA_TEST] Error reading sector " + String(sector), 0xF800);
            counterfeit = true;
            break;
        }
        
        uint8_t expectedBlock[512];
        memset(expectedBlock, 0, 512);
        snprintf((char*)expectedBlock, 64, "LBA_SIG_SEC_%u_IDX_%u", sector, i);
        for (size_t j = 64; j < 512; j++) {
            expectedBlock[j] = (uint8_t)((sector + j + i) & 0xFF);
        }
        
        if (memcmp(readBlock, expectedBlock, 512) != 0) {
            logToScreen("VERIFICATION FAIL at sector " + String(sector) + " (" + String(i) + " GB)!", 0xF800);
            counterfeit = true;
        } else {
            logToScreen("Verified: " + String(i) + " GB (" + String(i + 1) + "/" + String(numSectors) + " checked)", 0x07E0);
        }
        yield();
    }
    
    // 4. Restore original sector contents (CRITICAL for data safety!)
    logToScreen("Restoring original partition data...");
    bool restoreSuccess = true;
    for (uint32_t i = 0; i < numSectors; i++) {
        uint32_t sector = i * sectorStep;
        if (!getSdFat().card()->writeSector(sector, backupBuffer + (i * 512))) {
            logToScreen("[LBA_TEST] CRITICAL: Failed to restore " + String(sector), 0xF800);
            restoreSuccess = false;
        }
    }
    
    free(backupBuffer);
    
    if (counterfeit) {
        logToScreen("Result: FAIL! Wrap-around detected.", 0xF800);
        return false;
    }
    
    if (!restoreSuccess) {
        logToScreen("Result: Restore failed, signatures OK.", 0xFDA0);
        return false;
    }
    
    logToScreen("Result: PASS! No wrap-around detected.", 0x07E0);
    return true;
}

/**
 * @brief Erases card and formats to FAT32 (<=32GB) or exFAT (>32GB) automatically.
 * @param pr The Print stream to output progress formatting symbols.
 * @return true on success, false on error.
 */
inline bool formatCard(print_t* pr = &Serial) {
    if (!getSdFat().card()) {
        logToScreen("[FORMAT] Error: SD card not initialized.", 0xF800);
        return false;
    }
    
    uint32_t cardSectorCount = getSdFat().card()->sectorCount();
    if (cardSectorCount == 0) {
        logToScreen("[FORMAT] Error: Failed to read sector count.", 0xF800);
        return false;
    }
    
    Serial.printf("[FORMAT] Card size: %.2f GB (%u sectors)\n", 
                  (double)cardSectorCount * 512.0 / 1e9, cardSectorCount);
                  
    // 1. Perform low-level erase
    logToScreen("Erasing card (low-level block erase)...");
    uint32_t firstBlock = 0;
    uint32_t lastBlock;
    const uint32_t ERASE_SIZE = 262144L; // 128 MB chunks
    
    while (firstBlock < cardSectorCount) {
        lastBlock = firstBlock + ERASE_SIZE - 1;
        if (lastBlock >= cardSectorCount) {
            lastBlock = cardSectorCount - 1;
        }
        
        if (!getSdFat().card()->erase(firstBlock, lastBlock)) {
            logToScreen("[FORMAT] Warning: Erase not supported here.", 0xFDA0);
            break;
        }
        firstBlock += ERASE_SIZE;
        yield();
    }
    
    // 2. Format volume
    logToScreen("Rebuilding filesystems. Formatting...");
    uint8_t sectorBuffer[512];
    memset(sectorBuffer, 0, 512);
    
    FsFormatter formatter;
    bool formatSuccess = formatter.format(getSdFat().card(), sectorBuffer, pr);
    
    if (!formatSuccess) {
        logToScreen("[FORMAT] Error: Formatting failed!", 0xF800);
        return false;
    }
    
    logToScreen("Filesystem write completed successfully.", 0x07E0);
    
    // 3. Remount SdFat filesystem
    logToScreen("Remounting filesystem...");
    if (!initSPIAndSD(20)) {
        logToScreen("[FORMAT] Warning: Remount failed!", 0xFDA0);
        return false;
    }
    
    logToScreen("Remount complete! SD card ready.", 0x07E0);
    return true;
}

/**
 * @brief Reads CID and CSD registers and parses manufacturer, OEM, serial, and date info.
 * @return true on success, false on error.
 */
inline bool decodeRegisters() {
    if (!getSdFat().card()) {
        Serial.println("[DECODE] Error: SD card not initialized.");
        return false;
    }
    
    cid_t cid;
    csd_t csd;
    
    if (!getSdFat().card()->readCID(&cid)) {
        Serial.println("[DECODE] Error: Failed to read CID register.");
        return false;
    }
    
    if (!getSdFat().card()->readCSD(&csd)) {
        Serial.println("[DECODE] Error: Failed to read CSD register.");
        return false;
    }
    
    Serial.println("================ SD CARD REGISTERS ================");
    Serial.printf("Manufacturer ID (MID): 0x%02X\n", cid.mid);
    Serial.printf("OEM ID (OID):          %.2s\n", cid.oid);
    Serial.printf("Product Name (PNM):    %.5s\n", cid.pnm);
    Serial.printf("Product Revision:      %d.%d\n", cid.prvN(), cid.prvM());
    Serial.printf("Product Serial (PSN):  0x%08X (%u)\n", cid.psn(), cid.psn());
    Serial.printf("Manufacturing Date:    %02d/%d\n", cid.mdtMonth(), cid.mdtYear());
    Serial.printf("CSD Capacity (sectors):%u\n", csd.capacity());
    Serial.printf("CSD Copy Flag:         %s\n", csd.copy() ? "Yes" : "No");
    Serial.printf("Permanent Write Prot:  %s\n", csd.permWriteProtect() ? "Yes" : "No");
    Serial.printf("Temporary Write Prot:  %s\n", csd.tempWriteProtect() ? "Yes" : "No");
    Serial.println("===================================================");
    return true;
}

/**
 * @brief Writes 2000 blocks of 512 bytes sequentially, tracking max latency.
 * @param filepath The path of the temporary file to write.
 * @return true on success, false on error.
 */
inline bool latencyTest(const char* filepath = "/latency_test.tmp") {
    if (!getSdFat().card()) {
        Serial.println("[LATENCY] Error: SD card not initialized.");
        return false;
    }
    
    if (getSdFat().exists(filepath)) {
        getSdFat().remove(filepath);
    }
    
    FsFile file = getSdFat().open(filepath, O_WRONLY | O_CREAT | O_TRUNC);
    if (!file) {
        Serial.println("[LATENCY] Error: Failed to open file for latency test.");
        return false;
    }
    
    const size_t blockSize = 512;
    uint8_t* buf = (uint8_t*)malloc(blockSize);
    if (!buf) {
        Serial.println("[LATENCY] Error: Failed to allocate memory for write block.");
        file.close();
        return false;
    }
    
    // Fill buffer with dummy data
    for (size_t i = 0; i < blockSize; i++) {
        buf[i] = (uint8_t)(i % 256);
    }
    
    const uint32_t numWrites = 2000;
    uint32_t maxLatency = 0;
    uint32_t minLatency = 0xFFFFFFFF;
    uint64_t totalLatency = 0;
    
    Serial.printf("[LATENCY] Writing %u blocks of 512 bytes sequentially...\n", numWrites);
    
    for (uint32_t i = 0; i < numWrites; i++) {
        uint32_t startTime = micros();
        if (file.write(buf, blockSize) != blockSize) {
            Serial.printf("[LATENCY] Error: Write failed at block %u\n", i);
            free(buf);
            file.close();
            getSdFat().remove(filepath);
            return false;
        }
        uint32_t latency = micros() - startTime;
        
        totalLatency += latency;
        if (latency > maxLatency) {
            maxLatency = latency;
        }
        if (latency < minLatency) {
            minLatency = latency;
        }
        
        if (i % 200 == 0) {
            yield();
        }
    }
    
    uint32_t syncStart = micros();
    file.sync();
    uint32_t syncLatency = micros() - syncStart;
    
    file.close();
    free(buf);
    
    getSdFat().remove(filepath);
    
    double avgLatency = (double)totalLatency / numWrites;
    
    Serial.println("================ LATENCY TEST RESULTS ================");
    Serial.printf("Total blocks written:   %u\n", numWrites);
    Serial.printf("Min write latency:      %u us\n", minLatency);
    Serial.printf("Max write latency:      %u us\n", maxLatency);
    Serial.printf("Avg write latency:      %.2f us\n", avgLatency);
    Serial.printf("File sync latency:      %u us\n", syncLatency);
    Serial.println("======================================================");
    return true;
}

/**
 * @brief Reads a specified 512-byte raw block and prints it in hex/ASCII representation.
 * @param blockNumber The block sector number to dump (defaults to 0).
 * @return true on success, false on error.
 */
inline bool hexDumpBlock(uint32_t blockNumber = 0) {
    if (!getSdFat().card()) {
        Serial.println("[HEX_DUMP] Error: SD card not initialized.");
        return false;
    }
    
    uint8_t buf[512];
    if (!getSdFat().card()->readSector(blockNumber, buf)) {
        Serial.printf("[HEX_DUMP] Error: Failed to read raw block %u\n", blockNumber);
        return false;
    }
    
    Serial.printf("================ HEX DUMP OF RAW BLOCK %u ================\n", blockNumber);
    for (uint32_t i = 0; i < 512; i += 16) {
        Serial.printf("0x%03X: ", i);
        
        for (uint32_t j = 0; j < 16; j++) {
            Serial.printf("%02X ", buf[i + j]);
            if (j == 7) {
                Serial.print(" ");
            }
        }
        
        Serial.print(" |");
        for (uint32_t j = 0; j < 16; j++) {
            uint8_t c = buf[i + j];
            if (c >= 32 && c <= 126) {
                Serial.write(c);
            } else {
                Serial.print(".");
            }
        }
        Serial.println("|");
    }
    Serial.println("==========================================================");
    return true;
}

#endif // SD_DIAGNOSTICS_H
