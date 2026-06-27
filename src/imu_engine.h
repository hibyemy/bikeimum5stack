#ifndef IMU_ENGINE_H
#define IMU_ENGINE_H

#include <M5Core2.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <TinyGPS++.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Forward declarations from menu UI to avoid circular header dependencies
bool isHomeButtonPressed();
void waitForTouchRelease();

class IMUEngine {
public:
    // A simple, robust 3D vector helper class
    struct Vector3 {
        float x, y, z;
        Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        
        float magnitude() const {
            return sqrtf(x*x + y*y + z*z);
        }
        
        Vector3 normalized() const {
            float mag = magnitude();
            if (mag > 0.0001f) {
                return Vector3(x / mag, y / mag, z / mag);
            }
            return Vector3(0.0f, 0.0f, 0.0f);
        }
        
        float dot(const Vector3& v) const {
            return x * v.x + y * v.y + z * v.z;
        }
        
        Vector3 cross(const Vector3& v) const {
            return Vector3(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            );
        }
        
        Vector3 operator-(const Vector3& v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }
        
        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }
        
        Vector3 operator*(float s) const {
            return Vector3(x * s, y * s, z * s);
        }
    };

    // Aligned vehicle frame coordinate axes (Unit vectors in sensor frame)
    inline static Vector3 u_axis = Vector3(0.0f, 1.0f, 0.0f);   // Up axis
    inline static Vector3 f_axis = Vector3(0.0f, 0.0f, -1.0f);  // Forward axis
    inline static Vector3 r_axis = Vector3(1.0f, 0.0f, 0.0f);   // Right axis
    
    // Calibration parameters
    inline static Vector3 g_calib = Vector3(0.0f, 1.0f, 0.0f);
    inline static Vector3 gyro_bias = Vector3(0.0f, 0.0f, 0.0f);
    inline static volatile float g_scale = 1.0f;
    inline static volatile bool isCalibrated = false;
    inline static volatile bool pause_background_task = false;
    
    // Live fusion states (volatile and thread-safe)
    inline static volatile float roll = 0.0f;          // lean angle in degrees (positive = right, negative = left)
    inline static volatile float pitch = 0.0f;         // pitch angle in degrees (positive = nose up, negative = nose down)
    inline static volatile float linear_accel = 0.0f;  // gravity-compensated longitudinal acceleration (G)
    
    // Raw telemetry variables
    inline static Vector3 raw_accel = Vector3(0.0f, 0.0f, 0.0f);
    inline static Vector3 raw_gyro = Vector3(0.0f, 0.0f, 0.0f);
    inline static Vector3 raw_mag = Vector3(0.0f, 0.0f, 0.0f);
    
    // Session peak tracking (volatile and thread-safe)
    inline static volatile float peak_left_lean = 0.0f;   // degrees (always positive value)
    inline static volatile float peak_right_lean = 0.0f;  // degrees (always positive value)
    inline static volatile float peak_accel = 0.0f;       // G (always positive value)
    inline static volatile float peak_decel = 0.0f;       // G (always positive value)

    // Advanced 9DOF & GPS Telemetry variables
    inline static volatile bool usingExternalIMU = false;
    inline static uint8_t bmi160_addr = 0x69;
    inline static volatile float yaw = 0.0f;
    inline static volatile float mag_heading = 0.0f;
    inline static volatile float gps_speed_kmh = 0.0f;
    inline static volatile float gps_heading = 0.0f;
    inline static volatile float gps_altitude = 0.0f;
    inline static volatile uint32_t gps_sats = 0;
    inline static volatile float peak_wheelie = 0.0f;
    inline static volatile float peak_dive = 0.0f;
    inline static volatile bool bmi160_detected = false;
    inline static volatile bool onboard_imu_detected = false;
    inline static volatile bool lsm303_detected = false;
    inline static volatile bool gps_detected = false;
    inline static char gps_raw_buffer[64] = {0};
    inline static volatile uint8_t gps_raw_idx = 0;

    inline static TinyGPSPlus gps;
    
    // FreeRTOS Task and Sync Handles
    inline static TaskHandle_t imuTaskHandle = NULL;
    inline static SemaphoreHandle_t imuMutex = NULL;

    // Thread-safe raw GPS buffer reader
    inline static void getGpsRawBuffer(char* dest) {
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            uint8_t idx = gps_raw_idx;
            for (int i = 0; i < 63; i++) {
                uint8_t read_idx = (idx + i) % 64;
                char c = gps_raw_buffer[read_idx];
                dest[i] = (c == '\0' || c == '\r' || c == '\n') ? ' ' : c;
            }
            dest[63] = '\0';
            xSemaphoreGive(imuMutex);
        }
    }

    // Thread-safe getters
    inline static float getRoll() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = roll;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPitch() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = pitch;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getLinearAccel() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = linear_accel;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakLeftLean() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_left_lean;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakRightLean() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_right_lean;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakAccel() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_accel;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakDecel() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_decel;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getIsCalibrated() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = isCalibrated;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getUsingExternalIMU() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = usingExternalIMU;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getYaw() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = yaw;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getMagHeading() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = mag_heading;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getGpsSpeedKmh() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = gps_speed_kmh;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getGpsHeading() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = gps_heading;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getGpsAltitude() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = gps_altitude;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static uint32_t getGpsSats() {
        uint32_t val = 0;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = gps_sats;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakWheelie() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_wheelie;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static float getPeakDive() {
        float val = 0.0f;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = peak_dive;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getBmi160Detected() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = bmi160_detected;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getOnboardImuDetected() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = onboard_imu_detected;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getLsm303Detected() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = lsm303_detected;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    inline static bool getGpsDetected() {
        bool val = false;
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            val = gps_detected;
            xSemaphoreGive(imuMutex);
        }
        return val;
    }

    // Direct I2C utility functions for external sensors on Wire
    inline static void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
        Wire1.beginTransmission(addr);
        Wire1.write(reg);
        Wire1.write(val);
        Wire1.endTransmission();
    }

    inline static uint8_t i2cRead(uint8_t addr, uint8_t reg) {
        Wire1.beginTransmission(addr);
        Wire1.write(reg);
        if (Wire1.endTransmission() != 0) { // Send STOP. Repeated start (false) locks up ESP32 I2C.
            return 0;
        }
        Wire1.requestFrom(addr, (uint8_t)1);
        if (Wire1.available()) {
            return Wire1.read();
        }
        return 0;
    }

    inline static bool readBMI160(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
        Wire1.beginTransmission(bmi160_addr);
        Wire1.write(0x0C);
        if (Wire1.endTransmission() != 0) {
            return false;
        }
        uint8_t bytesReceived = Wire1.requestFrom(bmi160_addr, (uint8_t)12);
        if (bytesReceived == 12) {
            uint8_t d[12];
            for (int i = 0; i < 12; i++) {
                d[i] = Wire1.read();
            }
            // Little-endian: LSB is first, MSB is second
            int16_t raw_gx = (int16_t)(d[0] | (d[1] << 8));
            int16_t raw_gy = (int16_t)(d[2] | (d[3] << 8));
            int16_t raw_gz = (int16_t)(d[4] | (d[5] << 8));
            int16_t raw_ax = (int16_t)(d[6] | (d[7] << 8));
            int16_t raw_ay = (int16_t)(d[8] | (d[9] << 8));
            int16_t raw_az = (int16_t)(d[10] | (d[11] << 8));
            
            // +/-2000 dps -> 16.384 LSB/dps
            // +/-4G -> 8192 LSB/G
            *gx = (float)raw_gx * (2000.0f / 32768.0f);
            *gy = (float)raw_gy * (2000.0f / 32768.0f);
            *gz = (float)raw_gz * (2000.0f / 32768.0f);
            
            *ax = (float)raw_ax * (4.0f / 32768.0f);
            *ay = (float)raw_ay * (4.0f / 32768.0f);
            *az = (float)raw_az * (4.0f / 32768.0f);
            return true;
        }
        return false;
    }

    inline static bool readMag(int16_t &mx, int16_t &my, int16_t &mz) {
        Wire1.beginTransmission(0x1E);
        Wire1.write(0x03);
        if (Wire1.endTransmission() != 0) {
            return false;
        }
        uint8_t bytesReceived = Wire1.requestFrom((uint8_t)0x1E, (uint8_t)6);
        if (bytesReceived == 6) {
            uint8_t d[6];
            for (int i = 0; i < 6; i++) {
                d[i] = Wire1.read();
            }
            // LSM303 is Big Endian (MSB first, LSB second)
            // OUT_X_H, OUT_X_L, OUT_Z_H, OUT_Z_L, OUT_Y_H, OUT_Y_L
            mx = (int16_t)((d[0] << 8) | d[1]);
            mz = (int16_t)((d[2] << 8) | d[3]);
            my = (int16_t)((d[4] << 8) | d[5]);
            return true;
        }
        return false;
    }

    /**
     * @brief Background task that runs the IMU sensor fusion.
     */
    static void imuTask(void* pvParameters) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms (100Hz)
        uint32_t iterationCount = 0;
        
        while (true) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            if (pause_background_task) {
                continue;
            }

            // Read GPS bytes in background task
            while (Serial2.available() > 0) {
                char c = Serial2.read();
                gps.encode(c);
                if (imuMutex != NULL && xSemaphoreTake(imuMutex, 0) == pdTRUE) {
                    gps_raw_buffer[gps_raw_idx] = c;
                    gps_raw_idx = (gps_raw_idx + 1) % 64;
                    xSemaphoreGive(imuMutex);
                }
            }
            if (gps.charsProcessed() > 0) {
                if (imuMutex != NULL && xSemaphoreTake(imuMutex, 0) == pdTRUE) {
                    gps_detected = true;
                    xSemaphoreGive(imuMutex);
                }
            }
            
            // Call update (runs at 100Hz)
            update(0.01f);
            
            // Print telemetry at 20Hz (every 50ms, which is 5 iterations)
            iterationCount++;
            if (iterationCount >= 5) {
                printTelemetryJSON();
                iterationCount = 0;
            }
        }
    }

    /**
     * @brief Initializes the IMU devices and GPS, spawning the background task.
     * @return true if initialized successfully, false otherwise.
     */
    inline static bool init() {
        // Guard: if already initialized (task running), skip re-init to prevent
        // corrupting the I2C bus while the background task is mid-transaction.
        if (imuTaskHandle != NULL) {
            Serial.println("[IMU] Already initialized, skipping re-init.");
            return (bmi160_detected || onboard_imu_detected);
        }

        // Wire is already initialized on M-BUS (SDA=21, SCL=22) by M5.begin() in main.cpp.
        // Re-initializing it here can corrupt the I2C peripheral state.
        Wire1.setClock(100000); // Set clock speed to 100kHz for stability with external sensors
        delay(50); // Allow sensors time to power up

        // Initialize Serial2 (GPS UART)
        Serial2.begin(9600, SERIAL_8N1, 13, 14);

        Serial.println("[IMU] Wire I2C bus clock set to 100kHz");

        // ---- BMI160 Initialization ----
        // Step 1: Soft-reset BMI160 at both possible addresses to clear stale state.
        // Write 0xB6 to CMD register (0x7E). NACKs harmlessly if sensor is absent.
        Wire1.beginTransmission(0x69);
        Wire1.write(0x7E);
        Wire1.write(0xB6);
        Wire1.endTransmission();
        Wire1.beginTransmission(0x68);
        Wire1.write(0x7E);
        Wire1.write(0xB6);
        Wire1.endTransmission();
        delay(100); // BMI160 needs ~80ms after soft reset to become responsive

        // Step 2: Probe BMI160 CHIP_ID with retries (register 0x00 should return 0xD1)
        uint8_t addr_to_try = 0x69;
        uint8_t chip_id = 0;
        for (int attempt = 0; attempt < 3 && chip_id != 0xD1; attempt++) {
            chip_id = i2cRead(0x69, 0x00);
            if (chip_id == 0xD1) {
                addr_to_try = 0x69;
                break;
            }
            chip_id = i2cRead(0x68, 0x00);
            if (chip_id == 0xD1) {
                addr_to_try = 0x68;
                break;
            }
            Serial.printf("[IMU] BMI160 probe attempt %d: CHIP_ID=0x%02X (expected 0xD1)\n", attempt + 1, chip_id);
            delay(30);
        }

        bool temp_bmi = false;
        bool temp_onboard = false;
        if (chip_id == 0xD1) {
            bmi160_addr = addr_to_try;
            usingExternalIMU = true;
            temp_bmi = true;

            // CMD register 0x7E: write 0x11 to start accelerometer (normal power mode)
            i2cWrite(bmi160_addr, 0x7E, 0x11);
            delay(10);
            // CMD register 0x7E: write 0x15 to start gyroscope (normal power mode)
            i2cWrite(bmi160_addr, 0x7E, 0x15);
            delay(100);

            // ACC_RANGE register 0x41: write 0x05 for +/-4G
            i2cWrite(bmi160_addr, 0x41, 0x05);
            // GYR_RANGE register 0x43: write 0x00 for +/-2000 dps
            i2cWrite(bmi160_addr, 0x43, 0x00);

            Serial.printf("[IMU] External BMI160 successfully initialized at address 0x%02X.\n", bmi160_addr);
        } else {
            usingExternalIMU = false;
            Serial.printf("[IMU] BMI160 not found (last CHIP_ID read: 0x%02X). Trying onboard MPU6886...\n", chip_id);
            int res = M5.IMU.Init();
            if (res != 0) {
                Serial.println("[IMU] ERROR: Onboard MPU6886 initialization also failed!");
            } else {
                temp_onboard = true;
                Serial.println("[IMU] Onboard MPU6886 successfully initialized.");
            }
        }

        // ---- LSM303DLHC Magnetometer Initialization ----
        // Read identification register A (0x0A) with retries, should return 'H' (0x48)
        uint8_t mag_id = 0;
        for (int attempt = 0; attempt < 3 && mag_id != 0x48; attempt++) {
            mag_id = i2cRead(0x1E, 0x0A);
            if (mag_id == 0x48) break;
            Serial.printf("[IMU] LSM303 probe attempt %d: IRA_REG_M=0x%02X (expected 0x48)\n", attempt + 1, mag_id);
            delay(20);
        }
        bool temp_mag = (mag_id == 0x48);
        if (temp_mag) {
            // Write 0x14 to register 0x00 (CRA_REG_M): 75Hz rate
            i2cWrite(0x1E, 0x00, 0x14);
            // Write 0x20 to register 0x01 (CRB_REG_M): +/-1.3 Gauss range
            i2cWrite(0x1E, 0x01, 0x20);
            // Write 0x00 to register 0x02 (MR_REG_M): continuous conversion mode
            i2cWrite(0x1E, 0x02, 0x00);
            Serial.println("[IMU] LSM303DLHC Magnetometer initialized at address 0x1E.");
        } else {
            Serial.printf("[IMU] Warning: LSM303DLHC not found at 0x1E (last IRA read: 0x%02X).\n", mag_id);
        }

        // Create the mutex for shared variables
        if (imuMutex == NULL) {
            imuMutex = xSemaphoreCreateMutex();
        }

        // Store detected states under mutex
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            bmi160_detected = temp_bmi;
            onboard_imu_detected = temp_onboard;
            lsm303_detected = temp_mag;
            xSemaphoreGive(imuMutex);
        }

        // Spawn the FreeRTOS task if not already running (always spawn so GPS and terminal work)
        if (imuTaskHandle == NULL) {
            BaseType_t xReturned = xTaskCreatePinnedToCore(
                imuTask,
                "IMUTask",
                4096, // stack depth
                NULL,
                5, // priority
                &imuTaskHandle,
                0 // Core 0
            );
            if (xReturned != pdPASS) {
                Serial.println("[IMU] ERROR: Failed to create IMU FreeRTOS task!");
                return false;
            }
            Serial.println("[IMU] IMU FreeRTOS task spawned on Core 0.");
        }
        return (temp_bmi || temp_onboard);
    }
    
    /**
     * @brief Performs Zero Calibration to establish gyro bias offsets
     *        and record the static gravity vector defining the upward direction.
     *        Also performs stability check via variance threshold.
     */
    inline static bool calibrate() {
        Serial.println("[IMU] Calibrating. Please keep the device stable and level...");
        
        if (!bmi160_detected && !onboard_imu_detected) {
            Serial.println("[IMU] Calibration aborted: No IMU detected!");
            return false;
        }
        
        // Suspend task to prevent conflicts and I2C contention during raw sampling
        if (imuTaskHandle != NULL) {
            pause_background_task = true; delay(20);
        }
        
        Vector3 accelSum(0.0f, 0.0f, 0.0f);
        Vector3 gyroSum(0.0f, 0.0f, 0.0f);
        const int samples = 200;
        
        // Dynamically allocate to avoid stack overflow on Arduino loop task
        float* ax_samples = new float[samples];
        float* ay_samples = new float[samples];
        float* az_samples = new float[samples];
        float* gx_samples = new float[samples];
        float* gy_samples = new float[samples];
        float* gz_samples = new float[samples];
        
        for (int i = 0; i < samples; i++) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            float gx = 0.0f, gy = 0.0f, gz = 0.0f;
            if (bmi160_detected) {
                readBMI160(&ax, &ay, &az, &gx, &gy, &gz);
            } else if (onboard_imu_detected) {
                M5.IMU.getAccelData(&ax, &ay, &az);
                M5.IMU.getGyroData(&gx, &gy, &gz);
            }
            
            ax_samples[i] = ax;
            ay_samples[i] = ay;
            az_samples[i] = az;
            gx_samples[i] = gx;
            gy_samples[i] = gy;
            gz_samples[i] = gz;
            
            accelSum.x += ax;
            accelSum.y += ay;
            accelSum.z += az;
            
            gyroSum.x += gx;
            gyroSum.y += gy;
            gyroSum.z += gz;
            
            delay(10);
        }
        
        // Calculate averages
        float ax_mean = accelSum.x / samples;
        float ay_mean = accelSum.y / samples;
        float az_mean = accelSum.z / samples;
        float gx_mean = gyroSum.x / samples;
        float gy_mean = gyroSum.y / samples;
        float gz_mean = gyroSum.z / samples;
        
        // Calculate variance (sum of squared deviations / samples)
        float ax_var = 0.0f, ay_var = 0.0f, az_var = 0.0f;
        float gx_var = 0.0f, gy_var = 0.0f, gz_var = 0.0f;
        for (int i = 0; i < samples; i++) {
            float d_ax = ax_samples[i] - ax_mean;
            float d_ay = ay_samples[i] - ay_mean;
            float d_az = az_samples[i] - az_mean;
            float d_gx = gx_samples[i] - gx_mean;
            float d_gy = gy_samples[i] - gy_mean;
            float d_gz = gz_samples[i] - gz_mean;
            
            ax_var += d_ax * d_ax;
            ay_var += d_ay * d_ay;
            az_var += d_az * d_az;
            gx_var += d_gx * d_gx;
            gy_var += d_gy * d_gy;
            gz_var += d_gz * d_gz;
        }
        ax_var /= samples;
        ay_var /= samples;
        az_var /= samples;
        gx_var /= samples;
        gy_var /= samples;
        gz_var /= samples;
        
        // Cleanup dynamic memory
        delete[] ax_samples;
        delete[] ay_samples;
        delete[] az_samples;
        delete[] gx_samples;
        delete[] gy_samples;
        delete[] gz_samples;
        
        // Stability thresholds
        const float GYRO_VAR_THRESHOLD = 0.1f;    // deg^2/sec^2
        const float ACCEL_VAR_THRESHOLD = 0.005f; // G^2
        
        if (gx_var > GYRO_VAR_THRESHOLD || gy_var > GYRO_VAR_THRESHOLD || gz_var > GYRO_VAR_THRESHOLD ||
            ax_var > ACCEL_VAR_THRESHOLD || ay_var > ACCEL_VAR_THRESHOLD || az_var > ACCEL_VAR_THRESHOLD) {
            Serial.println("[IMU] Calibration aborted: Device was not stable!");
            Serial.printf("[IMU] Gyro Variances: [X:%.6f, Y:%.6f, Z:%.6f] (Thresh: %.3f)\n", gx_var, gy_var, gz_var, GYRO_VAR_THRESHOLD);
            Serial.printf("[IMU] Accel Variances: [X:%.6f, Y:%.6f, Z:%.6f] (Thresh: %.5f)\n", ax_var, ay_var, az_var, ACCEL_VAR_THRESHOLD);
            
            if (imuTaskHandle != NULL) {
                pause_background_task = false;
            }
            return false;
        }
        
        // Mutex lock for updating calibration parameters and resetting state
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            g_calib = Vector3(ax_mean, ay_mean, az_mean);
            gyro_bias = Vector3(gx_mean, gy_mean, gz_mean);
            
            float mag = g_calib.magnitude();
            if (mag > 0.001f) {
                g_scale = 1.0f / mag;
            } else {
                g_scale = 1.0f;
            }
            
            // 1. Establish vertical "Up" axis
            u_axis = g_calib.normalized();
            
            // 2. Project nominal forward vector [0, 0, -1] onto plane perpendicular to u_axis
            Vector3 f_nom(0.0f, 0.0f, -1.0f);
            Vector3 f_proj = f_nom - u_axis * f_nom.dot(u_axis);
            f_axis = f_proj.normalized();
            
            // 3. Complete orthogonal basis: Right axis = Forward x Up
            r_axis = f_axis.cross(u_axis).normalized();
            
            isCalibrated = true;
            
            // Reset state & peaks upon calibration
            roll = 0.0f;
            pitch = 0.0f;
            linear_accel = 0.0f;
            
            peak_left_lean = 0.0f;
            peak_right_lean = 0.0f;
            peak_accel = 0.0f;
            peak_decel = 0.0f;
            peak_wheelie = 0.0f;
            peak_dive = 0.0f;
            
            xSemaphoreGive(imuMutex);
        }
        
        Serial.println("[IMU] Zero calibration completed successfully.");
        Serial.printf("[IMU] Calib Gravity Vector: [%.3f, %.3f, %.3f] (Mag: %.3fG)\n", g_calib.x, g_calib.y, g_calib.z, g_calib.magnitude());
        Serial.printf("[IMU] Calib Gyro Biases   : [%.3f, %.3f, %.3f] dps\n", gyro_bias.x, gyro_bias.y, gyro_bias.z);
        Serial.printf("[IMU] Accel Scale Factor  : %.6f\n", g_scale);
        
        if (imuTaskHandle != NULL) {
            pause_background_task = false;
        }
        return true;
    }
    
    /**
     * @brief Resets all tracked session peak values.
     */
    inline static void resetPeaks() {
        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            peak_left_lean = 0.0f;
            peak_right_lean = 0.0f;
            peak_accel = 0.0f;
            peak_decel = 0.0f;
            peak_wheelie = 0.0f;
            peak_dive = 0.0f;
            xSemaphoreGive(imuMutex);
            Serial.println("[IMU] Peaks reset.");
        }
    }
    
    /**
     * @brief Updates sensor fusion filters, gravity subtraction, and peak tracking.
     * @param dt Elapsed time since last update (in seconds).
     */
    inline static void update(float dt) {
        // If background task is running, only allow background task to execute update logic.
        if (imuTaskHandle != NULL && xTaskGetCurrentTaskHandle() != imuTaskHandle) {
            return;
        }

        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            float gx = 0.0f, gy = 0.0f, gz = 0.0f;
            
            if (bmi160_detected) {
                readBMI160(&ax, &ay, &az, &gx, &gy, &gz);
            } else if (onboard_imu_detected) {
                M5.IMU.getAccelData(&ax, &ay, &az);
                M5.IMU.getGyroData(&gx, &gy, &gz);
            }
            
            // Apply g_scale to raw accelerometer inputs
            float ax_scaled = ax * g_scale;
            float ay_scaled = ay * g_scale;
            float az_scaled = az * g_scale;
            
            raw_accel = Vector3(ax_scaled, ay_scaled, az_scaled);
            raw_gyro = Vector3(gx, gy, gz);
            
            // Correct gyro measurements using calibration biases
            Vector3 gyro_corr = raw_gyro - gyro_bias;

            // Update GPS values if valid
            if (gps.speed.isValid()) {
                gps_speed_kmh = gps.speed.kmph();
            }
            if (gps.course.isValid()) {
                gps_heading = gps.course.deg();
            }
            if (gps.altitude.isValid()) {
                gps_altitude = gps.altitude.meters();
            }
            if (gps.satellites.isValid()) {
                gps_sats = gps.satellites.value();
            }

            // Read LSM303DLHC Magnetometer
            int16_t raw_mx = 0, raw_my = 0, raw_mz = 0;
            float mx = 0.0f, my = 0.0f, mz = 0.0f;
            if (lsm303_detected && readMag(raw_mx, raw_my, raw_mz)) {
                mx = (float)raw_mx;
                my = (float)raw_my;
                mz = (float)raw_mz;
                raw_mag = Vector3(mx, my, mz);
            }

            float gyro_yaw_dps = 0.0f;
            
            if (!isCalibrated) {
                // Uncalibrated Fallback (assumes standard portrait orientation)
                // In a right-handed system, if X=Left, Y=Up, Z=Forward:
                // Positive Roll (Lean Right) = +g.z
                float omega_roll = gz;
                float omega_pitch = -gx;
                gyro_yaw_dps = -gy;
                float omega_yaw = gyro_yaw_dps * (M_PI / 180.0f);

                // Centripetal compensation
                float ax_corr_fallback = -ax_scaled;
                if (gps.speed.isValid() && gps_speed_kmh > 5.0f) {
                    float v = gps_speed_kmh / 3.6f;
                    float a_centripetal = (v * omega_yaw) / 9.80665f;
                    ax_corr_fallback += a_centripetal;
                }

                float roll_acc = atan2f(ax_corr_fallback, ay_scaled) * RAD_TO_DEG;
                float pitch_acc = atan2f(-az_scaled, ay_scaled) * RAD_TO_DEG;
                
                // Standard Complementary Filter update
                const float alpha = 0.995f;
                roll = alpha * (roll + omega_roll * dt) + (1.0f - alpha) * roll_acc;
                pitch = alpha * (pitch + omega_pitch * dt) + (1.0f - alpha) * pitch_acc;
                
                float raw_accel_z_compensated = az_scaled - (-sinf(pitch * DEG_TO_RAD));
                // Low-pass filter (EMA) with 0.85 coefficient
                linear_accel = 0.85f * linear_accel + 0.15f * raw_accel_z_compensated;
            } else {
                // 1. Transform raw vectors into the aligned vehicle coordinate frame
                Vector3 a_veh(
                    raw_accel.dot(r_axis),
                    raw_accel.dot(u_axis),
                    raw_accel.dot(f_axis)
                );
                
                Vector3 g_veh(
                    gyro_corr.dot(r_axis),
                    gyro_corr.dot(u_axis),
                    gyro_corr.dot(f_axis)
                );
                
                // 2. Extract rotation rates for Roll (around Z), Pitch (around X), Yaw (around Y)
                // Invert the physical axes to match standard Right-Handed aeronautical conventions
                // Positive Roll = Right Lean, Positive Pitch = Wheelie, Positive Yaw = Right Turn
                float omega_roll = g_veh.z;
                float omega_pitch = -g_veh.x;
                gyro_yaw_dps = -g_veh.y;
                float omega_yaw = gyro_yaw_dps * (M_PI / 180.0f);

                // 3. Centripetal & Roll Acceleration Compensation
                // Invert X so that static Right Lean (gravity pulls right) yields positive a_veh_x_corrected
                float a_veh_x_corrected = -a_veh.x;
                if (gps.speed.isValid() && gps_speed_kmh > 5.0f) {
                    float v = gps_speed_kmh / 3.6f;
                    // Centripetal force pushes bike to the outside of the turn.
                    // A right turn (positive omega_yaw) pushes left. Accelerometer measures force right (-X).
                    // We add the calculated centripetal Gs to correct the vector back to true down.
                    float a_centripetal = (v * omega_yaw) / 9.80665f;
                    a_veh_x_corrected += a_centripetal;
                }
                
                // 4. Compute acceleration-based angles relative to vehicle frame
                float roll_acc = atan2f(a_veh_x_corrected, a_veh.y) * RAD_TO_DEG;
                // Wheelie (positive pitch) -> gravity pulls BACK (-Z). So we use -a_veh.z
                float pitch_acc = atan2f(-a_veh.z, a_veh.y) * RAD_TO_DEG;
                
                // 5. Complementary Filter
                const float alpha = 0.995f;
                roll = alpha * (roll + omega_roll * dt) + (1.0f - alpha) * roll_acc;
                pitch = alpha * (pitch + omega_pitch * dt) + (1.0f - alpha) * pitch_acc;
                
                // 6. Gravity compensation and EMA low-pass filter
                // During a wheelie, gravity's contribution to Z is -sin(pitch). Subtract it.
                float raw_accel_z_compensated = a_veh.z - (-sinf(pitch * DEG_TO_RAD));
                linear_accel = 0.85f * linear_accel + 0.15f * raw_accel_z_compensated;
            }

            // Yaw calculations
            float omega_yaw = gyro_yaw_dps * (M_PI / 180.0f);
            
            // Tilt compensation
            float theta = pitch * DEG_TO_RAD;
            float phi = roll * DEG_TO_RAD;
            float X_h = mx * cosf(theta) + my * sinf(phi) * sinf(theta) + mz * cosf(phi) * sinf(theta);
            float Y_h = my * cosf(phi) - mz * sinf(phi);
            mag_heading = atan2f(-Y_h, X_h) * (180.0f / M_PI);
            if (mag_heading < 0.0f) {
                mag_heading += 360.0f;
            }

            // Compass and Yaw Fusion
            if (lsm303_detected) {
                // Calculate shortest angular path to prevent 360-degree wrap-around fighting
                float heading_diff = mag_heading - yaw;
                while (heading_diff > 180.0f) heading_diff -= 360.0f;
                while (heading_diff < -180.0f) heading_diff += 360.0f;
                
                // Complementary filter: 99.8% Gyroscope, 0.2% Compass
                yaw = yaw + omega_yaw * dt + 0.002f * heading_diff;
            } else {
                // If unplugged, purely use the high-speed gyroscope (relative yaw)
                yaw = yaw + omega_yaw * dt;
            }
            
            // Normalize yaw to 0-360
            while (yaw >= 360.0f) yaw -= 360.0f;
            while (yaw < 0.0f) yaw += 360.0f;
            
            // 7. Update session peaks
            if (roll < 0.0f) {
                float left_lean = -roll;
                if (left_lean > peak_left_lean) {
                    peak_left_lean = left_lean;
                }
            } else {
                float right_lean = roll;
                if (right_lean > peak_right_lean) {
                    peak_right_lean = right_lean;
                }
            }
            
            if (linear_accel > 0.0f) {
                if (linear_accel > peak_accel) {
                    peak_accel = linear_accel;
                }
            } else {
                float decel = -linear_accel;
                if (decel > peak_decel) {
                    peak_decel = decel;
                }
            }

            // Wheelie & Dive Detection
            if (pitch > 0.0f) {
                if (pitch > peak_wheelie) {
                    peak_wheelie = pitch;
                }
            } else {
                float dive = -pitch;
                if (dive > peak_dive) {
                    peak_dive = dive;
                }
            }
            
            xSemaphoreGive(imuMutex);
        }
    }
    
    /**
     * @brief Outputs JSON-formatted telemetry via Serial at 20Hz.
     */
    inline static void printTelemetryJSON() {
        // If background task is running, only allow background task to print telemetry.
        if (imuTaskHandle != NULL && xTaskGetCurrentTaskHandle() != imuTaskHandle) {
            return;
        }

        // Lock mutex briefly to take a consistent snapshot
        float r = 0.0f, p = 0.0f, la = 0.0f;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float pl = 0.0f, pr = 0.0f, pa = 0.0f, pd = 0.0f;
        bool cal = false;

        bool extIMU = false;
        float y = 0.0f, mh = 0.0f;
        float g_speed = 0.0f, g_head = 0.0f, g_alt = 0.0f;
        uint32_t g_sats = 0;
        float p_wheelie = 0.0f, p_dive = 0.0f;

        if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            r = roll;
            p = pitch;
            la = linear_accel;
            ax = raw_accel.x;
            ay = raw_accel.y;
            az = raw_accel.z;
            pl = peak_left_lean;
            pr = peak_right_lean;
            pa = peak_accel;
            pd = peak_decel;
            cal = isCalibrated;

            extIMU = usingExternalIMU;
            y = yaw;
            mh = mag_heading;
            g_speed = gps_speed_kmh;
            g_head = gps_heading;
            g_alt = gps_altitude;
            g_sats = gps_sats;
            p_wheelie = peak_wheelie;
            p_dive = peak_dive;
            xSemaphoreGive(imuMutex);
        }

        Serial.printf("{\"timestamp\":%u,\"roll\":%.2f,\"pitch\":%.2f,\"linear_accel\":%.3f,\"raw_ax\":%.3f,\"raw_ay\":%.3f,\"raw_az\":%.3f,\"peak_left_lean\":%.2f,\"peak_right_lean\":%.2f,\"peak_accel\":%.3f,\"peak_decel\":%.3f,\"calibrated\":%s,\"usingExternalIMU\":%s,\"yaw\":%.2f,\"mag_heading\":%.2f,\"gps_speed_kmh\":%.2f,\"gps_heading\":%.2f,\"gps_altitude\":%.2f,\"gps_sats\":%u,\"peak_wheelie\":%.2f,\"peak_dive\":%.2f}\n",
                      millis(), r, p, la, ax, ay, az, pl, pr, pa, pd, cal ? "true" : "false", extIMU ? "true" : "false", y, mh, g_speed, g_head, g_alt, g_sats, p_wheelie, p_dive);
    }
    
    /**
     * @brief Visual telemetry monitor and dashboard UI app loop.
     */
    inline static void runIMUMonitor() {
        // Init screen background
        M5.Lcd.fillScreen(0x0842); // Sleek charcoal
        
        // Top Header
        M5.Lcd.fillRect(0, 0, 320, 45, 0x10A2); // Sapphire header
        M5.Lcd.drawFastHLine(0, 45, 320, 0x07FF); // Cyan divider line
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.drawString("IMU VEHICLE RADAR", 160, 22, 4);
        
        // Ensure sensor is up
        init();
        
        // Draw UI structure panels
        M5.Lcd.drawRect(10, 52, 145, 120, 0x07FF);   // Cyan panel frame for Angles
        M5.Lcd.drawRect(165, 52, 145, 120, 0xFDA0);  // Orange panel frame for G-forces
        
        // Calibration button
        M5.Lcd.fillRoundRect(20, 180, 120, 35, 6, 0x10A2);
        M5.Lcd.drawRoundRect(20, 180, 120, 35, 6, 0x07FF);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.drawString("Calibrate", 80, 197, 2);
        
        // Reset Peaks button
        M5.Lcd.fillRoundRect(180, 180, 120, 35, 6, 0x10A2);
        M5.Lcd.drawRoundRect(180, 180, 120, 35, 6, 0xFDA0);
        M5.Lcd.drawString("Reset Peaks", 240, 197, 2);
        
        // Navigation bottom bezel hint
        M5.Lcd.setTextColor(0x7BEF);
        M5.Lcd.drawString("Press Center Button [B] to return", 160, 230, 2);
        
        uint32_t lastIMUUpdate = micros();
        uint32_t lastTelemetryTime = 0;
        uint32_t lastUIUpdate = 0;
        
        waitForTouchRelease();
        
        while (true) {
            M5.update();
            
            // Handle Home / Exit button
            if (isHomeButtonPressed()) {
                break;
            }
            
            // Loop timing calculation
            uint32_t nowMicros = micros();
            float dt = (nowMicros - lastIMUUpdate) / 1000000.0f;
            if (dt <= 0.0f || dt > 0.5f) dt = 0.02f; // Fallback
            lastIMUUpdate = nowMicros;
            
            update(dt);
            
            uint32_t nowMillis = millis();
            
            // Output serial telemetry at 20Hz
            if (nowMillis - lastTelemetryTime >= 50) {
                printTelemetryJSON();
                lastTelemetryTime = nowMillis;
            }
            
            // Redraw UI texts at 10Hz
            if (nowMillis - lastUIUpdate >= 100) {
                lastUIUpdate = nowMillis;
                
                // Clear interior panel bodies
                M5.Lcd.fillRect(11, 53, 143, 118, 0x0842);
                M5.Lcd.fillRect(166, 53, 143, 118, 0x0842);
                
                M5.Lcd.setTextDatum(TL_DATUM);
                
                // Left Panel: Attitude Angle Details
                M5.Lcd.setTextColor(0x07FF);
                M5.Lcd.drawString("ATTITUDE", 20, 58, 2);
                M5.Lcd.setTextColor(TFT_WHITE);
                
                float current_roll = 0.0f;
                float current_pitch = 0.0f;
                float current_linear_accel = 0.0f;
                float current_raw_ax = 0.0f;
                float current_raw_ay = 0.0f;
                float current_raw_az = 0.0f;
                float current_peak_left_lean = 0.0f;
                float current_peak_right_lean = 0.0f;
                float current_peak_accel = 0.0f;
                float current_peak_decel = 0.0f;

                if (imuMutex != NULL && xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
                    current_roll = roll;
                    current_pitch = pitch;
                    current_linear_accel = linear_accel;
                    current_raw_ax = raw_accel.x;
                    current_raw_ay = raw_accel.y;
                    current_raw_az = raw_accel.z;
                    current_peak_left_lean = peak_left_lean;
                    current_peak_right_lean = peak_right_lean;
                    current_peak_accel = peak_accel;
                    current_peak_decel = peak_decel;
                    xSemaphoreGive(imuMutex);
                }

                if (current_roll < 0.0f) {
                    M5.Lcd.drawString("Lean: L " + String(-current_roll, 1) + " deg", 20, 78, 2);
                } else {
                    M5.Lcd.drawString("Lean: R " + String(current_roll, 1) + " deg", 20, 78, 2);
                }
                
                M5.Lcd.drawString("Pitch: " + String(current_pitch, 1) + " deg", 20, 98, 2);
                M5.Lcd.drawString("Peak L: " + String(current_peak_left_lean, 1) + " deg", 20, 122, 2);
                M5.Lcd.drawString("Peak R: " + String(current_peak_right_lean, 1) + " deg", 20, 142, 2);
                
                // Right Panel: G-Forces Details
                M5.Lcd.setTextColor(0xFDA0);
                M5.Lcd.drawString("G-FORCES", 175, 58, 2);
                M5.Lcd.setTextColor(TFT_WHITE);
                
                M5.Lcd.drawString("Linear: " + String(current_linear_accel, 3) + " G", 175, 78, 2);
                M5.Lcd.drawString("Raw Z:  " + String(current_raw_az, 3) + " G", 175, 98, 2);
                M5.Lcd.drawString("Peak +: " + String(current_peak_accel, 3) + " G", 175, 122, 2);
                M5.Lcd.drawString("Peak -: " + String(current_peak_decel, 3) + " G", 175, 142, 2);
            }
            
            // Check touch coordinate presses
            TouchPoint_t pos = M5.Touch.getPressPoint();
            if (pos.x != -1) {
                // Calibrate Button: X range [20, 140], Y range [180, 215]
                if (pos.x >= 20 && pos.x <= 140 && pos.y >= 180 && pos.y <= 215) {
                    M5.Lcd.fillRoundRect(20, 180, 120, 35, 6, 0x1CE7);
                    M5.Lcd.drawRoundRect(20, 180, 120, 35, 6, TFT_WHITE);
                    M5.Lcd.setTextColor(TFT_WHITE);
                    M5.Lcd.setTextDatum(MC_DATUM);
                    M5.Lcd.drawString("Calibrating...", 80, 197, 2);
                    
                    bool success = calibrate();
                    
                    M5.Lcd.fillRoundRect(20, 180, 120, 35, 6, 0x10A2);
                    if (success) {
                        M5.Lcd.drawRoundRect(20, 180, 120, 35, 6, 0x07E0); // Green
                        M5.Lcd.setTextColor(TFT_WHITE);
                        M5.Lcd.drawString("Calibrated", 80, 197, 2);
                    } else {
                        M5.Lcd.drawRoundRect(20, 180, 120, 35, 6, TFT_RED); // Red
                        M5.Lcd.setTextColor(TFT_WHITE);
                        M5.Lcd.drawString("Calib Fail", 80, 197, 2);
                    }
                    
                    delay(1000);
                    waitForTouchRelease();
                    
                    // Reset UI update timer
                    lastIMUUpdate = micros();
                }
                // Reset Peaks Button: X range [180, 300], Y range [180, 215]
                else if (pos.x >= 180 && pos.x <= 300 && pos.y >= 180 && pos.y <= 215) {
                    M5.Lcd.fillRoundRect(180, 180, 120, 35, 6, 0x1CE7);
                    M5.Lcd.drawRoundRect(180, 180, 120, 35, 6, TFT_WHITE);
                    M5.Lcd.setTextColor(TFT_WHITE);
                    M5.Lcd.setTextDatum(MC_DATUM);
                    M5.Lcd.drawString("Resetting...", 240, 197, 2);
                    
                    resetPeaks();
                    
                    M5.Lcd.fillRoundRect(180, 180, 120, 35, 6, 0x10A2);
                    M5.Lcd.drawRoundRect(180, 180, 120, 35, 6, 0xFDA0);
                    M5.Lcd.drawString("Reset Done", 240, 197, 2);
                    
                    delay(400);
                    waitForTouchRelease();
                    
                    lastIMUUpdate = micros();
                }
            }
            
            delay(10); // Loop execution yield delay
        }
        
        // Clean screen area when returning
        M5.Lcd.fillScreen(0x0842);
        waitForTouchRelease();
    }
};

#endif // IMU_ENGINE_H
