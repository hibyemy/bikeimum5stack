#ifndef WIFI_RADAR_H
#define WIFI_RADAR_H

#include <Arduino.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <math.h>

// DSP structures for vital signs extraction
struct BiquadCoeffs {
    float b0, b1, b2;
    float a1, a2;
};

inline BiquadCoeffs designBandpass(float f0, float bw, float fs) {
    float omega = 2.0f * M_PI * f0 / fs;
    float Q = f0 / bw;
    float alpha = sin(omega) / (2.0f * Q);
    float a0 = 1.0f + alpha;
    
    BiquadCoeffs coeffs;
    coeffs.b0 = (sin(omega) / 2.0f) / a0;
    coeffs.b1 = 0.0f;
    coeffs.b2 = -coeffs.b0;
    coeffs.a1 = (-2.0f * cos(omega)) / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    return coeffs;
}

struct BiquadFilter {
    BiquadCoeffs coeffs;
    float x1, x2;
    float y1, y2;
    
    void init(float f0, float bw, float fs) {
        coeffs = designBandpass(f0, bw, fs);
        x1 = x2 = y1 = y2 = 0.0f;
    }
    
    float process(float x) {
        float y = coeffs.b0 * x + coeffs.b1 * x1 + coeffs.b2 * x2 
                - coeffs.a1 * y1 - coeffs.a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
};

struct EnvelopeTracker {
    float envelope;
    
    void reset() {
        envelope = 0.0f;
    }
    
    float process(float val) {
        float abs_val = abs(val);
        float alpha = (abs_val > envelope) ? 0.20f : 0.02f; // Fast attack, slow decay
        envelope = envelope * (1.0f - alpha) + abs_val * alpha;
        return envelope;
    }
};

struct RateEstimator {
    float prev_val;
    uint32_t last_crossing_time;
    float estimated_bpm;
    float min_bpm;
    float max_bpm;
    
    void init(float minBpm, float maxBpm) {
        min_bpm = minBpm;
        max_bpm = maxBpm;
        prev_val = 0.0f;
        last_crossing_time = 0;
        estimated_bpm = 0.0f;
    }
    
    void reset() {
        prev_val = 0.0f;
        last_crossing_time = 0;
        estimated_bpm = 0.0f;
    }
    
    float process(float val, float envelope, float threshold, uint32_t now) {
        if (prev_val < 0.0f && val >= 0.0f) { // positive-going zero crossing
            if (envelope > threshold) {
                if (last_crossing_time != 0) {
                    uint32_t period_ms = now - last_crossing_time;
                    if (period_ms > 0) {
                        float instant_bpm = 60000.0f / period_ms;
                        if (instant_bpm >= min_bpm && instant_bpm <= max_bpm) {
                            if (estimated_bpm == 0.0f) {
                                estimated_bpm = instant_bpm;
                            } else {
                                estimated_bpm = estimated_bpm * 0.85f + instant_bpm * 0.15f;
                            }
                        }
                    }
                }
                last_crossing_time = now;
            }
        }
        
        // Squelch / decay BPM if signal is too weak
        if (envelope <= threshold) {
            estimated_bpm = estimated_bpm * 0.99f;
            if (estimated_bpm < 1.0f) {
                estimated_bpm = 0.0f;
                last_crossing_time = 0;
            }
        }
        
        prev_val = val;
        return estimated_bpm;
    }
};

// Configuration Constants
#ifndef ROLLING_WINDOW_SIZE
#define ROLLING_WINDOW_SIZE 20
#endif

#ifndef NUM_SUBCARRIERS
#define NUM_SUBCARRIERS 64
#endif

// Global State
// Use static variables to allow header-only definition without duplicate symbol link errors.
static volatile float latest_motion_score = 0.0f;
static float amplitude_history[NUM_SUBCARRIERS][ROLLING_WINDOW_SIZE] = {0};
static int history_index = 0;
static int history_count = 0;

// Exposed DSP outputs
static volatile float latest_breathing_val = 0.0f;
static volatile float latest_heart_val = 0.0f;
static volatile float latest_breathing_bpm = 0.0f;
static volatile float latest_heart_bpm = 0.0f;
static volatile float breathing_envelope = 0.0f;
static volatile float heart_envelope = 0.0f;

// Filter and estimator instances
static BiquadFilter breathingFilter;
static BiquadFilter heartRateFilter;
static EnvelopeTracker breathingEnv;
static EnvelopeTracker heartRateEnv;
static RateEstimator breathingBpmEst;
static RateEstimator heartRateBpmEst;

// CSI Callback Function
inline void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    if (info == NULL || info->buf == NULL) {
        return;
    }

    // Determine how many subcarriers we got, up to our max buffer size
    int num_subcarriers = info->len / 2;
    if (num_subcarriers > NUM_SUBCARRIERS) {
        num_subcarriers = NUM_SUBCARRIERS;
    }

    if (num_subcarriers <= 0) {
        return;
    }

    // Extract real and imaginary components, calculate amplitude for each subcarrier
    for (int i = 0; i < num_subcarriers; i++) {
        // info->buf contains real and imaginary signed 8-bit integers interleaved.
        // On ESP32, each subcarrier contains an imaginary and a real part (signed 8-bit integers):
        // imag = info->buf[2 * i]
        // real = info->buf[2 * i + 1]
        int8_t imag = info->buf[2 * i];
        int8_t real = info->buf[2 * i + 1];
        
        float amp = sqrt((float)(real * real + imag * imag));
        amplitude_history[i][history_index] = amp;
    }

    // Advance rolling window index
    history_index = (history_index + 1) % ROLLING_WINDOW_SIZE;
    if (history_count < ROLLING_WINDOW_SIZE) {
        history_count++;
    }

    // Compute variance of amplitude for each subcarrier over the rolling window
    float total_variance = 0.0f;
    for (int i = 0; i < num_subcarriers; i++) {
        float variance = 0.0f;
        if (history_count >= 2) {
            // Mean calculation
            float sum = 0.0f;
            for (int j = 0; j < history_count; j++) {
                sum += amplitude_history[i][j];
            }
            float mean = sum / history_count;

            // Variance calculation (two-pass method for numerical stability)
            float variance_sum = 0.0f;
            for (int j = 0; j < history_count; j++) {
                float diff = amplitude_history[i][j] - mean;
                variance_sum += diff * diff;
            }
            variance = variance_sum / history_count;
        }
        total_variance += variance;
    }

    // Compute latest_motion_score as the average variance across all subcarriers
    latest_motion_score = total_variance / num_subcarriers;
}

// Initialization Function
inline bool initWifiRadar(int channel = 6) {
    // 1. Safely initialize NVS (if not already initialized)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    // Note: If NVS is already initialized (e.g. ESP_OK), we proceed.
    
    // 2. Initialize ESP-IDF WiFi config and interface
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) {
        return false;
    }

    // 3. Set WiFi mode to WIFI_MODE_STA (Station mode supports promiscuous and CSI reliably)
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return false;
    }

    // 4. Start WiFi
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return false;
    }

    // 5. Enable promiscuous mode
    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        return false;
    }

    // 6. Configure promiscuous filter to receive data and management frames
    wifi_promiscuous_filter_t filter;
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
    err = esp_wifi_set_promiscuous_filter(&filter);
    if (err != ESP_OK) {
        return false;
    }

    // 7. Fixed to the specified WiFi channel
    err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        return false;
    }

    // 8. Set up CSI configuration, enabling it and selecting filters for data and management frames
    wifi_csi_config_t csi_config;
    memset(&csi_config, 0, sizeof(csi_config));
    csi_config.lltf_en = true;
    csi_config.htltf_en = true;
    csi_config.stbc_htltf2_en = true;
    csi_config.ltf_merge_en = true;
    csi_config.channel_filter_en = true;

    err = esp_wifi_set_csi_config(&csi_config);
    if (err != ESP_OK) {
        return false;
    }

    // 9. Register the callback wifi_csi_rx_cb
    err = esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL);
    if (err != ESP_OK) {
        return false;
    }

    // 10. Enable CSI collection
    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        return false;
    }

    // 11. Reset rolling buffer state and latest_motion_score
    history_index = 0;
    history_count = 0;
    memset(amplitude_history, 0, sizeof(amplitude_history));
    latest_motion_score = 0.0f;

    // 12. Initialize DSP filters & estimators
    breathingFilter.init(0.3f, 0.4f, 10.0f); // 0.1 - 0.5 Hz at 10Hz sampling
    heartRateFilter.init(1.4f, 1.2f, 10.0f); // 0.8 - 2.0 Hz at 10Hz sampling
    breathingBpmEst.init(5.0f, 35.0f);       // 5 to 35 breaths per minute
    heartRateBpmEst.init(40.0f, 130.0f);     // 40 to 130 heartbeats per minute
    breathingEnv.reset();
    heartRateEnv.reset();
    breathingBpmEst.reset();
    heartRateBpmEst.reset();

    latest_breathing_val = 0.0f;
    latest_heart_val = 0.0f;
    latest_breathing_bpm = 0.0f;
    latest_heart_bpm = 0.0f;
    breathing_envelope = 0.0f;
    heart_envelope = 0.0f;

    return true;
}

// Teardown Function
inline bool deinitWifiRadar() {
    // 1. Safely disable CSI
    esp_wifi_set_csi(false);

    // 2. Unregister CSI callback
    esp_wifi_set_csi_rx_cb(NULL, NULL);

    // 3. Turn off promiscuous mode
    esp_wifi_set_promiscuous(false);

    // 4. Stop WiFi
    esp_wifi_stop();

    // 5. Deinitialize WiFi to free memory
    esp_err_t err = esp_wifi_deinit();
    
    return (err == ESP_OK);
}

#endif // WIFI_RADAR_H
