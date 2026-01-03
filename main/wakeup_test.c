#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "rfm95.h"
#include "pump_config.h"
#include "crc.h"
#include "4b6b.h"

#define TAG "WAKEUP_TEST"

// Test configuration
#define WAKEUP_PAUSE_MS 10000  // Pause between wakeup strategies (ms)

// Pump ID from config
static const uint8_t PUMP_ID_BYTES[] = {0x90, 0x75, 0x91}; // 907591
#define CARELINK_DEVICE 0xA7

// Medtronic Commands
#define CMD_ACK      0x06
#define CMD_WAKEUP   0x5D
#define CMD_CLOCK    0x70
#define CMD_BATTERY  0x72
#define CMD_RESERVOIR 0x73
#define CMD_MODEL    0x8D
#define CMD_STATUS   0xCE

static uint8_t pkt_buf[107];
static uint8_t rx_buf[128];
static uint8_t decoded_buf[96];

// Global for best frequency from mmtune
static uint32_t g_best_frequency = PUMP_FREQUENCY;

// ===== PACKET BUILDING =====
static void build_pump_packet(uint8_t *out, size_t *out_len, uint8_t cmd) {
    uint8_t pkt[7];
    pkt[0] = CARELINK_DEVICE;
    pkt[1] = PUMP_ID_BYTES[0];
    pkt[2] = PUMP_ID_BYTES[1];
    pkt[3] = PUMP_ID_BYTES[2];
    pkt[4] = cmd;
    pkt[5] = 0x00;
    pkt[6] = crc8(pkt, 6);
    
    *out_len = encode_4b6b(pkt, out, 7);
    
    ESP_LOGD(TAG, "PKT[%d]: cmd=0x%02X", *out_len, cmd);
}

// ===== DECODE HELPERS =====
static bool find_crc_valid_packet(uint8_t *raw, int raw_len, int *out_start, int *out_enc_len) {
    uint8_t dec[128];
    int best_s = -1;
    int best_l = 0;
    int best_score = -1;
    
    for (int s = 0; s <= raw_len - 6; s++) {
        for (int l = 6; l <= raw_len - s; l++) {
            if ((l % 3) == 1) continue;
            
            int dec_len = decode_4b6b(raw + s, dec, l);
            if (dec_len <= 0) continue;
            if (dec[0] != CARELINK_DEVICE) continue;
            
            uint8_t computed_crc = crc8(dec, dec_len - 1);
            uint8_t packet_crc = dec[dec_len - 1];
            
            int score = 0;
            if (computed_crc == packet_crc) score += 100;
            score += dec_len;
            
            if (score > best_score) {
                best_score = score;
                best_s = s;
                best_l = l;
            }
        }
    }
    
    if (best_s < 0 || best_l < 6) return false;
    if (out_start) *out_start = best_s;
    if (out_enc_len) *out_enc_len = best_l;
    return true;
}

// ===== COMMAND HELPERS =====
static bool send_packet(uint8_t cmd, const char *desc) {
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, cmd);
    
    ESP_LOGI(TAG, "TX %s (0x%02X)...", desc, cmd);
    if (!transmit(pkt, len)) {
        ESP_LOGE(TAG, "TX failed");
        return false;
    }
    usleep(500);
    return true;
}

static bool wait_for_response(uint8_t *out_buf, int *out_len, int timeout_ms) {
    int n = receive(out_buf, 128, timeout_ms);
    if (n <= 0) {
        ESP_LOGD(TAG, "RX timeout");
        return false;
    }
    *out_len = n;
    return true;
}

static bool send_and_receive(uint8_t cmd, const char *desc) {
    int rx_len;
    
    if (!send_packet(cmd, desc)) return false;
    
    ESP_LOGI(TAG, "RX...");
    if (wait_for_response(rx_buf, &rx_len, 200)) {
        int start = 0, enc_len = 0;
        if (find_crc_valid_packet(rx_buf, rx_len, &start, &enc_len)) {
            int dec_len = decode_4b6b(rx_buf + start, decoded_buf, enc_len);
            if (dec_len > 0) {
                ESP_LOGI(TAG, "✓ Response: %d bytes, RSSI=%d", dec_len, read_rssi());
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, decoded_buf, dec_len, ESP_LOG_INFO);
                return true;
            }
        }
        ESP_LOGW(TAG, "Invalid packet received");
        return false;
    }
    ESP_LOGW(TAG, "No response");
    return false;
}


// ===== WAKEUP BURST =====
static bool perform_wakeup_burst(const char* name, int count, int delay_us, int rx_window_ms) {
    uint8_t wakeup_pkt[7];
    wakeup_pkt[0] = CARELINK_DEVICE;
    wakeup_pkt[1] = PUMP_ID_BYTES[0];
    wakeup_pkt[2] = PUMP_ID_BYTES[1];
    wakeup_pkt[3] = PUMP_ID_BYTES[2];
    wakeup_pkt[4] = CMD_WAKEUP;
    wakeup_pkt[5] = 0x00;
    wakeup_pkt[6] = crc8(wakeup_pkt, 6);
    
    uint8_t *tx_data = pkt_buf;
    int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
    
    ESP_LOGI(TAG, "\n=== WAKEUP BURST [%s] ===", name);
    ESP_LOGI(TAG, "Config: %d pkts, delay=%dus, rx_window=%dms", count, delay_us, rx_window_ms);
    
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (!transmit(tx_data, tx_len)) {
            ESP_LOGE(TAG, "TX failed at burst %d", i);
            continue;
        }
        
        if (delay_us > 0) {
            usleep(delay_us);
        }
        
        n = receive(rx_buf, sizeof(rx_buf), rx_window_ms);
        
        if (n > 0) {
            int start = 0, enc_len = 0;
            if (find_crc_valid_packet(rx_buf, n, &start, &enc_len)) {
                int dec_len = decode_4b6b(rx_buf + start, decoded_buf, enc_len);
                if (dec_len > 4 && decoded_buf[4] == CMD_ACK) {
                    ESP_LOGI(TAG, "✓ WAKEUP ACK at burst %d! RSSI=%d", i, read_rssi());
                    return true;
                }
            }
            ESP_LOGD(TAG, "RX at burst %d but not ACK", i);
        }
        
        if (i % 20 == 0 && i > 0) {
            printf(".");
            fflush(stdout);
        }
        
        if (i % 5 == 0) {
            taskYIELD(); // Let BLE task run
        }
    }
    printf("\n");
    ESP_LOGW(TAG, "✗ No ACK after %d bursts", count);
    return false;
}

// ===== COMBINED PATIENT_SPACED STRATEGY =====
static bool perform_patient_spaced_wakeup(void) {
    uint8_t wakeup_pkt[7];
    wakeup_pkt[0] = CARELINK_DEVICE;
    wakeup_pkt[1] = PUMP_ID_BYTES[0];
    wakeup_pkt[2] = PUMP_ID_BYTES[1];
    wakeup_pkt[3] = PUMP_ID_BYTES[2];
    wakeup_pkt[4] = CMD_WAKEUP;
    wakeup_pkt[5] = 0x00;
    wakeup_pkt[6] = crc8(wakeup_pkt, 6);
    
    uint8_t *tx_data = pkt_buf;
    int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
    
    ESP_LOGI(TAG, "\n=== WAKEUP BURST [PATIENT_SPACED] ===");
    ESP_LOGI(TAG, "Phase 1: PATIENT (50 pkts, 0us delay, 25ms RX)");
    ESP_LOGI(TAG, "Phase 2: SPACED (100 pkts, 5000us delay, 30ms RX)");
    
    int n = 0;
    
    // Phase 1: PATIENT (50 packets, 0µs delay, 25ms RX)
    for (int i = 0; i < 50; i++) {
        if (!transmit(tx_data, tx_len)) {
            ESP_LOGE(TAG, "TX failed at burst %d", i);
            continue;
        }
        
        n = receive(rx_buf, sizeof(rx_buf), 25);
        
        if (n > 0) {
            int start = 0, enc_len = 0;
            if (find_crc_valid_packet(rx_buf, n, &start, &enc_len)) {
                int dec_len = decode_4b6b(rx_buf + start, decoded_buf, enc_len);
                if (dec_len > 4 && decoded_buf[4] == CMD_ACK) {
                    ESP_LOGI(TAG, "✓ WAKEUP ACK at PATIENT phase, burst %d! RSSI=%d", i, read_rssi());
                    return true;
                }
            }
        }
        
        if (i % 10 == 0 && i > 0) {
            printf(".");
            fflush(stdout);
        }
        
        if (i % 5 == 0) {
            taskYIELD();
        }
    }
    
    ESP_LOGI(TAG, "Phase 1 complete, switching to SPACED...");
    
    // Phase 2: SPACED (100 packets, 5000µs delay, 30ms RX)
    for (int i = 0; i < 100; i++) {
        if (!transmit(tx_data, tx_len)) {
            ESP_LOGE(TAG, "TX failed at burst %d", 50 + i);
            continue;
        }
        
        usleep(5000);
        
        n = receive(rx_buf, sizeof(rx_buf), 30);
        
        if (n > 0) {
            int start = 0, enc_len = 0;
            if (find_crc_valid_packet(rx_buf, n, &start, &enc_len)) {
                int dec_len = decode_4b6b(rx_buf + start, decoded_buf, enc_len);
                if (dec_len > 4 && decoded_buf[4] == CMD_ACK) {
                    ESP_LOGI(TAG, "✓ WAKEUP ACK at SPACED phase, burst %d! RSSI=%d", 50 + i, read_rssi());
                    return true;
                }
            }
        }
        
        if (i % 10 == 0 && i > 0) {
            printf(".");
            fflush(stdout);
        }
        
        if (i % 5 == 0) {
            taskYIELD();
        }
    }
    
    printf("\n");
    ESP_LOGW(TAG, "✗ No ACK after 150 bursts (PATIENT_SPACED)");
    return false;
}

// ===== FREQUENCY SCAN (MMTUNE) =====
static void test_mmtune(void) {
    ESP_LOGI(TAG, "\n=== MMTUNE FREQUENCY SCAN ===");
    
    // STEP 1: Wake pump on default frequency first
    ESP_LOGI(TAG, "Step 1: Waking pump on default frequency %d Hz", PUMP_FREQUENCY);
    set_frequency(PUMP_FREQUENCY);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Use GNARL_STYLE wakeup burst (500µs delay, 30ms RX window)
    bool pump_awake = perform_wakeup_burst("INITIAL_WAKEUP", 200, 500, 30);
    
    if (!pump_awake) {
        ESP_LOGW(TAG, "✗ Failed to wake pump on default frequency!");
        ESP_LOGI(TAG, "===========================\n");
        return;
    }
    
    ESP_LOGI(TAG, "✓ Pump awake! Now scanning frequencies...\n");
    vTaskDelay(pdMS_TO_TICKS(500));  // Let pump stabilize
    
    // STEP 2: Scan all frequencies with GetModel
    ESP_LOGI(TAG, "Step 2: Scanning 21 frequencies from 868.0 to 869.0 MHz\n");
    
    const uint32_t START_FREQ = 868000000;
    const uint32_t STEP = 50000;
    const int NUM_FREQS = 21;
    
    int best_rssi = -128;
    uint32_t best_freq = 0;
    
    ESP_LOGI(TAG, "  Freq (MHz)    RSSI (dBm)");
    ESP_LOGI(TAG, "  ----------    ----------");
    
    for (int i = 0; i < NUM_FREQS; i++) {
        uint32_t freq = START_FREQ + (i * STEP);
        set_frequency(freq);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        int rssi = -128;
        
        // Send GetModel command and read RSSI from response
        uint8_t model_pkt[7];
        model_pkt[0] = CARELINK_DEVICE;
        model_pkt[1] = PUMP_ID_BYTES[0];
        model_pkt[2] = PUMP_ID_BYTES[1];
        model_pkt[3] = PUMP_ID_BYTES[2];
        model_pkt[4] = CMD_MODEL;
        model_pkt[5] = 0x00;
        model_pkt[6] = crc8(model_pkt, 6);
        
        uint8_t model_tx[256];
        int model_len = encode_4b6b(model_pkt, model_tx, 7);
        
        transmit(model_tx, model_len);
        int n = receive(rx_buf, sizeof(rx_buf), 100);
        
        if (n > 0) {
            int start = 0, enc_len = 0;
            if (find_crc_valid_packet(rx_buf, n, &start, &enc_len)) {
                int dec_len = decode_4b6b(rx_buf + start, decoded_buf, enc_len);
                if (dec_len > 4 && decoded_buf[4] == CMD_MODEL) {
                    rssi = read_rssi();
                }
            }
        }
        
        ESP_LOGI(TAG, "  %3d.%03d       %4d%s",
                 (int)(freq / 1000000), (int)((freq / 1000) % 1000), rssi,
                 (rssi > -128) ? " ✓" : "");
        
        if (rssi > best_rssi) {
            best_rssi = rssi;
            best_freq = freq;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "\n===========================");
    if (best_freq > 0) {
        ESP_LOGI(TAG, "✓ Best: %d.%03d MHz (RSSI=%d dBm)",
                 (int)(best_freq / 1000000), (int)((best_freq / 1000) % 1000), best_rssi);
        g_best_frequency = best_freq;
        set_frequency(best_freq);
    } else {
        ESP_LOGW(TAG, "✗ No response on any frequency");
    }
    ESP_LOGI(TAG, "===========================\n");
}

// ===== PREAMBLE VARIATION TEST =====
static void test_preamble_variations(void) {
    ESP_LOGI(TAG, "\n=== PREAMBLE LENGTH TEST ===");
    ESP_LOGI(TAG, "Using frequency: %lu Hz\n", (unsigned long)g_best_frequency);
    
    set_frequency(g_best_frequency);
    
    const struct {
        int preamble_ms;
        const char *desc;
    } tests[] = {
        {5,   "Minimal"},
        {20,  "Standard"},
        {50,  "GNARL typical"},
        {100, "GNARL max"},
        {200, "Extended"}
    };
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_MODEL);
    
    for (int t = 0; t < 5; t++) {
        ESP_LOGI(TAG, "\n--- Test %d: %s preamble (%d ms) ---",
                 t + 1, tests[t].desc, tests[t].preamble_ms);
        
        rfm95_set_preamble_ms(tests[t].preamble_ms);
        
        bool success = false;
        for (int attempt = 0; attempt < 3; attempt++) {
            ESP_LOGI(TAG, "Attempt %d/3...", attempt + 1);
            
            transmit(pkt, len);
            int n = receive(rx_buf, sizeof(rx_buf), 200);
            
            if (n > 0) {
                int start = 0, enc_len = 0;
                if (find_crc_valid_packet(rx_buf, n, &start, &enc_len)) {
                    ESP_LOGI(TAG, "✓ SUCCESS! RSSI=%d", read_rssi());
                    success = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        if (!success) {
            ESP_LOGW(TAG, "✗ Failed with %d ms preamble", tests[t].preamble_ms);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    rfm95_set_preamble_ms(0);
    ESP_LOGI(TAG, "\n=== PREAMBLE TEST COMPLETE ===\n");
}

// ===== COMMAND TEST SUITE =====
static void test_commands(void) {
    ESP_LOGI(TAG, "\n=== COMMAND TEST SUITE ===\n");
    
    const struct {
        uint8_t cmd;
        const char *name;
    } commands[] = {
        {CMD_MODEL,     "GetModel"},
        {CMD_CLOCK,     "GetClock"},
        {CMD_BATTERY,   "GetBattery"},
        {CMD_RESERVOIR, "GetReservoir"},
        {CMD_STATUS,    "GetStatus"}
    };
    
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "\n--- Test %d: %s (0x%02X) ---",
                 i + 1, commands[i].name, commands[i].cmd);
        
        if (send_and_receive(commands[i].cmd, commands[i].name)) {
            ESP_LOGI(TAG, "✓ %s success", commands[i].name);
        } else {
            ESP_LOGW(TAG, "✗ %s failed", commands[i].name);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "\n=== COMMAND TEST COMPLETE ===\n");
}

#define PUMP_SLEEP_MS 75000
// ===== WAKEUP STRATEGIES TEST =====
static void test_wakeup_strategies(void) {
    ESP_LOGI(TAG, "\n=== WAKEUP STRATEGIES TEST ===\n");
    
    // Only testing GNARL_STYLE with increased packets
    const char *name = "GNARL_STYLE_EXTENDED";
    // its work 100%!!!!
    // int count = 400; // 400
    // int delay_us = 500;
    // int rx_window_ms = 30;

    //     int count = 400; // 400
    // int delay_us = 100; // 200 works 
    // int rx_window_ms = 30; // 30 works


    int count = 300; // 400
    int delay_us = 200; // 200 works 
    int rx_window_ms = 30; // 30 works

    // Check if pump is already awake
    ESP_LOGI(TAG, "Checking if pump is asleep...");
    if (send_and_receive(CMD_MODEL, "Check Sleep")) {
        ESP_LOGW(TAG, "Pump is AWAKE! Waiting for it to sleep...");
        vTaskDelay(pdMS_TO_TICKS(PUMP_SLEEP_MS));
    } else {
        ESP_LOGI(TAG, "Pump appears to be asleep (no response).");
    }

    ESP_LOGI(TAG, "\n--- Strategy: %s ---", name);
    
    int64_t start_time = esp_timer_get_time();
    bool success = perform_wakeup_burst(name, count, delay_us, rx_window_ms);
    int64_t duration_ms = (esp_timer_get_time() - start_time) / 1000;

    if (success) {
        ESP_LOGI(TAG, "✓ Strategy '%s' succeeded in %lld ms!", name, duration_ms);
        
        // Test a command right after wakeup
        vTaskDelay(pdMS_TO_TICKS(100));
        if (send_and_receive(CMD_MODEL, "Model")) {
            ESP_LOGI(TAG, "✓ Post-wakeup command succeeded!");
        }            
    } else {
        ESP_LOGW(TAG, "✗ Strategy '%s' failed after %lld ms", name, duration_ms);
    }
    
    ESP_LOGI(TAG, "\n=== WAKEUP STRATEGIES COMPLETE ===\n");
}


// ===== MAIN TEST TASK =====
void wakeup_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "===   WAKEUP TEST SUITE STARTING    ===");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Pump ID: %02X%02X%02X", PUMP_ID_BYTES[0], PUMP_ID_BYTES[1], PUMP_ID_BYTES[2]);
    ESP_LOGI(TAG, "Default Freq: %lu Hz", (unsigned long)PUMP_FREQUENCY);
    ESP_LOGI(TAG, "\n");
    
    // Initialize radio
    set_frequency(PUMP_FREQUENCY);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1)
    {
        test_wakeup_strategies();
    // test_mmtune();
    // test_preamble_variations();
    // test_commands();
        vTaskDelay(pdMS_TO_TICKS(PUMP_SLEEP_MS)); // Wait 60 seconds before repeating
    }
    
    vTaskDelete(NULL);
}

void start_wakeup_test(void) {
    xTaskCreate(wakeup_test_task, "wakeup_test", 8192, NULL, 5, NULL);
}
