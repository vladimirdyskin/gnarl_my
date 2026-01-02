#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rfm95.h"
#include "pump_config.h"
#include "crc.h"
#include "4b6b.h"

#define TAG "WAKEUP_TEST"

// Pump ID from config
static const uint8_t PUMP_ID_BYTES[] = {0x90, 0x75, 0x91}; // 907591

static uint8_t pkt_buf[107];
static uint8_t rx_buf[128];

// Helper to listen for a specific duration
static int listen_only(int duration_ms) {
    ESP_LOGI(TAG, "TEST: Listening for %d ms...", duration_ms);
    int n = receive(rx_buf, sizeof(rx_buf), duration_ms);
    if (n > 0) {
        ESP_LOGI(TAG, "!!! ACTIVITY DETECTED !!! RSSI: %d", read_rssi());
        ESP_LOG_BUFFER_HEX(TAG, rx_buf, n);
        return 1;
    }
    return 0;
}

#define CMD_MODEL 0x8D

static void send_model_request(void) {
    uint8_t cmd_pkt[7];
    cmd_pkt[0] = 0xA7; 
    cmd_pkt[1] = PUMP_ID_BYTES[0];
    cmd_pkt[2] = PUMP_ID_BYTES[1];
    cmd_pkt[3] = PUMP_ID_BYTES[2];
    cmd_pkt[4] = CMD_MODEL; 
    cmd_pkt[5] = 0x00; 
    cmd_pkt[6] = crc8(cmd_pkt, 6);
    
    uint8_t *tx_data = pkt_buf;
    int tx_len = encode_4b6b(cmd_pkt, pkt_buf, 7);
    
    ESP_LOGI(TAG, "Sending CMD_MODEL...");
    if (transmit(tx_data, tx_len)) {
        int n = receive(rx_buf, sizeof(rx_buf), 200); // Wait longer for model response
        if (n > 0) {
            ESP_LOGI(TAG, "!!! MODEL RESPONSE RECEIVED !!! RSSI: %d", read_rssi());
            ESP_LOG_BUFFER_HEX(TAG, rx_buf, n);
        } else {
            ESP_LOGW(TAG, "No response to CMD_MODEL");
        }
    } else {
        ESP_LOGE(TAG, "Failed to transmit CMD_MODEL");
    }
}

static int perform_wakeup_burst(const char* name, int count, int delay_us, int rx_window_ms) {
    uint8_t wakeup_pkt[7];
    wakeup_pkt[0] = 0xA7; // Device Type (Carelink)
    wakeup_pkt[1] = PUMP_ID_BYTES[0];
    wakeup_pkt[2] = PUMP_ID_BYTES[1];
    wakeup_pkt[3] = PUMP_ID_BYTES[2];
    wakeup_pkt[4] = 0x5D; // CMD_WAKEUP
    wakeup_pkt[5] = 0x00; // Length
    wakeup_pkt[6] = crc8(wakeup_pkt, 6);
    
    uint8_t *tx_data = pkt_buf;
    int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
    
    ESP_LOGI(TAG, "START TEST [%s]: %d pkts, delay %dus, rx %dms", name, count, delay_us, rx_window_ms);
    
    int n = 0;
    for (int i = 0; i < count; i++) {
        if (!transmit(tx_data, tx_len)) {
             ESP_LOGE(TAG, "TX failed at burst %d", i);
             continue;
        }
        
        if (delay_us > 0) {
            usleep(delay_us);
        }
        
        // Listen for ACK
        n = receive(rx_buf, sizeof(rx_buf), rx_window_ms);
        
        if (n > 0) {
            ESP_LOGI(TAG, "!!! WAKEUP SUCCESS [%s] at burst %d !!! RSSI: %d", name, i, read_rssi());
            ESP_LOG_BUFFER_HEX(TAG, rx_buf, n);
            
            // Send Model Request immediately
            usleep(5000); // Small delay before next command
            send_model_request();
            
            return 1;
        }
        
        if (i % 20 == 0) {
            printf(".");
            fflush(stdout);
        }
    }
    printf("\n");
    ESP_LOGW(TAG, "TEST [%s] FINISHED: No response.", name);
    return 0;
}

void wakeup_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Initializing Radio...");
    set_frequency(PUMP_FREQUENCY);
    ESP_LOGI(TAG, "Frequency set to %lu Hz", (unsigned long)PUMP_FREQUENCY);

    // Give a moment for radio to stabilize, but short
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 1; i <= 5; i++) {
        ESP_LOGI(TAG, "=== STARTING TEST ITERATION %d/5 ===", i);

        // Test C: "Patient Listener" (100, 0ms, 25ms) - Give time to reply
        if (perform_wakeup_burst("PATIENT", 100, 0, 25)) {
            ESP_LOGI(TAG, "Iteration %d SUCCESS! Waiting 10s...", i);
            vTaskDelay(pdMS_TO_TICKS(10000));
        } else {
            ESP_LOGW(TAG, "Iteration %d FAILED. Retrying in 2s...", i);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    
    ESP_LOGI(TAG, "=== TEST COMPLETE ===");
    vTaskDelete(NULL);
}

void start_wakeup_test(void) {
    xTaskCreate(wakeup_test_task, "wakeup_test", 4096, NULL, 5, NULL);
}
