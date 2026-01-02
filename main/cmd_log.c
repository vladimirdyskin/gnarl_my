#include "cmd_log.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>

#define TAG "CMD_LOG"

#define MAX_LOG_ENTRIES 20
#define MAX_DATA_BYTES 10

typedef enum {
    LOG_BLE_RX,
    LOG_BLE_TX,
    LOG_RADIO_TX,
    LOG_RADIO_RX
} log_direction_t;

typedef struct {
    log_direction_t direction;
    uint8_t len;
    uint8_t data[MAX_DATA_BYTES];
} log_entry_t;

static log_entry_t log_buffer[MAX_LOG_ENTRIES];
static volatile int log_head = 0;
static volatile int log_tail = 0;

void cmd_log_init(void) {
    log_head = 0;
    log_tail = 0;
}

static void add_log_entry(log_direction_t dir, const uint8_t *data, int len) {
    int next_head = (log_head + 1) % MAX_LOG_ENTRIES;
    
    // Если буфер полон, перезаписываем старые записи
    if (next_head == log_tail) {
        log_tail = (log_tail + 1) % MAX_LOG_ENTRIES;
    }
    
    log_buffer[log_head].direction = dir;
    log_buffer[log_head].len = (len < MAX_DATA_BYTES) ? len : MAX_DATA_BYTES;
    memcpy(log_buffer[log_head].data, data, log_buffer[log_head].len);
    
    log_head = next_head;
}

void cmd_log_ble_rx(const uint8_t *data, int len) {
    add_log_entry(LOG_BLE_RX, data, len);
}

void cmd_log_ble_tx(const uint8_t *data, int len) {
    add_log_entry(LOG_BLE_TX, data, len);
}

void cmd_log_radio_tx(const uint8_t *data, int len) {
    add_log_entry(LOG_RADIO_TX, data, len);
}

void cmd_log_radio_rx(const uint8_t *data, int len) {
    add_log_entry(LOG_RADIO_RX, data, len);
}

void cmd_log_flush(void) {
    while (log_tail != log_head) {
        log_entry_t *entry = &log_buffer[log_tail];
        
        char hex[32];
        int pos = 0;
        for (int i = 0; i < entry->len && pos < 31; i++) {
            pos += snprintf(hex + pos, 32 - pos, "%02x", entry->data[i]);
        }
        
        const char *prefix;
        switch (entry->direction) {
            case LOG_BLE_RX:    prefix = "<BLE"; break;
            case LOG_BLE_TX:    prefix = ">BLE"; break;
            case LOG_RADIO_TX:  prefix = ">RF "; break;
            case LOG_RADIO_RX:  prefix = "<RF "; break;
            default:            prefix = "??? "; break;
        }
        
        ESP_LOGI(TAG, "%s %s%s", prefix, hex, entry->len >= MAX_DATA_BYTES ? ".." : "");
        
        log_tail = (log_tail + 1) % MAX_LOG_ENTRIES;
    }
}
