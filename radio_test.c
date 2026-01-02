/*
 * Radio Test - Active Communication
 * Tests multiple pump commands: Time, Battery, Reservoir
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "radio/radio_if.h"
#include "pump/minimed.h"

// Pump ID from successful scan: 90 75 91
#define PUMP_ID_0  0x90
#define PUMP_ID_1  0x75
#define PUMP_ID_2  0x91

// Medtronic Pump Commands
#define CMD_ACK                 0x06  // Acknowledge
#define CMD_NAK                 0x15  // Negative Acknowledge

// Write Commands (require special handling)
#define CMD_CGM_WRITE_TIMESTAMP 0x28  // CGM Write Timestamp
#define CMD_SET_BASAL_PATTERN_A 0x30  // Set Basal Pattern A
#define CMD_SET_BASAL_PATTERN_B 0x31  // Set Basal Pattern B
#define CMD_SET_CLOCK           0x40  // Set Clock/Time
#define CMD_SET_MAX_BOLUS       0x41  // Set Max Bolus
#define CMD_BOLUS               0x42  // Deliver Bolus
#define CMD_SELECT_BASAL_PATTERN 0x4A // Select Basal Pattern
#define CMD_SET_ABS_TEMP_BASAL  0x4C  // Set Absolute Temp Basal
#define CMD_SUSPEND             0x4D  // *** SUSPEND PUMP ***
#define CMD_RESUME              0x4D  // Resume = Suspend with param 0
#define CMD_BUTTON              0x5B  // Button Press
#define CMD_WAKEUP              0x5D  // Wakeup / RF Power On
#define CMD_SET_PCT_TEMP_BASAL  0x5E  // Set Percent Temp Basal

// Read Commands
#define CMD_CLOCK               0x70  // Read Clock/Time
#define CMD_BATTERY             0x72  // Read Battery Status
#define CMD_RESERVOIR           0x73  // Read Reservoir Level
#define CMD_HISTORY             0x80  // Read History Page
#define CMD_CARB_UNITS          0x88  // Carb Units (grams/exchanges)
#define CMD_GLUCOSE_UNITS       0x89  // Glucose Units (mg/dL or mmol/L)
#define CMD_CARB_RATIOS         0x8A  // Carb Ratios
#define CMD_SENSITIVITIES       0x8B  // Insulin Sensitivities
#define CMD_TARGETS_512         0x8C  // BG Targets (512 pumps)
#define CMD_MODEL               0x8D  // Pump Model Number
#define CMD_SETTINGS_512        0x91  // Settings (512 pumps)
#define CMD_BASAL_RATES         0x92  // Basal Rate Schedule
#define CMD_TEMP_BASAL          0x98  // Current Temp Basal
#define CMD_TARGETS             0x9F  // BG Targets (x22+ pumps)
#define CMD_SETTINGS            0xC0  // Settings (x22+ pumps)
#define CMD_STATUS              0xCE  // Pump Status

// Aliases for compatibility
#define CMD_READ_TIME      CMD_CLOCK
#define CMD_READ_BATTERY   CMD_BATTERY
#define CMD_READ_REMAINING CMD_RESERVOIR
#define CMD_GET_PUMP_MODEL CMD_MODEL

// Frequency found by scanner
#define PUMP_FREQUENCY     868350000

// Global variable for best frequency found by mmtune
static uint32_t g_best_frequency = PUMP_FREQUENCY;  // Default 868.35 MHz

// Static buffers to avoid stack overflow
static uint8_t rx_buf[128];
static uint8_t decoded_buf[96];  // Separate buffer for 4b6b decoding
static uint8_t long_pkt[150];

static void build_pump_packet(uint8_t *out, size_t *out_len, uint8_t cmd) {
    minimed_packet_t pkt = {
        .device_type = CARELINK_DEVICE,
        .pump_id = {PUMP_ID_0, PUMP_ID_1, PUMP_ID_2},
        .command = cmd,
        .length = 0,
        .crc = 0
    };
    pkt.crc = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);
    *out_len = encode_4b6b((uint8_t*)&pkt, out, sizeof(pkt));
    
    printk("PKT[%d]: ", *out_len);
    for (size_t i = 0; i < *out_len; i++) printk("%02X ", out[i]);
    printk("\n");
}

// Long packet structure: device_type + pump_id[3] + cmd + length + params[64] + crc = 71 bytes
// Encodes to 107 bytes
typedef struct __attribute__((packed)) {
    uint8_t device_type;
    uint8_t pump_id[3];
    uint8_t command;
    uint8_t length;
    uint8_t params[64];
    uint8_t crc;
} long_packet_t;

static void build_long_packet(uint8_t *out, size_t *out_len, uint8_t cmd, uint8_t *params, int params_len) {
    long_packet_t pkt = {
        .device_type = CARELINK_DEVICE,
        .pump_id = {PUMP_ID_0, PUMP_ID_1, PUMP_ID_2},
        .command = cmd,
        .length = params_len,
    };
    memset(pkt.params, 0, sizeof(pkt.params));
    if (params && params_len > 0) {
        memcpy(pkt.params, params, params_len);
    }
    pkt.crc = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);
    *out_len = encode_4b6b((uint8_t*)&pkt, out, sizeof(pkt));
    
    printk("LONG_PKT[%d]: cmd=0x%02X params_len=%d\n", *out_len, cmd, params_len);
}

static bool send_packet(uint8_t cmd, const char *desc) {
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, cmd);
    
    printk("TX %s (0x%02X)... ", desc, cmd);
    int ret = radio_tx(pkt, len);
    if (ret != 0) {
        printk("Failed (%d)\n", ret);
        return false;
    }
    k_busy_wait(500); 
    return true;
}

static void print_radio_status(const char *tag)
{
    uint8_t op = 0, irq1 = 0, irq2 = 0, dio_bits = 0;
    radio_debug_status(&op, &irq1, &irq2, &dio_bits);
    int rssi = radio_read_rssi();
    printk("%s: op=0x%02X irq1=0x%02X irq2=0x%02X dio_bits=%u fifo_empty=%u rssi=%d\n",
           tag, op, irq1, irq2, (unsigned)dio_bits, radio_fifo_empty() ? 1U : 0U, rssi);
}

static bool decode_ok_prefix(const uint8_t *enc, int enc_len, uint8_t *out_dec, int out_cap, int *out_dec_len)
{
    if (enc_len < 6) {
        return false;
    }
    if ((enc_len % 3) == 1) {
        return false;
    }
    int dec_len = decode_4b6b(enc, out_dec, (size_t)enc_len);
    if (dec_len <= 0 || dec_len > out_cap) {
        return false;
    }
    if (out_dec_len) {
        *out_dec_len = dec_len;
    }
    return true;
}

static void extract_best_4b6b_window(uint8_t *buf, int *len)
{
    if (!buf || !len || *len < 6) {
        return;
    }

    int raw_len = *len;
    int best_s = -1;
    int best_l = 0;
    int best_score = -1;

    uint8_t tmp_dec[96];
    int tmp_dec_len = 0;

    for (int start = 0; start <= raw_len - 6; start++) {
        for (int off = 0; off < 3; off++) {
            int s = start + off;
            if (s + 6 > raw_len) {
                continue;
            }

            int enc_len = 6;
            // Grow by full triplets.
            while (s + enc_len + 3 <= raw_len) {
                if (!decode_ok_prefix(buf + s, enc_len + 3, tmp_dec, (int)sizeof(tmp_dec), &tmp_dec_len)) {
                    break;
                }
                enc_len += 3;
            }
            // Allow a final 2-byte tail (3k+2).
            if (s + enc_len + 2 <= raw_len) {
                if (decode_ok_prefix(buf + s, enc_len + 2, tmp_dec, (int)sizeof(tmp_dec), &tmp_dec_len)) {
                    enc_len += 2;
                }
            }

            if (enc_len < 6) {
                continue;
            }

            // Score: prefer longer decodable runs; big bonus if it starts with CARELINK_DEVICE.
            int score = enc_len;
            if (decode_ok_prefix(buf + s, 6, tmp_dec, (int)sizeof(tmp_dec), &tmp_dec_len) && tmp_dec_len >= 1 && tmp_dec[0] == CARELINK_DEVICE) {
                score += 512;
            }

            if (score > best_score) {
                best_score = score;
                best_s = s;
                best_l = enc_len;
            }
        }
    }

    if (best_s >= 0 && best_l >= 6) {
        if (best_s != 0) {
            memmove(buf, buf + best_s, (size_t)best_l);
        }
        *len = best_l;
    }
}

static bool find_crc_valid_minimed_packet(uint8_t *raw, int raw_len, uint8_t expected_msg,
                                         int *out_start, int *out_enc_len)
{
    // GNARL-compatible scan:
    // - decode 4b6b
    // - starts with CARELINK_DEVICE
    // - (optional) message matches
    // - CRC8 OK where CRC byte is the LAST decoded byte, and CRC is computed over (dec_len-1)
    // This matters because pump responses are typically fixed-size long packets
    // (71 decoded bytes incl CRC; 107 encoded bytes), with payload padded to 64 bytes.
    // Supports encoded lengths 3k and 3k+2 (final 2 bytes -> 1 decoded byte).
    uint8_t dec[128];
    int best_s = -1;
    int best_l = 0;
    int best_score = -1;

    for (int s = 0; s <= raw_len - 6; s++) {
        for (int off = 0; off < 3; off++) {
            int start = s + off;
            if (start + 6 > raw_len) {
                continue;
            }

            int max_prefix = raw_len - start;
            for (int prefix = 6; prefix <= max_prefix; prefix++) {
                if ((prefix % 3) == 1) {
                    continue;
                }

                // Only consider canonical encoded sizes to reduce false positives:
                // - short packet: 7 decoded bytes incl CRC => 11 encoded bytes
                // - long packet: 71 decoded bytes incl CRC => 107 encoded bytes
                if (!(prefix == 11 || prefix == 107)) {
                    continue;
                }

                int dec_len = decode_4b6b(raw + start, dec, (size_t)prefix);
                if (dec_len < 7) {
                    continue;
                }
                // Only accept known packet sizes (short=7, long=71 incl CRC).
                if (!(dec_len == 7 || dec_len == 71)) {
                    continue;
                }
                if (dec[0] != CARELINK_DEVICE) {
                    continue;
                }
                uint8_t msg = dec[4];
                if (expected_msg != 0x00 && msg != expected_msg) {
                    // Allow ACK in general (rarely useful for read tests, but harmless).
                    if (msg != 0x06) {
                        continue;
                    }
                }

                // Length sanity: for a true short packet (7 bytes incl CRC), length must be 0.
                // If length is non-zero but we only decoded a short packet, this is almost
                // certainly a false-positive slice (it will later look 'truncated').
                uint8_t data_len = dec[5];
                if (data_len > 64) {
                    continue;
                }
                int expected_total = 7 + (int)data_len;
                if (dec_len == 7 && expected_total != 7) {
                    continue;
                }
                if (dec_len == 71 && expected_total > dec_len) {
                    continue;
                }

                uint8_t crc = crc8(dec, (size_t)(dec_len - 1));
                if (crc != dec[dec_len - 1]) {
                    continue;
                }

                // Scoring: strongly prefer long packets for reads; otherwise allow short.
                int score = 0;
                if (msg == expected_msg) {
                    score += 1000;
                }
                score += (dec_len == 71) ? 500 : 0;
                score += (prefix == 107) ? 50 : 0;
                score -= start; // earlier in capture is slightly better

                if (score > best_score) {
                    best_score = score;
                    best_s = start;
                    best_l = prefix;
                }
            }
        }
    }

done:
    if (best_s < 0 || best_l < 6) {
        return false;
    }
    if (out_start) {
        *out_start = best_s;
    }
    if (out_enc_len) {
        *out_enc_len = best_l;
    }
    return true;
}

static bool wait_for_response(uint8_t *out_buf, int *out_len, int timeout_ms) {
    int n = radio_receive(out_buf, 256, timeout_ms);
    if (n <= 0) {
        *out_len = 0;
        return false;
    }
    *out_len = n;
    return true;
}

static bool perform_wakeup(void) {
    printk("\n=== STARTING WAKEUP BURST ===\n");
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_WAKEUP);
    
    for (int i = 0; i < 150; i++) {
        radio_tx(pkt, len);
        k_busy_wait(500);
        
        radio_rx_start();
        bool ack = false;
        for(int j = 0; j < 8; j++) {
            if (radio_gpio_is_gdo0_active() || radio_sync_detected()) {
                ack = true;
                break;
            }
            k_msleep(2);
        }
        
        if (ack) {
            printk("Wakeup ACK at burst %d! RSSI: %d\n", i, radio_read_rssi());
            // Clear FIFO
            while (!radio_fifo_empty()) {
                radio_read_fifo();
            }
            radio_sleep();
            return true;
        }
        
        if (i % 10 == 0) printk(".");
    }
    printk("\nWakeup: No ACK after 150 bursts (pump may be already awake or off)\n");
    radio_sleep();
    return false;
}

static bool send_and_receive(uint8_t cmd, const char *desc) {
    int rx_len;
    
    if (!send_packet(cmd, desc)) {
        return false;
    }
    
    printk("RX... ");
    
    if (wait_for_response(rx_buf, &rx_len, 200)) {
        printk("Got %d bytes! RSSI: %d\n", rx_len, radio_read_rssi());

        if (rx_len < 6) {
            printk("Too short\n");
            radio_sleep();
            return false;
        }

        // First, try to find a CRC-valid minimed packet anywhere in the capture.
        int pkt_start = 0;
        int enc_len = 0;
        if (find_crc_valid_minimed_packet(rx_buf, rx_len, cmd, &pkt_start, &enc_len)) {
            if (pkt_start != 0) {
                memmove(rx_buf, rx_buf + pkt_start, (size_t)enc_len);
            }
            rx_len = enc_len;
            printk("CRC-valid window: start=%d enc_len=%d\n", pkt_start, enc_len);
        } else {
            // Fallback: best decodable 4b6b window (may still be CRC-bad).
            enc_len = rx_len;
            extract_best_4b6b_window(rx_buf, &enc_len);
            rx_len = enc_len;
            printk("Best window enc_len=%d (CRC not found)\n", enc_len);
        }

        uint8_t decoded[96];
        int dec_len = decode_4b6b(rx_buf, decoded, (size_t)rx_len);
        if (dec_len <= 0) {
            printk("Decode failed\n");
            radio_sleep();
            return false;
        }

        printk("DEC[%d] first=%02X pump=%02X%02X%02X msg=%02X len=%u\n",
               dec_len,
               decoded[0],
               (dec_len >= 4) ? decoded[1] : 0,
               (dec_len >= 4) ? decoded[2] : 0,
               (dec_len >= 4) ? decoded[3] : 0,
               (dec_len >= 5) ? decoded[4] : 0,
               (dec_len >= 6) ? decoded[5] : 0);

        if (dec_len >= 7 && decoded[0] == CARELINK_DEVICE) {
            // GNARL-compatible CRC: last decoded byte is CRC, computed over the rest.
            uint8_t crc_full = crc8(decoded, (size_t)(dec_len - 1));
            printk("CRC(full): calc=%02X got=%02X (%s)\n",
                   crc_full,
                   decoded[dec_len - 1],
                   (crc_full == decoded[dec_len - 1]) ? "OK" : "BAD");

            // Also print a payload-length sanity check (not used for CRC).
            uint8_t data_len = decoded[5];
            int expected_total = 7 + (int)data_len;
            if (data_len > 64 || dec_len < expected_total) {
                printk("Note: decoded packet shorter than payload length suggests: data_len=%u dec_len=%d need=%d\n",
                       data_len, dec_len, expected_total);
            }
        }

        // Keep the existing response parsing, but don't hard-fail solely on cmd mismatch.
        if (dec_len > 5 && decoded[0] == CARELINK_DEVICE) {
            uint8_t data_len = decoded[5];

            if (decoded[4] != cmd) {
                printk("Note: response msg=0x%02X (expected 0x%02X)\n", decoded[4], cmd);
            }

            if (data_len > 64) {
                printk("Invalid data_len=%u\n", data_len);
                radio_sleep();
                return false;
            }

            int expected_total = 7 + (int)data_len;
            if (dec_len < expected_total) {
                printk("Truncated response: dec_len=%d need=%d (data_len=%d)\n", dec_len, expected_total, data_len);
                radio_sleep();
                return false;
            }

            printk("Response msg=0x%02X len=%d: ", decoded[4], decoded[5]);
            for (int i = 0; i < dec_len && i < 24; i++) {
                printk("%02X ", decoded[i]);
            }
            printk("\n");
            
            // Parse specific responses
            // Header: A7 pump_id[3] cmd = 5 bytes, then len at [5], data starts at [6]
            uint8_t *data = &decoded[6];  // Points to first data byte after len
            
            if (cmd == CMD_READ_BATTERY && data_len >= 3) {
                // data[0]=status, data[1..2]=voltage (BE)
                uint16_t voltage_raw = (data[1] << 8) | data[2];
                uint16_t mV = voltage_raw * 10;
                printk("*** Battery: %d.%02d V (%d mV) ***\n", 
                       mV / 1000, (mV % 1000) / 10, mV);
            }
            else if (cmd == CMD_READ_REMAINING) {
                // 722: data_len=4, reservoir at data[2..3]
                // 522: data_len=2, reservoir at data[0..1]
                uint16_t remaining;
                if (data_len >= 4) {
                    remaining = (data[2] << 8) | data[3];
                    // 722: value * 25 / 1000 = units (milliUnits/40)
                    printk("*** Reservoir: %d.%d units ***\n", 
                           remaining * 25 / 1000, (remaining * 25 % 1000) / 100);
                } else {
                    remaining = (data[0] << 8) | data[1];
                    // 522: value * 100 / 1000 = units
                    printk("*** Reservoir: %d.%d units ***\n", 
                           remaining / 10, remaining % 10);
                }
            }
            else if (cmd == CMD_READ_TIME && data_len >= 7) {
                // data[0]=hour, data[1]=min, data[2]=sec, data[3..4]=year(BE), data[5]=month, data[6]=day
                uint16_t year = (data[3] << 8) | data[4];
                printk("*** Time: %02d:%02d:%02d  Date: %04d-%02d-%02d ***\n",
                       data[0], data[1], data[2], year, data[5], data[6]);
            }
            else if (cmd == CMD_TEMP_BASAL && data_len >= 6) {
                // data[0]=type (0=absolute), data[1]=rate_pct, data[2..3]=rate(BE), data[4..5]=minutes(BE)
                uint8_t tb_type = data[0];
                uint8_t rate_pct = data[1];
                uint16_t rate_abs = (data[2] << 8) | data[3];
                uint16_t minutes = (data[4] << 8) | data[5];
                
                if (minutes == 0) {
                    printk("*** Temp Basal: NONE active ***\n");
                } else if (tb_type == 0) {
                    uint32_t mU = rate_abs * 25;
                    printk("*** Temp Basal: %d.%03d U/hr for %d min ***\n", 
                           mU / 1000, mU % 1000, minutes);
                } else {
                    printk("*** Temp Basal: %d%% for %d min ***\n", rate_pct, minutes);
                }
            }
            else if (cmd == CMD_STATUS && data_len >= 3) {
                // data[0]=code, data[1]=bolusing, data[2]=suspended
                printk("*** Status: code=%d bolusing=%d suspended=%d ***\n",
                       data[0], data[1], data[2]);
            }
            else if (cmd == CMD_MODEL && data_len >= 2) {
                // data[0]=string_len, data[1..n]=model string (ASCII)
                int str_len = data[0];
                int model = 0;
                for (int i = 1; i <= str_len && i < data_len; i++) {
                    model = 10 * model + (data[i] - '0');
                }
                printk("*** Pump Model: %d (family=%d) ***\n", model, model % 100);
            }
            else if (cmd == CMD_CARB_UNITS && data_len >= 1) {
                // data[0]=units (1=grams, 2=exchanges)
                printk("*** Carb Units: %s ***\n", data[0] == 1 ? "grams" : "exchanges");
            }
            else if (cmd == CMD_GLUCOSE_UNITS && data_len >= 1) {
                // data[0]=units (1=mg/dL, 2=mmol/L)
                printk("*** Glucose Units: %s ***\n", data[0] == 1 ? "mg/dL" : "mmol/L");
            }
            else if (cmd == CMD_SETTINGS && data_len >= 21) {
                // x22+ pumps: data[6]=max_bolus, data[7..8]=max_basal(BE), data[13]=temp_basal_type, data[17]=DIA
                uint8_t max_bolus = data[6];
                uint16_t max_basal = (data[7] << 8) | data[8];
                uint8_t temp_type = data[13];
                uint8_t dia = data[17];
                printk("*** Settings: max_bolus=%d.%d U, max_basal=%d.%03d U/hr, DIA=%dh, temp_type=%s ***\n",
                       max_bolus / 10, max_bolus % 10,
                       max_basal * 25 / 1000, max_basal * 25 % 1000,
                       dia, temp_type == 0 ? "absolute" : "percent");
            }
            else if (cmd == CMD_CARB_RATIOS && data_len >= 3) {
                // data[0]=num+1, data[1]=units (1=grams, 2=exchanges), then pairs: [time, ratio] or [time, ratio_hi, ratio_lo]
                uint8_t units = data[1];
                printk("*** Carb Ratios (units=%s): ", units == 1 ? "grams" : "exchanges");
                // Just print first ratio for now
                if (data_len >= 5) {
                    uint16_t ratio = (data[3] << 8) | data[4];
                    printk("1g:%d.%d U", ratio / 10, ratio % 10);
                }
                printk(" ***\n");
            }
            else if (cmd == CMD_SENSITIVITIES && data_len >= 4) {
                // data[0]=num+1, data[1]=units, then pairs: [time|sensitivity_hi, sensitivity_lo]
                uint8_t units = data[1];
                printk("*** Sensitivities (units=%s): ", units == 1 ? "mg/dL" : "mmol/L");
                // First sensitivity
                if (data_len >= 4) {
                    uint8_t sens_hi = (data[2] >> 6) & 0x1;
                    uint8_t sens = (sens_hi << 8) | data[3];
                    printk("1U drops %d", sens);
                }
                printk(" ***\n");
            }
            else if (cmd == CMD_TARGETS && data_len >= 4) {
                // data[0]=num+1, data[1]=units, then [time, low, high]
                uint8_t units = data[1];
                printk("*** BG Targets (units=%s): ", units == 1 ? "mg/dL" : "mmol/L");
                if (data_len >= 5) {
                    printk("low=%d high=%d", data[3], data[4]);
                }
                printk(" ***\n");
            }
            else if (cmd == CMD_BASAL_RATES) {
                // Extended response - multiple fragments
                printk("*** Basal Rates: (extended response, %d bytes) ***\n", data_len);
            }
            radio_sleep();
            return true;
        }
    } else {
        printk("Timeout\n");
    }
    radio_sleep();
    return false;
}

// Long command: first send short packet, wait for ACK, then send long packet with params
static bool send_long_command(uint8_t cmd, uint8_t *params, int params_len, const char *desc) {
    int rx_len;
    
    // Step 1: Send short packet with command
    printk("\n=== LONG COMMAND: %s (0x%02X) ===\n", desc, cmd);
    printk("Step 1: Short packet... ");
    
    if (!send_packet(cmd, "init")) {
        return false;
    }
    
    // Wait for ACK
    printk("Waiting ACK... ");
    if (!wait_for_response(rx_buf, &rx_len, 500)) {
        printk("No ACK!\n");
        radio_sleep();
        return false;
    }
    
    // Decode and verify ACK
    int dec_len = decode_4b6b(rx_buf, rx_buf, (rx_len / 3) * 3);
    if (dec_len > 4 && rx_buf[4] == CMD_ACK) {
        printk("ACK received!\n");
    } else {
        printk("Invalid response (cmd=0x%02X)\n", dec_len > 4 ? rx_buf[4] : 0);
        radio_sleep();
        return false;
    }
    
    // Step 2: Send long packet with parameters
    printk("Step 2: Long packet with %d bytes params... ", params_len);
    
    size_t long_len;
    build_long_packet(long_pkt, &long_len, cmd, params, params_len);
    
    int ret = radio_tx(long_pkt, long_len);
    if (ret != 0) {
        printk("TX Failed (%d)\n", ret);
        radio_sleep();
        return false;
    }
    k_busy_wait(500);
    
    // Wait for response
    printk("Waiting response... ");
    if (wait_for_response(rx_buf, &rx_len, 500)) {
        printk("Got %d bytes! RSSI: %d\n", rx_len, radio_read_rssi());
        
        dec_len = decode_4b6b(rx_buf, rx_buf, (rx_len / 3) * 3);
        if (dec_len > 5) {
            printk("Response: ");
            for (int i = 0; i < dec_len && i < 20; i++) {
                printk("%02X ", rx_buf[i]);
            }
            printk("\n");
            
            if (rx_buf[4] == CMD_ACK) {
                printk("*** COMMAND ACCEPTED! ***\n");
            }
        }
        radio_sleep();
        return true;
    }
    
    printk("Timeout\n");
    radio_sleep();
    return false;
}

// Test temp basal (safe: just reads current, doesn't change anything)
static void test_read_temp_basal(void) {
    printk("\n--- Reading Temp Basal Status ---\n");
    send_and_receive(CMD_TEMP_BASAL, "TempBasal");
}

// Test pump status (extended response)
static void test_pump_status(void) {
    printk("\n--- Reading Pump Status ---\n");
    send_and_receive(CMD_STATUS, "Status");
}

// ⚠️ DANGEROUS: Test suspend and resume
static void test_suspend_resume(void) {
    printk("\n\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("!!!  WARNING: SUSPEND TEST STARTING  !!!\n");
    printk("!!!  Pump will stop for 5 seconds    !!!\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    
    // Check status before
    printk("--- Status BEFORE suspend ---\n");
    send_and_receive(CMD_STATUS, "Status");
    k_msleep(100);
    
    // SUSPEND
    printk("\n*** SUSPENDING PUMP ***\n");
    uint8_t suspend_param = 1;  // 1 = suspend
    bool suspended = send_long_command(CMD_SUSPEND, &suspend_param, 1, "SUSPEND");
    
    if (suspended) {
        printk("*** PUMP SUSPENDED! ***\n");
        
        // Verify suspended
        k_msleep(500);
        send_and_receive(CMD_STATUS, "Status");
        
        // Wait 5 seconds
        printk("\n*** Waiting 5 seconds... ***\n");
        for (int i = 5; i > 0; i--) {
            printk("%d... ", i);
            k_msleep(1000);
        }
        printk("\n");
        
        // RESUME
        printk("\n*** RESUMING PUMP ***\n");
        uint8_t resume_param = 0;  // 0 = resume
        bool resumed = send_long_command(CMD_SUSPEND, &resume_param, 1, "RESUME");
        
        if (resumed) {
            printk("*** PUMP RESUMED! ***\n");
        } else {
            printk("*** RESUME FAILED! Check pump manually! ***\n");
        }
        
        // Verify resumed
        k_msleep(500);
        send_and_receive(CMD_STATUS, "Status");
    } else {
        printk("*** SUSPEND FAILED ***\n");
    }
    
    printk("\n!!! SUSPEND/RESUME TEST COMPLETE !!!\n\n");
}

// ⚠️ Test SetTempBasal sequence: 0.9 → 1.0 U/hr
static void test_set_temp_basal_sequence(void) {
    printk("\n\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("!!!  TEMP BASAL SEQUENCE TEST       !!!\n");
    printk("!!!  Step 1: Set 0.9 U/hr           !!!\n");
    printk("!!!  Step 2: Set 1.0 U/hr (normal)  !!!\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    
    // STEP 1: Set 0.9 U/hr for 30 min
    {
        uint16_t rate_mU = 900;  // 0.9 U/hr
        uint16_t duration_min = 30;
        
        printk("\n=== STEP 1: Setting 0.9 U/hr for 30 min ===\n");
        
        printk("--- Current TempBasal ---\n");
        send_and_receive(CMD_TEMP_BASAL, "TempBasal");
        k_msleep(100);
        
        uint16_t strokes = (rate_mU * 40) / 1000;
        uint8_t time_segments = duration_min / 30;
        
        uint8_t params[4];
        params[0] = 3;
        params[1] = (strokes >> 8) & 0xFF;
        params[2] = strokes & 0xFF;
        params[3] = time_segments;
        
        printk("*** Setting TempBasal: %d strokes (%d.%03d U/hr) for %d segments (%d min) ***\n",
               strokes, rate_mU / 1000, rate_mU % 1000, time_segments, duration_min);
        printk("Params: %02X %02X %02X %02X\n", params[0], params[1], params[2], params[3]);
        
        bool ok = send_long_command(CMD_SET_ABS_TEMP_BASAL, params, 4, "SET_TEMP_BASAL");
        
        if (ok) {
            printk("*** STEP 1: 0.9 U/hr SET! ***\n");
        } else {
            printk("*** STEP 1: FAILED! ***\n");
        }
        
        k_msleep(500);
        printk("--- Verifying TempBasal ---\n");
        send_and_receive(CMD_TEMP_BASAL, "TempBasal");
        
        printk("\n*** Waiting 10 seconds before next step... ***\n");
        for (int i = 10; i > 0; i--) {
            printk("%d... ", i);
            k_msleep(1000);
        }
        printk("\n");
    }
    
    // STEP 2: Set 1.0 U/hr for 30 min (normal basal)
    {
        uint16_t rate_mU = 1000;  // 1.0 U/hr
        uint16_t duration_min = 30;
        
        printk("\n=== STEP 2: Setting 1.0 U/hr for 30 min (normal) ===\n");
        
        printk("--- Current TempBasal ---\n");
        send_and_receive(CMD_TEMP_BASAL, "TempBasal");
        k_msleep(100);
        
        uint16_t strokes = (rate_mU * 40) / 1000;
        uint8_t time_segments = duration_min / 30;
        
        uint8_t params[4];
        params[0] = 3;
        params[1] = (strokes >> 8) & 0xFF;
        params[2] = strokes & 0xFF;
        params[3] = time_segments;
        
        printk("*** Setting TempBasal: %d strokes (%d.%03d U/hr) for %d segments (%d min) ***\n",
               strokes, rate_mU / 1000, rate_mU % 1000, time_segments, duration_min);
        printk("Params: %02X %02X %02X %02X\n", params[0], params[1], params[2], params[3]);
        
        bool ok = send_long_command(CMD_SET_ABS_TEMP_BASAL, params, 4, "SET_TEMP_BASAL");
        
        if (ok) {
            printk("*** STEP 2: 1.0 U/hr SET! ***\n");
        } else {
            printk("*** STEP 2: FAILED! ***\n");
        }
        
        k_msleep(500);
        printk("--- Verifying TempBasal ---\n");
        send_and_receive(CMD_TEMP_BASAL, "TempBasal");
    }
    
    printk("\n!!! TEMP BASAL SEQUENCE COMPLETE !!!\n");
    printk("!!! Check pump: should show 1.0 U/hr !!!\n\n");
}

// ⚠️ DANGEROUS: Test single temp basal setting
static void test_set_temp_basal(uint16_t rate_mU, uint16_t duration_min) {
    printk("\n\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("!!!  WARNING: SET TEMP BASAL TEST    !!!\n");
    printk("!!!  Rate: %d.%03d U/hr for %d min    !!!\n", rate_mU / 1000, rate_mU % 1000, duration_min);
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    
    printk("--- Current TempBasal ---\n");
    send_and_receive(CMD_TEMP_BASAL, "TempBasal");
    k_msleep(100);
    
    uint16_t strokes = (rate_mU * 40) / 1000;
    uint8_t time_segments = duration_min / 30;
    
    uint8_t params[4];
    params[0] = 3;
    params[1] = (strokes >> 8) & 0xFF;
    params[2] = strokes & 0xFF;
    params[3] = time_segments;
    
    printk("*** Setting TempBasal: %d strokes (%d.%03d U/hr) for %d segments (%d min) ***\n",
           strokes, rate_mU / 1000, rate_mU % 1000, time_segments, duration_min);
    printk("Params: %02X %02X %02X %02X\n", params[0], params[1], params[2], params[3]);
    
    bool ok = send_long_command(CMD_SET_ABS_TEMP_BASAL, params, 4, "SET_TEMP_BASAL");
    
    if (ok) {
        printk("*** TEMP BASAL SET! ***\n");
    } else {
        printk("*** TEMP BASAL FAILED! ***\n");
    }
    
    k_msleep(500);
    printk("--- Verifying TempBasal ---\n");
    send_and_receive(CMD_TEMP_BASAL, "TempBasal");
    
    printk("\n!!! TEMP BASAL TEST COMPLETE !!!\n\n");
}

// ⚠️ DANGEROUS: Test Bolus - delivers insulin!
static void test_bolus(uint16_t amount_mU) {
    printk("\n\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printk("!!!  ⚠️⚠️⚠️ BOLUS TEST ⚠️⚠️⚠️        !!!\n");
    printk("!!!  Amount: %d.%02d U                  !!!\n", amount_mU / 1000, (amount_mU % 1000) / 10);
    printk("!!!  THIS WILL DELIVER REAL INSULIN! !!!\n");
    printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    
    // Read status before
    printk("--- Status BEFORE bolus ---\n");
    send_and_receive(CMD_STATUS, "Status");
    k_msleep(100);
    
    // Bolus (0x42) params for 722 (family=22):
    // param[0] = strokes
    // For x22 pumps (512, 712, 522, 722): 1 stroke = 0.1 U
    // For x23+ pumps: 1 stroke = 0.025 U
    uint8_t strokes = amount_mU / 100;  // 100 mU / 100 = 1 stroke = 0.1 U for 722
    
    uint8_t params[1];
    params[0] = strokes;
    
    printk("*** Delivering Bolus: %d strokes (%d.%02d U) ***\n",
           strokes, amount_mU / 1000, (amount_mU % 1000) / 10);
    
    bool ok = send_long_command(CMD_BOLUS, params, 1, "BOLUS");
    
    if (ok) {
        printk("*** BOLUS STARTED! ***\n");
        
        // Monitor bolusing status
        for (int i = 0; i < 10; i++) {
            k_msleep(1000);
            printk("Checking status... ");
            send_and_receive(CMD_STATUS, "Status");
        }
    } else {
        printk("*** BOLUS FAILED! ***\n");
    }
    
    printk("\n!!! BOLUS TEST COMPLETE !!!\n\n");
}

// --- BLE SIMULATION HELPERS ---
static bool send_raw_packet(const uint8_t *data, size_t len) {
    printk("TX RAW %d bytes... ", len);
    int ret = radio_tx((uint8_t*)data, len);
    if (ret != 0) {
        printk("Failed (%d)\n", ret);
        return false;
    }
    k_busy_wait(500);
    return true;
}

static bool send_raw_and_receive(const uint8_t *tx_data, size_t tx_len, uint8_t *rx_buf, int *rx_len, int timeout_ms, int preamble_ms) {
    radio_set_preamble_ms(preamble_ms);
    
    if (!send_raw_packet(tx_data, tx_len)) {
        return false;
    }
    
    printk("RX... ");
    if (wait_for_response(rx_buf, rx_len, timeout_ms)) {
        printk("Got %d bytes! RSSI: %d\n", *rx_len, radio_read_rssi());
        return true;
    }
    printk("Timeout\n");
    return false;
}

static void simulate_ble_command(const uint8_t *ble_data, int len) {
    printk("\n=== SIMULATING BLE COMMAND ===\n");
    printk("Input BLE Data (%d bytes):\n", len);
    for(int i=0; i<len; i++) printk("%02X ", ble_data[i]);
    printk("\n");
    
    if (len < 2) return;
    uint8_t cmd = ble_data[1]; // 0x05 = SendAndListen
    
    if (cmd == 0x05) { 
        // Header is 12 bytes (offsets 0-11 in payload, so 2-13 in ble_data)
        // ble_data[0] = len
        // ble_data[1] = cmd
        // ble_data[2] = send_channel
        // ble_data[3] = repeat_count
        // ble_data[4-5] = delay_ms
        // ble_data[6] = listen_channel
        // ble_data[7-10] = timeout_ms
        // ble_data[11] = retry_count
        // ble_data[12-13] = preamble_ms
        // ble_data[14...] = packet

        if (len < 14) {
            printk("Packet too short (min 14 bytes for SendAndListen)\n");
            return;
        }
        
        const uint8_t *data = &ble_data[2];
        
        uint8_t send_channel = data[0];
        uint8_t repeat_count = data[1];
        uint16_t delay_ms = (data[2] << 8) | data[3];
        uint8_t listen_channel = data[4];
        uint32_t timeout_ms = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
        uint8_t retry_count = data[9];
        uint16_t preamble_ms = (data[10] << 8) | data[11];
        
        const uint8_t *tx_data = &data[12];
        int tx_len = len - 14;
        
        // Strip trailing 0x00 if present (some implementations might add padding)
        if (tx_len > 0 && tx_data[tx_len-1] == 0x00) tx_len--;
        
        printk("SendAndListen:\n");
        printk("  Send Chan: %d\n", send_channel);
        printk("  Repeat: %d\n", repeat_count);
        printk("  Delay: %d ms\n", delay_ms);
        printk("  Listen Chan: %d\n", listen_channel);
        printk("  Timeout: %d ms\n", timeout_ms);
        printk("  Retry: %d\n", retry_count);
        printk("  Preamble: %d ms\n", preamble_ms);
        printk("  Packet Len: %d\n", tx_len);
               
        int rx_len;
        bool success = false;
        
        // Initial attempt + retries
        for (int i = 0; i <= retry_count; i++) {
            if (i > 0) {
                printk("Retry %d...\n", i);
            }
            
            // Note: We ignore send_channel/listen_channel for now and use the configured frequency
            // In a real implementation, we should switch channels if requested.
            
            if (send_raw_and_receive(tx_data, tx_len, rx_buf, &rx_len, timeout_ms, preamble_ms)) {
                printk("Success!\n");
                // Decode response
                int dec_len = decode_4b6b(rx_buf, rx_buf, (rx_len/3)*3);
                printk("Decoded response (%d bytes): ", dec_len);
                for(int k=0; k<dec_len; k++) printk("%02X ", rx_buf[k]);
                printk("\n");
                success = true;
                break;
            }
            
            if (i < retry_count) {
                 // Delay before retry? Gnarl doesn't seem to have explicit delay between retries other than processing time
                 // But the command has 'delay_ms' which is used between repeats of the SEND.
            }
        }
        if (!success) printk("Command Failed after retries\n");
    } else {
        printk("Unsupported BLE command: 0x%02X\n", cmd);
    }
    printk("=== END SIMULATION ===\n\n");
}

static void test_ble_simulation(void) {
    printk("\n*** STARTING BLE SIMULATION TEST ***\n");
    
    // 1. Construct a valid Pump Packet (Get Model)
    uint8_t pump_packet[64];
    size_t pump_packet_len;
    build_pump_packet(pump_packet, &pump_packet_len, CMD_MODEL);
    
    // 2. Construct the BLE Command Buffer
    // Structure: [Len, Cmd, SendCh, Repeat, Delay(2), ListenCh, Timeout(4), Retry, Preamble(2), Packet...]
    uint8_t ble_buf[128];
    int idx = 0;
    
    // Payload length will be 12 (header) + pump_packet_len
    uint8_t payload_len = 12 + pump_packet_len;
    
    ble_buf[idx++] = payload_len;       // Byte 0: Length
    ble_buf[idx++] = 0x05;              // Byte 1: CmdSendAndListen
    
    // Payload starts here
    ble_buf[idx++] = 0;                 // Send Channel (0 = use current)
    ble_buf[idx++] = 0;                 // Repeat Count
    
    // Delay MS (0)
    ble_buf[idx++] = 0;
    ble_buf[idx++] = 0;
    
    ble_buf[idx++] = 0;                 // Listen Channel (0 = use current)
    
    // Timeout MS (200ms = 0x000000C8)
    ble_buf[idx++] = 0x00;
    ble_buf[idx++] = 0x00;
    ble_buf[idx++] = 0x00;
    ble_buf[idx++] = 0xC8;
    
    ble_buf[idx++] = 3;                 // Retry Count
    
    // Preamble MS (20ms = 0x0014) - Standard for wakeup
    ble_buf[idx++] = 0x00;
    ble_buf[idx++] = 0x14;
    
    // Packet Data
    memcpy(&ble_buf[idx], pump_packet, pump_packet_len);
    idx += pump_packet_len;
    
    // 3. Run Simulation
    simulate_ble_command(ble_buf, idx);
}

// ===== PREAMBLE TEST: Test different preamble lengths =====
static void test_preamble_variations(void)
{
    printk("\n\n================================================\n");
    printk("=== PREAMBLE LENGTH TEST ===\n");
    printk("================================================\n");
    printk("Testing GNARL-style wakeup with different preamble lengths\n");
    printk("Using frequency: %d.%03d MHz\n\n",
           g_best_frequency / 1000000, (g_best_frequency % 1000000) / 1000);
    
    radio_set_frequency(g_best_frequency);  // Use best frequency from mmtune
    
    // Test configurations: {preamble_ms, description}
    const struct {
        int preamble_ms;
        const char *desc;
    } tests[] = {
        {5,   "Short (default)"},
        {20,  "Standard wakeup"},
        {46,  "GNARL active probe"},
        {50,  "GNARL typical"},
        {100, "GNARL max (sleeping pump)"}
    };
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_MODEL);
    
    for (int t = 0; t < 5; t++) {
        int preamble = tests[t].preamble_ms;
        const char *desc = tests[t].desc;
        
        printk("\n--- Test %d: Preamble = %d ms (%s) ---\n", t+1, preamble, desc);
        
        // Set preamble
        radio_set_preamble_ms(preamble);
        printk("Preamble set to %d ms\n", preamble);
        
        // TX
        printk("TX MODEL... ");
        uint64_t tx_start = k_uptime_get();
        radio_tx(pkt, len);
        k_busy_wait(500);
        printk("Done\n");
        
        // RX with timeout
        printk("RX (timeout 300ms)... ");
        int n = radio_receive(rx_buf, sizeof(rx_buf), 300);
        uint64_t rx_time = k_uptime_get() - tx_start;
        
        if (n > 0) {
            printk("✓ SUCCESS in %lld ms!\n", rx_time);
            printk("  Got %d bytes, RSSI=%d dBm\n", n, radio_read_rssi());
            
            // Show raw data (first 20 bytes)
            printk("  RAW: ");
            for (int i = 0; i < n && i < 20; i++) printk("%02X ", rx_buf[i]);
            if (n > 20) printk("...");
            printk("\n");
            
            // Find CRC-valid packet using GNARL method
            int pkt_start = 0;
            int enc_len = 0;
            if (find_crc_valid_minimed_packet(rx_buf, n, CMD_MODEL, &pkt_start, &enc_len)) {
                printk("  CRC-valid packet found: start=%d enc_len=%d\n", pkt_start, enc_len);
                
                // Decode the valid portion
                int dec_len = decode_4b6b(rx_buf + pkt_start, decoded_buf, enc_len);
                printk("  DEC[%d]: Full packet:\n  ", dec_len);
                
                // Show all bytes in rows of 16
                for (int i = 0; i < dec_len; i++) {
                    printk("%02X ", decoded_buf[i]);
                    if ((i + 1) % 16 == 0 && (i + 1) < dec_len) printk("\n  ");
                }
                printk("\n");
                
                if (dec_len >= 7) {
                    uint8_t crc = crc8(decoded_buf, dec_len - 1);
                    printk("  CRC: calc=%02X got=%02X %s\n", 
                           crc, decoded_buf[dec_len-1],
                           (crc == decoded_buf[dec_len-1]) ? "✓ OK" : "✗ BAD");
                    
                    // Decode model from response
                    if (decoded_buf[4] == CMD_MODEL && dec_len >= 10) {
                        uint8_t data_len = decoded_buf[5];
                        if (data_len >= 2) {
                            uint8_t str_len = decoded_buf[6];
                            int model = 0;
                            for (int i = 1; i <= str_len && (6 + i) < dec_len; i++) {
                                model = 10 * model + (decoded_buf[6 + i] - '0');
                            }
                            printk("  *** Model: %d ***\n", model);
                        }
                    }
                }
            } else {
                printk("  ✗ No CRC-valid packet found, trying raw decode...\n");
                
                // Fallback: decode directly
                int dec_len = decode_4b6b(rx_buf, decoded_buf, (n/3)*3);
                printk("  DEC[%d]: ", dec_len);
                for (int i = 0; i < dec_len && i < 16; i++) printk("%02X ", decoded_buf[i]);
                printk("\n");
            }
        } else {
            printk("✗ TIMEOUT (no response in %lld ms)\n", rx_time);
        }
        
        // Wait between tests
        k_msleep(2000);
        radio_sleep();
        k_msleep(500);
    }
    
    printk("\n================================================\n");
    printk("=== PREAMBLE TEST COMPLETE ===\n");
    printk("================================================\n");
}

// ===== MMTUNE TEST: Frequency scanning like GNARL mmtune =====
static void test_mmtune(void)
{
    printk("\n\n================================================\n");
    printk("=== MMTUNE FREQUENCY SCAN ===\n");
    printk("================================================\n");
    printk("Scanning 21 frequencies from 868.0 to 869.0 MHz\n\n");
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_MODEL);
    
    const uint32_t START_FREQ = 868000000;  // 868.0 MHz
    const uint32_t STEP = 50000;             // 50 kHz
    const int NUM_FREQS = 21;                // 868.0 - 869.0 MHz
    
    int best_rssi = -128;
    uint32_t best_freq = 0;
    int freq_index = -1;
    
    // Store results for display
    int rssi_values[NUM_FREQS];
    
    printk("  Freq (MHz)    RSSI (dBm)\n");
    printk("  ----------    ----------\n");
    
    radio_set_preamble_ms(50);  // GNARL typical preamble
    
    for (int i = 0; i < NUM_FREQS; i++) {
        uint32_t freq = START_FREQ + (STEP * i);
        
        // Set frequency
        radio_set_frequency(freq);
        
        // Send command with retry (like GNARL)
        bool success = false;
        int rssi = -128;
        
        for (int retry = 0; retry <= 2; retry++) {
            radio_tx(pkt, len);
            k_busy_wait(500);
            
            int n = radio_receive(rx_buf, sizeof(rx_buf), 200);
            if (n > 0) {
                rssi = radio_read_rssi();
                
                // Quick CRC check
                int pkt_start = 0, enc_len = 0;
                if (find_crc_valid_minimed_packet(rx_buf, n, CMD_MODEL, &pkt_start, &enc_len)) {
                    success = true;
                    break;
                }
            }
            
            if (retry < 2) k_msleep(50);
        }
        
        rssi_values[i] = rssi;
        
        if (success && rssi > best_rssi) {
            best_rssi = rssi;
            best_freq = freq;
            freq_index = i;
        }
        
        // Print result
        printk("  %3d.%03d      %4d      %s\n",
               freq / 1000000, (freq % 1000000) / 1000,
               rssi,
               success ? "✓" : "✗");
        
        k_msleep(500);  // Short delay between frequencies
    }
    
    printk("\n================================================\n");
    if (best_freq > 0) {
        printk("=== BEST FREQUENCY: %d.%03d MHz ===\n",
               best_freq / 1000000, (best_freq % 1000000) / 1000);
        printk("=== BEST RSSI: %d dBm ===\n", best_rssi);
        
        // Simple ASCII bar chart
        printk("\nRSSI Chart:\n");
        for (int i = 0; i < NUM_FREQS; i++) {
            int bar_len = (rssi_values[i] + 128) / 2;  // Scale -128..0 to 0..64
            if (bar_len < 0) bar_len = 0;
            if (bar_len > 50) bar_len = 50;
            
            printk("%3d.%03d [%4d]: ",
                   (START_FREQ + STEP * i) / 1000000,
                   ((START_FREQ + STEP * i) % 1000000) / 1000,
                   rssi_values[i]);
            
            for (int j = 0; j < bar_len; j++) printk("#");
            if (i == freq_index) printk(" ← BEST");
            printk("\n");
        }
    } else {
        printk("=== NO VALID FREQUENCY FOUND ===\n");
    }
    printk("================================================\n");
    
    // Restore best frequency and save globally
    if (best_freq > 0) {
        g_best_frequency = best_freq;  // Save for other tests
        radio_set_frequency(best_freq);
        printk("\nFrequency set to best: %d.%03d MHz\n",
               best_freq / 1000000, (best_freq % 1000000) / 1000);
        printk("(Saved to g_best_frequency for other tests)\n");
    }
}

// ===== COMBINED TEST: Different preambles with retry + wakeup fallback =====
static void test_combined_wakeup(void)
{
    printk("\n\n================================================\n");
    printk("=== COMBINED WAKEUP TEST ===\n");
    printk("================================================\n");
    printk("Testing different preambles with retry + wakeup fallback\n");
    printk("Using frequency: %d.%03d MHz\n\n",
           g_best_frequency / 1000000, (g_best_frequency % 1000000) / 1000);
    
    radio_set_frequency(g_best_frequency);  // Use best frequency from mmtune
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_MODEL);
    
    // Test different preambles
    const int preambles[] = {5, 20, 50, 100};
    const char *descriptions[] = {"Short", "Standard", "GNARL typical", "GNARL max"};
    const int retry_count = 2;  // 2 retries = 3 total attempts
    const int timeout_ms = 200;
    
    for (int p = 0; p < 4; p++) {
        int preamble_ms = preambles[p];
        const char *desc = descriptions[p];
        
        printk("\n=== Test %d: Preamble %d ms (%s) ===\n", p+1, preamble_ms, desc);
        printk("Retry: %d attempts, timeout %d ms each\n\n", retry_count + 1, timeout_ms);
        
        radio_set_preamble_ms(preamble_ms);
        
        bool success = false;
        uint64_t total_start = k_uptime_get();
        
        // Try with retry
        for (int attempt = 0; attempt <= retry_count; attempt++) {
            printk("Attempt %d/%d: TX... ", attempt + 1, retry_count + 1);
            radio_tx(pkt, len);
            k_busy_wait(500);
            printk("RX... ");
            
            int n = radio_receive(rx_buf, sizeof(rx_buf), timeout_ms);
            
            if (n > 0) {
                uint64_t elapsed = k_uptime_get() - total_start;
                printk("✓ SUCCESS (%lld ms total, RSSI=%d dBm)\n", elapsed, radio_read_rssi());
                
                // Quick CRC check
                int pkt_start = 0, enc_len = 0;
                if (find_crc_valid_minimed_packet(rx_buf, n, CMD_MODEL, &pkt_start, &enc_len)) {
                    int dec_len = decode_4b6b(rx_buf + pkt_start, decoded_buf, enc_len);
                    if (dec_len >= 7) {
                        uint8_t crc = crc8(decoded_buf, dec_len - 1);
                        printk("  CRC: %s\n", (crc == decoded_buf[dec_len-1]) ? "✓ OK" : "✗ BAD");
                    }
                }
                success = true;
                break;
            } else {
                printk("✗ Timeout\n");
            }
            
            if (attempt < retry_count) {
                k_msleep(50);
            }
        }
        
        // If failed after retries, try wakeup burst
        if (!success) {
            printk("\n⚠️  All attempts failed. Trying WAKEUP burst...\n");
            
            uint8_t wakeup_pkt[64];
            size_t wakeup_len;
            build_pump_packet(wakeup_pkt, &wakeup_len, CMD_WAKEUP);
            
            radio_set_preamble_ms(5);  // Short preamble for burst
            
            for (int i = 0; i < 50; i++) {  // 50 burst packets (faster than 150)
                radio_tx(wakeup_pkt, wakeup_len);
                k_busy_wait(500);
                
                // Check for ACK every 10 bursts
                if (i % 10 == 9) {
                    radio_rx_start();
                    k_msleep(5);
                    if (radio_sync_detected() || !radio_fifo_empty()) {
                        printk("  Wakeup ACK at burst %d!\n", i + 1);
                        while (!radio_fifo_empty()) radio_read_fifo();
                        radio_sleep();
                        
                        // Try command again
                        k_msleep(50);
                        radio_set_preamble_ms(preamble_ms);
                        printk("  Retry command after wakeup... ");
                        radio_tx(pkt, len);
                        k_busy_wait(500);
                        int n = radio_receive(rx_buf, sizeof(rx_buf), timeout_ms);
                        if (n > 0) {
                            printk("✓ SUCCESS (RSSI=%d dBm)\n", radio_read_rssi());
                            success = true;
                        } else {
                            printk("✗ Still no response\n");
                        }
                        break;
                    }
                    radio_sleep();
                }
            }
            
            if (!success) {
                printk("  ✗ Wakeup burst failed (pump may be off)\n");
            }
        }
        
        uint64_t total_time = k_uptime_get() - total_start;
        if (success) {
            printk("=== RESULT: ✓ SUCCESS (total %lld ms) ===\n", total_time);
        } else {
            printk("=== RESULT: ✗ FAILED (total %lld ms) ===\n", total_time);
        }
        
        // DON'T sleep radio between tests - keep pump awake like mmtune!
        k_msleep(1000);  // Just pause 1 sec between tests
    }
    
    printk("\n================================================\n");
    printk("=== ALL TESTS COMPLETE ===\n");
    printk("================================================\n");
    
    // Sleep radio only AFTER all tests
    radio_sleep();
}

// ===== GNARL RETRY TEST: Test retry logic like GNARL =====
static void test_gnarl_retry(void)
{
    printk("\n\n================================================\n");
    printk("=== GNARL RETRY TEST ===\n");
    printk("================================================\n");
    printk("Testing retry logic with long preamble (like GNARL)\n");
    printk("Using frequency: %d.%03d MHz\n\n",
           g_best_frequency / 1000000, (g_best_frequency % 1000000) / 1000);
    
    radio_set_frequency(g_best_frequency);  // Use best frequency from mmtune
    
    uint8_t pkt[64];
    size_t len;
    build_pump_packet(pkt, &len, CMD_MODEL);
    
    const int preamble_ms = 50;  // GNARL typical
    const int retry_count = 3;   // 3 retries = 4 total attempts
    const int timeout_ms = 200;
    
    printk("Configuration:\n");
    printk("  Preamble: %d ms\n", preamble_ms);
    printk("  Retry count: %d (total %d attempts)\n", retry_count, retry_count + 1);
    printk("  Timeout per attempt: %d ms\n\n", timeout_ms);
    
    radio_set_preamble_ms(preamble_ms);
    
    bool success = false;
    uint64_t total_start = k_uptime_get();
    
    for (int attempt = 0; attempt <= retry_count; attempt++) {
        if (attempt == 0) {
            printk("Initial attempt:\n");
        } else {
            printk("\nRetry %d/%d:\n", attempt, retry_count);
        }
        
        // TX
        printk("  TX... ");
        uint64_t tx_start = k_uptime_get();
        radio_tx(pkt, len);
        k_busy_wait(500);
        printk("Done\n");
        
        // RX
        printk("  RX... ");
        int n = radio_receive(rx_buf, sizeof(rx_buf), timeout_ms);
        uint64_t elapsed = k_uptime_get() - tx_start;
        
        if (n > 0) {
            uint64_t total_time = k_uptime_get() - total_start;
            printk("✓ SUCCESS in %lld ms (total %lld ms, attempt %d)!\n", 
                   elapsed, total_time, attempt + 1);
            printk("  Got %d bytes, RSSI=%d dBm\n", n, radio_read_rssi());
            
            // Decode
            int pkt_start = 0;
            int enc_len = 0;
            if (find_crc_valid_minimed_packet(rx_buf, n, CMD_MODEL, &pkt_start, &enc_len)) {
                int dec_len = decode_4b6b(rx_buf + pkt_start, decoded_buf, enc_len);
                if (dec_len >= 7) {
                    uint8_t crc = crc8(decoded_buf, dec_len - 1);
                    printk("  CRC: %s\n", (crc == decoded_buf[dec_len-1]) ? "✓ OK" : "✗ BAD");
                    
                    if (decoded_buf[4] == CMD_MODEL && dec_len >= 10) {
                        uint8_t str_len = decoded_buf[6];
                        int model = 0;
                        for (int i = 1; i <= str_len && (6 + i) < dec_len; i++) {
                            model = 10 * model + (decoded_buf[6 + i] - '0');
                        }
                        printk("  *** Model: %d ***\n", model);
                    }
                }
            }
            
            success = true;
            break;
        } else {
            printk("✗ Timeout (%lld ms)\n", elapsed);
        }
        
        // Small delay between retries (GNARL doesn't have explicit delay, just processing time)
        if (attempt < retry_count) {
            k_msleep(100);
        }
    }
    
    uint64_t total_time = k_uptime_get() - total_start;
    
    printk("\n================================================\n");
    if (success) {
        printk("=== RESULT: SUCCESS (total time %lld ms) ===\n", total_time);
    } else {
        printk("=== RESULT: FAILED after %d attempts (%lld ms) ===\n", 
               retry_count + 1, total_time);
    }
    printk("================================================\n");
}

// ===== INTERRUPT TEST: Compare interrupt vs polling RX =====
static void test_interrupt_rx(void)
{
    // Reset radio to known state before each test iteration
    radio_sleep();
    k_msleep(10);
    
    printk("\n\n================================================\n");
    printk("=== INTERRUPT vs POLLING COMPARISON TEST ===\n");
    printk("================================================\n");
    printk("Using frequency: %d.%03d MHz\n\n",
           g_best_frequency / 1000000, (g_best_frequency % 1000000) / 1000);
    
    radio_set_frequency(g_best_frequency);  // Use best frequency from mmtune
    
    // ===== SKIP WAKEUP - Test like old code (no wakeup) =====
    printk("--- Skipping WAKEUP entirely (like old main) ---\n");
    printk("--- Pump should respond if it's awake ---\n\n");
    
    // Build packets 
    uint8_t pkt1[64], pkt2[64];
    size_t len1, len2;
    build_pump_packet(pkt1, &len1, CMD_MODEL);      // 0x8D - Model
    build_pump_packet(pkt2, &len2, CMD_READ_TIME);  // 0x70 - Time
    
    printk("*** Testing RX modes (NO wakeup)... ***\n\n");
    
    // ===== TEST 1: POLLING MODE (manual implementation) =====
    printk("--- TEST 1: MODEL (0x8D) with MANUAL POLLING ---\n");
    printk("This uses direct register polling (no interrupt)\n\n");
    
    // TX MODEL command
    printk("TX MODEL... ");
    radio_tx(pkt1, len1);
    k_busy_wait(500);
    printk("Done\n");
    
    // Manual polling RX (bypass radio_receive)
    printk("RX with POLLING (checking SYNC every 5ms):\n");
    radio_rx_start();
    
    bool sync_detected = false;
    int poll_time = 0;
    for(int i=0; i<40; i++) {  // 40 * 5ms = 200ms
        if (radio_sync_detected() || radio_gpio_is_gdo0_active()) {
            sync_detected = true;
            poll_time = i * 5;
            printk("  → SYNC detected at %d ms!\n", poll_time);
            break;
        }
        k_msleep(5);
    }
    
    if (sync_detected) {
        // Read FIFO
        int cnt = 0;
        int wait = 0;
        while(cnt < sizeof(rx_buf) && wait < 1000) {
            if (!radio_fifo_empty()) {
                rx_buf[cnt++] = radio_read_fifo();
                wait = 0;
            } else {
                k_usleep(500);
                wait++;
            }
        }
        radio_sleep();
        
        printk("  → SUCCESS! Got %d bytes, RSSI=%d dBm\n", cnt, radio_read_rssi());
        printk("  → RAW: ");
        for(int i=0; i<cnt && i<20; i++) printk("%02X ", rx_buf[i]);
        printk("\n");
        
        int dec_len = decode_4b6b(rx_buf, rx_buf, (cnt/3)*3);
        if (dec_len > 0) {
            printk("  → DEC[%d]: ", dec_len);
            for(int i=0; i<dec_len && i<12; i++) printk("%02X ", rx_buf[i]);
            printk("\n");
            if (dec_len >= 5) {
                printk("  → Response msg=0x%02X (expected 0x%02X)\n", rx_buf[4], CMD_MODEL);
            }
        }
    } else {
        printk("  → TIMEOUT (no SYNC in 200ms)\n");
        radio_sleep();
    }
    
    k_msleep(2000);  // Wait between tests
    
    // ===== TEST 2: INTERRUPT MODE (radio_receive) =====
    printk("\n--- TEST 2: TIME (0x70) with INTERRUPT ---\n");
    printk("This uses radio_receive() with DIO2 interrupt\n\n");
    
    // TX TIME command
    printk("TX TIME... ");
    radio_tx(pkt2, len2);
    k_busy_wait(500);
    printk("Done\n");
    
    // RX with interrupt (radio_receive uses k_sem_take on DIO2)
    printk("RX with INTERRUPT (waiting on DIO2 semaphore):\n");
    int n = radio_receive(rx_buf, sizeof(rx_buf), 200);
    
    if (n > 0) {
        printk("  → SUCCESS! Got %d bytes, RSSI=%d dBm\n", n, radio_read_rssi());
        printk("  → RAW: ");
        for(int i=0; i<n && i<20; i++) printk("%02X ", rx_buf[i]);
        printk("\n");
        
        int dec_len = decode_4b6b(rx_buf, rx_buf, (n/3)*3);
        if (dec_len > 0) {
            printk("  → DEC[%d]: ", dec_len);
            for(int i=0; i<dec_len && i<12; i++) printk("%02X ", rx_buf[i]);
            printk("\n");
            if (dec_len >= 5) {
                printk("  → Response msg=0x%02X (expected 0x%02X)\n", rx_buf[4], CMD_READ_TIME);
            }
        }
    } else {
        printk("  → TIMEOUT (interrupt never fired, k_sem_take timeout)\n");
        printk("  → This means DIO2 interrupt is NOT working!\n");
    }
    
    k_msleep(1000);
    
    // ===== SUMMARY =====
    printk("\n================================================\n");
    printk("=== SUMMARY ===\n");
    printk("POLLING mode:   %s\n", sync_detected ? "✓ WORKS" : "✗ FAILED");
    printk("INTERRUPT mode: %s\n", n > 0 ? "✓ WORKS" : "✗ FAILED");
    
    if (sync_detected && n <= 0) {
        printk("\n⚠️  DIAGNOSIS: DIO2 interrupt is broken!\n");
        printk("   - Polling detects SYNC correctly\n");
        printk("   - Interrupt never fires\n");
        printk("   - Possible causes:\n");
        printk("     1. DIO2 pin not physically connected\n");
        printk("     2. GPIO interrupt config issue\n");
        printk("     3. DIO_MAPPING_1 register wrong\n");
    } else if (!sync_detected && n <= 0) {
        printk("\n⚠️  DIAGNOSIS: Both modes failed!\n");
        printk("   - Pump was awake but not responding\n");
        printk("   - May be RF issue or pump sleeping again\n");
    } else if (sync_detected && n > 0) {
        printk("\n✓ DIAGNOSIS: Both modes working!\n");
        printk("   - DIO2 interrupt is functional\n");
    }
    printk("================================================\n");
}

// ===== MAIN: Selectable tests =====
int main(void)
{
    k_msleep(2000);
    printk("\n\n=== RADIO TEST SUITE ===\n");
    printk("nRF52840 + RFM95 @ 868 MHz\n");
    printk("Pump ID: %02X %02X %02X\n", PUMP_ID_0, PUMP_ID_1, PUMP_ID_2);
    printk("Waiting 5 seconds...\n");
    for (int i = 5; i > 0; i--) {
        printk("%d... ", i);
        k_msleep(1000);
    }
    printk("\n\nStarting!\n\n");
    
    radio_init();
    radio_set_frequency(PUMP_FREQUENCY);
    printk("Radio initialized on %d Hz\n", PUMP_FREQUENCY);
    radio_debug_print();
    
    // ========================================
    // SELECT TESTS TO RUN (can enable multiple)
    // Tests run in sequence: MMTUNE → PREAMBLE → RETRY → COMBINED → INTERRUPT → BLE_SIM → SUSPEND/BOLUS
    // ========================================
    
    #define RUN_MMTUNE      1  // Scan frequencies to find best RSSI
    #define RUN_PREAMBLE    1  // Test different preamble lengths
    #define RUN_RETRY       0  // Test GNARL retry logic
    #define RUN_COMBINED    0  // Test combined wakeup strategy
    #define RUN_INTERRUPT   0  // Test DIO2 interrupt vs polling
    #define RUN_BLE_SIM     0  // Test BLE SendAndListen simulation
    #define RUN_SUSPEND     1  // Test SUSPEND/RESUME (dangerous!)
    #define RUN_TEMP_BASAL  1  // Test temp basal 0.9→1.0 U/hr (safe if disconnected)
    #define RUN_BOLUS       1  // Test BOLUS 0.1U delivery (VERY dangerous!)
    
    // Run tests once (no loop - manual reboot for next test)
    {
        #if RUN_MMTUNE
        test_mmtune();
        #endif
        
        #if RUN_PREAMBLE
        test_preamble_variations();
        #endif
        
        #if RUN_RETRY
        test_gnarl_retry();
        #endif
        
        #if RUN_COMBINED
        test_combined_wakeup();
        #endif
        
        #if RUN_INTERRUPT
        test_interrupt_rx();
        #endif
        
        #if RUN_BLE_SIM
        test_ble_simulation();
        #endif
        
        #if RUN_SUSPEND
        test_suspend_resume();
        #endif
        
        #if RUN_TEMP_BASAL
        test_set_temp_basal_sequence();  // 0.9 → 1.0 U/hr
        #endif
        
        #if RUN_BOLUS
        test_bolus(100);  // 0.1 U
        #endif
        
        // All tests complete
        #if !RUN_MMTUNE && !RUN_PREAMBLE && !RUN_RETRY && !RUN_COMBINED && !RUN_INTERRUPT && !RUN_BLE_SIM && !RUN_SUSPEND && !RUN_TEMP_BASAL && !RUN_BOLUS
        printk("\n⚠️ No tests enabled! Enable at least one test.\n");
        #else
        printk("\n\n=== ALL TESTS COMPLETE ===\n");
        printk("=== System sleeping. Reboot to run again. ===\n\n");
        #endif
        
        radio_sleep();
        while(1) k_msleep(60000);  // Sleep forever - reboot to test again
    }
    
    return 0;
}


