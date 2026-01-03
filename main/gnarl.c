#include "gnarl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "4b6b.h"
#include "adc.h"
#include "cmd_log.h"
#include "commands.h"
#include "display.h"
#include "medtronic.h"
#include "rfm95.h"
#include "crc.h"
#include "pump_config.h"

#define MAX_PARAM_LEN (16)
#define MAX_PACKET_LEN (107)

#define CARELINK_DEVICE 0xA7
#define MINIMED_CMD_MODEL 0x8D
#define WAKEUP_BURST_DEFAULT_MS 8000u
#define WAKEUP_FIRST_TIMEOUT_MS 25000u
#define QUICK_MODEL_PROBE_MS 100u

// Optimized wakeup parameters (tested working values)
#define WAKEUP_BURST_COUNT 400
#define WAKEUP_BURST_DELAY_US 100
#define WAKEUP_RX_WINDOW_MS 30
#define WAKEUP_EARLY_TIMEOUT_US 9000000 // 9 seconds

static const char* cmd_name(uint8_t cmd) {
    switch(cmd) {
        case 0x00: return "ACK";
        case 0x5D: return "WAKEUP";
        case 0x8D: return "MODEL";
        default: return "UNKNOWN";
    }
}

static bool extract_minimed_info(const uint8_t *encoded, uint8_t len,
                                  uint8_t pump_id[3], uint8_t *cmd) {
    uint8_t decoded[96];
    int dec_len = decode_4b6b(encoded, decoded, len);
    
    if (dec_len < 5) return false;
    if (decoded[0] != CARELINK_DEVICE) return false;
    
    pump_id[0] = decoded[1];
    pump_id[1] = decoded[2];
    pump_id[2] = decoded[3];
    *cmd = decoded[4];
    
    ESP_LOGD(TAG, "Minimed: PumpID=%02X%02X%02X Cmd=0x%02X (%s)", 
             pump_id[0], pump_id[1], pump_id[2], *cmd, cmd_name(*cmd));
    
    return true;
}

typedef enum
{
	ENCODING_NONE = 0,
	ENCODING_4B6B = 2,
} encoding_type_t;

static encoding_type_t encoding_type = ENCODING_NONE;
static int64_t last_pump_comm_time = 0;

typedef enum CommandCode rfspy_cmd_t;

typedef struct
{
	rfspy_cmd_t command;
	int length;
	int rssi;
	uint8_t data[MAX_PARAM_LEN + MAX_PACKET_LEN];
} rfspy_request_t;

#define QUEUE_LENGTH (20)

static QueueHandle_t request_queue;

static TaskHandle_t gnarl_loop_handle = NULL;

static TaskHandle_t pump_probe_handle = NULL;

static TaskHandle_t wakeup_task_handle = NULL;

static volatile bool cancel_current_command;
static volatile bool pump_awake = false;
static volatile bool wakeup_in_progress = false;
static volatile bool wakeup_early_timeout_sent = false;

static inline bool pump_is_asleep(void) {
	// Consider pump awake only if we have a recent successful RX.
	if (pump_awake) {
		int64_t now = esp_timer_get_time();
		if (last_pump_comm_time > 0 && (now - last_pump_comm_time) <= (45 * SECONDS)) {
			return false;
		}
	}
	// If no recent comms (>45s) or never awake, treat as asleep and clear flag.
	pump_awake = false;
	return true;
}

void gnarl_cancel_current_command(void)
{
	cancel_current_command = true;
	if (gnarl_loop_handle != NULL)
	{
		xTaskNotifyGive(gnarl_loop_handle);
	}
}

static SemaphoreHandle_t radio_mutex;

// Tracking for RX timeouts (send_and_listen failures)
// Note: mode change failures are tracked separately in rfm95.c via rfm95_needs_reset()
static volatile int consecutive_rx_failures = 0;
#define RX_FAILURE_THRESHOLD 10  // Reset after 10 consecutive RX timeouts

#define PUMP_BG_LISTEN_INTERVAL_MS 5000
// 250ms was often too short to catch sporadic on-air traffic.
// Keep it modest to avoid delaying foreground commands.
#define PUMP_BG_LISTEN_TIMEOUT_MS 1500

// Active probe (short Medtronic CMD_STATUS) to populate pump RSSI after reboot.
// Runs rarely and skips if the radio is busy.
#define PUMP_ACTIVE_PROBE_START_DELAY_MS 3000
#define PUMP_ACTIVE_PROBE_INTERVAL_MS 30000
#define PUMP_ACTIVE_PROBE_PREAMBLE_MS 50
// Fast-start: try a few times quickly after boot so RSSI shows up sooner.
#define PUMP_ACTIVE_PROBE_FAST_TRIES 3
#define PUMP_ACTIVE_PROBE_FAST_INTERVAL_MS 2000

static inline bool radio_lock(TickType_t ticks_to_wait)
{
	return (radio_mutex != NULL) && (xSemaphoreTake(radio_mutex, ticks_to_wait) == pdTRUE);
}

static inline void radio_unlock(void)
{
	if (radio_mutex != NULL)
	{
		xSemaphoreGive(radio_mutex);
	}
}

// Check if radio is currently busy (for ADC to avoid interference)
bool radio_is_busy(void) {
	if (radio_mutex == NULL) {
		return false;
	}
	// Try to take with zero timeout - if we can't, it's busy
	if (xSemaphoreTake(radio_mutex, 0) == pdTRUE) {
		xSemaphoreGive(radio_mutex);
		return false;
	}
	return true;
}

// Check both RX failures and mode change failures (from rfm95.c)
static void reset_radio_if_needed(void) {
	bool needs_reset = (consecutive_rx_failures >= RX_FAILURE_THRESHOLD) || rfm95_needs_reset();
	
	if (needs_reset) {
		int rfm_failures = rfm95_get_failure_count();
		ESP_LOGW(TAG, "RADIO: Performing soft reinit (RX failures=%d, mode failures=%d)", 
				 consecutive_rx_failures, rfm_failures);
		
		// NOTE: This function is called only from within radio-locked contexts,
		// so we don't need to acquire the lock here.
		uint32_t saved_freq = PUMP_FREQUENCY;
		
		// Use soft reinit - NO hardware reset! Just reconfigure registers.
		// Hardware reset can kill the RFM95 chip on Heltec boards.
		rfm95_soft_reinit();
		set_frequency(saved_freq);
		
		// Reset both counters
		consecutive_rx_failures = 0;
		rfm95_reset_failure_count();
		ESP_LOGI(TAG, "RADIO: Soft reinit complete, frequency restored to %lu Hz", saved_freq);
	}
}

typedef struct __attribute__((packed))
{
	uint8_t listen_channel;
	uint32_t timeout_ms;
} get_packet_cmd_t;

typedef struct __attribute__((packed))
{
	uint8_t send_channel;
	uint8_t repeat_count;
	uint16_t delay_ms;
	uint8_t packet[];
} send_packet_cmd_t;

typedef struct __attribute__((packed))
{
	uint8_t send_channel;
	uint8_t repeat_count;
	uint16_t delay_ms;
	uint8_t listen_channel;
	uint32_t timeout_ms;
	uint8_t retry_count;
	uint16_t preamble_ms;
	uint8_t packet[];
} send_and_listen_cmd_t;

typedef struct __attribute__((packed))
{
	uint8_t rssi;
	uint8_t packet_count;
	uint8_t packet[MAX_PACKET_LEN];
} response_packet_t;

typedef struct __attribute__((packed))
{
	uint32_t uptime;
	uint16_t rx_overflow;
	uint16_t rx_fifo_overflow;
	uint16_t packet_rx_count;
	uint16_t packet_tx_count;
	uint16_t crc_failure_count;
	uint16_t spi_sync_failure_count;
	uint16_t placeholder0;
	uint16_t placeholder1;
} statistics_cmd_t;
static statistics_cmd_t statistics;

static response_packet_t rx_buf;
static connection_stats_t connections_stats[2];

connection_stats_t *get_connection_stats(void)
{
	return connections_stats;
}

void set_rssi(int value, connection_stat radio)
{
	connections_stats[radio].rssi = value;
	connections_stats[radio].timestamp = esp_timer_get_time() / SECONDS;
}

static inline void swap_bytes(uint8_t *p, uint8_t *q)
{
	uint8_t t = *p;
	*p = *q;
	*q = t;
}

static inline void reverse_two_bytes(uint16_t *n)
{
	uint8_t *p = (uint8_t *)n;
	swap_bytes(&p[0], &p[1]);
}

static inline void reverse_four_bytes(uint32_t *n)
{
	uint8_t *p = (uint8_t *)n;
	swap_bytes(&p[0], &p[3]);
	swap_bytes(&p[1], &p[2]);
}

// 71-byte long packet encodes to 107 bytes.
static uint8_t pkt_buf[107];

static void send(uint8_t *data, int len, int repeat_count, int delay_ms)
{
	if (len > 0 && data[len - 1] == 0)
	{
		len--;
	}

	bool skip_encoding = false;  // Set true if we've already produced encoded bytes.

	// --- ID SUBSTITUTION HACK ---
	// Force ID rewrite even if host did not request ENCODING_4B6B.
	// Strategy: try decode->rewrite->encode; if decode fails but header looks like Carelink, rewrite and encode anyway.
	if (len > 6) {
		uint8_t decoded[96];
		int dec_len = decode_4b6b(data, decoded, len);
		bool decoded_ok = (dec_len > 0 && dec_len <= (int)sizeof(decoded));
		uint8_t *payload = decoded_ok ? decoded : data;
		int payload_len = decoded_ok ? dec_len : len;
		if (payload_len > 6 && payload[0] == CARELINK_DEVICE) {
			uint8_t pid0 = payload[1];
			uint8_t pid1 = payload[2];
			uint8_t pid2 = payload[3];
			if (pid0 == 0x66 && pid1 == 0x55 && pid2 == 0x5A) {
				ESP_LOGI(TAG, "HACK: Substituting ID 66555A -> 907591 in TX packet");
				payload[1] = 0x90;
				payload[2] = 0x75;
				payload[3] = 0x91;
				payload[payload_len - 1] = crc8(payload, payload_len - 1);
				int new_len = encode_4b6b(payload, pkt_buf, payload_len);
				data = pkt_buf;
				len = new_len;
				skip_encoding = true;
			}
		}
	}
	// -----------------------------

	cmd_log_radio_tx(data, len);
	if (!skip_encoding)
	{
		switch (encoding_type)
		{
		case ENCODING_NONE:
			break;
		case ENCODING_4B6B:
			len = encode_4b6b(data, pkt_buf, len);
			data = pkt_buf;
			break;
		default:
			ESP_LOGE(TAG, "send: unknown encoding type %d", encoding_type);
			break;
		}
	}

	transmit(data, len);
	while (repeat_count > 0)
	{
		usleep(delay_ms * MILLISECONDS);
		transmit(data, len);
		repeat_count--;
	}
}

// Transform an RSSI value back into the raw encoding that the TI CC111x radios use.
// See section 13.10.3 of the CC1110 data sheet.
static uint8_t raw_rssi(int rssi)
{
	const int rssi_offset = 73;
	return (rssi + rssi_offset) * 2;
}

static void rx_common(int n, int rssi)
{
	if (n > 0)
	{
		pump_awake = true;
		last_pump_comm_time = esp_timer_get_time();
		display_pulse_radio_rx();
	}

	if (n == 0)
	{
		consecutive_rx_failures++;
		if ((consecutive_rx_failures % 5) == 0)
		{
			ESP_LOGW(TAG, "RX: timeout (failure #%d, noise floor RSSI=%d)", consecutive_rx_failures, rssi);
		}
		else
		{
			ESP_LOGD(TAG, "RX: timeout (failure #%d, noise floor RSSI=%d)", consecutive_rx_failures, rssi);
		}
		reset_radio_if_needed();
		send_code(RESPONSE_CODE_RX_TIMEOUT);
		return;
	}

	if (consecutive_rx_failures > 0)
	{
		ESP_LOGI(TAG, "RADIO: Successful RX after %d failures", consecutive_rx_failures);
		consecutive_rx_failures = 0;
	}

	rfm95_reset_failure_count();
	cmd_log_radio_rx(rx_buf.packet, n);
	set_pump_rssi(rssi);
	rx_buf.rssi = raw_rssi(rssi);
	if (rx_buf.rssi == 0)
	{
		rx_buf.rssi = 1;
	}
	rx_buf.packet_count = rx_packet_count();
	if (rx_buf.packet_count == 0)
	{
		rx_buf.packet_count = 1;
	}
	int d;
	switch (encoding_type)
	{
	case ENCODING_NONE:
		break;
	case ENCODING_4B6B:
		d = decode_4b6b(rx_buf.packet, pkt_buf, n);
		if (d != -1)
		{
			memcpy(rx_buf.packet, pkt_buf, d);
			n = d;
		}
		break;
	default:
		ESP_LOGE(TAG, "RX: unknown encoding type %d", encoding_type);
		break;
	}

	cmd_log_flush();
	uint8_t rx_pid[3];
	uint8_t rx_cmd;
	if (extract_minimed_info(rx_buf.packet, n, rx_pid, &rx_cmd)) {
		ESP_LOGD(TAG, "RX Minimed dev=%02X id=%02X%02X%02X cmd=0x%02X", CARELINK_DEVICE, rx_pid[0], rx_pid[1], rx_pid[2], rx_cmd);
	}
	send_bytes((uint8_t *)&rx_buf, 2 + n);
}

static volatile int in_get_packet = 0;

static void get_packet(const uint8_t *buf, int len)
{
	get_packet_cmd_t *p = (get_packet_cmd_t *)buf;
	uint32_t timeout_ms;
	memcpy(&timeout_ms, &p->timeout_ms, sizeof(timeout_ms));
	reverse_four_bytes(&timeout_ms);
	memcpy(&p->timeout_ms, &timeout_ms, sizeof(timeout_ms));
	uint32_t original_timeout_ms = p->timeout_ms;
	bool pump_asleep = !pump_awake;
	if (pump_asleep) {
		const uint32_t bump_ms = 1000u;
		if (p->timeout_ms > 0xFFFFFFFFu - bump_ms) {
			p->timeout_ms = 0xFFFFFFFFu;
		} else {
			p->timeout_ms += bump_ms;
		}
	}
	ESP_LOGD(TAG, "get_packet: listen_channel %d timeout_ms %lu (raw %lu%s)",
			 p->listen_channel, p->timeout_ms, (unsigned long)original_timeout_ms,
			 pump_asleep ? ", +1000ms (pump asleep)" : "");
	if (!radio_lock(portMAX_DELAY))
	{
		ESP_LOGW(TAG, "get_packet: radio mutex unavailable");
		return;
	}

	in_get_packet = 1;
	int n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
	if (!cancel_current_command)
	{
		rx_common(n, read_rssi());
	}
	in_get_packet = 0;

	radio_unlock();
}

static void send_packet(const uint8_t *buf, int len)
{
	send_packet_cmd_t *p = (send_packet_cmd_t *)buf;
	uint16_t delay_ms_val;
	memcpy(&delay_ms_val, &p->delay_ms, sizeof(delay_ms_val));
	reverse_two_bytes(&delay_ms_val);
	memcpy(&p->delay_ms, &delay_ms_val, sizeof(delay_ms_val));
	ESP_LOGD(TAG, "send_packet: len %d send_channel %d repeat_count %d delay_ms %d",
			 len, p->send_channel, p->repeat_count, p->delay_ms);
	len -= (p->packet - (uint8_t *)p);
	int repeat_count = p->repeat_count;
	int delay_ms = p->delay_ms;
	// Some clients use 0xFF as a sentinel for "default". Treat it as "no repeats".
	if (repeat_count == 0xFF || delay_ms == 0) {
		repeat_count = 0;
	}
	if (!radio_lock(portMAX_DELAY))
	{
		ESP_LOGW(TAG, "send_packet: radio mutex unavailable");
		return;
	}
	send(p->packet, len, repeat_count, delay_ms);
	radio_unlock();
	send_code(RESPONSE_CODE_SUCCESS);
}

static bool quick_model_probe(const uint8_t *pkt, int pkt_len, uint32_t timeout_ms) {
	if (pkt_len < 4) {
		return false;
	}

	if (timeout_ms == 0) {
		timeout_ms = QUICK_MODEL_PROBE_MS;
	}

	uint32_t prev_freq = read_frequency();
	if (prev_freq != PUMP_FREQUENCY) {
		set_frequency(PUMP_FREQUENCY);
	}

	uint8_t probe_pkt[7];
	probe_pkt[0] = CARELINK_DEVICE;
	probe_pkt[1] = pkt[1];
	probe_pkt[2] = pkt[2];
	probe_pkt[3] = pkt[3];
	probe_pkt[4] = MINIMED_CMD_MODEL;
	probe_pkt[5] = 0;
	probe_pkt[6] = crc8(probe_pkt, 6);

	uint8_t encoded[16];
	int tx_len = encode_4b6b(probe_pkt, encoded, sizeof(probe_pkt));
	if (tx_len <= 0) {
		if (prev_freq != PUMP_FREQUENCY) {
			set_frequency(prev_freq);
		}
		return false;
	}

	uint8_t rx_tmp[150];
	transmit(encoded, tx_len);
	int n = receive(rx_tmp, sizeof(rx_tmp), timeout_ms);

	if (prev_freq != PUMP_FREQUENCY) {
		set_frequency(prev_freq);
	}

	if (n > 0) {
		pump_awake = true;
		last_pump_comm_time = esp_timer_get_time();
		return true;
	}

	return false;
}

// Helper: Parse and byte-swap the command structure in-place
static void parse_sl_cmd(send_and_listen_cmd_t *p) {
	uint16_t delay, preamble;
	uint32_t timeout;
	
	memcpy(&delay, &p->delay_ms, 2);
	memcpy(&timeout, &p->timeout_ms, 4);
	memcpy(&preamble, &p->preamble_ms, 2);
	
	reverse_two_bytes(&delay);
	reverse_four_bytes(&timeout);
	reverse_two_bytes(&preamble);
	
	p->delay_ms = delay;
	p->timeout_ms = timeout;
	p->preamble_ms = preamble;
}

// Async wakeup task - runs in background
// IMPORTANT: Uses tight loop like wakeup_test.c - no per-iteration radio_lock!
static void async_wakeup_task(void *unused) {
	ESP_LOGI(TAG, "ASYNC_WAKEUP: Task started");
	
	// Flags already set in start_async_wakeup()
	
	// Use LOCAL buffer - don't pollute global rx_buf!
	uint8_t local_rx[128];
	
	// Prepare WAKEUP packet
	uint8_t wakeup_pkt[7] = { 0xA7, 0x90, 0x75, 0x91, 0x5D, 0x00, 0x00 };
	wakeup_pkt[6] = crc8(wakeup_pkt, 6);
	
	// Use local TX buffer too
	uint8_t local_tx[16];
	int tx_len = encode_4b6b(wakeup_pkt, local_tx, 7);
	
	int64_t start_time = esp_timer_get_time();
	int64_t deadline = start_time + ((int64_t)WAKEUP_FIRST_TIMEOUT_MS * MILLISECONDS);
	
	int n = 0;
	
	// Take radio lock ONCE at start (like wakeup_test.c - tight loop!)
	if (!radio_lock(pdMS_TO_TICKS(5000))) {
		ESP_LOGE(TAG, "ASYNC_WAKEUP: Cannot get radio lock!");
		wakeup_in_progress = false;
		wakeup_task_handle = NULL;
		vTaskDelete(NULL);
		return;
	}
	
	// CRITICAL: Set frequency before burst! (like wakeup_test.c)
	set_frequency(PUMP_FREQUENCY);
	
	ESP_LOGI(TAG, "ASYNC_WAKEUP: Starting burst (count=%d, delay=%dus, rx=%dms) at %lu Hz", 
			 WAKEUP_BURST_COUNT, WAKEUP_BURST_DELAY_US, WAKEUP_RX_WINDOW_MS,
			 (unsigned long)PUMP_FREQUENCY);
	
	for (int i = 0; i < WAKEUP_BURST_COUNT && wakeup_in_progress; i++) {
		int64_t now = esp_timer_get_time();
		
		if (now >= deadline) {
			ESP_LOGW(TAG, "ASYNC_WAKEUP: Timeout at burst %d", i);
			break;
		}
		
		// Send early timeout at 9 seconds (do this OUTSIDE tight loop timing)
		if (!wakeup_early_timeout_sent && (now - start_time) > WAKEUP_EARLY_TIMEOUT_US) {
			ESP_LOGI(TAG, "ASYNC_WAKEUP: Sending early timeout to phone at 9s (burst %d)...", i);
			send_code(RESPONSE_CODE_RX_TIMEOUT);
			wakeup_early_timeout_sent = true;
		}
		
		// Tight loop like wakeup_test.c - NO radio_lock per iteration!
		transmit(local_tx, tx_len);
		
		if (WAKEUP_BURST_DELAY_US > 0) {
			usleep(WAKEUP_BURST_DELAY_US);
		}
		
		n = receive(local_rx, sizeof(local_rx), WAKEUP_RX_WINDOW_MS);
		
		if (n > 0) {
			ESP_LOGI(TAG, "ASYNC_WAKEUP: Response at burst %d! (RSSI: %d)", i, read_rssi());
			pump_awake = true;
			last_pump_comm_time = esp_timer_get_time();
			
			// CRITICAL: Clear early timeout flag so next command executes!
			wakeup_early_timeout_sent = false;
			
			// NOTE: Do NOT send GetModel here! It interferes with iAPS commands.
			// The pump is already awake after responding to WAKEUP.
			// iAPS will send its own commands.
			
			break;
		}
		
		// Yield every 50 bursts to let other tasks run (was 20 - too often)
		if (i % 50 == 0 && i > 0) {
			taskYIELD();
		}
	}
	
	// Release radio lock at the end
	radio_unlock();
	
	int64_t total_time = (esp_timer_get_time() - start_time) / 1000;
	
	if (n == 0) {
		ESP_LOGW(TAG, "ASYNC_WAKEUP: Failed (no response after %d bursts, %lld ms)", 
				 WAKEUP_BURST_COUNT, (long long)total_time);
	} else {
		ESP_LOGI(TAG, "ASYNC_WAKEUP: Success in %lld ms", (long long)total_time);
	}
	
	wakeup_in_progress = false;
	ESP_LOGI(TAG, "ASYNC_WAKEUP: Task finished (pump_awake=%d)", pump_awake);
	
	wakeup_task_handle = NULL;
	vTaskDelete(NULL);
}

// Start async wakeup if not already running
static void start_async_wakeup(void) {
	if (wakeup_in_progress) {
		ESP_LOGD(TAG, "Async wakeup already in progress");
		return;
	}
	
	if (wakeup_task_handle != NULL) {
		ESP_LOGW(TAG, "Wakeup task handle not null, cleaning up");
		wakeup_task_handle = NULL;
	}
	
	// Set flag BEFORE creating task so callers know wakeup is starting
	wakeup_in_progress = true;
	wakeup_early_timeout_sent = false;
	
	ESP_LOGI(TAG, "Starting async wakeup task...");
	xTaskCreate(async_wakeup_task, "async_wake", 4096, NULL, tskIDLE_PRIORITY + 8, &wakeup_task_handle);
}

// Helper: Handle explicit WAKEUP command logic
static void handle_wakeup_request(const uint8_t *packet, int len, uint32_t timeout_ms, 
                                int *n, int *rssi, int *tries, bool *early_timeout_sent) {
	bool asleep = pump_is_asleep();
	
	// If pump is already awake, just send one wakeup packet and get quick response
	if (!asleep) {
		ESP_LOGI(TAG, "WAKEUP: Pump already awake, sending quick wakeup");
		*tries = 1;
		
		// Need radio lock for quick TX/RX
		if (!radio_lock(pdMS_TO_TICKS(1000))) {
			ESP_LOGW(TAG, "WAKEUP: Radio busy");
			*n = 0;
			return;
		}
		
		display_set_radio_active(true);
		
		// Prepare and send single wakeup packet
		uint8_t wakeup_pkt[7] = { 0xA7, 0x90, 0x75, 0x91, 0x5D, 0x00, 0x00 };
		wakeup_pkt[6] = crc8(wakeup_pkt, 6);
		int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
		
		transmit(pkt_buf, tx_len);
		*n = receive(rx_buf.packet, sizeof(rx_buf.packet), 200);
		
		if (*n > 0) {
			*rssi = read_rssi();
			pump_awake = true;
			last_pump_comm_time = esp_timer_get_time();
		}
		
		display_set_radio_active(false);
		radio_unlock();
		return;
	}
	
	// Check if async wakeup is already running (second command from iAPS)
	if (wakeup_in_progress) {
		ESP_LOGI(TAG, "WAKEUP: Async wakeup in progress, waiting for completion...");
		
		// Wait for async wakeup to COMPLETE (check every 1 sec)
		while (wakeup_in_progress) {
			vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1 sec
			ESP_LOGD(TAG, "WAKEUP: Still waiting for async wakeup... (pump_awake=%d)", pump_awake);
		}
		
		// Async wakeup finished - check result
		if (pump_awake) {
			ESP_LOGI(TAG, "WAKEUP: Async wakeup completed successfully!");
			
			// Send quick wakeup packet to get response for iAPS
			if (!radio_lock(pdMS_TO_TICKS(1000))) {
				ESP_LOGW(TAG, "WAKEUP: Radio busy after async wakeup");
				*n = 0;
				return;
			}
			
			display_set_radio_active(true);
			
			uint8_t wakeup_pkt[7] = { 0xA7, 0x90, 0x75, 0x91, 0x5D, 0x00, 0x00 };
			wakeup_pkt[6] = crc8(wakeup_pkt, 6);
			int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
			
			transmit(pkt_buf, tx_len);
			*n = receive(rx_buf.packet, sizeof(rx_buf.packet), 200);
			
			if (*n > 0) {
				*rssi = read_rssi();
				last_pump_comm_time = esp_timer_get_time();
				ESP_LOGI(TAG, "WAKEUP: Got response, sending OK to iAPS");
			} else {
				ESP_LOGW(TAG, "WAKEUP: No response after async wakeup, but pump should be awake");
				// Still report success since async wakeup succeeded
			}
			
			display_set_radio_active(false);
			radio_unlock();
			*tries = 1;
			return;
		}
		
		// Async wakeup finished but pump didn't wake
		ESP_LOGW(TAG, "WAKEUP: Async wakeup finished but pump not responding");
		*early_timeout_sent = true; // Signal timeout
		*tries = 1;
		*n = 0;
		return;
	}
	
	// First WAKEUP command - pump is asleep and no async wakeup running
	// Start async wakeup and wait for early timeout (9 seconds)
	ESP_LOGI(TAG, "WAKEUP: Starting async wakeup burst...");
	start_async_wakeup();
	
	// Wait for either:
	// 1. Early timeout at 9 seconds (async task will send it)
	// 2. Pump wakes up before 9 seconds
	// 3. Async wakeup completes
	ESP_LOGI(TAG, "WAKEUP: Waiting for early timeout or pump response...");
	while (wakeup_in_progress && !wakeup_early_timeout_sent && !pump_awake) {
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	
	// Check what happened
	if (pump_awake) {
		// Pump woke up before 9 seconds! Send quick response
		ESP_LOGI(TAG, "WAKEUP: Pump woke up before timeout!");
		
		if (!radio_lock(pdMS_TO_TICKS(1000))) {
			ESP_LOGW(TAG, "WAKEUP: Radio busy");
			*n = 0;
			return;
		}
		
		display_set_radio_active(true);
		
		uint8_t wakeup_pkt[7] = { 0xA7, 0x90, 0x75, 0x91, 0x5D, 0x00, 0x00 };
		wakeup_pkt[6] = crc8(wakeup_pkt, 6);
		int tx_len = encode_4b6b(wakeup_pkt, pkt_buf, 7);
		
		transmit(pkt_buf, tx_len);
		*n = receive(rx_buf.packet, sizeof(rx_buf.packet), 200);
		
		if (*n > 0) {
			*rssi = read_rssi();
			last_pump_comm_time = esp_timer_get_time();
		}
		
		display_set_radio_active(false);
		radio_unlock();
		*tries = 1;
		return;
	}
	
	// Early timeout was sent by async task at 9 seconds
	// Let iAPS retry with second command
	ESP_LOGI(TAG, "WAKEUP: Early timeout sent at 9s, waiting for iAPS retry");
	*early_timeout_sent = true;
	*tries = 1;
	*n = 0;
	
	// Don't send another timeout - async task already sent it
}

// Helper: Standard command retry loop
static void run_standard_loop(send_and_listen_cmd_t *p, int len, int repeat_count, int delay_ms,
                            bool woke_now, int *n, int *rssi, int *tries) {
	
	uint16_t base_preamble = (p->preamble_ms > 0) ? p->preamble_ms : 60;
	if (woke_now && base_preamble < 180) base_preamble = 180;
	
	if (woke_now) vTaskDelay(pdMS_TO_TICKS(20));
	
	for (int i = 0; i <= p->retry_count; i++) {
		if (cancel_current_command) break;
		
		(*tries)++;
		
		// Adaptive preamble
		uint16_t current_preamble = base_preamble;
		if (i > 0) {
			current_preamble += (i * 50);
			if (current_preamble > 300) current_preamble = 300;
		}
		if (i == 0 && current_preamble < 100) current_preamble = 100; // Min start
		
		rfm95_set_preamble_ms(current_preamble);
		
		ESP_LOGD(TAG, "SendAndListen: try %d/%d (preamble=%u, timeout=%lu)", 
				 i+1, p->retry_count+1, current_preamble, (unsigned long)p->timeout_ms);
		
		send(p->packet, len, repeat_count, delay_ms);
		
		if (!cancel_current_command) {
			*n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
			if (*n > 0) {
				*rssi = read_rssi();
				break;
			}
		}
	}
	rfm95_set_preamble_ms(0);
}

static void send_and_listen(const uint8_t *buf, int len)
{
	send_and_listen_cmd_t *p = (send_and_listen_cmd_t *)buf;
	parse_sl_cmd(p);
	uint32_t original_timeout_ms = p->timeout_ms;

	int payload_len = len - (p->packet - (uint8_t *)p);
	uint8_t pump_id[3];
	uint8_t cmd_code;
	bool is_minimed = extract_minimed_info(p->packet, payload_len, pump_id, &cmd_code);
	bool is_wakeup = (is_minimed && cmd_code == 0x5D);
	
	// Fallback: still treat as WAKEUP if the raw 5th byte matches
	if (!is_wakeup && payload_len >= 5 && p->packet[4] == 0x5D) {
		is_wakeup = true;
	}
	
	bool pump_asleep = pump_is_asleep();
	if (pump_asleep) {
		if (is_wakeup) {
			if (p->timeout_ms < WAKEUP_FIRST_TIMEOUT_MS) {
				p->timeout_ms = WAKEUP_FIRST_TIMEOUT_MS;
			}
		} else if (p->timeout_ms == 0) {
			p->timeout_ms = 45000u;
		}
	}

	ESP_LOGD(TAG, "send_and_listen: len %d ch %d/%d repeat %d delay %d timeout %lu (raw %lu) retry %d preamble %u",
			 len, p->send_channel, p->listen_channel, p->repeat_count, p->delay_ms,
			 (unsigned long)p->timeout_ms, (unsigned long)original_timeout_ms,
			 p->retry_count, (unsigned)p->preamble_ms);

	int repeat_count = (p->repeat_count == 0xFF) ? 0 : p->repeat_count;
	int delay_ms = p->delay_ms;

	int64_t t0 = esp_timer_get_time();
	int n = 0;
	int rssi = 0;
	int tries_used = 0;
	bool early_timeout_sent = false;

	// Handle wakeup request (may use async task)
	if (is_wakeup) {
		// For WAKEUP, we handle without holding radio lock initially
		// because async wakeup task needs the lock
		handle_wakeup_request(p->packet, payload_len, p->timeout_ms, &n, &rssi, &tries_used, &early_timeout_sent);
		
		// If early timeout sent, respond now
		if (early_timeout_sent) {
			ESP_LOGI(TAG, "WAKEUP: Early timeout, sending RX_TIMEOUT to phone");
			// Already sent in async task or handle_wakeup_request
			int64_t dt = esp_timer_get_time() - t0;
			ESP_LOGD(TAG, "CmdSendAndListen (WAKEUP async) took %lld ms", (long long)(dt / 1000));
			return;
		}
		
		// Got response, send it
		if (n > 0 && !cancel_current_command) {
			rx_common(n, rssi);
		}
		
		int64_t dt = esp_timer_get_time() - t0;
		if (dt > 2 * SECONDS) {
			ESP_LOGW(TAG, "CmdSendAndListen (WAKEUP) took %lld ms (tries=%d, n=%d)", (long long)(dt / 1000), tries_used, n);
		}
		return;
	}
	
	// Non-wakeup command - check if pump is asleep or wakeup in progress
	if (pump_is_asleep() || wakeup_in_progress) {
		// Start async wakeup if not already running
		if (!wakeup_in_progress) {
			ESP_LOGI(TAG, "CMD: Pump is asleep, starting async wakeup...");
			start_async_wakeup();
		} else {
			ESP_LOGI(TAG, "CMD: Async wakeup already in progress, waiting...");
		}
		
		// ALWAYS wait for async wakeup to COMPLETE (don't skip early!)
		// This ensures we execute the command when pump wakes up
		int wait_count = 0;
		while (wakeup_in_progress && wait_count < 300) { // max 30 sec
			vTaskDelay(pdMS_TO_TICKS(100));
			wait_count++;
			
			// Check if pump woke up - exit loop early
			if (pump_awake) {
				ESP_LOGI(TAG, "CMD: Pump woke up! (waited %d ms)", wait_count * 100);
				break;
			}
		}
		
		// Check result
		if (pump_awake) {
			ESP_LOGI(TAG, "CMD: Pump is awake, executing command");
			wakeup_early_timeout_sent = false;
		} else {
			ESP_LOGW(TAG, "CMD: Wakeup failed (timeout after %d ms)", wait_count * 100);
			send_code(RESPONSE_CODE_RX_TIMEOUT);
			return;
		}
	} else {
		// Pump is awake - reset early timeout flag so commands execute normally
		if (wakeup_early_timeout_sent) {
			ESP_LOGD(TAG, "CMD: Pump awake, clearing early_timeout flag");
			wakeup_early_timeout_sent = false;
		}
	}
	
	// Now take radio lock for the actual command
	if (!radio_lock(portMAX_DELAY)) {
		ESP_LOGW(TAG, "send_and_listen: radio mutex unavailable");
		return;
	}

	display_set_radio_active(true);
	
	// NOTE: Do NOT use quick_model_probe here!
	// If async_wakeup succeeded, pump is already awake.
	// quick_model_probe sends GetModel which causes "Unexpected response GetPumpModel(722)" error in iAPS.
	bool woke_now = false;
	
	if (woke_now && p->timeout_ms < 500) {
		ESP_LOGD(TAG, "Bumping timeout after wake to 500ms");
		p->timeout_ms = 500;
	}
	
	run_standard_loop(p, payload_len, repeat_count, delay_ms, woke_now, &n, &rssi, &tries_used);

	if (!cancel_current_command) {
		rx_common(n, rssi);
	}

	display_set_radio_active(false);
	radio_unlock();

	int64_t dt = esp_timer_get_time() - t0;
	if (dt > 2 * SECONDS) {
		ESP_LOGW(TAG, "CmdSendAndListen took %lld ms (tries=%d, n=%d)", (long long)(dt / 1000), tries_used, n);
	}
}

static void pump_bg_listen_task(void *unused)
{
	ESP_LOGI(TAG, "pump_bg_listen: started (interval=%dms timeout=%dms)",
			 PUMP_BG_LISTEN_INTERVAL_MS, PUMP_BG_LISTEN_TIMEOUT_MS);

	display_set_radio_active(false);

	uint32_t loops = 0;

	for (;;)
	{
		loops++;
		vTaskDelay(pdMS_TO_TICKS(PUMP_BG_LISTEN_INTERVAL_MS));

		// Don't interfere with active radio operations.
		if (!radio_lock(0))
		{
			continue;
		}

		// Temporarily tune to the pump frequency for passive listening.
		uint32_t prev_freq = read_frequency();
		if (prev_freq != PUMP_FREQUENCY)
		{
			set_frequency(PUMP_FREQUENCY);
		}

		int n = receive(rx_buf.packet, sizeof(rx_buf.packet), PUMP_BG_LISTEN_TIMEOUT_MS);
		int rssi = read_rssi();

		if (prev_freq != PUMP_FREQUENCY)
		{
			set_frequency(prev_freq);
		}

		if (n > 0)
		{
			ESP_LOGD(TAG, "pump_bg_listen: got packet len=%d rssi=%d", n, rssi);
			set_pump_rssi(rssi);
		}
		else if ((loops % 6) == 0)
		{
			ESP_LOGD(TAG, "pump_bg_listen: no packets yet (listening on %lu Hz, window=%dms)",
					 (unsigned long)PUMP_FREQUENCY, PUMP_BG_LISTEN_TIMEOUT_MS);
		}

		radio_unlock();
	}
}

__attribute__((unused))
static void pump_active_probe_task(void *unused)
{
	ESP_LOGI(TAG, "pump_active_probe: started (interval=%dms)", PUMP_ACTIVE_PROBE_INTERVAL_MS);

	vTaskDelay(pdMS_TO_TICKS(PUMP_ACTIVE_PROBE_START_DELAY_MS));

	uint32_t loops = 0;
	int fast_tries_left = PUMP_ACTIVE_PROBE_FAST_TRIES;
	bool had_success = false;
	for (;;)
	{
		TickType_t wait_ticks;
		if (!had_success && fast_tries_left > 0)
		{
			wait_ticks = pdMS_TO_TICKS(PUMP_ACTIVE_PROBE_FAST_INTERVAL_MS);
		}
		else
		{
			wait_ticks = pdMS_TO_TICKS(PUMP_ACTIVE_PROBE_INTERVAL_MS);
		}

		// Sleep until next scheduled attempt or until explicitly requested (e.g. display button).
		uint32_t notif = 0;
		(void)xTaskNotifyWait(0, UINT32_MAX, &notif, wait_ticks);

		loops++;

		// Only probe when BLE is connected
		if (!ble_is_connected())
		{
			continue;
		}

		// Don't interfere with active radio operations.
		if (!radio_lock(0))
		{
			continue;
		}

		uint32_t prev_freq = read_frequency();
		if (prev_freq != PUMP_FREQUENCY)
		{
			set_frequency(PUMP_FREQUENCY);
		}

		// Longer preamble helps with a sleeping pump radio.
		rfm95_set_preamble_ms(PUMP_ACTIVE_PROBE_PREAMBLE_MS);
		bool woke = pump_wakeup();

		status_t st;
		int rc = pump_get_status(&st);
		int rssi = read_rssi();

		rfm95_set_preamble_ms(0);

		if (prev_freq != PUMP_FREQUENCY)
		{
			set_frequency(prev_freq);
		}

		radio_unlock();

		if (rc == 0)
		{
			ESP_LOGI(TAG, "pump_active_probe: ok (rssi=%d status=0x%02x)", rssi, (unsigned)st.code);
			// Successful communication - reset failure counters
			consecutive_rx_failures = 0;
			rfm95_reset_failure_count();
			// -127 is a common sentinel (last_rssi=0xFF) meaning "no valid RSSI".
			// Don't overwrite a previous good reading with it.
			if (rssi != -127)
			{
				set_pump_rssi(rssi);
			}
			had_success = true;
		}
		else if ((loops % 1) == 0)
		{
			ESP_LOGD(TAG, "pump_active_probe: no response yet (woke=%d)", woke ? 1 : 0);
		}

		if (!had_success && fast_tries_left > 0)
		{
			fast_tries_left--;
		}
	}
}

int gnarl_request_pump_probe_isr(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	if (pump_probe_handle != NULL)
	{
		xTaskNotifyFromISR(pump_probe_handle, 1, eSetValueWithOverwrite, &higher_priority_task_woken);
	}
	return higher_priority_task_woken == pdTRUE ? 1 : 0;
}

static uint8_t fr[3];

static inline bool valid_frequency(uint32_t f)
{
	if (863 * MHz <= f && f <= 870 * MHz)
	{
		return true;
	}
	if (910 * MHz <= f && f <= 920 * MHz)
	{
		return true;
	}
	return false;
}

// Change the radio frequency if the current register values make sense.
static void check_frequency(void)
{
	uint32_t f = ((uint32_t)fr[0] << 16) + ((uint32_t)fr[1] << 8) + ((uint32_t)fr[2]);
	uint32_t freq = (uint32_t)(((uint64_t)f * 24 * MHz) >> 16);
	if (valid_frequency(freq))
	{
		ESP_LOGD(TAG, "setting frequency to %lu Hz", freq);
		set_frequency(freq);
	}
	else
	{
		ESP_LOGD(TAG, "invalid frequency (%lu Hz)", freq);
	}
}

static void update_register(const uint8_t *buf, int len)
{
	// AAPS sends 2 bytes, Loop sends 10
	if (len < 2)
	{
		ESP_LOGE(TAG, "update_register: len = %d", len);
		return;
	}
	uint8_t addr = buf[0];
	uint8_t value = buf[1];
	ESP_LOGD(TAG, "update_register: addr %02X value %02X", addr, value);
	switch (addr)
	{
	case 0x09 ... 0x0B:
		fr[addr - 0x09] = value;
		// set_frequency() hits SPI; serialize with other radio users (pump probe, bg listen, etc).
		if (!radio_lock(portMAX_DELAY))
		{
			ESP_LOGW(TAG, "update_register: radio mutex unavailable");
			break;
		}
		check_frequency();
		radio_unlock();
		break;
	default:
		ESP_LOGD(TAG, "update_register: addr %02X ignored", addr);
		break;
	}
	send_code(RESPONSE_CODE_SUCCESS);
}

static void led_mode(const uint8_t *buf, int len)
{
	if (len < 2)
	{
		ESP_LOGE(TAG, "led_mode: len = %d", len);
		return;
	}
	ESP_LOGD(TAG, "led_mode: led %02X mode %02X", buf[0], buf[1]);
	send_code(RESPONSE_CODE_SUCCESS);
}

static void read_register(const uint8_t *buf, int len)
{
	uint8_t addr = buf[0];
	uint8_t value = 0;
	switch (addr)
	{
	case 0x09 ... 0x0B:
		value = fr[addr - 0x09];
		break;
	}
	ESP_LOGD(TAG, "read_register: addr %02X value %02X", addr, value);
	send_bytes(&value, sizeof(value));
}

static void set_sw_encoding(const uint8_t *buf, int len)
{
	ESP_LOGD(TAG, "encoding mode %02X", buf[0]);
	switch (buf[0])
	{
	case ENCODING_NONE:
	case ENCODING_4B6B:
		encoding_type = buf[0];
		break;
	default:
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	send_code(RESPONSE_CODE_SUCCESS);
}

static void send_stats()
{
	statistics.uptime = xTaskGetTickCount();
	// From rfm95:
	statistics.packet_rx_count = rx_packet_count();
	statistics.packet_tx_count = tx_packet_count();
	// Use placeholders for battery info (keeps protocol size stable).
	// placeholder0: battery voltage in mV, placeholder1: battery percent (0..100).
	statistics.placeholder0 = (uint16_t)get_battery_voltage();
	statistics.placeholder1 = (uint16_t)battery_percent(get_battery_voltage());
	ESP_LOGD(TAG, "send_stats len %d uptime %lu rx %d tx %d",
			 sizeof(statistics), statistics.uptime,
			 statistics.packet_rx_count, statistics.packet_tx_count);
	
	// Use local copies to avoid taking address of packed struct members
	uint32_t uptime = statistics.uptime;
	uint16_t rx_count = statistics.packet_rx_count;
	uint16_t tx_count = statistics.packet_tx_count;
	uint16_t ph0 = statistics.placeholder0;
	uint16_t ph1 = statistics.placeholder1;
	reverse_four_bytes(&uptime);
	reverse_two_bytes(&rx_count);
	reverse_two_bytes(&tx_count);
	reverse_two_bytes(&ph0);
	reverse_two_bytes(&ph1);
	statistics.uptime = uptime;
	statistics.packet_rx_count = rx_count;
	statistics.packet_tx_count = tx_count;
	statistics.placeholder0 = ph0;
	statistics.placeholder1 = ph1;
	
	send_bytes((const uint8_t *)&statistics, sizeof(statistics));
}

// This is called from the ble task.
void rfspy_command(const uint8_t *buf, int count, int rssi)
{
	if (count == 0)
	{
		ESP_LOGE(TAG, "rfspy_command: count == 0");
		return;
	}
	if (buf[0] != count - 1 || count == 1)
	{
		ESP_LOGE(TAG, "rfspy_command: length = %d, byte 0 == %d", count, buf[0]);
		return;
	}
	rfspy_cmd_t cmd = buf[1];

	// Log ALL incoming BLE commands at INFO level for debugging
	ESP_LOGI(TAG, ">>> BLE CMD: 0x%02X (%s) Len: %d", cmd, 
			 cmd == 0x00 ? "GetState" :
			 cmd == 0x01 ? "GetVersion" :
			 cmd == 0x02 ? "GetPacket" :
			 cmd == 0x03 ? "SendPacket" :
			 cmd == 0x04 ? "SendAndListen" :
			 cmd == 0x05 ? "UpdateRegister" :
			 cmd == 0x06 ? "Reset" :
			 cmd == 0x07 ? "LED" :
			 cmd == 0x08 ? "ReadRegister" :
			 cmd == 0x09 ? "SetModeReg" :
			 cmd == 0x0A ? "SetSWEncoding" :
			 cmd == 0x0B ? "SetPreamble" :
			 cmd == 0x0C ? "ResetRadio" :
			 cmd == 0x0D ? "GetStatistics" : "Unknown",
			 count);

	// GetPacket is used by Loop to wait for MySentry packets.
	// It is fine to ignore subsequent calls if in_get_packet is true
	// because we are already looping in the code to send a response.
	// The commands and responses do not seem to have a sequence number.
	if ((cmd == CmdGetPacket) && in_get_packet)
	{
		ESP_LOGD(TAG, "ignoring CmdGetPacket while GetPacket is active");
		return;
	}

	// Do this before enqueueing, otherwise there is a risk of self-inflicting
	// the notification due to concurrency.
	if (uxQueueMessagesWaiting(request_queue) > 0)
	{
		if (in_get_packet)
		{
			ESP_LOGD(TAG, "rfspy_command: xTaskNotifyGive");
			xTaskNotifyGive(gnarl_loop_handle);
		}
	}
	rfspy_request_t req = {
		.command = cmd,
		.length = count - 2,
		.rssi = rssi,
	};
	memcpy(req.data, buf + 2, req.length);
	if (!xQueueSend(request_queue, &req, 0))
	{
		ESP_LOGE(TAG, "rfspy_command: cannot queue request for command %d", cmd);
		statistics.rx_fifo_overflow += 1;
		return;
	}
	ESP_LOGI(TAG, ">>> BLE queued 0x%02X, queue=%d", cmd, uxQueueMessagesWaiting(request_queue));
}

static void gnarl_loop(void *unused)
{
	ESP_LOGD(TAG, "starting gnarl_loop");
	for (;;)
	{
		rfspy_request_t req;
		xQueueReceive(request_queue, &req, portMAX_DELAY);

		// New command begins; clear any previous cancellation request.
		cancel_current_command = false;

		switch (req.command)
		{
		case CmdGetState:
			ESP_LOGI(TAG, "<<< EXEC: CmdGetState");
			send_bytes((const uint8_t *)STATE_OK, strlen(STATE_OK));
			break;
		case CmdGetVersion:
			ESP_LOGI(TAG, "<<< EXEC: CmdGetVersion");
			send_bytes((const uint8_t *)SUBG_RFSPY_VERSION, strlen(SUBG_RFSPY_VERSION));
			break;
		case CmdGetPacket:
			ESP_LOGI(TAG, "<<< EXEC: CmdGetPacket");
			get_packet(req.data, req.length);
			break;
		case CmdSendPacket:
			ESP_LOGI(TAG, "<<< EXEC: CmdSendPacket");
			send_packet(req.data, req.length);
			break;
		case CmdSendAndListen:
			ESP_LOGI(TAG, "<<< EXEC: CmdSendAndListen");
			send_and_listen(req.data, req.length);
			break;
		case CmdUpdateRegister:
			ESP_LOGD(TAG, "CmdUpdateRegister");
			update_register(req.data, req.length);
			break;
		case CmdLED:
			ESP_LOGD(TAG, "CmdLED");
			led_mode(req.data, req.length);
			break;
		case CmdReadRegister:
			ESP_LOGD(TAG, "CmdReadRegister");
			read_register(req.data, req.length);
			break;
		case CmdSetSWEncoding:
			ESP_LOGD(TAG, "CmdSetSWEncoding");
			set_sw_encoding(req.data, req.length);
			break;
		case CmdResetRadioConfig:
			ESP_LOGD(TAG, "CmdResetRadioConfig");
			send_code(RESPONSE_CODE_SUCCESS);
			break;
		case CmdGetStatistics:
			ESP_LOGD(TAG, "CmdGetStatistics");
			send_stats();
			break;
		default:
			ESP_LOGE(TAG, "unimplemented rfspy command %d", req.command);
			break;
		}
		set_ble_rssi(req.rssi);
	}
}

static void initial_pump_probe_task(void *unused) {
    ESP_LOGI(TAG, "Initial Probe: Waiting for radio init...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for radio setup

    if (!radio_lock(portMAX_DELAY)) {
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Initial Probe: Checking pump state (GetModel)...");

    // Parse PUMP_ID string to bytes
    const char *id_str = PUMP_ID;
    uint8_t id[3];
    if (strlen(id_str) == 6) {
        char tmp[3] = {0};
        for (int i = 0; i < 3; i++) {
            tmp[0] = id_str[i*2];
            tmp[1] = id_str[i*2+1];
            id[i] = (uint8_t)strtol(tmp, NULL, 16);
        }
    } else {
        ESP_LOGE(TAG, "Invalid PUMP_ID format");
        radio_unlock();
        vTaskDelete(NULL);
        return;
    }

    // Construct dummy packet for quick_model_probe
    // It expects [?, ID0, ID1, ID2, ...]
    uint8_t dummy_pkt[10];
    dummy_pkt[1] = id[0];
    dummy_pkt[2] = id[1];
    dummy_pkt[3] = id[2];

    bool awake = quick_model_probe(dummy_pkt, 4, 200); // 200ms timeout

    if (awake) {
        ESP_LOGI(TAG, "Initial Probe: Pump is AWAKE");
    } else {
        ESP_LOGI(TAG, "Initial Probe: Pump is ASLEEP (no response)");
    }

    radio_unlock();
    vTaskDelete(NULL);
}

void start_gnarl_task(void)
{
	// Configure Medtronic pump ID for active probe and other optional uses.
	pump_set_id(PUMP_ID);

	radio_mutex = xSemaphoreCreateMutex();
	if (radio_mutex == NULL)
	{
		ESP_LOGW(TAG, "radio mutex create failed");
	}

	request_queue = xQueueCreate(QUEUE_LENGTH, sizeof(rfspy_request_t));
	// Keep this task responsive, but avoid starving NimBLE/BT tasks.
	// Too-high priority here can lead to periodic BLE disconnects under load.
	xTaskCreate(gnarl_loop, "gnarl", 4096, 0, tskIDLE_PRIORITY + 10, &gnarl_loop_handle);

	// Background short listen to populate pump RSSI after reboot.
	xTaskCreate(pump_bg_listen_task, "pump_bg", 3072, 0, tskIDLE_PRIORITY + 6, NULL);

	// One-time initial probe to check pump state
	xTaskCreate(initial_pump_probe_task, "init_probe", 3072, 0, tskIDLE_PRIORITY + 6, NULL);

	// Active probe disabled - causes too much radio traffic
	// Loop/phone commands provide enough activity to keep pump RSSI updated
	pump_probe_handle = NULL;
}
