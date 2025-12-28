#include <unistd.h>

#define TAG		"rfm95"

#include <esp_log.h>
#include <esp_err.h>
#include <esp_pm.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "module.h"
#include "rfm95.h"
#include "spi.h"

#define MILLISECOND	1000

// Radio bitrate is configured to ~16384 bps in rfm95_init().
#define BITRATE_BPS		16384

#define DEFAULT_PREAMBLE_BYTES	0x18

// Radio failure tracking for auto-reset
static volatile int radio_failures = 0;
#define RADIO_FAILURE_THRESHOLD 5

static uint16_t current_preamble_bytes = DEFAULT_PREAMBLE_BYTES;

// PM lock to prevent light sleep during radio operations
static esp_pm_lock_handle_t radio_pm_lock = NULL;

static void radio_pm_lock_acquire(void) {
	if (radio_pm_lock != NULL) {
		esp_pm_lock_acquire(radio_pm_lock);
	}
}

static void radio_pm_lock_release(void) {
	if (radio_pm_lock != NULL) {
		esp_pm_lock_release(radio_pm_lock);
	}
}

void rfm95_set_tx_power_dbm(int8_t dbm) {
	// PA_BOOST output: 2..17 dBm range (SX1276 typical).
	if (dbm < 2) {
		dbm = 2;
	}
	if (dbm > 17) {
		dbm = 17;
	}
	uint8_t output_power = (uint8_t)(dbm - 2) & 0x0F;
	write_register(REG_PA_CONFIG, PA_BOOST | PA_MAX_POWER_7 | output_power);
}

void rfm95_set_preamble_bytes(uint16_t preamble_bytes) {
	if (preamble_bytes == 0) {
		preamble_bytes = DEFAULT_PREAMBLE_BYTES;
	}
	write_register(REG_PREAMBLE_MSB, (uint8_t)(preamble_bytes >> 8));
	write_register(REG_PREAMBLE_LSB, (uint8_t)(preamble_bytes & 0xFF));
	current_preamble_bytes = preamble_bytes;
}

void rfm95_set_preamble_ms(uint16_t preamble_ms) {
	if (preamble_ms == 0) {
		rfm95_set_preamble_bytes(DEFAULT_PREAMBLE_BYTES);
		return;
	}
	// Convert milliseconds to preamble bytes: bytes ~= bitrate_bps * ms / 8000.
	uint32_t bytes = ((uint32_t)BITRATE_BPS * (uint32_t)preamble_ms + 7999) / 8000;
	if (bytes < DEFAULT_PREAMBLE_BYTES) {
		bytes = DEFAULT_PREAMBLE_BYTES;
	}
	if (bytes > 0xFFFF) {
		bytes = 0xFFFF;
	}
	rfm95_set_preamble_bytes((uint16_t)bytes);
}

// The FIFO_THRESHOLD value should allow a maximum-sized packet to be
// written in two bursts, but be large enough to avoid fifo underflow.
#define FIFO_THRESHOLD	20

#ifndef USE_POLLING
static void install_isr(void);
#endif

static inline uint8_t read_mode(void) {
	return read_register(REG_OP_MODE) & OP_MODE_MASK;
}

#define MAX_WAIT	1000

static void set_mode(uint8_t mode) {
	uint8_t cur_mode = read_mode();
	if (mode == cur_mode) {
		ESP_LOGD(TAG, "RADIO: Already in mode %d, skipping", mode);
		return;
	}
	ESP_LOGD(TAG, "RADIO: Changing mode %d -> %d", cur_mode, mode);
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | mode);
	if (cur_mode == MODE_SLEEP) {
		usleep(100);
	}
	for (int w = 0; w < MAX_WAIT; w++) {
		cur_mode = read_mode();
		if (cur_mode == mode) {
			// Success - reset failure count on any successful mode change
			radio_failures = 0;
			return;
		}
		// Avoid tight spinning; keeps higher-level stacks (e.g. BLE) responsive.
		taskYIELD();
	}
	// Mode change timeout - increment failure counter
	radio_failures++;
	ESP_LOGE(TAG, "set_mode(%d) timeout in mode %d (failure #%d)", mode, cur_mode, radio_failures);
}

static inline void set_mode_sleep(void) {
	set_mode(MODE_SLEEP);
}

static inline void set_mode_standby(void) {
	set_mode(MODE_STDBY);
}

static inline void set_mode_receive(void) {
	set_mode(MODE_RX);
}

static inline void set_mode_transmit(void) {
	set_mode(MODE_TX);
}

static inline void sequencer_stop(void) {
	write_register(REG_SEQ_CONFIG_1, SEQUENCER_STOP);
}

// Reset the radio device.  See section 7.2.2 of data sheet.
// NOTE: For Heltec/TTGO boards we keep RST as OUTPUT and drive HIGH explicitly,
// because the internal pull-up may be too weak or absent on these modules.

void rfm95_reset(void) {
	ESP_LOGD(TAG, "RADIO: Resetting RFM95 module");
	
	// Configure RST as output
	gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
	
	// Drive RST HIGH first to ensure known state
	gpio_set_level(LORA_RST, 1);
	usleep(10 * MILLISECOND);
	
	// Pull RESET LOW to reset the module (datasheet minimum 100us)
	gpio_set_level(LORA_RST, 0);
	usleep(1 * MILLISECOND);  // Hold LOW for 1ms
	
	// Drive RST HIGH to release reset (keep as OUTPUT, don't rely on pull-up!)
	gpio_set_level(LORA_RST, 1);
	
	// Wait for module to stabilize (datasheet says 5ms, use 50ms for safety)
	usleep(50 * MILLISECOND);
	ESP_LOGD(TAG, "RADIO: Reset complete, waiting for module to stabilize");
}

static volatile int rx_packets;

int rx_packet_count(void) {
	return rx_packets;
}

static volatile int tx_packets;

int tx_packet_count(void) {
	return tx_packets;
}

// Radio failure tracking functions (variable defined at top of file)
int rfm95_get_failure_count(void) {
	return radio_failures;
}

void rfm95_reset_failure_count(void) {
	radio_failures = 0;
}

bool rfm95_needs_reset(void) {
	return radio_failures >= RADIO_FAILURE_THRESHOLD;
}

// Soft reinit - reconfigure registers WITHOUT hardware reset
// Use this for recovery, as hardware reset can kill the chip on some boards
void rfm95_soft_reinit(void) {
	ESP_LOGD(TAG, "RADIO: Soft reinit (no hardware reset)");
	
	// Reset failure counter first
	radio_failures = 0;
	
	// Force standby mode to stop any ongoing operation
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | MODE_STDBY);
	usleep(1 * MILLISECOND);
	
	// Clear FIFO
	write_register(REG_IRQ_FLAGS_2, FIFO_OVERRUN);
	
	// Re-apply all configuration
	write_register(REG_PA_CONFIG, 0x8F);
	write_register(REG_LNA, 0x23);
	write_register(REG_DIO_MAPPING_1, 3 << DIO2_MAPPING_SHIFT);
	
	// Force sleep mode twice (required for FSK/OOK mode change)
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | MODE_SLEEP);
	usleep(100);
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | MODE_SLEEP);
	usleep(100);
	
	// Bitrate
	write_register(REG_BITRATE_MSB, 0x07);
	write_register(REG_BITRATE_LSB, 0xA1);
	
	// RSSI config
	write_register(REG_RSSI_CONFIG, 5);
	
	// RX bandwidth
	write_register(REG_RX_BW, (1 << RX_BW_MANT_SHIFT) | 1);
	
	// Preamble
	rfm95_set_preamble_bytes(DEFAULT_PREAMBLE_BYTES);
	
	// Sync word config
	write_register(REG_SYNC_CONFIG, SYNC_ON | 3);
	write_register(REG_SYNC_VALUE_1, 0xFF);
	write_register(REG_SYNC_VALUE_2, 0x00);
	write_register(REG_SYNC_VALUE_3, 0xFF);
	write_register(REG_SYNC_VALUE_4, 0x00);
	
	// Packet format
	write_register(REG_PACKET_CONFIG_1, PACKET_FORMAT_FIXED);
	write_register(REG_PAYLOAD_LENGTH, 0);
	write_register(REG_PACKET_CONFIG_2, PACKET_MODE | 0);
	
	// Go to standby
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | MODE_STDBY);
	usleep(1 * MILLISECOND);
	
	int version = read_version();
	ESP_LOGI(TAG, "RADIO: Soft reinit complete, Version: 0x%02X", version);
}

void rfm95_init(void) {
	ESP_LOGI(TAG, "RADIO: Initializing RFM95 radio module");
	spi_init();
	ESP_LOGD(TAG, "RADIO: SPI initialized");
	rfm95_reset();
	ESP_LOGD(TAG, "RADIO: Reset complete");

	// Enable PA_BOOST for TTGO/Heltec LoRa32 boards (required for TX)
	// Using exact same value as original GNARL: 0x8F
	write_register(REG_PA_CONFIG, 0x8F);

	// Enable LNA with max gain and boost (critical for RX sensitivity!)
	write_register(REG_LNA, 0x23);

	gpio_set_direction(LORA_DIO2, GPIO_MODE_INPUT);
	gpio_set_intr_type(LORA_DIO2, GPIO_INTR_POSEDGE);
	// Interrupt on LORA_DIO2 when SyncMatch occurs.
	write_register(REG_DIO_MAPPING_1, 3 << DIO2_MAPPING_SHIFT);

#ifndef USE_POLLING
	// Ensure the ISR handler is ready before the first receive(). This avoids
	// one-time latency and prevents late ISR setup during active radio ops.
	install_isr();
#endif

	// Must be in Sleep mode first before the second call can change to FSK/OOK mode.
	set_mode_sleep();
	set_mode_sleep();

	// Ideal bit rate is 16384 bps; this works out to 16385 bps.
	write_register(REG_BITRATE_MSB, 0x07);
	write_register(REG_BITRATE_LSB, 0xA1);

	// Use 64 samples for RSSI.
	write_register(REG_RSSI_CONFIG, 5);

	// 200 kHz channel bandwidth (mantissa = 20, exp = 1)
	write_register(REG_RX_BW, (1 << RX_BW_MANT_SHIFT) | 1);

	// Make sure enough preamble bytes are sent.
	rfm95_set_preamble_bytes(DEFAULT_PREAMBLE_BYTES);

	// Use 4 bytes for Sync word.
	write_register(REG_SYNC_CONFIG, SYNC_ON | 3);

	// Sync word.
	write_register(REG_SYNC_VALUE_1, 0xFF);
	write_register(REG_SYNC_VALUE_2, 0x00);
	write_register(REG_SYNC_VALUE_3, 0xFF);
	write_register(REG_SYNC_VALUE_4, 0x00);

	// Use unlimited length packet format (data sheet section 4.2.13.2).
	write_register(REG_PACKET_CONFIG_1, PACKET_FORMAT_FIXED);
	write_register(REG_PAYLOAD_LENGTH, 0);
	write_register(REG_PACKET_CONFIG_2, PACKET_MODE | 0);

	int version = read_version();
	ESP_LOGI(TAG, "RADIO: RFM95 Version: 0x%02X", version);
	
	// Create PM lock to prevent light sleep during radio operations
	if (radio_pm_lock == NULL) {
		esp_err_t err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "radio", &radio_pm_lock);
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "Failed to create radio PM lock: %s", esp_err_to_name(err));
			radio_pm_lock = NULL;
		}
	}
}

static inline bool fifo_empty(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_EMPTY) != 0;
}

static inline bool fifo_full(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_FULL) != 0;
}

static inline bool fifo_threshold_exceeded(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_LEVEL) != 0;
}

static inline void clear_fifo(void) {
	write_register(REG_IRQ_FLAGS_2, FIFO_OVERRUN);
}

static inline uint8_t read_fifo_flags(void) {
	return read_register(REG_IRQ_FLAGS_2);
}

static inline void xmit_byte(uint8_t b) {
	write_register(REG_FIFO, b);
}

static inline void xmit(uint8_t* data, int len) {
	write_burst(REG_FIFO, data, len);
}

static bool wait_for_fifo_room(void) {
	for (int w = 0; w < MAX_WAIT; w++) {
		if (!fifo_full()) {
			if (w > 0) {
				ESP_LOGD(TAG, "RADIO TX: FIFO room available after %d waits", w);
			}
			return true;
		}
		// Avoid burning CPU while waiting for FIFO to drain.
		taskYIELD();
	}
	ESP_LOGE(TAG, "RADIO TX: FIFO timeout! Still full after %d waits, flags = %02X", MAX_WAIT, read_fifo_flags());
	sequencer_stop();
	set_mode_sleep();
	return false;
}

static void wait_for_transmit_done(void) {
	uint8_t mode;
	for (int w = 0; w < MAX_WAIT; w++) {
		mode = read_mode();
		if (mode == MODE_STDBY) {
			ESP_LOGD(TAG, "RADIO TX: Transmission completed after %d iterations", w);
			return;
		}
		usleep(1*MILLISECOND);
	}
	ESP_LOGE(TAG, "RADIO TX: Transmission timeout! Still in mode %d", mode);
	sequencer_stop();
	set_mode_sleep();
}

void transmit(uint8_t *buf, int count) {
	ESP_LOGI(TAG, "TX: %d bytes", count);
	if (count > 0) {
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, count, ESP_LOG_DEBUG);
	}
	clear_fifo();
	ESP_LOGD(TAG, "RADIO TX: FIFO cleared");
	set_mode_standby();
	ESP_LOGD(TAG, "RADIO TX: Mode set to standby");
	// Automatically enter Transmit state on FifoLevel interrupt.
	write_register(REG_FIFO_THRESH, TX_START_CONDITION | FIFO_THRESHOLD);
	write_register(REG_SEQ_CONFIG_1, SEQUENCER_START | IDLE_MODE_STANDBY | FROM_START_TO_TX);
	ESP_LOGD(TAG, "RADIO TX: Sequencer configured, FIFO threshold=%d", FIFO_THRESHOLD);
	int avail = FIFO_SIZE;
	for (;;) {
		if (avail > count) {
			avail = count;
		}
		ESP_LOGD(TAG, "RADIO TX: Writing %d bytes to TX FIFO (remaining: %d)", avail, count);
		xmit(buf, avail);
		ESP_LOGD(TAG, "RADIO TX: After xmit: mode = %d", read_mode());
		buf += avail;
		count -= avail;
		if (count == 0) {
			break;
		}
		// Wait until there is room for at least fifoSize - fifoThreshold bytes in the FIFO.
		// Err on the short side here to avoid TXFIFO underflow.
		usleep(FIFO_SIZE / 4 * MILLISECOND);
		for (;;) {
			if (!fifo_threshold_exceeded()) {
				avail = FIFO_SIZE - FIFO_THRESHOLD;
				break;
			}
			// Don't hog the CPU at high task priority.
			taskYIELD();
		}
	}
	if (!wait_for_fifo_room()) {
		return;
	}
	ESP_LOGD(TAG, "RADIO TX: Sending terminating byte");
	xmit_byte(0);
	ESP_LOGD(TAG, "RADIO TX: Waiting for transmission to complete");
	wait_for_transmit_done();
	set_mode_standby();
	tx_packets++;
	ESP_LOGI(TAG, "TX: done (total: %d)", tx_packets);
}

static bool packet_seen(void) {
	bool seen = (read_register(REG_IRQ_FLAGS_1) & SYNC_ADDRESS_MATCH) != 0;
	if (seen) {
		ESP_LOGD(TAG, "RADIO RX: Incoming packet detected (SyncAddress match)");
	}
	return seen;
}

static inline uint8_t recv_byte(void) {
	return read_register(REG_FIFO);
}

static uint8_t last_rssi = 0xFF;

int read_rssi(void) {
	// last_rssi starts at 0xFF before we've captured an RSSI sample.
	// Returning -127 here is misleading; treat it as "unknown".
	if (last_rssi == 0xFF) {
		return 0;
	}
	return -(int)last_rssi / 2;
}

typedef void wait_fn_t(int);

static int rx_common(wait_fn_t wait_fn, uint8_t *buf, int count, int timeout) {
	ESP_LOGI(TAG, "RX: listening (timeout: %d ms)", timeout);
	radio_pm_lock_acquire();  // Prevent light sleep during RX
	gpio_intr_enable(LORA_DIO2);
	set_mode_receive();
	uint8_t mode_after = read_mode();
	if (mode_after != MODE_RX) {
		ESP_LOGE(TAG, "RX: Failed to enter RX mode! Current mode=%d", mode_after);
	}
	if (!packet_seen()) {
		// Stay in RX mode.
		wait_fn(timeout);
		if (!packet_seen()) {
			// Read RSSI even on timeout to get noise floor
			last_rssi = read_register(REG_RSSI);
			uint8_t mode = read_mode();
			uint8_t irq1 = read_register(REG_IRQ_FLAGS_1);
			uint8_t irq2 = read_register(REG_IRQ_FLAGS_2);
			set_mode_sleep();
			gpio_intr_disable(LORA_DIO2);
			radio_pm_lock_release();  // Allow light sleep again
			ESP_LOGW(TAG, "RX: timeout, mode=%d, rssi=0x%02X(%d dBm), irq1=0x%02X, irq2=0x%02X", 
					 mode, last_rssi, read_rssi(), irq1, irq2);
			return 0;
		}
	}
	ESP_LOGD(TAG, "RADIO RX: Packet detected, reading data");
	last_rssi = read_register(REG_RSSI);
	ESP_LOGD(TAG, "RADIO RX: RSSI = %d dBm", read_rssi());
	int n = 0;
	int w = 0;
	ESP_LOGD(TAG, "RADIO RX: Reading bytes from FIFO");
	while (n < count) {
		if (fifo_empty()) {
			usleep(500);
			w++;
			if (w >= MAX_WAIT) {
				ESP_LOGD(TAG, "max RX FIFO wait reached");
				break;
			}
			continue;
		}
		uint8_t b = recv_byte();
		if (b == 0) {
			break;
		}
		buf[n++] = b;
		w = 0;
	}
	set_mode_sleep();
	ESP_LOGD(TAG, "RADIO RX: Mode set to sleep");
	clear_fifo();
	gpio_intr_disable(LORA_DIO2);
	ESP_LOGD(TAG, "RADIO RX: GPIO interrupt disabled");
	if (n > 0) {
		// Remove spurious final byte consisting of just one or two high bits.
		uint8_t b = buf[n-1];
		if (b == 0x80 || b == 0xC0) {
			ESP_LOGD(TAG, "end-of-packet glitch %X with RSSI %d", b >> 6, read_rssi());
			n--;
		}
	}
	if (n > 0) {
		rx_packets++;
		ESP_LOGI(TAG, "RX: %d bytes, RSSI: %d dBm (total: %d)", n, read_rssi(), rx_packets);
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, n, ESP_LOG_DEBUG);
	}
	radio_pm_lock_release();  // Allow light sleep again
	return n;
}

static void sleep_until_interrupt(int timeout) {
	uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
	uint64_t us = (uint64_t)timeout * MILLISECOND;
	esp_sleep_enable_timer_wakeup(us);
	// Wake up on receive interrupt.
	gpio_wakeup_enable(LORA_DIO2, GPIO_INTR_HIGH_LEVEL);
	esp_sleep_enable_gpio_wakeup();
	esp_light_sleep_start();
}

int sleep_receive(uint8_t *buf, int count, int timeout) {
	return rx_common(sleep_until_interrupt, buf, count, timeout);
}

#ifdef USE_POLLING

#define POLL_INTERVAL	5  // milliseconds

static void wait_until_interrupt(int timeout) {
	while (!packet_seen() && timeout > 0) {
		int t = timeout < POLL_INTERVAL ? timeout : POLL_INTERVAL;
		usleep(t * MILLISECOND);
		timeout -= t;
	}
}

#else

static volatile TaskHandle_t rx_waiting_task;

static void rx_interrupt(void *unused) {
	if (rx_waiting_task != 0) {
		vTaskNotifyGiveFromISR(rx_waiting_task, 0);
	}
}

static void install_isr(void) {
	static int isr_installed = 0;
	if (isr_installed) {
		return;
	}
	// The ISR service is installed once at startup (see main/app_main).
	// Only add our handler here.
	esp_err_t err = gpio_isr_handler_add(LORA_DIO2, rx_interrupt, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "gpio_isr_handler_add(%d) failed: %s", (int)LORA_DIO2, esp_err_to_name(err));
		return;
	}
	isr_installed = 1;
}

static void wait_until_interrupt(int timeout) {
	install_isr();
	ESP_LOGD(TAG, "waiting until interrupt");
	rx_waiting_task = xTaskGetCurrentTaskHandle();
	xTaskNotifyWait(0, 0, 0, pdMS_TO_TICKS(timeout));
	rx_waiting_task = 0;
	ESP_LOGD(TAG, "finished waiting");
}

#endif

int receive(uint8_t *buf, int count, int timeout) {
	return rx_common(wait_until_interrupt, buf, count, timeout);
}

uint32_t read_frequency(void) {
	uint8_t frf[3];
	read_burst(REG_FRF_MSB, frf, sizeof(frf));
	uint32_t f = (frf[0] << 16) | (frf[1] << 8) | frf[2];
	return ((uint64_t)f * FXOSC) >> 19;
}

void set_frequency(uint32_t freq_hz) {
	// Override frequency in 868-869 MHz range to 868.35 MHz (pump frequency for EU region)
	// This matches original GNARL behavior - Loop may request different frequencies but pump uses fixed one
	if (freq_hz > 868000000 && freq_hz < 869000000) {
		ESP_LOGD(TAG, "set_frequency: Overriding %lu Hz to 868350000 Hz", (unsigned long)freq_hz);
		freq_hz = 868350000;
	} else {
		ESP_LOGD(TAG, "set_frequency: %lu Hz", (unsigned long)freq_hz);
	}
	uint32_t f = (((uint64_t)freq_hz << 19) + FXOSC/2) / FXOSC;
	uint8_t frf[3];
	frf[0] = f >> 16;
	frf[1] = f >> 8;
	frf[2] = f;
	write_burst(REG_FRF_MSB, frf, sizeof(frf));
}

int read_version(void) {
	return read_register(REG_VERSION);
}

int version_major(int v) {
	return v >> 4;
}

int version_minor(int v) {
	return v & 0xF;
}
