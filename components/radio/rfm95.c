#include <unistd.h>

#define TAG		"rfm95"

#include <esp_log.h>
#include <esp_err.h>
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

static uint16_t current_preamble_bytes = DEFAULT_PREAMBLE_BYTES;

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
	ESP_LOGI(TAG, "RADIO: Changing mode %d -> %d", cur_mode, mode);
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | mode);
	if (cur_mode == MODE_SLEEP) {
		usleep(100);
	}
	for (int w = 0; w < MAX_WAIT; w++) {
		cur_mode = read_mode();
		if (cur_mode == mode) {
			return;
		}
		// Avoid tight spinning; keeps higher-level stacks (e.g. BLE) responsive.
		taskYIELD();
	}
	ESP_LOGI(TAG, "set_mode(%d) timeout in mode %d", mode, cur_mode);
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
// NOTE: the RFM95 requires the reset pin to be in input mode
// except while resetting the chip, unlike the RFM69 for example.

void rfm95_reset(void) {
	ESP_LOGI(TAG, "RADIO: Resetting RFM95 module");
	gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(LORA_RST, 0);
	usleep(100);
	gpio_set_direction(LORA_RST, GPIO_MODE_INPUT);
	usleep(5*MILLISECOND);
	ESP_LOGI(TAG, "RADIO: Reset complete, waiting for module to stabilize");
}

static volatile int rx_packets;

int rx_packet_count(void) {
	return rx_packets;
}

static volatile int tx_packets;

int tx_packet_count(void) {
	return tx_packets;
}

void rfm95_init(void) {
	ESP_LOGI(TAG, "RADIO: Initializing RFM95 radio module");
	spi_init();
	ESP_LOGD(TAG, "RADIO: SPI initialized");
	rfm95_reset();
	ESP_LOGD(TAG, "RADIO: Reset complete");

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

	// use PA_BOOST output pin
	rfm95_set_tx_power_dbm(17);
	int version = read_version();
	ESP_LOGI(TAG, "RADIO: RFM95 Version: 0x%02X", version);
	ESP_LOGI(TAG, "RADIO: RFM95 initialization complete, TX power=17 dBm, Bitrate=16384 bps, BW=200 kHz");
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
			ESP_LOGI(TAG, "RADIO TX: Transmission completed after %d iterations", w);
			return;
		}
		usleep(1*MILLISECOND);
	}
	ESP_LOGE(TAG, "RADIO TX: Transmission timeout! Still in mode %d", mode);
	sequencer_stop();
	set_mode_sleep();
}

void transmit(uint8_t *buf, int count) {
	ESP_LOGI(TAG, "RADIO TX: Starting transmission of %d-byte packet", count);
	if (count > 0) {
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, count, ESP_LOG_INFO);
	}
	clear_fifo();
	ESP_LOGD(TAG, "RADIO TX: FIFO cleared");
	set_mode_standby();
	ESP_LOGD(TAG, "RADIO TX: Mode set to standby");
	// Automatically enter Transmit state on FifoLevel interrupt.
	write_register(REG_FIFO_THRESH, TX_START_CONDITION | FIFO_THRESHOLD);
	write_register(REG_SEQ_CONFIG_1, SEQUENCER_START | IDLE_MODE_STANDBY | FROM_START_TO_TX);
	ESP_LOGI(TAG, "RADIO TX: Sequencer configured, FIFO threshold=%d", FIFO_THRESHOLD);
	int avail = FIFO_SIZE;
	for (;;) {
		if (avail > count) {
			avail = count;
		}
		ESP_LOGI(TAG, "RADIO TX: Writing %d bytes to TX FIFO (remaining: %d)", avail, count);
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
	ESP_LOGI(TAG, "RADIO TX: Transmission complete. Total TX packets: %d", tx_packets);
}

static bool packet_seen(void) {
	bool seen = (read_register(REG_IRQ_FLAGS_1) & SYNC_ADDRESS_MATCH) != 0;
	if (seen) {
		ESP_LOGI(TAG, "RADIO RX: Incoming packet detected (SyncAddress match)");
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
	ESP_LOGI(TAG, "RADIO RX: Starting receive (buffer size: %d, timeout: %d ms)", count, timeout);
	gpio_intr_enable(LORA_DIO2);
	ESP_LOGD(TAG, "RADIO RX: GPIO interrupt enabled on DIO2");
	set_mode_receive();
	ESP_LOGD(TAG, "RADIO RX: Mode set to receive");
	if (!packet_seen()) {
		// Stay in RX mode.
		ESP_LOGD(TAG, "RADIO RX: No packet detected yet, waiting...");
		wait_fn(timeout);
		if (!packet_seen()) {
			set_mode_sleep();
			ESP_LOGI(TAG, "RADIO RX: Receive timeout after %d ms", timeout);
			return 0;
		}
	}
	ESP_LOGI(TAG, "RADIO RX: Packet detected, reading data");
	last_rssi = read_register(REG_RSSI);
	ESP_LOGI(TAG, "RADIO RX: RSSI = %d dBm", read_rssi());
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
		ESP_LOGI(TAG, "RADIO RX: Received %d bytes, RSSI: %d dBm, Total RX packets: %d", n, read_rssi(), rx_packets);
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, n, ESP_LOG_INFO);
	} else {
		ESP_LOGI(TAG, "RADIO RX: No data received");
	}
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
	ESP_LOGI(TAG, "RADIO: Setting frequency to %lu Hz", (unsigned long)freq_hz);
	uint32_t f = (((uint64_t)freq_hz << 19) + FXOSC/2) / FXOSC;
	uint8_t frf[3];
	frf[0] = f >> 16;
	frf[1] = f >> 8;
	frf[2] = f;
	write_burst(REG_FRF_MSB, frf, sizeof(frf));
	ESP_LOGD(TAG, "RADIO: Frequency set complete");
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
