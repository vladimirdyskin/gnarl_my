#include "gnarl.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "4b6b.h"
#include "commands.h"
#include "display.h"
#include "medtronic.h"
#include "rfm95.h"

#define MAX_PARAM_LEN (16)
#define MAX_PACKET_LEN (107)

typedef enum
{
	ENCODING_NONE = 0,
	ENCODING_4B6B = 2,
} encoding_type_t;

static encoding_type_t encoding_type = ENCODING_NONE;

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

static SemaphoreHandle_t radio_mutex;

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
	if (n == 0)
	{
		ESP_LOGD(TAG, "RX: timeout");
		send_code(RESPONSE_CODE_RX_TIMEOUT);
		return;
	}
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
	send_bytes((uint8_t *)&rx_buf, 2 + n);
}

static volatile int in_get_packet = 0;

static void get_packet(const uint8_t *buf, int len)
{
	get_packet_cmd_t *p = (get_packet_cmd_t *)buf;
	reverse_four_bytes(&p->timeout_ms);
	ESP_LOGD(TAG, "get_packet: listen_channel %d timeout_ms %lu",
			 p->listen_channel, p->timeout_ms);
	if (!radio_lock(portMAX_DELAY))
	{
		ESP_LOGW(TAG, "get_packet: radio mutex unavailable");
		return;
	}

	in_get_packet = 1;
	int n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
	rx_common(n, read_rssi());
	in_get_packet = 0;

	radio_unlock();
}

static void send_packet(const uint8_t *buf, int len)
{
	send_packet_cmd_t *p = (send_packet_cmd_t *)buf;
	reverse_two_bytes(&p->delay_ms);
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

static void send_and_listen(const uint8_t *buf, int len)
{
	send_and_listen_cmd_t *p = (send_and_listen_cmd_t *)buf;
	reverse_two_bytes(&p->delay_ms);
	reverse_four_bytes(&p->timeout_ms);
	reverse_two_bytes(&p->preamble_ms);
	ESP_LOGD(TAG, "send_and_listen: len %d send_channel %d repeat_count %d delay_ms %d",
			 len, p->send_channel, p->repeat_count, p->delay_ms);
	ESP_LOGD(TAG, "send_and_listen: listen_channel %d timeout_ms %lu retry_count %d preamble_ms %u",
			 p->listen_channel, p->timeout_ms, p->retry_count, (unsigned)p->preamble_ms);
	len -= (p->packet - (uint8_t *)p);

	int repeat_count = p->repeat_count;
	int delay_ms = p->delay_ms;
	// Some clients use 0xFF as a sentinel for "default". Treat it as "no repeats".
	// Also, delay_ms==0 makes repeated TX effectively back-to-back; avoid huge bursts.
	if (repeat_count == 0xFF || delay_ms == 0) {
		repeat_count = 0;
	}

	if (!radio_lock(portMAX_DELAY))
	{
		ESP_LOGW(TAG, "send_and_listen: radio mutex unavailable");
		return;
	}

	int64_t t0 = esp_timer_get_time();

	int n = 0;
	int rssi = 0;
	int tries_used = 0;

	// Longer preamble often reduces first-command latency by avoiding retries
	// while the pump radio is still waking up.
	rfm95_set_preamble_ms(p->preamble_ms);

	for (int retries = p->retry_count + 1; retries > 0; retries--)
	{
		tries_used++;
		send(p->packet, len, repeat_count, delay_ms);
		n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
		rssi = read_rssi();
		if (n != 0)
		{
			break;
		}
	}

	// Restore default preamble to keep behavior consistent for other commands.
	rfm95_set_preamble_ms(0);

	radio_unlock();

	int64_t dt = esp_timer_get_time() - t0;
	if (dt > 2 * SECONDS)
	{
		ESP_LOGW(TAG, "CmdSendAndListen took %lld ms (tries=%d, send_ch=%u, listen_ch=%u, repeat=%u, delay_ms=%u, preamble_ms=%u, timeout_ms=%lu, rx_len=%d)",
				 (long long)(dt / 1000),
				 tries_used,
				 (unsigned)p->send_channel,
				 (unsigned)p->listen_channel,
				 (unsigned)repeat_count,
				 (unsigned)delay_ms,
				 (unsigned)p->preamble_ms,
				 (unsigned long)p->timeout_ms,
				 n);
	}
	rx_common(n, rssi);
}

static void pump_bg_listen_task(void *unused)
{
	ESP_LOGI(TAG, "pump_bg_listen: started (interval=%dms timeout=%dms)",
			 PUMP_BG_LISTEN_INTERVAL_MS, PUMP_BG_LISTEN_TIMEOUT_MS);

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
		// Client commands (e.g. mmtune scans) may change the active frequency.
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
			ESP_LOGI(TAG, "pump_bg_listen: got packet len=%d rssi=%d", n, rssi);
			set_pump_rssi(rssi);
		}
		else if ((loops % 6) == 0)
		{
			ESP_LOGI(TAG, "pump_bg_listen: no packets yet (listening on %lu Hz, window=%dms)",
					 (unsigned long)PUMP_FREQUENCY, PUMP_BG_LISTEN_TIMEOUT_MS);
		}

		radio_unlock();
	}
}

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
			ESP_LOGI(TAG, "pump_active_probe: no response yet (woke=%d)", woke ? 1 : 0);
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
		ESP_LOGI(TAG, "setting frequency to %lu Hz", freq);
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
		check_frequency();
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
	ESP_LOGD(TAG, "send_stats len %d uptime %lu rx %d tx %d",
			 sizeof(statistics), statistics.uptime,
			 statistics.packet_rx_count, statistics.packet_tx_count);
	reverse_four_bytes(&statistics.uptime);
	reverse_two_bytes(&statistics.packet_rx_count);
	reverse_two_bytes(&statistics.packet_tx_count);
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

	// GetPacket is used by Loop to wait for MySentry packets.
	// It is fine to ignore subsequent calls if in_get_packet is true
	// because we are already looping in the code to send a response.
	// The commands and responses do not seem to have a sequence number.
	if ((cmd == CmdGetPacket) && in_get_packet)
	{
		ESP_LOGI(TAG, "ignoring CmdGetPacket while GetPacket is active");
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
	ESP_LOGD(TAG, "rfspy_command 0x%x, queue length %d", cmd, uxQueueMessagesWaiting(request_queue));
}

static void gnarl_loop(void *unused)
{
	ESP_LOGD(TAG, "starting gnarl_loop");
	for (;;)
	{
		rfspy_request_t req;
		xQueueReceive(request_queue, &req, portMAX_DELAY);

		switch (req.command)
		{
		case CmdGetState:
			ESP_LOGI(TAG, "CmdGetState");
			send_bytes((const uint8_t *)STATE_OK, strlen(STATE_OK));
			break;
		case CmdGetVersion:
			ESP_LOGI(TAG, "CmdGetVersion");
			send_bytes((const uint8_t *)SUBG_RFSPY_VERSION, strlen(SUBG_RFSPY_VERSION));
			break;
		case CmdGetPacket:
			ESP_LOGI(TAG, "CmdGetPacket");
			get_packet(req.data, req.length);
			break;
		case CmdSendPacket:
			ESP_LOGI(TAG, "CmdSendPacket");
			send_packet(req.data, req.length);
			break;
		case CmdSendAndListen:
			ESP_LOGI(TAG, "CmdSendAndListen");
			send_and_listen(req.data, req.length);
			break;
		case CmdUpdateRegister:
			ESP_LOGI(TAG, "CmdUpdateRegister");
			update_register(req.data, req.length);
			break;
		case CmdLED:
			ESP_LOGI(TAG, "CmdLED");
			led_mode(req.data, req.length);
			break;
		case CmdReadRegister:
			ESP_LOGI(TAG, "CmdReadRegister");
			read_register(req.data, req.length);
			break;
		case CmdSetSWEncoding:
			ESP_LOGI(TAG, "CmdSetSWEncoding");
			set_sw_encoding(req.data, req.length);
			break;
		case CmdResetRadioConfig:
			ESP_LOGI(TAG, "CmdResetRadioConfig");
			send_code(RESPONSE_CODE_SUCCESS);
			break;
		case CmdGetStatistics:
			ESP_LOGI(TAG, "CmdGetStatistics");
			send_stats();
			break;
		default:
			ESP_LOGE(TAG, "unimplemented rfspy command %d", req.command);
			break;
		}
		set_ble_rssi(req.rssi);
	}
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

	// Active probe to populate pump RSSI even when the pump is otherwise silent.
	BaseType_t ok = xTaskCreate(pump_active_probe_task, "pump_probe", 4096, 0, tskIDLE_PRIORITY + 6, &pump_probe_handle);
	if (ok != pdPASS)
	{
		ESP_LOGW(TAG, "pump_active_probe: task create failed");
		pump_probe_handle = NULL;
	}
}
