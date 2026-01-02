#include "gnarl.h"

#include <string.h>
#include <unistd.h>

#include <esp_timer.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "adc.h"
#include "display.h"
#include "module.h"
#include "oled.h"

#define DISPLAY_TIMEOUT 15 // seconds
#define ROW_HEIGHT (OLED_HEIGHT / 4)
#define CENTER_OFFSET(x, object_width) ((x - object_width) / 2)

#define LEFT_COL_WIDTH 15
#define GAP_PX 2

static inline int row_baseline_y(int row)
{
	const int ascent = oled_font_ascent();
	const int descent = oled_font_descent();
	const int center = row * ROW_HEIGHT + ROW_HEIGHT / 2;
	// u8g2 draws strings relative to baseline.
	return center + (ascent - descent) / 2;
}

static inline int left_col_x_center(int object_width)
{
	return CENTER_OFFSET(LEFT_COL_WIDTH, object_width);
}

static inline int right_limit_x(void)
{
	return OLED_WIDTH - 1;
}

static inline int left_limit_x(void)
{
	return LEFT_COL_WIDTH + GAP_PX;
}

static inline int center_limit_x(int object_width)
{
	return CENTER_OFFSET(OLED_WIDTH, object_width);
}

static int draw_right_field(int x_right, int y, const char *s)
{
	oled_align_right();
	oled_draw_string(x_right, y, s);
	return x_right - oled_string_width(s) - GAP_PX;
}

typedef struct
{
	uint8_t width;
	uint8_t height;
	const uint8_t *bits;
} glyph_t;

static TaskHandle_t task_handle;
static TimerHandle_t shutdown_timer;

static const glyph_t PHONE = {
	.height = 15,
	.width = 9,
	.bits = (uint8_t[]){0xfe, 0x00, 0x39, 0x01, 0x01, 0x01, 0x7d, 0x01, 0x45, 0x01, 0x45, 0x01, 0x45, 0x01, 0x45, 0x01, 0x45, 0x01, 0x45, 0x01, 0x7d, 0x01, 0x01, 0x01, 0x11, 0x01, 0x01, 0x01, 0xfe}};

static const glyph_t BATTERY = {
	.height = 9,
	.width = 15,
	.bits = (uint8_t[]){0xfc, 0x7f, 0x04, 0x7f, 0x07, 0x7f, 0x01, 0x7f, 0x01, 0x7f, 0x01, 0x7e, 0x07, 0x7e, 0x04, 0x7e, 0xfc, 0x7f}};

static const glyph_t PUMP = {
	.height = 9,
	.width = 15,
	.bits = (uint8_t[]){0xf0, 0x7f, 0x10, 0x40, 0xd1, 0x57, 0x51, 0x44, 0x51, 0x44, 0xd1, 0x57, 0x1d, 0x40, 0x06, 0x40, 0xfc, 0x7f}};

static const glyph_t CLOCK = {
	.height = 15,
	.width = 15,
	.bits = (uint8_t[]){0xe0, 0x03, 0x98, 0x0c, 0x84, 0x10, 0x02, 0x20, 0x82, 0x20, 0x81, 0x40, 0x81, 0x40, 0x87, 0x70, 0x01, 0x41, 0x01, 0x42, 0x02, 0x20, 0x02, 0x20, 0x84, 0x10, 0x98, 0x0c, 0xe0, 0x03}};

static void format_time_period(char *buf, uint32_t period)
{
	uint8_t seconds = period % 60;
	period /= 60;
	uint8_t minutes = period % 60;
	period /= 60;
	uint8_t hours = period % 24;
	period /= 24;
	uint8_t days = period;

	if (days > 0)
	{
		sprintf(buf, "%2dd%2dh", days, hours);
	}
	else if (hours > 0)
	{
		sprintf(buf, "%2dh%2dm", hours, minutes);
	}
	else if (minutes > 0)
	{
		sprintf(buf, "%2dm%2ds", minutes, seconds);
	}
	else
	{
		sprintf(buf, "%5ds", seconds);
	}
}

typedef struct
{
	int8_t rssi;
	uint32_t uptime;
} connection_display_data_t;

typedef struct
{
	connection_display_data_t connections[2];
	uint16_t battery_voltage;
	uint32_t uptime;
} display_data_t;

static bool radio_active;
static int64_t rx_flash_until;

static inline uint32_t timestamp_to_period(uint32_t timestamp)
{
	return (esp_timer_get_time() / SECONDS) - timestamp;
}

static void draw_initial()
{
	oled_init();
	oled_font_medium();
	oled_align_right();
	oled_clear();

	// draw phone glyph_t
	// 0-15
	oled_draw_xbm(left_col_x_center(PHONE.width), row_baseline_y(0) - PHONE.height + 1, PHONE.width, PHONE.height, PHONE.bits);
	// draw pump glyph_t
	// 16-31
	oled_draw_xbm(left_col_x_center(PUMP.width), row_baseline_y(1) - PUMP.height + 1, PUMP.width, PUMP.height, PUMP.bits);
	// draw battery glyph_t
	// 32-47
	oled_draw_xbm(left_col_x_center(BATTERY.width), row_baseline_y(2) - BATTERY.height + 1, BATTERY.width, BATTERY.height, BATTERY.bits);

	// draw clock glyph_t centered horizontally on bottom row
	oled_draw_xbm(center_limit_x(CLOCK.width), row_baseline_y(3) - CLOCK.height + 1, CLOCK.width, CLOCK.height, CLOCK.bits);

	oled_update();
}

static void draw_static_glyphs(void)
{
	oled_font_medium();
	// glyphs are placed in the left column
	oled_draw_xbm(left_col_x_center(PHONE.width), row_baseline_y(0) - PHONE.height + 1, PHONE.width, PHONE.height, PHONE.bits);
	oled_draw_xbm(left_col_x_center(PUMP.width), row_baseline_y(1) - PUMP.height + 1, PUMP.width, PUMP.height, PUMP.bits);
	oled_draw_xbm(left_col_x_center(BATTERY.width), row_baseline_y(2) - BATTERY.height + 1, BATTERY.width, BATTERY.height, BATTERY.bits);
	oled_draw_xbm(center_limit_x(CLOCK.width), row_baseline_y(3) - CLOCK.height + 1, CLOCK.width, CLOCK.height, CLOCK.bits);
}

static void draw_row_rssi_and_age(int row, const char *rssi_str, const char *age_str)
{
	int x = right_limit_x();
	const int y = row_baseline_y(row);

	// Always show age (right-most)
	x = draw_right_field(x, y, age_str);

	// Prefer keeping RSSI; only show unit label if it fits
	int x_after_label = x;
	const int label_w = oled_string_width("dBm");
	if (x_after_label - label_w >= left_limit_x())
	{
		x_after_label = draw_right_field(x_after_label, y, "dBm");
	}

	if (x_after_label > left_limit_x())
	{
		draw_right_field(x_after_label, y, rssi_str);
	}
}

static void draw_row_battery(const char *percent_str, const char *mv_str)
{
	int x = right_limit_x();
	const int y = row_baseline_y(2);

	// Prefer keeping numeric values; only show unit labels if they fit.
	int x_after_mv_label = x;
	const int mv_label_w = oled_string_width("mV");
	if (x_after_mv_label - mv_label_w >= left_limit_x())
	{
		x_after_mv_label = draw_right_field(x_after_mv_label, y, "mV");
	}

	int x_after_mv = draw_right_field(x_after_mv_label, y, mv_str);

	int x_after_percent_label = x_after_mv;
	const int pct_label_w = oled_string_width("%");
	if (x_after_percent_label - pct_label_w >= left_limit_x())
	{
		x_after_percent_label = draw_right_field(x_after_percent_label, y, "%");
	}

	if (x_after_percent_label > left_limit_x())
	{
		draw_right_field(x_after_percent_label, y, percent_str);
	}
}
static void draw_data()
{
	char buf[7];
	char rssi_str[8];
	char age_str[8];
	char percent_str[8];
	char mv_str[8];

	static display_data_t last_display_data = {
		.battery_voltage = UINT16_MAX,
		.uptime = UINT32_MAX,
		.connections = {
			{.uptime = UINT32_MAX, .rssi = INT8_MAX},
			{.uptime = UINT32_MAX, .rssi = INT8_MAX},
		},
	};
	uint8_t should_update = 0;

	// draw phone + pump RSSI
	for (int i = 0; i < 2; i++)
	{
		connection_display_data_t *last_connection_data = &last_display_data.connections[i];
		connection_stats_t *connection_data = &get_connection_stats()[i];

		const int8_t old_rssi = last_connection_data->rssi;
		const int8_t new_rssi = connection_data->rssi;

		if (old_rssi != new_rssi)
		{
			last_connection_data->rssi = new_rssi;
			should_update = 1;
		}

		const uint32_t old_period = last_connection_data->uptime;
		const uint32_t new_period = timestamp_to_period(connection_data->timestamp);

		if (new_period != old_period)
		{
			last_connection_data->uptime = new_period;
			should_update = 1;
		}
	}

	// draw battery stats
	const uint16_t old_battery_voltage = last_display_data.battery_voltage;
	const uint16_t new_battery_voltage = get_battery_voltage();
	if (new_battery_voltage != old_battery_voltage)
	{
		last_display_data.battery_voltage = new_battery_voltage;
		should_update = 1;
	}

	// draw uptime
	const uint32_t old_uptime = last_display_data.uptime;
	const uint32_t new_uptime = timestamp_to_period(0);

	if (new_uptime != old_uptime)
	{
		last_display_data.uptime = new_uptime;
		should_update = 1;
	}

	if (!should_update)
	{
		return;
	}

	// Full redraw avoids leftover pixels when values shrink (e.g., 100 -> 99).
	oled_clear();
	draw_static_glyphs();

	// Phone + pump rows
	for (int i = 0; i < 2; i++)
	{
		connection_display_data_t *last_connection_data = &last_display_data.connections[i];
		connection_stats_t *connection_data = &get_connection_stats()[i];
		const int8_t rssi = connection_data->rssi;
		const uint32_t age = timestamp_to_period(connection_data->timestamp);

		// -127 is commonly used as a sentinel for "invalid/unknown RSSI".
		snprintf(rssi_str, sizeof(rssi_str), (rssi == 0 || rssi == -127) ? " ---" : "%4d", (int)rssi);
		format_time_period(age_str, age);

		draw_row_rssi_and_age(i, rssi_str, age_str);
		last_connection_data->rssi = rssi;
		last_connection_data->uptime = age;
	}

	// Bottom row: TX/RX markers on the left
	bool rx_on = (rx_flash_until != 0) && (esp_timer_get_time() < rx_flash_until);
	char status[8] = "";
	if (radio_active && rx_on)
	{
		strncpy(status, "TX RX", sizeof(status));
	}
	else if (radio_active)
	{
		strncpy(status, "TX", sizeof(status));
	}
	else if (rx_on)
	{
		strncpy(status, "RX", sizeof(status));
	}

	oled_align_left();
	oled_draw_string(left_limit_x(), row_baseline_y(3), status);

	// Battery row
	const uint16_t battery_mv = get_battery_voltage();
	snprintf(percent_str, sizeof(percent_str), "%4d", battery_percent(battery_mv));
	snprintf(mv_str, sizeof(mv_str), "%4d", (int)battery_mv);
	draw_row_battery(percent_str, mv_str);
	last_display_data.battery_voltage = battery_mv;

	// Uptime row
	format_time_period(buf, timestamp_to_period(0));
	draw_right_field(right_limit_x(), row_baseline_y(3), buf);
	last_display_data.uptime = new_uptime;

	oled_update();
}

void display_set_radio_active(bool active)
{
	radio_active = active;
	if (task_handle != NULL)
	{
		xTaskNotify(task_handle, pdTRUE, eSetValueWithOverwrite);
	}
}

void display_pulse_radio_rx(void)
{
	rx_flash_until = esp_timer_get_time() + 3 * SECONDS;
	if (task_handle != NULL)
	{
		xTaskNotify(task_handle, pdTRUE, eSetValueWithOverwrite);
	}
}

static void display_loop(void *unused)
{
	draw_initial();

	TickType_t timeout = 0;

	// Notify the task itself to draw data immediately.
	xTaskNotify(task_handle, pdTRUE, eSetValueWithOverwrite);

	for (;;)
	{
		uint32_t notification_value;
		BaseType_t notified = xTaskNotifyWait(0, 0, &notification_value, timeout);
		if (notified == pdTRUE && notification_value == pdFALSE)
		{
			timeout = portMAX_DELAY;
			oled_off();
		}
		else
		{
			// Lower periodic wakeups to reduce power; notifications still trigger immediate redraw.
			timeout = pdMS_TO_TICKS(1000);
			draw_data();
			oled_on();
		}
	}
}

static inline void disable_display(TimerHandle_t xTimer)
{
	xTaskNotify(task_handle, pdFALSE, eSetValueWithOverwrite);
}

static void button_interrupt(void *unused)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (!gpio_get_level(BUTTON))
	{
		xTaskNotifyFromISR(task_handle, pdTRUE, eSetValueWithOverwrite, &higher_priority_task_woken);
		if (gnarl_request_pump_probe_isr())
		{
			higher_priority_task_woken = pdTRUE;
		}
		xTimerStopFromISR(shutdown_timer, &higher_priority_task_woken);
	}
	else
		xTimerResetFromISR(shutdown_timer, &higher_priority_task_woken);

	portYIELD_FROM_ISR(higher_priority_task_woken);
}

void display_init(void)
{
	ESP_LOGI(TAG, "display_init: starting");
	xTaskCreate(display_loop, "display", 4096, 0, 10, &task_handle);
	shutdown_timer = xTimerCreate("display_off", pdMS_TO_TICKS(DISPLAY_TIMEOUT * 1000), pdFALSE, 0, disable_display);

	// Enable interrupt on button press.
	gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
	gpio_set_intr_type(BUTTON, GPIO_INTR_ANYEDGE);
	gpio_isr_handler_add(BUTTON, button_interrupt, 0);
	gpio_intr_enable(BUTTON);
	ESP_LOGI(TAG, "display_init: button ISR configured on GPIO %d", (int)BUTTON);

	xTimerStart(shutdown_timer, 0);
	ESP_LOGI(TAG, "display_init: shutdown timer started (%ds)", DISPLAY_TIMEOUT);
}
