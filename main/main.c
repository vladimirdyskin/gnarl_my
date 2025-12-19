#include <stdio.h>

#include <esp_chip_info.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <esp_system.h>

#include <driver/gpio.h>

#include "adc.h"
#include "display.h"
#include "gnarl.h"
#include "rfm95.h"
#include "spi.h"

static const char *reset_reason_str(esp_reset_reason_t reason)
{
	switch (reason)
	{
	case ESP_RST_UNKNOWN:
		return "UNKNOWN";
	case ESP_RST_POWERON:
		return "POWERON";
	case ESP_RST_EXT:
		return "EXT";
	case ESP_RST_SW:
		return "SW";
	case ESP_RST_PANIC:
		return "PANIC";
	case ESP_RST_INT_WDT:
		return "INT_WDT";
	case ESP_RST_TASK_WDT:
		return "TASK_WDT";
	case ESP_RST_WDT:
		return "WDT";
	case ESP_RST_DEEPSLEEP:
		return "DEEPSLEEP";
	case ESP_RST_BROWNOUT:
		return "BROWNOUT";
	case ESP_RST_SDIO:
		return "SDIO";
	default:
		return "(unmapped)";
	}
}

    

void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI(TAG, "==== GNARL boot ====");
	esp_reset_reason_t reason = esp_reset_reason();
	ESP_LOGI(TAG, "reset reason: %d (%s)", (int)reason, reset_reason_str(reason));

	esp_chip_info_t chip;
	esp_chip_info(&chip);
	ESP_LOGI(TAG, "chip: model=%d rev=%d cores=%d", (int)chip.model, (int)chip.revision, (int)chip.cores);
	ESP_LOGI(TAG, "heap free: %u", (unsigned)esp_get_free_heap_size());
	ESP_LOGI(TAG, "firmware: %s / %s", SUBG_RFSPY_VERSION, BLE_RFSPY_VERSION);

	esp_pm_config_t pm_config = {
		.max_freq_mhz = 80,
		.min_freq_mhz = 20,
		// For BLE stability while debugging, keep light sleep off.
		.light_sleep_enable = false,
	};

	ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	ESP_LOGI(TAG, "PM configured: min=%d max=%d light_sleep=%d", pm_config.min_freq_mhz, pm_config.max_freq_mhz, pm_config.light_sleep_enable);

	// Install GPIO ISR service once for the whole app. Individual modules should
	// only add their handlers and must not call gpio_install_isr_service() again.
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

	rfm95_init();
	uint8_t v = read_version();
	ESP_LOGD(TAG, "radio version %d.%d", version_major(v), version_minor(v));
	set_frequency(PUMP_FREQUENCY);
	ESP_LOGD(TAG, "frequency set to %lu Hz", read_frequency());
	ESP_LOGI(TAG, "radio ready; pump freq=%lu", (unsigned long)PUMP_FREQUENCY);
	adc_init();
	display_init();
	gnarl_init();
	ESP_LOGI(TAG, "init complete; advertising should start soon");
}
