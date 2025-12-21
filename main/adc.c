#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>

#include "module.h"

#include "adc.h"

const static char *TAG = "ADC";

// Keep ADC sampling lightweight to avoid impacting radio/BLE.
#define AVG_SAMPLES (4)
#define AVG_PERIOD (1) // seconds total time spent sampling per update
// With a high-impedance VBAT divider (e.g. 100k/100k), the ADC sampling cap
// can hold charge and readings may lag. Do a few dummy reads to settle.
#define DUMMY_READS (2)

// How often to refresh the cached battery voltage.
#define UPDATE_PERIOD_MS (30000)
// Scale the ADC input voltage back to the actual battery voltage using the
// board's resistor divider (see include/module.h).
#define SCALE_VOLTAGE(raw_mv) ((int)((int64_t)(raw_mv) * (VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM))

// LiPo/Li-ion sanity bounds. If the computed value is outside these bounds,
// it's very likely the divider/pin definition doesn't match the actual board.
#define LIPO_MIN_MV (2500)
#define LIPO_MAX_MV (4500)

// Raw ADC saturation indicator (12-bit).
#define ADC_RAW_SAT (4090)

static volatile int battery_voltage;

static bool warned_divider_mismatch;
static bool warned_adc_saturation;

static int choose_battery_mv(int adc_mv, int scaled_mv)
{
    // Prefer scaled value when it looks like a plausible battery voltage.
    if (scaled_mv >= LIPO_MIN_MV && scaled_mv <= LIPO_MAX_MV)
    {
        return scaled_mv;
    }

    // If scaling produces an impossible value (e.g., >5V on a 1S LiPo), fall
    // back to unscaled and warn once.
    if (!warned_divider_mismatch)
    {
        warned_divider_mismatch = true;
        ESP_LOGW(TAG,
                 "battery scaling looks wrong: adc=%dmV scaled=%dmV (VDIV_R1_KOHM=%d VDIV_R2_KOHM=%d). "
                 "Falling back to unscaled. Check BATTERY_ADC/VDIV in include/module.h.",
                 adc_mv,
                 scaled_mv,
                 (int)VDIV_R1_KOHM,
                 (int)VDIV_R2_KOHM);
    }
    return adc_mv;
}

static void probe_adc1_pins(adc_oneshot_unit_handle_t adc_handle, adc_cali_handle_t cali_handle, const adc_oneshot_chan_cfg_t *config)
{
    // Quick probe to help identify which GPIO actually reflects battery voltage.
    // Only ADC1 pins are probed to avoid ADC2/WiFi conflicts.
    const gpio_num_t gpios[] = {GPIO_NUM_36, GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_39};

    for (size_t i = 0; i < sizeof(gpios) / sizeof(gpios[0]); i++)
    {
        adc_unit_t unit;
        adc_channel_t channel;
        esp_err_t err = adc_oneshot_io_to_channel(gpios[i], &unit, &channel);
        if (err != ESP_OK || unit != ADC_UNIT_1)
        {
            continue;
        }

        err = adc_oneshot_config_channel(adc_handle, channel, config);
        if (err != ESP_OK)
        {
            continue;
        }

        int raw = 0;
        int mv = 0;

        for (int d = 0; d < DUMMY_READS; d++)
        {
            (void)adc_oneshot_read(adc_handle, channel, &raw);
        }

        err = adc_oneshot_read(adc_handle, channel, &raw);
        if (err != ESP_OK)
        {
            continue;
        }

        if (cali_handle)
        {
            err = adc_cali_raw_to_voltage(cali_handle, raw, &mv);
            if (err != ESP_OK)
            {
                continue;
            }
            ESP_LOGD(TAG, "probe GPIO%d: raw=%d mv=%d scaled=%d", (int)gpios[i], raw, mv, SCALE_VOLTAGE(mv));
        }
        else
        {
            ESP_LOGD(TAG, "probe GPIO%d: raw=%d (no calibration)", (int)gpios[i], raw);
        }
    }
}

uint16_t get_battery_voltage(void)
{
    return battery_voltage;
}

uint8_t battery_percent(uint16_t battery_voltage)
{
    uint16_t full_bat = 4200;
    uint16_t empty_bat = 3000;

    if (battery_voltage > full_bat)
        battery_voltage = full_bat;
    else if (battery_voltage < empty_bat)
        battery_voltage = empty_bat;

    uint8_t percent = (battery_voltage - empty_bat) * 100 / (full_bat - empty_bat);

    ESP_LOGI(TAG, "(Conv) Battery voltage: %d mV, percentage: %d%%", battery_voltage, percent);

    return percent;
}

static void adc_task(void *arg)
{
    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(BATTERY_ADC, &unit, &channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_io_to_channel(%d) failed: %s", (int)BATTERY_ADC, esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
    };
    err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_chan_cfg_t config = {
		// Use higher attenuation to avoid saturation near full battery.
		.atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle, channel, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_cali_handle_t cali_handle = NULL;

    adc_cali_line_fitting_config_t cali_config = {
        .atten = config.atten,
        .bitwidth = config.bitwidth,
        .unit_id = unit,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (err != ESP_OK)
    {
        // Calibration is optional; don't reboot if unavailable.
        ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(err));
        cali_handle = NULL;
    }

    // Heltec VBAT sense: GPIO21 must be asserted (LOW) to connect the divider
    // and power VEXT. Some OLED drivers may alter this pin, so we also
    // re-assert it during sampling below.
    gpio_set_direction(BATTERY_SENSE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(BATTERY_SENSE_EN, BATTERY_SENSE_EN_ACTIVE);

    // One-time probe (DEBUG only): helps confirm the correct VBAT ADC GPIO on this board.
    // Note: we only probe ADC1 GPIOs here.
    if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG)
    {
        probe_adc1_pins(adc_handle, cali_handle, &config);
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // Ensure VBAT divider is connected for this update window.
        gpio_set_level(BATTERY_SENSE_EN, BATTERY_SENSE_EN_ACTIVE);

        int averaged_result = 0;
        int valid_samples = 0;
        for (int i = 0; i < AVG_SAMPLES; i++)
        {
            int raw_output, voltage;

			// Dummy reads to help settle when source impedance is high.
			for (int d = 0; d < DUMMY_READS; d++)
			{
				(void)adc_oneshot_read(adc_handle, channel, &raw_output);
			}

            err = adc_oneshot_read(adc_handle, channel, &raw_output);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                continue;
            }

            if (cali_handle)
            {
                err = adc_cali_raw_to_voltage(cali_handle, raw_output, &voltage);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed: %s", esp_err_to_name(err));
                    vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                    continue;
                }
                if (raw_output >= ADC_RAW_SAT && !warned_adc_saturation)
                {
                    warned_adc_saturation = true;
                    ESP_LOGW(TAG,
                             "ADC is saturated (raw=%d). This usually means wrong BATTERY_ADC pin or missing/incorrect divider.",
                             raw_output);
                }
                ESP_LOGD(TAG, "ADC raw: %d, voltage: %d mV", raw_output, voltage);
            }
            else
            {
                ESP_LOGI(TAG, "ADC raw: %d (no calibration)", raw_output);
                vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                continue;
            }

            if (battery_voltage <= 1000)
            {
                const int scaled = SCALE_VOLTAGE(voltage);
                battery_voltage = choose_battery_mv(voltage, scaled);
            }

            averaged_result += voltage;
            valid_samples++;

            vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
        }
        if (valid_samples == 0)
        {
            ESP_LOGW(TAG, "ADC: no valid samples");
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UPDATE_PERIOD_MS));
            continue;
        }

        averaged_result /= valid_samples;

        const int scaled_avg = SCALE_VOLTAGE(averaged_result);
        const int chosen = choose_battery_mv(averaged_result, scaled_avg);
        ESP_LOGI(TAG, "ADC voltage: %d mV (%d mV scaled; chosen=%d mV)", averaged_result, scaled_avg, chosen);
        battery_voltage = chosen;

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UPDATE_PERIOD_MS));
    }
}

void adc_init(void)
{
	ESP_LOGI(TAG, "adc_init: starting task, BATTERY_ADC=%d", (int)BATTERY_ADC);
	// Low priority: ADC is best-effort telemetry and must not interfere with radio/BLE.
    xTaskCreate(adc_task, "adc", 3072, NULL, 1, NULL);
}