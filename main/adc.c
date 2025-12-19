#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "module.h"

#include "adc.h"

const static char *TAG = "ADC";

#define AVG_SAMPLES (10)
#define AVG_PERIOD (4) // seconds
// Scale the ADC input voltage back to the actual battery voltage using the
// board's resistor divider (see include/module.h).
#define SCALE_VOLTAGE(raw_mv) ((int)((int64_t)(raw_mv) * (VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM))

static int battery_voltage;

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

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        int averaged_result = 0;
        for (int i = 0; i < AVG_SAMPLES; i++)
        {
            int raw_output, voltage;
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
                ESP_LOGI(TAG, "ADC raw: %d, voltage: %d mV", raw_output, voltage);
            }
            else
            {
                ESP_LOGI(TAG, "ADC raw: %d (no calibration)", raw_output);
                vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                continue;
            }

            if (battery_voltage <= 1000)
                battery_voltage = SCALE_VOLTAGE(voltage);

            averaged_result += voltage;

            vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
        }
        averaged_result /= AVG_SAMPLES;

        ESP_LOGI(TAG, "ADC voltage: %d mV (%d mV scaled)", averaged_result, SCALE_VOLTAGE(averaged_result));
        battery_voltage = SCALE_VOLTAGE(averaged_result);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(60000));
    }
}

void adc_init(void)
{
	ESP_LOGI(TAG, "adc_init: starting task, BATTERY_ADC=%d", (int)BATTERY_ADC);
    xTaskCreate(adc_task, "adc", 3072, NULL, 5, NULL);
}