#include "stdint.h"

void adc_init(void);
uint16_t get_battery_voltage(void);
uint8_t battery_percent(uint16_t battery_voltage);
