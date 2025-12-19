#ifndef _MODULE_H
#define _MODULE_H

// Board configuration.
// This workspace targets Heltec WiFi LoRa 32 V2 (ESP32 + SX1276 + OLED + VBAT sense).

#define BUTTON		GPIO_NUM_0
#define LED		    GPIO_NUM_25

#define OLED_SDA	GPIO_NUM_4
#define OLED_SCL	GPIO_NUM_15
#define OLED_RST	GPIO_NUM_16

#define LORA_SCK	GPIO_NUM_5
#define LORA_SDI	GPIO_NUM_19
#define LORA_SDO	GPIO_NUM_27
#define LORA_CS		GPIO_NUM_18
#define LORA_RST	GPIO_NUM_14
#define LORA_DIO0	GPIO_NUM_26
#define LORA_DIO1	GPIO_NUM_35
#define LORA_DIO2	GPIO_NUM_34

// Battery sense ADC pin (Heltec WiFi LoRa 32 V2)
#define BATTERY_ADC	GPIO_NUM_37

// VBAT voltage divider (in kΩ): R1 from VBAT to ADC, R2 from ADC to GND.
// Heltec WiFi LoRa 32 V2 uses ~100k/100k => VBAT ~= ADC * 2.
#define VDIV_R1_KOHM	100
#define VDIV_R2_KOHM	100

// Comment out the following line for a TTGO ESP LoRa v1 module w/o OLED
#define OLED_ENABLE

// Pin mappings on TTGO ESP32 LoRa OLED v1 module


#endif // _MODULE_H
