#ifndef _CMD_LOG_H
#define _CMD_LOG_H

#include <stdint.h>

// Инициализация системы логирования команд
void cmd_log_init(void);

// Логирование команд (не блокирующие, сохраняются в буфер)
void cmd_log_ble_rx(const uint8_t *data, int len);
void cmd_log_ble_tx(const uint8_t *data, int len);
void cmd_log_radio_tx(const uint8_t *data, int len);
void cmd_log_radio_rx(const uint8_t *data, int len);

// Вывод накопленных логов (вызывать когда радио свободно)
void cmd_log_flush(void);

#endif // _CMD_LOG_H
