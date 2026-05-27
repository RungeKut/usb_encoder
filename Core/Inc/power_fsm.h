#ifndef POWER_FSM_H
#define POWER_FSM_H

#include <stdbool.h>
#include <stdint.h>

#define INSTALL_PC // Наличие компьютера в этой модификации прибора

typedef enum {
    PWR_STATE_IDLE = 0,
    PWR_STATE_TURNING_ON,          // Подали питание на реле, ждём USB
    PWR_STATE_WAIT_USB_CONNECT,    // Ждём PC_RunState == true
    PWR_STATE_WAIT_STABLE_CONNECT, // Стабилизация 1 сек после подключения
    PWR_STATE_TURNING_OFF,         // Отправили команду выключения, ждём отключения USB
    PWR_STATE_WAIT_USB_DISCONNECT, // Ждём PC_RunState == false
    PWR_STATE_DELAY_RELAY_OFF,     // Задержка 5 сек перед снятием питания
    PWR_STATE_ERROR_TIMEOUT_ON,    // Таймаут ожидания включения от USB
	PWR_STATE_ERROR_TIMEOUT_OFF    // Таймаут ожидания включения от USB
} power_state_t;

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_RED,
	LED_STATE_GREEN,
	LED_STATE_YELLOW
} led_state_t;

void PowerLed_Set(led_state_t state);
void PowerFSM_Init(void);
void PowerFSM_Request(bool turn_on); // true = включить, false = выключить
void PowerFSM_Tick(void);            // Вызывать в main() каждый цикл
bool PowerFSM_IsBusy(void);

extern power_state_t current_power_state;
extern bool global_power_state;

#endif
