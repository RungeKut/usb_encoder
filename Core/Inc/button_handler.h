#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"

// Тип события кнопки
typedef enum {
    BUTTON_EVT_NONE = 0,
    BUTTON_EVT_SHORT_PRESS,
    BUTTON_EVT_DOUBLE_CLICK,
    BUTTON_EVT_LONG_PRESS
} button_event_t;

// Контекст состояния одной кнопки
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    
    // Состояние дребезга
    bool last_raw;
    bool last_stable;
    uint32_t last_change_time;
    
    // Состояние нажатия
    bool pressed;
    uint32_t press_start_time;
    bool long_press_executed;
    uint32_t last_release_time;
    bool waiting_for_double;
    
    // Очередь событий
    volatile button_event_t pending_event;
} button_ctx_t;

// Инициализация контекста
void Button_Init(button_ctx_t* ctx, GPIO_TypeDef* port, uint16_t pin);

// Функция для получения последнего события (и сброса)
button_event_t GetButtonEvent(button_ctx_t* ctx);

// Основной обработчик — вызывать в цикле
void HandleButton(button_ctx_t* ctx);

#endif /* BUTTON_HANDLER_H */
