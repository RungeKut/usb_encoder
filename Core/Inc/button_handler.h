#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

// Тип события кнопки
typedef enum {
    BUTTON_EVT_NONE = 0,
    BUTTON_EVT_SHORT_PRESS,
    BUTTON_EVT_DOUBLE_CLICK,
    BUTTON_EVT_LONG_PRESS
} button_event_t;

// Функция для получения последнего события (и сброса)
button_event_t GetButtonEvent(void);

// Основной обработчик — вызывать в цикле
void HandleButton(void);

#endif /* BUTTON_HANDLER_H */