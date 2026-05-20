#ifndef BUTTON_ACTIONS_H
#define BUTTON_ACTIONS_H
#include "button_handler.h"
#include "hid_codes.h"

#define NUM_BUTTONS 4

extern button_ctx_t buttons[NUM_BUTTONS];

extern device_mode_t current_mode;

// Инициализация структуры кнопок
void System_Init_Buttons(void);

// Вызов одиночного действия
void ExecuteSingleAction(uint8_t btn_idx, button_event_t evt);

#endif /* BUTTON_ACTIONS_H */
