#ifndef COMBO_RESOLVER_H
#define COMBO_RESOLVER_H
#include "button_handler.h"

#define COMBO_TIMEOUT_MS 250U  // Окно для одновременного нажатия
#define MAX_PENDING_BUTTONS 4  // Макс. кнопок в комбинации

typedef void (*combo_cb_t)(void);

// Таблица комбинаций: маска кнопок -> действие
typedef struct {
    uint8_t mask;           // (1<<0)|(1<<1)
    combo_cb_t action;
} combo_entry_t;

extern const combo_entry_t combo_table[];
extern const uint8_t COMBO_TABLE_SIZE;

void Combo_Init(void);
void Combo_AddEvent(uint8_t btn_idx, button_event_t evt);
void Combo_Tick(void); // вызывать в цикле while(1)

#endif /* COMBO_RESOLVER_H */
