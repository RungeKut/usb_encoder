#include "combo_resolver.h"
#include "button_actions.h"
#include "main.h"
#include "hid_keyboard.h"
#include "hid_mouse.h"
#include "hid_consumer.h"

// Таблица комбинаций
void Combo_0_1(void) // Enter + ON
{
	HID_KB_PressKeyOnce(HIDKEY_F10, HIDKEY_MODIFIER_NONE);
}
void Combo_0_3(void) // С + Enter
{
	HID_KB_PressKeyOnce(HIDKEY_SPACEBAR, HIDKEY_MODIFIER_NONE);
}
void Combo_2_3(void) // С + WIN
{
	HID_KB_PressKeyOnce(HIDKEY_DELETE, HIDKEY_MODIFIER_NONE);
}
void Combo_3_4(void) // С + PRESET
{
	HID_KB_PressKeyOnce(HIDKEY_F5, HIDKEY_MODIFIER_NONE);
}
void Combo_3_5(void) // С + MUTE
{
	HID_KB_PressKeyOnce(HIDKEY_F4, HIDKEY_MODIFIER_LEFT_ALT);
}
void Combo_3_6(void) // С + LEFT
{
	HID_KB_PressKeyOnce(HIDKEY_KP_MINUS, HIDKEY_MODIFIER_NONE);
}
void Combo_3_7(void) // С + RIGHT
{
	HID_KB_PressKeyOnce(HIDKEY_KP_PLUS, HIDKEY_MODIFIER_NONE);
}

const combo_entry_t combo_table[] = {
    { (1<<0) | (1<<1), Combo_0_1 },
	{ (1<<0) | (1<<3), Combo_0_3 },
    { (1<<2) | (1<<3), Combo_2_3 },
	{ (1<<3) | (1<<4), Combo_3_4 },
	{ (1<<3) | (1<<5), Combo_3_5 },
	{ (1<<3) | (1<<6), Combo_3_6 },
	{ (1<<3) | (1<<7), Combo_3_7 },
};
const uint8_t COMBO_TABLE_SIZE = sizeof(combo_table)/sizeof(combo_table[0]);

// Внутренний буфер
typedef struct {
    uint8_t btn_idx;
    button_event_t evt;
} pending_btn_t;

static pending_btn_t pending[MAX_PENDING_BUTTONS];
static uint8_t pending_count = 0;
static uint32_t last_event_time = 0;
static bool window_active = false;

void Combo_Init(void) {
    pending_count = 0;
    window_active = false;
}

static void Combo_Resolve(void) {
    window_active = false;
    
    // Собираем маску
    uint8_t mask = 0;
    for (int i = 0; i < pending_count; i++) mask |= (1U << pending[i].btn_idx);

    // Ищем в таблице
    bool matched = false;
    for (int i = 0; i < COMBO_TABLE_SIZE; i++) {
        if (combo_table[i].mask == mask) {
            combo_table[i].action();
            matched = true;
            break;
        }
    }

    // Если комбинация не найдена → выполняем одиночные
    if (!matched) {
        for (int i = 0; i < pending_count; i++) {
            ExecuteSingleAction(pending[i].btn_idx, pending[i].evt);
        }
    }

    pending_count = 0; // Очистка
}

void Combo_AddEvent(uint8_t btn_idx, button_event_t evt) {
    if (evt == BUTTON_EVT_NONE || btn_idx >= NUM_BUTTONS) return;
    uint32_t now = HAL_GetTick();

    // Если окно истекло, сначала разрешаем старые
    if (window_active && (now - last_event_time > COMBO_TIMEOUT_MS)) {
        Combo_Resolve();
    }

    if (pending_count < MAX_PENDING_BUTTONS) {
        pending[pending_count].btn_idx = btn_idx;
        pending[pending_count].evt = evt;
        pending_count++;
    }

    window_active = true;
    last_event_time = now;
}

void Combo_Tick(void) {
    if (window_active && (HAL_GetTick() - last_event_time > COMBO_TIMEOUT_MS)) {
        Combo_Resolve();
    }
}
