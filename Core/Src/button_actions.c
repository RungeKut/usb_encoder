#include "button_actions.h"
#include "main.h"
#include "hid_keyboard.h"
#include "hid_mouse.h"
#include "hid_consumer.h"
#include "power_fsm.h"

// === Реализация действий ===
/* ========================================================================== */
/*                 --- Функции обработки кнопки энкодера ---                  */
/* ========================================================================== */

device_mode_t current_mode = MODE_ENCODER;
bool apply_input_key = false;

//Одно короткое нажатие
void EncoderActionShortPress(void) {
	if (current_mode == MODE_ENCODER) {
		HID_KB_PressKeyOnce(HIDKEY_ENTER, HIDKEY_MODIFIER_NONE);  // press 'Enter'
	} else if (current_mode == MODE_KEYBOARD) {
		apply_input_key = true;
	} else if (current_mode == MODE_MOUSE) {
		HID_Mouse_Click(0x01); // Left click
	} else if (current_mode == MODE_WHEEL) {
		HID_Mouse_Click(0x01); // Left click
	} else if (current_mode == MODE_CONSUMER) {
		HID_Media_PressKeyOnce(HID_MEDIA_Vol_Mute);
	}
}

//bool powerFlag = false;

//Одно двойное нажатие
void EncoderActionDoubleClick(void) {
	if (current_mode == MODE_ENCODER) {
		HID_KB_PressKeyOnce(HIDKEY_BACKSPACE, HIDKEY_MODIFIER_NONE);
	} else if (current_mode == MODE_KEYBOARD) {
		// Переключить раскладку (отправить Alt+Shift)
		HID_KB_PressKeyOnce(HIDKEY_NONE, HIDKEY_MODIFIER_LEFT_ALT + HIDKEY_MODIFIER_LEFT_SHIFT); // Left Shift, Left Alt
	} else if (current_mode == MODE_CONSUMER) {
//		if (powerFlag) {
//			SendCustomCommand(HID_CUSTOM_SystemWakeUp);
//			powerFlag = false;
//		} else {
			HID_System_SendCommand(HID_CUSTOM_SystemPowerDown);
//			powerFlag = true;
//		}
//		// Мигни количеством режимов:
//		// Отключить LED
//		HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
//		HAL_Delay(500);
//		for (int i = 0; i <= (uint8_t)powerFlag; i++) {
//			// Включить LED
//			HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
//			HAL_Delay(200);
//			// Отключить LED
//			HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
//			HAL_Delay(400);
//		}
	} else if (current_mode == MODE_MOUSE) {
		HID_Mouse_Click(0x02); // Right click
		axis_mouse_move = axis_mouse_move ? false : true;
	} else if (current_mode == MODE_WHEEL) {
		//MouseClick(0x03); // Middle click
		HID_KB_PressKeyOnce(HIDKEY_NONE, HIDKEY_MODIFIER_LEFT_UI);
	}
}

//Одно долгое нажатие
void EncoderActionLongPress(void) {
	// Долгое нажатие → смена режима
	current_mode = (current_mode + 1) % MODE_COUNT;
	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	// Мигни количеством режимов:
	// Отключить LED
	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	for (int i = 0; i <= current_mode; i++) {
		// Включить LED
		HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
		// Отключить LED
		HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
		HAL_Delay(400);
	}
}

/* ========================================================================== */
/*              --- Функции обработки кнопки питания POW_ON ---               */
/* ========================================================================== */
//Одно короткое нажатие
void POW_ON_ActionShortPress(void) {
	PowerFSM_Request(!global_power_state);
}
//Одно двойное нажатие
void POW_ON_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void POW_ON_ActionLongPress(void) {
	// Аварийное выключение реле без ожидания ПК
	HAL_GPIO_WritePin(COMP_ON_GPIO_Port, COMP_ON_Pin, GPIO_PIN_RESET);
	
	//Желтый
	HAL_GPIO_WritePin(LED_220_GPIO_Port, LED_220_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_POW_GPIO_Port, LED_POW_Pin, GPIO_PIN_RESET);
	
	global_power_state = false;
    current_power_state = PWR_STATE_IDLE;
}

/* ========================================================================== */
/*                   --- Функции обработки кнопки WIN ---                     */
/* ========================================================================== */
//Одно короткое нажатие
void WIN_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_NONE, HIDKEY_MODIFIER_LEFT_UI);
}

//Одно двойное нажатие
void WIN_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void WIN_ActionLongPress(void) {
}

/* ========================================================================== */
/*                     --- Функции обработки кнопки C ---                     */
/* ========================================================================== */
//Одно короткое нажатие
void C_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_ESCAPE, HIDKEY_MODIFIER_NONE);
}

//Одно двойное нажатие
void C_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void C_ActionLongPress(void) {
}

/* ========================================================================== */
/*                  --- Функции обработки кнопки PRESET ---                   */
/* ========================================================================== */
//Одно короткое нажатие
void PRESET_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_P, HIDKEY_MODIFIER_NONE);
}

//Одно двойное нажатие
void PRESET_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void PRESET_ActionLongPress(void) {
}

/* ========================================================================== */
/*                   --- Функции обработки кнопки MUTE ---                    */
/* ========================================================================== */
//Одно короткое нажатие
void MUTE_ActionShortPress(void) {
	HID_Media_PressKeyOnce(HID_MEDIA_Vol_Mute);
}

//Одно двойное нажатие
void MUTE_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void MUTE_ActionLongPress(void) {
}

/* ========================================================================== */
/*                   --- Функции обработки кнопки LEFT ---                    */
/* ========================================================================== */
//Одно короткое нажатие
void LEFT_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_LEFT, HIDKEY_MODIFIER_NONE);
}

//Одно двойное нажатие
void LEFT_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void LEFT_ActionLongPress(void) {
}

/* ========================================================================== */
/*                   --- Функции обработки кнопки RIGHT ---                   */
/* ========================================================================== */
//Одно короткое нажатие
void RIGHT_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_RIGHT, HIDKEY_MODIFIER_NONE);
}

//Одно двойное нажатие
void RIGHT_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void RIGHT_ActionLongPress(void) {
}

/* ========================================================================== */
/*                --- Функции обработки кнопки PRINTSCREEN ---                */
/* ========================================================================== */
//Одно короткое нажатие
void PRNSCR_ActionShortPress(void) {
	HID_KB_PressKeyOnce(HIDKEY_PRINTSCREEN, HIDKEY_MODIFIER_NONE);
}

//Одно двойное нажатие
void PRNSCR_ActionDoubleClick(void) {
}

//Одно долгое нажатие
void PRNSCR_ActionLongPress(void) {
}

/* ========================================================================== */
/*              --- Функция инициализации структуры кнопок ---                */
/* ========================================================================== */

button_ctx_t buttons[NUM_BUTTONS] = {0};

void System_Init_Buttons(void)
{
    // Порты и пины кнопок
    Button_Init(&buttons[0], ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin);
    Button_Init(&buttons[1], KEY_ON_GPIO_Port, KEY_ON_Pin);
    Button_Init(&buttons[2], KEY_WIN_GPIO_Port, KEY_WIN_Pin);
    Button_Init(&buttons[3], KEY_reset_GPIO_Port, KEY_reset_Pin);
	Button_Init(&buttons[4], KEY_PRESET_GPIO_Port, KEY_PRESET_Pin);
	Button_Init(&buttons[5], KEY_MUTE_GPIO_Port, KEY_MUTE_Pin);
	Button_Init(&buttons[6], KEY_LEFT_GPIO_Port, KEY_LEFT_Pin);
	Button_Init(&buttons[7], KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin);
	Button_Init(&buttons[8], KEY_PRINTSCREEN_GPIO_Port, KEY_PRINTSCREEN_Pin);
}

// === Таблица указателей на функции (dispatch matrix) для одиночных кнопок ===
//     (не для сочетаний клавиш)
typedef void (*action_cb_t)(void);
static const action_cb_t action_table[NUM_BUTTONS][3] = {
    { EncoderActionShortPress, EncoderActionDoubleClick, EncoderActionLongPress },
    { POW_ON_ActionShortPress, POW_ON_ActionDoubleClick, POW_ON_ActionLongPress },
    { WIN_ActionShortPress,    WIN_ActionDoubleClick,    WIN_ActionLongPress },
    { C_ActionShortPress,      C_ActionDoubleClick,      C_ActionLongPress },
	{ PRESET_ActionShortPress, PRESET_ActionDoubleClick, PRESET_ActionLongPress },
	{ MUTE_ActionShortPress,   MUTE_ActionDoubleClick,   MUTE_ActionLongPress },
	{ LEFT_ActionShortPress,   LEFT_ActionDoubleClick,   LEFT_ActionLongPress },
	{ RIGHT_ActionShortPress,  RIGHT_ActionDoubleClick,  RIGHT_ActionLongPress },
	{ PRNSCR_ActionShortPress, PRNSCR_ActionDoubleClick, PRNSCR_ActionLongPress }
};

void ExecuteSingleAction(uint8_t btn_idx, button_event_t evt) {
    if (btn_idx >= NUM_BUTTONS || evt == BUTTON_EVT_NONE) return;
    
    // Преобразуем enum в индекс массива (0..2)
    uint8_t idx = (evt == BUTTON_EVT_SHORT_PRESS) ? 0 :
                  (evt == BUTTON_EVT_DOUBLE_CLICK) ? 1 : 2;
                  
    if (action_table[btn_idx][idx]) {
        action_table[btn_idx][idx]();
    }
}
