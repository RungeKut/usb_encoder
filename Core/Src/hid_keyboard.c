#include "hid_keyboard.h"
#include "hid_queue.h"

/* ========================================================================== */
/*                 --- Функции отправки для клавиатуры ---                    */
/* ========================================================================== */

// Удобная обёртка: нажать и отпустить одну клавишу
void HID_KB_PressKeyOnce(uint8_t keycode, uint8_t modifier) {
	// Нажатие
	keyboardHID report = {.id = 1, .modifier = modifier, .keycode[0] = keycode};
	HID_Queue_Push((uint8_t*)&report, sizeof(report));
	// Отпускание (очередь сама выдержит паузу между ними)
	report.modifier = 0;
	report.keycode[0] = 0;
	HID_Queue_Push((uint8_t*)&report, sizeof(report));
}
