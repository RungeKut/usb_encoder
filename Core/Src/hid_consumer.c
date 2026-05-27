#include "hid_consumer.h"
#include "hid_queue.h"

// Удобная обёртка: нажать и отпустить одну клавишу
void HID_Media_PressKeyOnce(uint16_t keycode) {
	// Нажатие
	mediaHID report = {.id = HID_MEDIA_REPORT, .lsb = LOBYTE(keycode), .msb = HIBYTE(keycode)};
	HID_Queue_Push((uint8_t*)&report, sizeof(report));
	// Отпускание (очередь сама выдержит паузу между ними)
	report.lsb = 0x00;
	report.msb = 0x00;
	HID_Queue_Push((uint8_t*)&report, sizeof(report));
}

/* ========================================================================== */
/*               --- Функции отправки для System Control ---                  */
/* ========================================================================== */

void HID_System_SendCommand(uint8_t usage) {
	// Нажатие
	customHID report = {.id = 4, .keys = usage};
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
    // Отпускание (очередь сама выдержит паузу между ними)
    report.keys = 0;
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
}
