#include "hid_mouse.h"
#include "hid_queue.h"

/* ========================================================================== */
/*                    --- Функции отправки для мыши ---                       */
/* ========================================================================== */

bool axis_mouse_move = false; // Ось перемещения мыши: false - Ось X, true - Ось Y

void HID_Mouse_Move(int8_t x, int8_t y) {
    mouseHID report = {.id = 3, .x = x, .y = y, .buttons = 0, .wheel = 0};
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
}

// wheel: положительное — вверх, отрицательное — вниз
void HID_Mouse_Wheel(int8_t delta) {
    mouseHID report = {.id = 3, .wheel = delta};
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
}

void HID_Mouse_Click(uint8_t button) {
    // Нажатие
    mouseHID report = {.id = 3, .buttons = button};
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
    // Отпускание (очередь сама выдержит паузу между ними)
    report.buttons = 0;
    HID_Queue_Push((uint8_t*)&report, sizeof(report));
}
