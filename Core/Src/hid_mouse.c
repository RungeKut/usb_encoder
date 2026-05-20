#include "hid_mouse.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ========================================================================== */
/*                    --- Функции отправки для мыши ---                       */
/* ========================================================================== */

// Ось перемещения мыши: false - Ось X, true - Ось Y
bool axis_mouse_move = false;

static void HID_Mouse_SendReport(mouseHID *mouse) {
    //while (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)mouse, sizeof(mouseHID)) == USBD_BUSY);
	HAL_Delay(30);
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)mouse, sizeof(mouseHID));
}

void HID_Mouse_Move(int8_t x, int8_t y) {
    mouseHID report = {0};
	report.id = 3;
    report.x = x;
    report.y = y;
    HID_Mouse_SendReport(&report);
}

// wheel: положительное — вверх, отрицательное — вниз
void HID_Mouse_Wheel(int8_t delta) {
    mouseHID report = {0};
	report.id = 3;
    report.wheel = delta;
    HID_Mouse_SendReport(&report);
}

void HID_Mouse_Click(uint8_t button) {
    mouseHID report = {0};
	report.id = 3;
    report.buttons = button;
    HID_Mouse_SendReport(&report);
    report.buttons = 0;
    HID_Mouse_SendReport(&report);
}
