#include "hid_keyboard.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ========================================================================== */
/*                 --- Функции отправки для клавиатуры ---                    */
/* ========================================================================== */

void HID_Keyboard_SendReport(keyboardHID *kb) {
	// Неблокирующее ожидание готовности USB
    //while (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)kb, sizeof(keyboardHID)) == USBD_BUSY);
	//HAL_Delay(30);
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)kb, sizeof(keyboardHID));
}

// Удобная обёртка: нажать и отпустить одну клавишу
void HID_KB_PressKeyOnce(uint8_t keycode, uint8_t modifier) {
	keyboardHID report = {0};
	report.id = 1;
	report.modifier = modifier;
	report.keycode[0] = keycode;
	HID_Keyboard_SendReport(&report);
	// Отпустить
	report.id = 1;
	report.modifier = 0;
	report.keycode[0] = 0;
	HID_Keyboard_SendReport(&report);
}
