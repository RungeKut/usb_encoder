#include "hid_consumer.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static void HID_Media_SendReport(mediaHID *kb) {
    //while (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)kb, sizeof(mediaHID)) == USBD_BUSY);
	HAL_Delay(30);
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)kb, sizeof(mediaHID));
}

// Удобная обёртка: нажать и отпустить одну клавишу
void HID_Media_PressKeyOnce(uint16_t keycode) {
	mediaHID report = {0};
	report.id = HID_MEDIA_REPORT;
	report.lsb = LOBYTE(keycode);
	report.msb = HIBYTE(keycode);
	HID_Media_SendReport(&report);
	HAL_Delay(30);

	report.lsb = 0x00;
	report.msb = 0x00;
	HID_Media_SendReport(&report);
	HAL_Delay(30);
}

/* ========================================================================== */
/*               --- Функции отправки для System Control ---                  */
/* ========================================================================== */

static void HID_System_SendReport(customHID *custom) {
    //while (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)custom, sizeof(customHID)) == USBD_BUSY);
	HAL_Delay(30);
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)custom, sizeof(customHID));
}

void HID_System_SendCommand(uint8_t usage) {
	customHID report = {0};
	report.id = 4;
	report.keys = usage;
    HID_System_SendReport(&report);
    // Обязательно отпустить!
    report.keys = 0;
    HID_System_SendReport(&report);
}
