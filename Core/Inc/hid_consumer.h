#ifndef HID_CONSUMER_H
#define HID_CONSUMER_H
#include "hid_codes.h"
#include "usbd_hid.h"

void HID_Media_PressKeyOnce(uint16_t keycode);
void HID_System_SendCommand(uint8_t usage);

#endif /* HID_CONSUMER_H */
