#ifndef HID_KEYBOARD_H
#define HID_KEYBOARD_H
#include "hid_codes.h"
#include "usbd_hid.h"

void HID_KB_PressKeyOnce(uint8_t keycode, uint8_t modifier);

#endif /* HID_KEYBOARD_H */
