#ifndef HID_MOUSE_H
#define HID_MOUSE_H
#include "hid_codes.h"
#include "usbd_hid.h"

#define MOUSE_STEP 5 // Шаг перемещения мыши

extern bool axis_mouse_move;

void HID_Mouse_Move(int8_t x, int8_t y);
void HID_Mouse_Wheel(int8_t delta);
void HID_Mouse_Click(uint8_t button);

#endif /* HID_MOUSE_H */
