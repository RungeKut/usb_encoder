#ifndef HID_QUEUE_H
#define HID_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define HID_QUEUE_SIZE      5
#define HID_REPORT_MAX_LEN  16  // Достаточно для любого вашего репорта (макс 9 байт + запас)

void HID_Queue_Init(void);
bool HID_Queue_Push(const uint8_t *report, uint16_t len);
void HID_Queue_Tick(void);
void HID_Queue_SetMinPause(uint16_t ms);
bool HID_Queue_IsFull(void);
bool HID_Queue_IsEmpty(void);

#endif
