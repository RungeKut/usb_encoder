#include "hid_queue.h"
#include "usbd_hid.h"
#include "usb_device.h"
#include "main.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
    uint8_t data[HID_REPORT_MAX_LEN];
    uint16_t len;
} hid_report_t;

static hid_report_t queue[HID_QUEUE_SIZE];
static uint8_t head = 0;
static uint8_t tail = 0;
static uint8_t count = 0;
static uint16_t min_pause_ms = 30;
static uint32_t last_send_tick = 0;

/* Проверка готовности USB-эндпоинта (не блокирует, проверяет стек) */
static bool is_usb_tx_ready(void) {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return false;
    if (hUsbDeviceFS.pClassData == NULL) return false;
    
    USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hhid->state == HID_IDLE);
}

void HID_Queue_Init(void) {
    head = 0; tail = 0; count = 0;
    last_send_tick = 0;
    min_pause_ms = 30;
}

bool HID_Queue_Push(const uint8_t *report, uint16_t len) {
    if (count >= HID_QUEUE_SIZE || len == 0 || len > HID_REPORT_MAX_LEN) {
        return false; // Очередь полна или некорректная длина
    }
    
    queue[tail].len = len;
    for (uint8_t i = 0; i < len; i++) {
        queue[tail].data[i] = report[i];
    }
    tail = (tail + 1) % HID_QUEUE_SIZE;
    count++;
    return true;
}

void HID_Queue_SetMinPause(uint16_t ms) {
    min_pause_ms = (ms == 0) ? 1 : ms;
}

bool HID_Queue_IsFull(void)  { return count >= HID_QUEUE_SIZE; }
bool HID_Queue_IsEmpty(void) { return count == 0; }

/* Вызывать в main() каждый цикл. Неблокирующая. */
void HID_Queue_Tick(void) {
    if (count == 0) return;
    
    uint32_t now = HAL_GetTick();
    // Проверка минимальной паузы
    if ((now - last_send_tick) < min_pause_ms) return;
    // Проверка готовности USB
    if (!is_usb_tx_ready()) return;

    // Отправляем старейший отчёт
    USBD_HID_SendReport(&hUsbDeviceFS, queue[head].data, queue[head].len);
    
    last_send_tick = now;
    head = (head + 1) % HID_QUEUE_SIZE;
    count--;
}
