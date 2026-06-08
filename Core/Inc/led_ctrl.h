#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <stdint.h>
#include <stdbool.h>

// Соответствие битов регистра 595 → светодиодов
typedef enum {
    LED_ERR = 0,
    LED_OK,
    LED_REMOTE,
    LED_BUSY,
    LED_RX,
    LED_TX,
    LED_ATTOFF,
    LED_OVPOW,
    LED_COUNT
} led_id_t;

void LED_Init(void);
void LED_Set(led_id_t led, bool on);   // on=true → зажечь, false → погасить
void LED_SetMask(uint8_t mask);        // Прямая запись маски (0=горит, 1=погашен)
void LED_Update(void);                 // Отправка в SPI + защелкивание RCLK

#endif
