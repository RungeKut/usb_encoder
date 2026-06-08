#include "led_ctrl.h"
#include "main.h"
#include "spi.h"

// ===== НАСТРОЙКА PIN LATCH (RCLK) =====
// Укажите порт и пин, который вы назначили в CubeMX для RCLK регистра 595
#define LED_LATCH_GPIO_Port  lc_clk_GPIO_Port
#define LED_LATCH_Pin        lc_clk_Pin
// ==============================

extern SPI_HandleTypeDef hspi2; // Или hspi2, если используете другой SPI

// Внутренний буфер. Изначально 0xFF → все светодиоды ПОГАШЕНЫ (т.к. 1 = OFF при active-low)
static uint8_t led_buffer = 0xFF;

void LED_Init(void) {
    HAL_GPIO_WritePin(LED_LATCH_GPIO_Port, LED_LATCH_Pin, GPIO_PIN_RESET);
    led_buffer = 0xFF;
    LED_Update();
}

void LED_Set(led_id_t led, bool on) {
    if (led >= LED_COUNT) return;
    
    if (on) {
        led_buffer &= ~(1U << led); // Бит = 0 → светодиод ГОРИТ (активный низ)
    } else {
        led_buffer |= (1U << led);  // Бит = 1 → светодиод ПОГАШЕН
    }
}

void LED_SetMask(uint8_t mask) {
    // mask: 0 = горит, 1 = погашен (совпадает с внутренней логикой буфера)
    led_buffer = mask;
}

void LED_Update(void) {
	// 1. Проверка, что SPI инициализирован и готов
    if (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) return;
	
    // 2. Отправляем байт по SPI с таймаутом 1000 мс
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, &led_buffer, 1, 1000);
    
	if (status != HAL_OK) {
        // Если ошибка (например, не включено тактирование) → флаг или обработка
        return; 
    }
	
    // 3. Импульс защелкивания (RCLK)
    HAL_GPIO_WritePin(LED_LATCH_GPIO_Port, LED_LATCH_Pin, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP(); // Минимальная задержка для стабильности фронта
    HAL_GPIO_WritePin(LED_LATCH_GPIO_Port, LED_LATCH_Pin, GPIO_PIN_RESET);
}

//Включение/выключение отдельных LED
//LED_Set(LED_RX, true);   // Зажечь RX
//LED_Set(LED_BUSY, true); // Зажечь BUSY
//LED_Set(LED_ERR, false); // Погасить ERR
//LED_Update();            // Одна отправка в SPI (быстро и атомарно)

//Обновление статуса целиком (например, при обработке UART)
//// Например: ERR=1, OK=0, REMOTE=0, BUSY=1, RX=0, TX=1, ATTOFF=0, OVPOW=0
//// В битах: 1 0 0 1 0 1 0 0 = 0x94
//LED_SetMask(0x94); 
//LED_Update();

//Индикация работы шины
//void UART_TxEvent(void) {
//    LED_Set(LED_TX, true);
//    LED_Update();
//    // Погасить через 50 мс асинхронно:
//    HAL_Delay(50); // Или используйте таймер, чтобы не блокировать
//    LED_Set(LED_TX, false);
//    LED_Update();
//}
