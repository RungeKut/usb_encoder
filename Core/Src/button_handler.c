#include "button_handler.h"
#include "main.h"

// === Настройки ===
#define DEBOUNCE_MS      10U   // Антидребезг (меньше 30 может ловить помехи)
#define LONG_PRESS_MS    3000U // Порог долгого нажатия

// === Инициализация контекста ===
void Button_Init(button_ctx_t* ctx, GPIO_TypeDef* port, uint16_t pin)
{
    ctx->port = port;
    ctx->pin = pin;
    ctx->state = 0; // IDLE
    ctx->pending_event = BUTTON_EVT_NONE;
    ctx->last_raw = true; // Pull-up по умолчанию (не нажата)
}

// === Основной обработчик кнопки ===
void HandleButton(button_ctx_t* ctx)
{
    uint32_t now = HAL_GetTick();
    bool is_active = (HAL_GPIO_ReadPin(ctx->port, ctx->pin) == GPIO_PIN_RESET);

    // 1. Антидребезг
    if (is_active != ctx->last_raw) {
        ctx->last_raw = is_active;
        ctx->debounce_timer = now;
    }
    if ((now - ctx->debounce_timer) < DEBOUNCE_MS) return;
    bool stable = ctx->last_raw;

    // 2. Упрощённая машина состояний без Double Click
    switch (ctx->state) {
        case 0: // IDLE
            if (stable) {
                ctx->pending_event = BUTTON_EVT_SHORT_PRESS; // МГНОВЕННО при нажатии
                ctx->state = 1;
                ctx->press_start = now;
            }
            break;

        case 1: // PRESSED (ждём отпускания или долгое нажатие)
            if (!stable) {
                ctx->state = 0; // Кнопка отпущена
            } else if ((now - ctx->press_start) >= LONG_PRESS_MS) {
                ctx->pending_event = BUTTON_EVT_LONG_PRESS;
                ctx->state = 2;
            }
            break;

        case 2: // LONG_HELD
            if (!stable) ctx->state = 0; // Ждём физического отпускания
            break;
    }
}

// === Получение события (вызывать в main) ===
button_event_t GetButtonEvent(button_ctx_t* ctx)
{
    button_event_t evt = ctx->pending_event;
    ctx->pending_event = BUTTON_EVT_NONE;
    return evt;
}
