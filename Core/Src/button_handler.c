#include "button_handler.h"
#include "main.h"

// === Настройки ===
#define DEBOUNCE_MS				40U
#define SHORT_PRESS_MAX_MS		300U
#define LONG_PRESS_MS			1000U
#define DOUBLE_CLICK_GAP_MS		350U

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

    // 1. Debounce
    if (is_active != ctx->last_raw) {
        ctx->last_raw = is_active;
        ctx->debounce_timer = now;
    }
    if ((now - ctx->debounce_timer) < DEBOUNCE_MS) return;
    bool stable = ctx->last_raw;

    // 2. State Machine
    switch (ctx->state)
    {
        case 0: // IDLE
            if (stable) {
                ctx->state = 1; // DEBOUNCE_DOWN
                ctx->debounce_timer = now;
            }
            break;

        case 1: // DEBOUNCE_DOWN
            if (!stable) { ctx->state = 0; break; } // Дребезг отпустил
            ctx->state = 2; // PRESSED
            ctx->press_start = now;
            break;

        case 2: // PRESSED
            if (!stable) {
                uint32_t dur = now - ctx->press_start;
                if (dur >= LONG_PRESS_MS) {
                    ctx->pending_event = BUTTON_EVT_LONG_PRESS;
                    ctx->state = 3; // LONG_HELD
                } else if (dur < SHORT_PRESS_MAX_MS) {
                    ctx->state = 4; // WAIT_DOUBLE
                    ctx->double_wait_start = now;
                } else {
                    ctx->state = 0; // Среднее нажатие (300..1000мс) игнорируем
                }
            } else if ((now - ctx->press_start) >= LONG_PRESS_MS) {
                ctx->pending_event = BUTTON_EVT_LONG_PRESS;
                ctx->state = 3; // LONG_HELD
            }
            break;

        case 3: // LONG_HELD
            if (!stable) ctx->state = 0; // Ждем физического отпускания
            break;

        case 4: // WAIT_DOUBLE (после первого короткого отпускания)
            if (stable) {
                // Второе нажатие обнаружено -> фиксируем Double Click
                ctx->pending_event = BUTTON_EVT_DOUBLE_CLICK;
                ctx->state = 5; // Переходим в состояние подавления
            } else if ((now - ctx->double_wait_start) >= DOUBLE_CLICK_GAP_MS) {
                // Таймаут истек -> это был одиночный клик
                ctx->pending_event = BUTTON_EVT_SHORT_PRESS;
                ctx->state = 0;
            }
            break;

        case 5: // WAIT_DOUBLE_RELEASE
            // Игнорируем всё, пока вторая кнопка не будет отпущена.
            // Это предотвращает повторный заход в логику короткого нажатия.
            if (!stable) ctx->state = 0; 
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
