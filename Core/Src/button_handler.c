#include "button_handler.h"
#include "main.h"

// === Настройки ===
#define DEBOUNCE_DELAY_MS       40U
#define SHORT_PRESS_MAX_MS     300U
#define LONG_PRESS_DURATION_MS 1000U
#define DOUBLE_CLICK_TIMEOUT_MS 350U

// === Инициализация контекста ===
void Button_Init(button_ctx_t* ctx, GPIO_TypeDef* port, uint16_t pin)
{
    ctx->port = port;
    ctx->pin = pin;
    
    // Инициализируем состояние текущим уровнем пина
    bool initial_state = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
    ctx->last_raw = initial_state;
    ctx->last_stable = initial_state;
    ctx->last_change_time = HAL_GetTick();
    
    ctx->pressed = false;
    ctx->press_start_time = 0;
    ctx->long_press_executed = false;
    ctx->last_release_time = 0;
    ctx->waiting_for_double = false;
    ctx->pending_event = BUTTON_EVT_NONE;
}

// === Вспомогательная функция: чтение с debounce ===
static bool ReadButtonStable(button_ctx_t* ctx)
{
    bool raw = (HAL_GPIO_ReadPin(ctx->port, ctx->pin) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    
    if (raw != ctx->last_raw) {
        ctx->last_raw = raw;
        ctx->last_change_time = now;
    }

    // Состояние стабильно, если не менялось дольше DEBOUNCE_DELAY_MS
    if ((now - ctx->last_change_time) > DEBOUNCE_DELAY_MS) {
        ctx->last_stable = raw;
    }
    return ctx->last_stable;
}

// === Основной обработчик кнопки ===
void HandleButton(button_ctx_t* ctx)
{
    uint32_t now = HAL_GetTick();
    bool is_pressed = ReadButtonStable(ctx);

    if (is_pressed) {
        // Кнопка нажата
        if (!ctx->pressed) {
            // Только что нажали
            ctx->pressed = true;
            ctx->press_start_time = now;
            ctx->long_press_executed = false;
            // Не сбрасываем waiting_for_double — может быть второй клик!
        } else {
            // Уже нажата — проверяем долгое нажатие
            if (!ctx->long_press_executed && (now - ctx->press_start_time >= LONG_PRESS_DURATION_MS)) {
                ctx->long_press_executed = true;
                ctx->waiting_for_double = false;
                ctx->pending_event = BUTTON_EVT_LONG_PRESS;
            }
        }
    } else {
        // Кнопка отпущена
        if (ctx->pressed) {
            uint32_t press_duration = now - ctx->press_start_time;
            ctx->pressed = false;
            ctx->last_release_time = now;
            
            if (!ctx->long_press_executed) {
                // Уже обработано выше
            } else {
                if (press_duration < SHORT_PRESS_MAX_MS) {
                    if (ctx->waiting_for_double) {
                        // Второй клик
                        ctx->waiting_for_double = false;
                        ctx->pending_event = BUTTON_EVT_DOUBLE_CLICK;
                    } else {
                        // Первый клик — ждём второй
                        ctx->waiting_for_double = true;
                        ctx->last_release_time = now; // обновляем время для таймаута
                    }
                }
                // Игнорируем нажатия от 300 до 1000 мс
            }
        }
    }

    // Проверка таймаута двойного клика
    if (ctx->waiting_for_double && (now - ctx->last_release_time > DOUBLE_CLICK_TIMEOUT_MS)) {
        ctx->waiting_for_double = false;
        ctx->pending_event = BUTTON_EVT_SHORT_PRESS;
    }
}

// === Получение события (вызывать в main) ===
button_event_t GetButtonEvent(button_ctx_t* ctx)
{
    button_event_t evt = ctx->pending_event;
    ctx->pending_event = BUTTON_EVT_NONE;
    return evt;
}
