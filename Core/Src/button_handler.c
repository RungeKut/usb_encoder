#include "button_handler.h"
#include "main.h"
#include "gpio.h"

// === Настройки ===
#define DEBOUNCE_DELAY_MS       40U
#define SHORT_PRESS_MAX_MS     300U
#define LONG_PRESS_DURATION_MS 1000U
#define DOUBLE_CLICK_TIMEOUT_MS 350U

// === Состояния кнопки ===
static bool button_pressed = false;
static uint32_t press_start_time = 0;
static bool long_press_executed = false;
static uint32_t last_release_time = 0;
static bool waiting_for_double = false;

// === Событие (защищено от оптимизации)
static volatile button_event_t pending_event = BUTTON_EVT_NONE;

// === Вспомогательная функция: чтение с debounce ===
static bool ReadButtonStable(void)
{
    static bool last_raw = true;
    static bool last_stable = true;
    static uint32_t last_change_time = 0;

    bool raw = (HAL_GPIO_ReadPin(ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin) == GPIO_PIN_RESET);

    uint32_t now = HAL_GetTick();

    if (raw != last_raw) {
        last_raw = raw;
        last_change_time = now;
    }

    // Состояние стабильно, если не менялось дольше DEBOUNCE_DELAY_MS
    if ((now - last_change_time) > DEBOUNCE_DELAY_MS) {
        last_stable = raw;
    }

    return last_stable;
}

// === Основной обработчик кнопки ===
void HandleButton(void)
{
    uint32_t now = HAL_GetTick();
    bool is_pressed = ReadButtonStable();

    if (is_pressed) {
        // Кнопка нажата
        if (!button_pressed) {
            // Только что нажали
            button_pressed = true;
            press_start_time = now;
            long_press_executed = false;
            // Не сбрасываем waiting_for_double — может быть второй клик!
        } else {
            // Уже нажата — проверяем долгое нажатие
            if (!long_press_executed && (now - press_start_time >= LONG_PRESS_DURATION_MS)) {
                long_press_executed = true;
                waiting_for_double = false;
                pending_event = BUTTON_EVT_LONG_PRESS;
            }
        }
    } else {
        // Кнопка отпущена
        if (button_pressed) {
            uint32_t press_duration = now - press_start_time;
            button_pressed = false;
            last_release_time = now;

            if (long_press_executed) {
                // Уже обработано выше
            } else {
                if (press_duration < SHORT_PRESS_MAX_MS) {
                    if (waiting_for_double) {
                        // Второй клик
                        waiting_for_double = false;
                        pending_event = BUTTON_EVT_DOUBLE_CLICK;
                    } else {
                        // Первый клик — ждём второй
                        waiting_for_double = true;
                        last_release_time = now; // обновляем время для таймаута
                    }
                }
                // Игнорируем нажатия от 300 до 1000 мс
            }
        }
    }

    // Проверка таймаута двойного клика
    if (waiting_for_double && (now - last_release_time > DOUBLE_CLICK_TIMEOUT_MS)) {
        waiting_for_double = false;
        pending_event = BUTTON_EVT_SHORT_PRESS;
    }
}

// === Получение события (вызывать в main) ===
button_event_t GetButtonEvent(void)
{
    button_event_t evt = pending_event;
    pending_event = BUTTON_EVT_NONE;
    return evt;
}
