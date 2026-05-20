#include "encoder_handler.h"
#include "main.h"
#include "tim.h"
#include "hid_keyboard.h"
#include "hid_consumer.h"
#include "hid_mouse.h"

/* ========================================================================== */
/*                     --- Функция обработки энкодера ---                     */
/* ========================================================================== */
// Счётчики энкодера
static int32_t prevCounter = 0;
static int32_t currCounter = 0;
static int32_t delta = 0;
static uint32_t encoder_last_tick = 0;
static uint8_t logical_index = 0;

void HandleEncoder(void) {
	// Для выполнения функции раз в 50мс.
	if ((now_tick - encoder_last_tick) < 50) { return; }
	encoder_last_tick = now_tick;
	
	currCounter = __HAL_TIM_GET_COUNTER(&htim1);
	currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
	if(currCounter > 32768/2) {
		// Преобразуем значения счетчика из:
		//  ... 32766, 32767, 0, 1, 2 ...
		// в значения:
		//  ... -2, -1, 0, 1, 2 ...
		currCounter = currCounter - 32768;
	}
	if(currCounter != prevCounter) {
		delta = currCounter-prevCounter;
		prevCounter = currCounter;
		// защита от дребезга контактов и переполнения счетчика
		// (переполнение будет случаться очень редко)
		//HAL_Delay(10);
		if((delta > -10) && (delta < 10)) {
			// здесь обрабатываем поворот энкодера на delta щелчков
			HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);

			if (current_mode == MODE_ENCODER) {
				// delta положительная или отрицательная в зависимости от направления вращения
				if (delta < 0)
				{
					HID_KB_PressKeyOnce(HIDKEY_UP, HIDKEY_MODIFIER_NONE);  // press 'UP'
				}
				if (delta > 0)
				{
					HID_KB_PressKeyOnce(HIDKEY_DOWN, HIDKEY_MODIFIER_NONE);  // press 'DOWN'
				}
				
			} else if (current_mode == MODE_CONSUMER) {
				//consumer_index = (consumer_index - delta + CONSUMER_COUNT) % CONSUMER_COUNT;
				if (delta < 0)
				{
					HID_Media_PressKeyOnce(HID_MEDIA_Vol_Up);
				}
				if (delta > 0)
				{
					HID_Media_PressKeyOnce(HID_MEDIA_Vol_Down);
				}
				
			} else if (current_mode == MODE_KEYBOARD) {
				logical_index = (logical_index - delta + 2 * KEY_COUNT) % (2 * KEY_COUNT);
				// Определяем физическую клавишу и нужно ли Shift
				uint8_t physical_index = logical_index / 2; // 0,0,1,1,2,2,...
				bool use_shift = (logical_index % 2 == 0);  // чётный → с Shift, нечётный → без Shift
				
				// Стираем предыдущую букву (если не первое нажатие)
				if (!apply_input_key) {
					HID_KB_PressKeyOnce(HIDKEY_BACKSPACE, HIDKEY_MODIFIER_NONE); // Backspace
				}
				
				uint8_t modifier = use_shift ? HIDKEY_MODIFIER_LEFT_SHIFT : HIDKEY_MODIFIER_NONE;
				HID_KB_PressKeyOnce(key_list[physical_index], modifier);

				apply_input_key = false;

			} else if (current_mode == MODE_MOUSE) {
				uint8_t RoadLength = delta * MOUSE_STEP;
				if (axis_mouse_move)
					HID_Mouse_Move(-RoadLength, 0);
				else
					HID_Mouse_Move(0, RoadLength);
			
			} else if (current_mode == MODE_WHEEL) {
				HID_Mouse_Wheel(delta);
			}
		}
	}
}
