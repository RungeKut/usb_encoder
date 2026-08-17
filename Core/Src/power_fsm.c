#include "power_fsm.h"
#include "main.h"
#include "usbd_hid.h"
#include "hid_consumer.h"
#include "tim.h"

power_state_t current_power_state = PWR_STATE_IDLE;
bool global_power_state = false;

static uint32_t fsm_start_tick = 0;
#define TIMEOUT_CONNECT_MS    15000  // 15 сек на подключение
#define TIMEOUT_DISCONNECT_MS 20000  // 20 сек на отключение (Windows может тормозить)
#define STABLE_CONNECT_MS     2000   // 2 сек стабилизации
#define RELAY_OFF_DELAY_MS    5000   // 5 сек перед снятием питания

void PowerLed_Set(led_state_t state) {
    switch (state) {
		case LED_STATE_OFF: {
			HAL_GPIO_WritePin(LED_220_GPIO_Port, LED_220_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_POW_GPIO_Port, LED_POW_Pin, GPIO_PIN_SET);
			break;
		}
		case LED_STATE_RED: {
			HAL_GPIO_WritePin(LED_220_GPIO_Port, LED_220_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_POW_GPIO_Port, LED_POW_Pin, GPIO_PIN_SET);
			break;
		}
		case LED_STATE_GREEN: {
			HAL_GPIO_WritePin(LED_220_GPIO_Port, LED_220_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_POW_GPIO_Port, LED_POW_Pin, GPIO_PIN_RESET);
			break;
		}
		case LED_STATE_YELLOW: {
			HAL_GPIO_WritePin(LED_220_GPIO_Port, LED_220_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_POW_GPIO_Port, LED_POW_Pin, GPIO_PIN_RESET);
			break;
		}
		default: break;
	}
}

void PowerFSM_Init(void) {
    current_power_state = PWR_STATE_IDLE;
    global_power_state = false;
    HAL_GPIO_WritePin(COMP_ON_GPIO_Port, COMP_ON_Pin, GPIO_PIN_RESET);
	
	//HAL_GPIO_WritePin(ZYNQ_ON_GPIO_Port, ZYNQ_ON_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	TIM2->CCR4=0;
	
    PowerLed_Set(LED_STATE_OFF);
}

void PowerFSM_Request(bool turn_on) {
    if (current_power_state != PWR_STATE_IDLE) return; // Уже выполняется операция
    if ((turn_on && global_power_state) || (!turn_on && !global_power_state)) return; // Уже в нужном состоянии

    fsm_start_tick = HAL_GetTick();
    current_power_state = turn_on ? PWR_STATE_TURNING_ON : PWR_STATE_TURNING_OFF;
}

bool PowerFSM_IsBusy(void) {
    return current_power_state != PWR_STATE_IDLE;
}

uint32_t i;

void PowerFSM_Tick(void) {
    uint32_t now = HAL_GetTick();
    extern bool PC_RunState; // из usbd_hid.c

    switch (current_power_state) {
        case PWR_STATE_TURNING_ON: {
			// Включаем ключ платы SDR
//			HAL_GPIO_WritePin(ZYNQ_ON_GPIO_Port, ZYNQ_ON_Pin, GPIO_PIN_SET);
			for(uint16_t i = 0; i <= 65535; i++)
			{
				TIM2->CCR4 = i;
				
				volatile uint32_t d;
				// Число подобрано эмпирически для 72МГц и оптимизации -O3
				for(d = 0; d < 80; d++) // 500мс
				{
					__NOP(); // Пустая инструкция, чтобы компилятор точно не выкинул цикл
				}
			}
			TIM2->CCR4 = 720; // Гарантируем 100% в конце
//			TIM2->CCR4=5000;
//			HAL_Delay(500);
//			TIM2->CCR4=65535;
#ifdef INSTALL_PC
            // Включаем ключ компьютера
            HAL_GPIO_WritePin(COMP_ON_GPIO_Port, COMP_ON_Pin, GPIO_PIN_SET);
            current_power_state = PWR_STATE_WAIT_USB_CONNECT;
            fsm_start_tick = now;
            break;
        }

        case PWR_STATE_WAIT_USB_CONNECT: {
			uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_YELLOW);
            }
			
            if (PC_RunState) {
                current_power_state = PWR_STATE_WAIT_STABLE_CONNECT;
                fsm_start_tick = now;
            } else if (elapsed > TIMEOUT_CONNECT_MS) {
                current_power_state = PWR_STATE_ERROR_TIMEOUT_ON;
				fsm_start_tick = now;
            }
            break;
        }
#else
            current_power_state = PWR_STATE_WAIT_STABLE_CONNECT;
            fsm_start_tick = now;
            break;
        }
#endif
        case PWR_STATE_WAIT_STABLE_CONNECT: {
			uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_GREEN);
            }
			
            if (elapsed >= STABLE_CONNECT_MS) {
                // Стабильное подключение → зелёный
                PowerLed_Set(LED_STATE_GREEN);
                global_power_state = true;
                current_power_state = PWR_STATE_IDLE;
            }
            break;
        }

        case PWR_STATE_TURNING_OFF: {
#ifdef INSTALL_PC
            // Отправляем команду выключения ПК
            HID_System_SendCommand(HID_CUSTOM_SystemPowerDown);
            current_power_state = PWR_STATE_WAIT_USB_DISCONNECT;
            fsm_start_tick = now;
            break;
        }

        case PWR_STATE_WAIT_USB_DISCONNECT: {
			uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_YELLOW);
            }
			
            if (!PC_RunState) {
                current_power_state = PWR_STATE_DELAY_RELAY_OFF;
                fsm_start_tick = now;
            } else if (elapsed > TIMEOUT_DISCONNECT_MS) {
                current_power_state = PWR_STATE_ERROR_TIMEOUT_OFF;
				fsm_start_tick = now;
            }
            break;
        }
#else
            current_power_state = PWR_STATE_DELAY_RELAY_OFF;
            fsm_start_tick = now;
            break;
        }
#endif
        case PWR_STATE_DELAY_RELAY_OFF: {
			uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_YELLOW);
            }
			
            // Ждём 5 сек для разряда БП
            if (elapsed >= RELAY_OFF_DELAY_MS) {
				// Отключаем ключ платы SDR
				//HAL_GPIO_WritePin(ZYNQ_ON_GPIO_Port, ZYNQ_ON_Pin, GPIO_PIN_RESET);
				TIM2->CCR4=0;
				// Отключаем ключ компьютера
                HAL_GPIO_WritePin(COMP_ON_GPIO_Port, COMP_ON_Pin, GPIO_PIN_RESET);
                PowerLed_Set(LED_STATE_YELLOW);
                global_power_state = false;
                current_power_state = PWR_STATE_IDLE;
            }
            break;
        }

        case PWR_STATE_ERROR_TIMEOUT_ON: {
            uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_RED);
            }

            // Автоматический сброс FSM через 3 секунды
            if (elapsed >= 3000) {
				// Отключаем ключ платы SDR
			    //HAL_GPIO_WritePin(ZYNQ_ON_GPIO_Port, ZYNQ_ON_Pin, GPIO_PIN_RESET);
				TIM2->CCR4=0;
				// Отключаем ключ компьютера
                HAL_GPIO_WritePin(COMP_ON_GPIO_Port, COMP_ON_Pin, GPIO_PIN_RESET);
                PowerLed_Set(LED_STATE_YELLOW);
                global_power_state = false;
                current_power_state = PWR_STATE_IDLE;
            }
            break;
        }
		
		case PWR_STATE_ERROR_TIMEOUT_OFF: {
            uint32_t elapsed = now - fsm_start_tick;
            
            // Мигание каждые 500 мс (250 мс ВКЛ, 250 мс ВЫКЛ)
            if ((elapsed % 500) < 250) {
                PowerLed_Set(LED_STATE_OFF);
            } else {
                PowerLed_Set(LED_STATE_GREEN);
            }

            // Автоматический сброс FSM через 3 секунды
            if (elapsed >= 3000) {
                PowerLed_Set(LED_STATE_GREEN);
                global_power_state = true;
                current_power_state = PWR_STATE_IDLE;
            }
            break;
        }

        default: break;
    }
}
