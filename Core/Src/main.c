/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//https://www.usb.org/sites/default/files/hut1_3_0.pdf
//Переключение режима энкодера
typedef enum {
  MODE_ENCODER,
	MODE_CONSUMER,
  MODE_KEYBOARD,
  MODE_MOUSE,
	MODE_COUNT        // ← специальный "счётчик" количества элементов — всегда последним!
} device_mode_t;

device_mode_t current_mode = MODE_ENCODER;

// Состояние кнопки энкодера
bool button_pressed = false;
uint32_t button_press_time = 0;

// Счётчики энкодера
int32_t prevCounter = 0;
int32_t currCounter = 0;
int32_t delta = 0;

// Для двойного нажатия
#define DOUBLE_CLICK_TIMEOUT  200  // мс
static uint32_t last_click_time = 0;
static bool waiting_for_double = false;

// Consumer команды
typedef struct {
    const char* name;
    uint16_t usage;
} consumer_cmd_t;

consumer_cmd_t consumer_list[] = {
    // === Системные команды ===
    {"Power",        0x0030},
    {"Sleep",        0x0032},
    {"Wake Up",      0x0034},
    {"Power Down",   0x0031},
    {"Reset",        0x0033},

    // === Управление экраном ===
    {"Screen Off",   0x019F}, // Brightness Down до 0
    {"Screen On",    0x01A0}, // Brightness Up
    {"Screen Dim",   0x01B2}, // Dimmer

    // === Аудио: громкость ===
    {"Vol Up",       0x00E9},
    {"Vol Down",     0x00EA},
    {"Vol Mute",     0x00E2},

    // === Медиа: воспроизведение ===
    {"Play/Pause",   0x00CD},
    {"Stop",         0x00B7},
    {"Next Track",   0x00B5},
    {"Prev Track",   0x00B6},
    {"Fast Forward", 0x00B3},
    {"Rewind",       0x00B4},

    // === Запуск приложений ===
    {"Mail",         0x018A},
    {"Browser",      0x0182},
    {"Calc",         0x0192},
    {"Music",        0x0183},
    {"Search",       0x0221},

    // === Управление питанием ноутбука ===
    {"Battery",      0x01A5}, // Show battery status
    {"Hibernate",    0x01AC},

    // === Дополнительно ===
    {"Eject",        0x00B8},
    {"Record",       0x00B2},
    {"Random Play",  0x00C1},
    {"Repeat",       0x00C2},
    {"Menu",         0x0180}, // Show menu
    {"Help",         0x0181},
    {"WWW Home",     0x0194},
    {"Back",         0x0195}, // Browser back
    {"Forward",      0x0196}, // Browser forward
    {"Refresh",      0x0197},
    {"Bookmarks",    0x0198},
};

#define CONSUMER_COUNT (sizeof(consumer_list)/sizeof(consumer_list[0]))
uint8_t consumer_index = 0;

// Список кнопок клавиатуры
// Все печатающие клавиши, которые имеют альтернативу с Shift (US QWERTY)
uint8_t key_list[] = {
    // Буквы (a-z)
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
    0x1C, 0x1D,

    // Цифры и символы верхнего ряда
    0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
    0x26, 0x27,

    // Ряд под цифрами
    0x2D, 0x2E, // - =
    0x2F, 0x30, // [ ]
    0x31,       // \
    0x33, 0x34, // ; '
    0x35,       // `
    0x36, 0x37, 0x38 // , . /
};

#define KEY_COUNT (sizeof(key_list)/sizeof(key_list[0]))
uint8_t logical_index = 0;
bool apply_input_key = true;

// Шаг перемещения мыши
#define MOUSE_STEP 5

//Mouse HID Report
typedef struct {
	uint8_t buttons;   // bit 0 = left, 1 = right, 2 = middle
	int8_t x;          // movement X
	int8_t y;          // movement Y
	int8_t wheel;      // scroll
} mouseHID;

//Keyboard HID Report
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} keyboardHID;

keyboardHID keyboardhid = {0};

/* ========================================================================== */
/*                 --- Функции отправки для клавиатуры ---                    */
/* ========================================================================== */

void SendKeyboardReport(keyboardHID *kb) {
    USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)kb, sizeof(keyboardHID), HID_KEYBOARD_EP);
    HAL_Delay(2); // небольшая задержка для надёжности
}

// Удобная обёртка: нажать и отпустить одну клавишу
void PressKeyOnce(uint8_t keycode, uint8_t modifier) {
    keyboardHID report = {0};
    report.MODIFIER = modifier;
    report.KEYCODE1 = keycode;
    SendKeyboardReport(&report);
    HAL_Delay(10);
    // Отпустить
    report.MODIFIER = 0;
    report.KEYCODE1 = 0;
    SendKeyboardReport(&report);
}

/* ========================================================================== */
/*                    --- Функции отправки для мыши ---                       */
/* ========================================================================== */

void SendMouseReport(mouseHID *mouse) {
    USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)mouse, sizeof(mouseHID), HID_MOUSE_EP);
    HAL_Delay(2);
}

// Ось перемещения мыши: false - Ось X, true - Ось Y
bool axis_mouse_move = false;

void MouseMove(int8_t x, int8_t y) {
    mouseHID report = {0};
    report.x = x;
    report.y = y;
    SendMouseReport(&report);
}

void MouseClick(uint8_t button) {
    mouseHID report = {0};
    report.buttons = button;
    SendMouseReport(&report);
    HAL_Delay(20);
    report.buttons = 0;
    SendMouseReport(&report);
}

/* ========================================================================== */
/*             --- Функции отправки для медиа устройства ---                  */
/* ========================================================================== */

void SendConsumerCommand(uint16_t usage) {
    uint8_t report[2] = {
        (uint8_t)(usage & 0xFF),        // младший байт
        (uint8_t)((usage >> 8) & 0xFF)  // старший байт
    };
    USBD_HID_SendReport_EP(&hUsbDeviceFS, report, 2, HID_CONSUMER_EP);
    HAL_Delay(100);
    // Обязательно отпустить!
    uint8_t release[2] = {0};
    USBD_HID_SendReport_EP(&hUsbDeviceFS, release, 2, HID_CONSUMER_EP);
		HAL_Delay(100);
}

void SendConsumerByIndex(uint8_t index) {
    if (index < sizeof(consumer_list) / sizeof(consumer_list[0])) {
        SendConsumerCommand(consumer_list[index].usage);
    }
}

/* ========================================================================== */
/*                     --- Функция обработки энкодера ---                     */
/* ========================================================================== */
void HandleEncoder(void) {
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
		HAL_Delay(10);
    if((delta > -10) && (delta < 10)) {
      // здесь обрабатываем поворот энкодера на delta щелчков
			HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
			
			if (current_mode == MODE_ENCODER) {
				// delta положительная или отрицательная в зависимости от направления вращения
				if (delta < 0)
				{
					PressKeyOnce(0x52, 0x00);  // press 'UP'
				}
				if (delta > 0)
				{
					PressKeyOnce(0x51, 0x00);  // press 'DOWN'
				}
				
			} else if (current_mode == MODE_CONSUMER) {
                consumer_index = (consumer_index - delta + CONSUMER_COUNT) % CONSUMER_COUNT;
				
			} else if (current_mode == MODE_KEYBOARD) {
        logical_index = (logical_index - delta + 2 * KEY_COUNT) % (2 * KEY_COUNT);
				// Определяем физическую клавишу и нужно ли Shift
        uint8_t physical_index = logical_index / 2; // 0,0,1,1,2,2,...
        bool use_shift = (logical_index % 2 == 0);  // чётный → с Shift, нечётный → без Shift
				
				// Стираем предыдущую букву (если не первое нажатие)
        if (!apply_input_key) {
          PressKeyOnce(0x2A, 0); // Backspace
        }
				
				uint8_t modifier = use_shift ? 0x02 : 0x00;
        PressKeyOnce(key_list[physical_index], modifier);

        apply_input_key = false;
				
      } else if (current_mode == MODE_MOUSE) {
				uint8_t RoadLength = delta * MOUSE_STEP;
				if (axis_mouse_move)
					MouseMove(RoadLength, 0);
				else
					MouseMove(0, RoadLength);
      }
    }
  }
}

/* ========================================================================== */
/*                 --- Функция обработки кнопки энкодера ---                  */
/* ========================================================================== */
//Одно короткое нажатие
void OneShortPress(void) {
	if (current_mode == MODE_ENCODER) {
    PressKeyOnce(0x28, 0x00);  // press 'Enter'
  } else if (current_mode == MODE_KEYBOARD) {
		apply_input_key = true;
  } else if (current_mode == MODE_MOUSE) {
    MouseClick(0x01); // Left click
  } else if (current_mode == MODE_CONSUMER) {
    SendConsumerCommand(consumer_list[consumer_index].usage);
		for (int i = 0; i <= consumer_index; i++) {
      // Включить LED
      HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
      HAL_Delay(20);
      // Отключить LED
      HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
      HAL_Delay(80);
    }
  }
}

//Одно долгое нажатие
void OneLongPress(void) {
  // Долгое нажатие → смена режима
  current_mode = (current_mode + 1) % MODE_COUNT;
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  // Мигни количеством режимов:
  // Отключить LED
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  for (int i = 0; i <= current_mode; i++) {
    // Включить LED
    HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    // Отключить LED
    HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
    HAL_Delay(400);
  }
}

//Одно двойное нажатие
void OneDoubleClick(void) {
	if (current_mode == MODE_ENCODER) {
    PressKeyOnce(0x2A, 0); // press 'Backspace'
	} else if (current_mode == MODE_KEYBOARD) {
    // Переключить раскладку (отправить Alt+Shift)
		PressKeyOnce(0x1F, 0x04); // Left Shift, Left Alt
  } else if (current_mode == MODE_CONSUMER) {
    SendConsumerCommand(consumer_list[28].usage); // Menu
	} else if (current_mode == MODE_MOUSE) {
    MouseClick(0x02); // Right click
		axis_mouse_move = axis_mouse_move ? false : true;
  }
}

//Обработчик нажатия
void HandleButton(void) {
  static bool button_pressed = false;
  static uint32_t press_start_time = 0;
  if (HAL_GPIO_ReadPin(ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin) == GPIO_PIN_RESET) {
    if (!button_pressed) {
      button_pressed = true;
      press_start_time = HAL_GetTick();
    }
  } else {
    if (button_pressed) {
      uint32_t press_duration = HAL_GetTick() - press_start_time;
      button_pressed = false;
					
      if (press_duration > 1000) { // Долгое нажатие
        OneLongPress();
				
      } else if (press_duration < 300) { // Короткое нажатие
        //Проверяем, не двойное ли
        if (waiting_for_double) {
          // Это второе нажатие → двойной клик!
          waiting_for_double = false;
          OneDoubleClick();
        } else {
          // Первое нажатие — ждём второе
          waiting_for_double = true;
          last_click_time = HAL_GetTick();
        }
      }
    }
  }

  // Проверка таймаута для двойного нажатия
  if (waiting_for_double && (HAL_GetTick() - last_click_time > DOUBLE_CLICK_TIMEOUT)) {
    waiting_for_double = false;
    // Обрабатываем как одиночное нажатие
    OneShortPress();
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_RTC_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
//	HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
//	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HandleEncoder();
    HandleButton();
    HAL_Delay(5); // небольшая задержка для стабильности
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
