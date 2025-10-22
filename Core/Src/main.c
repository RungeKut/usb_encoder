

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

#define CONSUMER_COUNT (sizeof(consumer_list)/sizeof(consumer_list[0]))
uint8_t consumer_index = 0;

enum {
	HIDKEY_NONE    = 0x00, // No key pressed
	HIDKEY_ERR_OVF = 0x01, // Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")
	//  0x02 Keyboard POST Fail
	//  0x03 Keyboard Error Undefined
	HIDKEY_A = 0x04, // Keyboard a and A
	HIDKEY_B = 0x05, // Keyboard b and B
	HIDKEY_C = 0x06, // Keyboard c and C
	HIDKEY_D = 0x07, // Keyboard d and D
	HIDKEY_E = 0x08, // Keyboard e and E
	HIDKEY_F = 0x09, // Keyboard f and F
	HIDKEY_G = 0x0a, // Keyboard g and G
	HIDKEY_H = 0x0b, // Keyboard h and H
	HIDKEY_I = 0x0c, // Keyboard i and I
	HIDKEY_J = 0x0d, // Keyboard j and J
	HIDKEY_K = 0x0e, // Keyboard k and K
	HIDKEY_L = 0x0f, // Keyboard l and L
	HIDKEY_M = 0x10, // Keyboard m and M
	HIDKEY_N = 0x11, // Keyboard n and N
	HIDKEY_O = 0x12, // Keyboard o and O
	HIDKEY_P = 0x13, // Keyboard p and P
	HIDKEY_Q = 0x14, // Keyboard q and Q
	HIDKEY_R = 0x15, // Keyboard r and R
	HIDKEY_S = 0x16, // Keyboard s and S
	HIDKEY_T = 0x17, // Keyboard t and T
	HIDKEY_U = 0x18, // Keyboard u and U
	HIDKEY_V = 0x19, // Keyboard v and V
	HIDKEY_W = 0x1a, // Keyboard w and W
	HIDKEY_X = 0x1b, // Keyboard x and X
	HIDKEY_Y = 0x1c, // Keyboard y and Y
	HIDKEY_Z = 0x1d, // Keyboard z and Z

	HIDKEY_1 = 0x1e, // Keyboard 1 and !
	HIDKEY_2 = 0x1f, // Keyboard 2 and @
	HIDKEY_3 = 0x20, // Keyboard 3 and #
	HIDKEY_4 = 0x21, // Keyboard 4 and $
	HIDKEY_5 = 0x22, // Keyboard 5 and %
	HIDKEY_6 = 0x23, // Keyboard 6 and ^
	HIDKEY_7 = 0x24, // Keyboard 7 and &
	HIDKEY_8 = 0x25, // Keyboard 8 and *
	HIDKEY_9 = 0x26, // Keyboard 9 and (
	HIDKEY_0 = 0x27, // Keyboard 0 and )

	HIDKEY_ENTER         = 0x28, // Keyboard Return (ENTER)
	HIDKEY_ESCAPE        = 0x29, // Keyboard ESCAPE
	HIDKEY_BACKSPACE     = 0x2a, // Keyboard DELETE (Backspace)
	HIDKEY_TAB           = 0x2b, // Keyboard Tab
	HIDKEY_SPACEBAR      = 0x2c, // Keyboard Spacebar
	HIDKEY_UNDERSCORE    = 0x2d, // Keyboard - and _
	HIDKEY_PLUS          = 0x2e, // Keyboard = and +
	HIDKEY_OPEN_BRACKET  = 0x2f, // Keyboard [ and {
	HIDKEY_CLOSE_BRACKET = 0x30, // Keyboard ] and }
	HIDKEY_BACKSLASH     = 0x31, // Keyboard backslash \ and pipe |
	HIDKEY_HASH          = 0x32, // Keyboard hash # and tilde ~
	HIDKEY_COLON         = 0x33, // Keyboard semicolon ; and :
	HIDKEY_QUOTE         = 0x34, // Keyboard quote ' and single quote "
	HIDKEY_TILDE         = 0x35, // Keyboard ` and ~ grave accent
	HIDKEY_COMMA         = 0x36, // Keyboard , and <
	HIDKEY_DOT           = 0x37, // Keyboard . and >
	HIDKEY_SLASH         = 0x38, // Keyboard / and ?
	HIDKEY_CAPS_LOCK     = 0x39, // Keyboard Caps Lock

	HIDKEY_F1  = 0x3a, // Keyboard F1
	HIDKEY_F2  = 0x3b, // Keyboard F2
	HIDKEY_F3  = 0x3c, // Keyboard F3
	HIDKEY_F4  = 0x3d, // Keyboard F4
	HIDKEY_F5  = 0x3e, // Keyboard F5
	HIDKEY_F6  = 0x3f, // Keyboard F6
	HIDKEY_F7  = 0x40, // Keyboard F7
	HIDKEY_F8  = 0x41, // Keyboard F8
	HIDKEY_F9  = 0x42, // Keyboard F9
	HIDKEY_F10 = 0x43, // Keyboard F10
	HIDKEY_F11 = 0x44, // Keyboard F11
	HIDKEY_F12 = 0x45, // Keyboard F12
	
	HIDKEY_PRINTSCREEN = 0x46, // Keyboard Print Screen70, 
	HIDKEY_SCROLL_LOCK = 0x47, // Keyboard Scroll Lock71, 
	HIDKEY_PAUSE       = 0x48, // Keyboard Pause72, 
	HIDKEY_INSERT      = 0x49, // Keyboard Insert73, 
	HIDKEY_HOME        = 0x4a, // Keyboard Home74, 
	HIDKEY_PAGEUP      = 0x4b, // Keyboard Page Up75, 
	HIDKEY_DELETE      = 0x4c, // Keyboard Delete Forward76, 
	HIDKEY_END         = 0x4d, // Keyboard End77, 
	HIDKEY_PAGEDOWN    = 0x4e, // Keyboard Page Down78, 
	HIDKEY_RIGHT       = 0x4f, // Keyboard Right Arrow79, 
	HIDKEY_LEFT        = 0x50, // Keyboard Left Arrow80, 
	HIDKEY_DOWN        = 0x51, // Keyboard Down Arrow81, 
	HIDKEY_UP          = 0x52, // Keyboard Up Arrow82, 

	HIDKEY_KP_NUM_LOCK = 0x53, // Keyboard Num Lock and Clear
	HIDKEY_KP_SLASH    = 0x54, // Keypad /
	HIDKEY_KP_ASTERISK = 0x55, // Keypad *
	HIDKEY_KP_MINUS    = 0x56, // Keypad -
	HIDKEY_KP_PLUS     = 0x57, // Keypad +
	HIDKEY_KP_ENTER    = 0x58, // Keypad ENTER
	HIDKEY_KP_1        = 0x59, // Keypad 1 and End
	HIDKEY_KP_2        = 0x5a, // Keypad 2 and Down Arrow
	HIDKEY_KP_3        = 0x5b, // Keypad 3 and PageDn
	HIDKEY_KP_4        = 0x5c, // Keypad 4 and Left Arrow
	HIDKEY_KP_5        = 0x5d, // Keypad 5
	HIDKEY_KP_6        = 0x5e, // Keypad 6 and Right Arrow
	HIDKEY_KP_7        = 0x5f, // Keypad 7 and Home
	HIDKEY_KP_8        = 0x60, // Keypad 8 and Up Arrow
	HIDKEY_KP_9        = 0x61, // Keypad 9 and Page Up
	HIDKEY_KP_0        = 0x62, // Keypad 0 and Insert
	HIDKEY_KP_DOT      = 0x63, // Keypad . and Delete

	HIDKEY_NON_US_BACKSLASH_AND_SLASH = 0x64, // Keyboard Non-US \ and |
	HIDKEY_APPLICATION = 0x65, // Keyboard Application
	HIDKEY_POWER   = 0x66, // Keyboard Power
	HIDKEY_KPEQUAL = 0x67, // Keypad =

	HIDKEY_F13 = 0x68, // Keyboard F13
	HIDKEY_F14 = 0x69, // Keyboard F14
	HIDKEY_F15 = 0x6a, // Keyboard F15
	HIDKEY_F16 = 0x6b, // Keyboard F16
	HIDKEY_F17 = 0x6c, // Keyboard F17
	HIDKEY_F18 = 0x6d, // Keyboard F18
	HIDKEY_F19 = 0x6e, // Keyboard F19
	HIDKEY_F20 = 0x6f, // Keyboard F20
	HIDKEY_F21 = 0x70, // Keyboard F21
	HIDKEY_F22 = 0x71, // Keyboard F22
	HIDKEY_F23 = 0x72, // Keyboard F23
	HIDKEY_F24 = 0x73, // Keyboard F24

	HIDKEY_EXECUTE     = 0x74, // Keyboard Execute
	HIDKEY_HELP        = 0x75, // Keyboard Help
	HIDKEY_MENU        = 0x76, // Keyboard Menu
	HIDKEY_SELECT      = 0x77, // Keyboard Select
	HIDKEY_STOP        = 0x78, // Keyboard Stop
	HIDKEY_AGAIN       = 0x79, // Keyboard Again
	HIDKEY_UNDO        = 0x7a, // Keyboard Undo
	HIDKEY_CUT         = 0x7b, // Keyboard Cut
	HIDKEY_COPY        = 0x7c, // Keyboard Copy
	HIDKEY_PASTE       = 0x7d, // Keyboard Paste
	HIDKEY_FIND        = 0x7e, // Keyboard Find
	HIDKEY_MUTE        = 0x7f, // Keyboard Mute
	HIDKEY_VOLUME_UP   = 0x80, // Keyboard Volume Up
	HIDKEY_VOLUME_DOWN = 0x81, // Keyboard Volume Down

	// 0x82  Keyboard Locking Caps Lock
	// 0x83  Keyboard Locking Num Lock
	// 0x84  Keyboard Locking Scroll Lock

	HIDKEY__KP_COMMA = 0x85, // Keypad Comma
	// 0x86  Keypad Equal Sign
	HIDKEY_RO = 0x87, // Keyboard International1
	HIDKEY_KATAKANAHIRAGANA = 0x88, // Keyboard International2
	HIDKEY_YEN = 0x89, // Keyboard International3
	HIDKEY_HENKAN = 0x8a, // Keyboard International4
	HIDKEY_MUHENKAN = 0x8b, // Keyboard International5
	HIDKEY_KPJPCOMMA = 0x8c, // Keyboard International6
	// 0x8d  Keyboard International7
	// 0x8e  Keyboard International8
	// 0x8f  Keyboard International9
	HIDKEY_HANGEUL = 0x90, // Keyboard LANG1
	HIDKEY_HANJA = 0x91, // Keyboard LANG2
	HIDKEY_KATAKANA = 0x92, // Keyboard LANG3
	HIDKEY_HIRAGANA = 0x93, // Keyboard LANG4
	HIDKEY_ZENKAKUHANKAKU = 0x94, // Keyboard LANG5
	// 0x95  Keyboard LANG6
	// 0x96  Keyboard LANG7
	// 0x97  Keyboard LANG8
	// 0x98  Keyboard LANG9
	// 0x99  Keyboard Alternate Erase
	// 0x9a  Keyboard SysReq/Attention
	// 0x9b  Keyboard Cancel
	// 0x9c  Keyboard Clear
	// 0x9d  Keyboard Prior
	// 0x9e  Keyboard Return
	// 0x9f  Keyboard Separator
	// 0xa0  Keyboard Out
	// 0xa1  Keyboard Oper
	// 0xa2  Keyboard Clear/Again
	// 0xa3  Keyboard CrSel/Props
	// 0xa4  Keyboard ExSel

	// 0xb0  Keypad 00
	// 0xb1  Keypad 000
	// 0xb2  Thousands Separator
	// 0xb3  Decimal Separator
	// 0xb4  Currency Unit
	// 0xb5  Currency Sub-unit
	HIDKEY_KPLEFTPAREN = 0xb6, // Keypad (
	HIDKEY_KPRIGHTPAREN = 0xb7, // Keypad )
	// 0xb8  Keypad {
	// 0xb9  Keypad }
	// 0xba  Keypad Tab
	// 0xbb  Keypad Backspace
	// 0xbc  Keypad A
	// 0xbd  Keypad B
	// 0xbe  Keypad C
	// 0xbf  Keypad D
	// 0xc0  Keypad E
	// 0xc1  Keypad F
	// 0xc2  Keypad XOR
	// 0xc3  Keypad ^
	// 0xc4  Keypad %
	// 0xc5  Keypad <
	// 0xc6  Keypad >
	// 0xc7  Keypad &
	// 0xc8  Keypad &&
	// 0xc9  Keypad |
	// 0xca  Keypad ||
	// 0xcb  Keypad :
	// 0xcc  Keypad #
	// 0xcd  Keypad Space
	// 0xce  Keypad @
	// 0xcf  Keypad !
	// 0xd0  Keypad Memory Store
	// 0xd1  Keypad Memory Recall
	// 0xd2  Keypad Memory Clear
	// 0xd3  Keypad Memory Add
	// 0xd4  Keypad Memory Subtract
	// 0xd5  Keypad Memory Multiply
	// 0xd6  Keypad Memory Divide
	// 0xd7  Keypad +/-
	// 0xd8  Keypad Clear
	// 0xd9  Keypad Clear Entry
	// 0xda  Keypad Binary
	// 0xdb  Keypad Octal
	// 0xdc  Keypad Decimal
	// 0xdd  Keypad Hexadecimal

	// modifier bit sets
	HIDKEY_MODIFIER_NONE = 0x00, 
	HIDKEY_MODIFIER_LEFT_CTRL = 0x01, 
	HIDKEY_MODIFIER_LEFT_SHIFT = 0x02, 
	HIDKEY_MODIFIER_LEFT_ALT = 0x04, 
	HIDKEY_MODIFIER_LEFT_UI = 0x08, 
	HIDKEY_MODIFIER_RIGHT_CTRL = 0x10, 
	HIDKEY_MODIFIER_RIGHT_SHIFT = 0x20, 
	HIDKEY_MODIFIER_RIGHT_ALT = 0x40, 
	HIDKEY_MODIFIER_RIGHT_UI = 0x80,

	// media keys
	HIDKEY_MEDIA_SCAN_NEXT   = 0x01,
	HIDKEY_MEDIA_SCAN_PREV   = 0x02,
	HIDKEY_MEDIA_STOP        = 0x04,
	HIDKEY_MEDIA_EJECT       = 0x08,
	HIDKEY_MEDIA_PAUSE       = 0x10,
	HIDKEY_MEDIA_MUTE        = 0x20,
	HIDKEY_MEDIA_VOLUME_UP   = 0x40,
	HIDKEY_MEDIA_VOLUME_DOWN = 0x80,
	
	HID_MEDIA_PAUSE     = 0x00B1, // pause
	HID_MEDIA_RECORD    = 0x00B3,
	HID_MEDIA_SCAN_NEXT = 0x00B5,
	HID_MEDIA_SCAN_PREV = 0x00B6,
	HID_MEDIA_STOP      = 0x00B7,
	HID_MEDIA_EJECT     = 0x00B8,
	HID_MEDIA_VOL_UP    = 0x00E9,
	HID_MEDIA_VOL_DONW  = 0x00EA,
	HID_MEDIA_PLAY      = 0x00CD, // play/pause

	KEY_MEDIA_PLAYPAUSE = 0xe8,
	KEY_MEDIA_STOPCD = 0xe9,
	KEY_MEDIA_PREVIOUSSONG = 0xea,
	KEY_MEDIA_NEXTSONG = 0xeb,
	KEY_MEDIA_EJECTCD = 0xec,
	KEY_MEDIA_VOLUMEUP = 0xed,
	KEY_MEDIA_VOLUMEDOWN = 0xee,
	KEY_MEDIA_MUTE = 0xef,
	KEY_MEDIA_WWW = 0xf0,
	KEY_MEDIA_BACK = 0xf1,
	KEY_MEDIA_FORWARD = 0xf2,
	KEY_MEDIA_STOP = 0xf3,
	KEY_MEDIA_FIND = 0xf4,
	KEY_MEDIA_SCROLLUP = 0xf5,
	KEY_MEDIA_SCROLLDOWN = 0xf6,
	KEY_MEDIA_EDIT = 0xf7,
	KEY_MEDIA_SLEEP = 0xf8,
	KEY_MEDIA_COFFEE = 0xf9,
	KEY_MEDIA_REFRESH = 0xfa,
	KEY_MEDIA_CALC = 0xfb,

	// === Системные команды ===
    HID_MEDIA_Power      = 0x0030,
    HID_MEDIA_Sleep      = 0x0032,
    HID_MEDIA_Wake_Up    = 0x0034,
    HID_MEDIA_Power_Down = 0x0031,
    HID_MEDIA_Reset      = 0x0033,

    // === Управление экраном ===
    HID_MEDIA_Screen_Off = 0x019F, // Brightness Down до 0
    HID_MEDIA_Screen_On  = 0x01A0, // Brightness Up
    HID_MEDIA_Screen_Dim = 0x01B2, // Dimmer

    // === Аудио: громкость ===
    HID_MEDIA_Vol_Up     = 0x00E9,
    HID_MEDIA_Vol_Down   = 0x00EA,
    HID_MEDIA_Vol_Mute   = 0x00E2,

    // === Медиа: воспроизведение ===
    HID_MEDIA_Play_Pause   = 0x00CD,
    HID_MEDIA_Stop         = 0x00B7,
    HID_MEDIA_Next_Track   = 0x00B5,
    HID_MEDIA_Prev_Track   = 0x00B6,
    HID_MEDIA_Fast_Forward = 0x00B3,
    HID_MEDIA_Rewind       = 0x00B4,

    // === Запуск приложений ===
    HID_MEDIA_Mail    = 0x018A,
    HID_MEDIA_Browser = 0x0182,
    HID_MEDIA_Calc    = 0x0192,
    HID_MEDIA_Music   = 0x0183,
    HID_MEDIA_Search  = 0x0221,

    // === Управление питанием ноутбука ===
    HID_MEDIA_Battery   = 0x01A5, // Show battery status
    HID_MEDIA_Hibernate = 0x01AC,

    // === Дополнительно ===
    HID_MEDIA_Eject       = 0x00B8,
    HID_MEDIA_Record      = 0x00B2,
    HID_MEDIA_Random_Play = 0x00C1,
    HID_MEDIA_Repeat      = 0x00C2,
    HID_MEDIA_Menu        = 0x0180, // Show menu
    HID_MEDIA_Help        = 0x0181,
    HID_MEDIA_WWW_Home    = 0x0194,
    HID_MEDIA_Back        = 0x0195, // Browser back
    HID_MEDIA_Forward     = 0x0196, // Browser forward
    HID_MEDIA_Refresh     = 0x0197,
    HID_MEDIA_Bookmarks   = 0x0198,
} KEYBOARD_KEY_LIST;

// Список кнопок клавиатуры
// Все печатающие клавиши, которые имеют альтернативу с Shift (US QWERTY)
uint8_t key_list[] = {
	// Нижний ряд клавиатуры
	HIDKEY_Z, HIDKEY_X, HIDKEY_C, HIDKEY_V, HIDKEY_B, HIDKEY_N, HIDKEY_M, HIDKEY_COMMA, HIDKEY_DOT, HIDKEY_SLASH,

	// Второй ряд снизу клавиатуры
	HIDKEY_A, HIDKEY_S, HIDKEY_D, HIDKEY_F, HIDKEY_G, HIDKEY_H, HIDKEY_J, HIDKEY_K, HIDKEY_L, HIDKEY_COLON, HIDKEY_QUOTE, HIDKEY_BACKSLASH,

	// Второй ряд сверху клавиатуры
	HIDKEY_Q, HIDKEY_W, HIDKEY_E, HIDKEY_R, HIDKEY_T, HIDKEY_Y, HIDKEY_U, HIDKEY_I, HIDKEY_O, HIDKEY_P, HIDKEY_OPEN_BRACKET, HIDKEY_CLOSE_BRACKET,

	// Верхний ряд клавиатуры
	HIDKEY_TILDE, HIDKEY_1, HIDKEY_2, HIDKEY_3, HIDKEY_4, HIDKEY_5, HIDKEY_6, HIDKEY_7, HIDKEY_8, HIDKEY_9, HIDKEY_0, HIDKEY_UNDERSCORE, HIDKEY_PLUS
};

#define KEY_COUNT (sizeof(key_list)/sizeof(key_list[0]))
uint8_t logical_index = 0;
bool apply_input_key = true;

// Шаг перемещения мыши
#define MOUSE_STEP 5

/* ========================================================================== */
/*                 --- Функции отправки для клавиатуры ---                    */
/* ========================================================================== */

void SendKeyboardReport(keyboardHID *kb) {
	USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)kb, HID_KEYBOARD_EP_SIZE, HID_KEYBOARD_EP);
	HAL_Delay(10);
}

void SendMediaReport(mediaHID *kb) {
	USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)kb, HID_MEDIA_EP_SIZE, HID_KEYBOARD_EP);
	HAL_Delay(10);
}

// Удобная обёртка: нажать и отпустить одну клавишу
void PressKeyOnce(uint8_t keycode, uint8_t modifier) {
	keyboardHID report = {0};
	report.id = 1;
	report.modifier = modifier;
	report.keycode[0] = keycode;
	SendKeyboardReport(&report);
	// Отпустить
	report.id = 1;
	report.modifier = 0;
	report.keycode[0] = 0;
	SendKeyboardReport(&report);
}


// Удобная обёртка: нажать и отпустить одну клавишу
void PressMediaKeyOnce(uint16_t keycode) {
	uint8_t report[3];
	report[0]= HID_MEDIA_REPORT;
	report[1]= LOBYTE(keycode);
	report[2]= HIBYTE(keycode);
	USBD_HID_SendReport_EP(&hUsbDeviceFS, report, sizeof(report), HID_KEYBOARD_EP);
	HAL_Delay(30);

	report[0]= HID_MEDIA_REPORT;
	report[1]= 0x00;
	report[2]= 0x00;
	USBD_HID_SendReport_EP(&hUsbDeviceFS, report, sizeof(report), HID_KEYBOARD_EP);
	HAL_Delay(30);
}

/* ========================================================================== */
/*                    --- Функции отправки для мыши ---                       */
/* ========================================================================== */

void SendMouseReport(mouseHID *mouse) {
    USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)mouse, sizeof(mouseHID), HID_MOUSE_EP);
	HAL_Delay(20);
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
    report.buttons = 0;
    SendMouseReport(&report);
}

/* ========================================================================== */
/*             --- Функции отправки для медиа устройства ---                  */
/* ========================================================================== */

void SendConsumerReport(сonsumerHID *сonsumer) {
	HAL_Delay(10);
    USBD_HID_SendReport_EP(&hUsbDeviceFS, (uint8_t *)сonsumer, sizeof(сonsumerHID), HID_CONSUMER_EP);
}

void SendConsumerCommand(uint16_t usage) {
	сonsumerHID report = {0};
	report.id = 2;
	report.keys1 = LOBYTE(usage);
	report.keys2 = HIBYTE(usage);
    SendConsumerReport(&report);
    // Обязательно отпустить!
    report.keys1 = 0;
	report.keys2 = 0;
    SendConsumerReport(&report);
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
					PressKeyOnce(HIDKEY_UP, HIDKEY_MODIFIER_NONE);  // press 'UP'
				}
				if (delta > 0)
				{
					PressKeyOnce(HIDKEY_DOWN, HIDKEY_MODIFIER_NONE);  // press 'DOWN'
				}
				
			} else if (current_mode == MODE_CONSUMER) {
                //consumer_index = (consumer_index - delta + CONSUMER_COUNT) % CONSUMER_COUNT;
				if (delta < 0)
				{
					PressMediaKeyOnce(0xE9);
				}
				if (delta > 0)
				{
					PressMediaKeyOnce(0xEA);
				}
			} else if (current_mode == MODE_KEYBOARD) {
        logical_index = (logical_index - delta + 2 * KEY_COUNT) % (2 * KEY_COUNT);
				// Определяем физическую клавишу и нужно ли Shift
        uint8_t physical_index = logical_index / 2; // 0,0,1,1,2,2,...
        bool use_shift = (logical_index % 2 == 0);  // чётный → с Shift, нечётный → без Shift
				
				// Стираем предыдущую букву (если не первое нажатие)
        if (!apply_input_key) {
          PressKeyOnce(HIDKEY_BACKSPACE, HIDKEY_MODIFIER_NONE); // Backspace
        }
				
				uint8_t modifier = use_shift ? HIDKEY_MODIFIER_LEFT_SHIFT : HIDKEY_MODIFIER_NONE;
        PressKeyOnce(key_list[physical_index], modifier);

        apply_input_key = false;
				
      } else if (current_mode == MODE_MOUSE) {
				uint8_t RoadLength = delta * MOUSE_STEP;
				if (axis_mouse_move)
					MouseMove(-RoadLength, 0);
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
		PressKeyOnce(HIDKEY_POWER, HIDKEY_MODIFIER_NONE);  // press 'Enter'
	} else if (current_mode == MODE_KEYBOARD) {
		apply_input_key = true;
	} else if (current_mode == MODE_MOUSE) {
		MouseClick(0x01); // Left click
	} else if (current_mode == MODE_CONSUMER) {
		PressMediaKeyOnce(0x0030);
//		SendConsumerCommand(consumer_list[consumer_index].usage);
//		for (int i = 0; i <= consumer_index; i++) {
//			// Включить LED
//			HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
//			HAL_Delay(20);
//			// Отключить LED
//			HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
//			HAL_Delay(80);
//		}
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
		//PressMediaKeyOnce(HIDKEY_MEDIA_VOLUME_UP);
		PressKeyOnce(HIDKEY_POWER, HIDKEY_MODIFIER_NONE);
	} else if (current_mode == MODE_KEYBOARD) {
		// Переключить раскладку (отправить Alt+Shift)
		PressKeyOnce(HIDKEY_MODIFIER_LEFT_SHIFT, HIDKEY_MODIFIER_LEFT_ALT); // Left Shift, Left Alt
	} else if (current_mode == MODE_CONSUMER) {
//		SendConsumerCommand(consumer_list[8].usage);
		PressMediaKeyOnce(0x0032);
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
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
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
