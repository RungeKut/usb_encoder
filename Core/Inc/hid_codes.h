#ifndef HID_CODES_H
#define HID_CODES_H
#include <stdint.h>
#include <stdbool.h>

/* === Режимы энкодера === */
typedef enum {
	MODE_ENCODER,
	MODE_CONSUMER,
	MODE_KEYBOARD,
	MODE_MOUSE,
	MODE_WHEEL,
	MODE_COUNT        // ← специальный "счётчик" количества элементов — всегда последним!
} device_mode_t;

extern device_mode_t current_mode;
extern bool apply_input_key;

/* === HID Key Codes (Keyboard) === */
static enum {
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
	
	// === Управление питанием ===
    HID_CUSTOM_SystemPowerDown = 0x81,
    HID_CUSTOM_SystemSleep     = 0x82,
    HID_CUSTOM_SystemWakeUp    = 0x83,
} KEYBOARD_KEY_LIST;

/* === Таблица клавиш (US QWERTY) === */
// Список кнопок клавиатуры
// Все печатающие клавиши, которые имеют альтернативу с Shift (US QWERTY)

static const uint8_t key_list[] = {
	// Нижний ряд клавиатуры
	HIDKEY_Z, HIDKEY_X, HIDKEY_C, HIDKEY_V, HIDKEY_B, HIDKEY_N, HIDKEY_M, HIDKEY_COMMA, HIDKEY_DOT, HIDKEY_SLASH,

	// Второй ряд снизу клавиатуры
	HIDKEY_A, HIDKEY_S, HIDKEY_D, HIDKEY_F, HIDKEY_G, HIDKEY_H, HIDKEY_J, HIDKEY_K, HIDKEY_L, HIDKEY_COLON, HIDKEY_QUOTE, HIDKEY_BACKSLASH,

	// Второй ряд сверху клавиатуры
	HIDKEY_Q, HIDKEY_W, HIDKEY_E, HIDKEY_R, HIDKEY_T, HIDKEY_Y, HIDKEY_U, HIDKEY_I, HIDKEY_O, HIDKEY_P, HIDKEY_OPEN_BRACKET, HIDKEY_CLOSE_BRACKET,

	// Верхний ряд клавиатуры
	HIDKEY_TILDE, HIDKEY_1, HIDKEY_2, HIDKEY_3, HIDKEY_4, HIDKEY_5, HIDKEY_6, HIDKEY_7, HIDKEY_8, HIDKEY_9, HIDKEY_0, HIDKEY_UNDERSCORE, HIDKEY_PLUS
};

#define KEY_COUNT 47
	
#endif /* HID_CODES_H */
