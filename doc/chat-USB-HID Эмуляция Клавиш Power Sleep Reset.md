### USER
на stm32f103c8t6 мне нужно сделать USB-HID эмулятор нажатия на клавиатуре кнопок power, sleep и reset. Как будет выглядеть настройка дескриптора?

### ASSISTANT
Для реализации USB-HID устройства на STM32F103C8T6, эмулирующего нажатие **специальных клавиш** — **Power**, **Sleep** и **Reset** — вам нужно использовать **Consumer Page** (0x0C) или **Generic Desktop Page** (0x01) в HID-дескрипторе, в зависимости от того, как именно вы хотите эмулировать эти команды.

Однако важно понимать следующее:

- **Power**, **Sleep** и **Wake** — это **Consumer Usage IDs**, определённые в **HID Usage Tables** (раздел **Consumer Page**, Usage ID 0x30, 0x31, 0x32 и т.д.).
- **Reset** — **не является стандартной HID-клавишей**. Обычно "Reset" не передаётся через HID — это аппаратное действие. Но если вы хотите эмулировать "перезагрузку системы", то в Windows/Linux это не делается через HID. Однако можно использовать **System Control Page** (0x01, Usage 0x81–0x83) для **System Power Down**, **System Sleep**, **System Wake Up**.

---

### ✅ Рекомендуемый подход

Используйте **System Control Page** (Generic Desktop, Usage = 0x81–0x83):

| Действие        | Usage ID | Page       | Описание                    |
|------------------|----------|------------|-----------------------------|
| System Power Down| 0x81     | 0x01 (GDC) | Выключает систему           |
| System Sleep     | 0x82     | 0x01 (GDC) | Переводит в сон             |
| System Wake Up   | 0x83     | 0x01 (GDC) | Пробуждает систему          |

> **Примечание**: "Reset" (перезагрузка) **не поддерживается** стандартным HID. Вы можете использовать **Power Down**, а ОС может интерпретировать это как выключение или перезагрузку в зависимости от настроек, но **прямой "Reset" через HID невозможен**.

---

### 📄 Пример HID-дескриптора (System Control)

Этот дескриптор описывает **1-байтовый отчёт**, где каждый бит соответствует одному из действий (Power Down, Sleep, Wake Up). Но по стандарту **System Control** передаётся как **16-битное значение** (bitmask), где биты 0–7 — зарезервированы, а биты 8–15 — управляют действиями.

Однако проще использовать **1-байтный отчёт**, где вы передаёте **одну команду за раз** (например, 0x01 = Power Down, 0x02 = Sleep и т.д.), но это **не соответствует стандарту**.

Правильнее — использовать **2-байтный System Control Report**, как в официальных HID-спецификациях.

---

### ✅ Стандартный HID Report Descriptor для System Control

```c
0x05, 0x01,        // Usage Page (Generic Desktop)
0x09, 0x80,        // Usage (System Control)
0xA1, 0x01,        // Collection (Application)
0x15, 0x00,        // Logical Minimum (0)
0x26, 0xFF, 0x00,  // Logical Maximum (255)
0x19, 0x00,        // Usage Minimum (0)
0x2A, 0xFF, 0x00,  // Usage Maximum (255)
0x75, 0x08,        // Report Size (8)
0x95, 0x02,        // Report Count (2)
0x81, 0x00,        // Input (Data, Array, Absolute)
0xC0               // End Collection
```

> Этот дескриптор описывает **2-байтный Input Report**, но для **Output (от устройства к хосту)** лучше использовать **Feature или Output**, но HID-клавиатура обычно использует **Interrupt OUT endpoint** с **Output Report**.

Однако для **System Control** чаще используется **Input Report** (устройство → хост), и хост его читает.

Но в случае эмуляции клавиш (как клавиатура), вы отправляете **отчёт через Interrupt OUT endpoint**, и хост интерпретирует его как команду.

---

### 🔧 Альтернатива: Использовать Consumer Page (если система поддерживает)

Если вы хотите использовать **Consumer Page**, дескриптор будет таким:

```c
0x05, 0x0C,        // Usage Page (Consumer)
0x09, 0x01,        // Usage (Consumer Control)
0xA1, 0x01,        // Collection (Application)
0x15, 0x00,        // Logical Minimum (0)
0x26, 0xFF, 0x00,  // Logical Maximum (255)
0x19, 0x00,        // Usage Minimum (0)
0x2A, 0xFF, 0x00,  // Usage Maximum (255)
0x75, 0x08,        // Report Size (8)
0x95, 0x02,        // Report Count (2)
0x81, 0x00,        // Input (Data, Array, Absolute)
0xC0               // End Collection
```

И тогда вы отправляете **Consumer Usage ID**:

- **Power**: `0x30`
- **Sleep**: `0x31`
- **Wake Up**: `0x32`

Но опять же — **Reset нет**.

---

### 📌 Рекомендация

Для максимальной совместимости с Windows/Linux используйте **System Control Page** с **2-байтным отчётом**, где:

- `0x01 0x00` → System Power Down  
- `0x02 0x00` → System Sleep  
- `0x03 0x00` → System Wake Up  

> Формат: младший байт — команда, старший — 0.

---

### 🧩 Пример кода отправки (псевдокод для STM32 + USB)

```c
uint8_t report[2] = {0};

// Power Down
report[0] = 0x01;
USBD_HID_SendReport(&hUsbDeviceFS, report, 2);

// Sleep
report[0] = 0x02;
USBD_HID_SendReport(&hUsbDeviceFS, report, 2);
```

> Убедитесь, что ваш USB-стек (например, STM32CubeMX + USB Device HID) настроен на **2-байтный отчёт** и соответствует дескриптору выше.

---

### 📎 Важно

1. **"Reset" не поддерживается HID** — вы не сможете перезагрузить ПК через HID-команду.
2. Убедитесь, что ваше устройство определяется как **System Control** или **Consumer Control**, а не как обычная клавиатура.
3. В дескрипторе устройства (USB Device Descriptor) укажите:
   - `bInterfaceClass = 0x03` (HID)
   - `bInterfaceSubClass = 0x00` или `0x01`
   - `bInterfaceProtocol = 0x00` (никакой, или 0x01 для клавиатуры, но лучше 0x00 для System Control)

---

Если вы используете **STM32CubeMX**, вы можете:

1. Создать проект с USB Device → HID.
2. Заменить стандартный HID-дескриптор на приведённый выше.
3. Настроить размер отчёта на 2 байта.
4. Отправлять нужные команды через `USBD_HID_SendReport()`.

---

Хочешь — могу сгенерировать полный пример для STM32CubeIDE с правильным дескриптором и функцией отправки.

### USER
Я его настроил как 3й интерфейс:
	// Configuration Descriptor (9 byte)
	/*  1 byte*/ 0x09,	//bLength: Configuration Descriptor size - длина дескриптора конфигурации
	/*  2 byte*/ USB_DESC_TYPE_CONFIGURATION,	//bDescriptorType: Configuration - тип дескриптора - конфигурация
	/*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ),	//wTotalLength - общий размер всего дерева под данной конфигурацией в байтах
	/*  5 byte*/ 0x03,	//bNumInterfaces -  в конфигурации три интерфейса
	/*  6 byte*/ 0x01,	//bConfigurationValue - индекс текущей конфигурации
	/*  7 byte*/ 0x00,	//iConfiguration - индекс строки, которая описывает эту конфигурацию
	/*  8 byte*/ 0x40,	//bmAttributes - признак того, что устройство будет питаться от шины USB
	/*  9 byte*/ 0x32,	//MaxPower (100 mA)

	#ifdef KEYBOARD_CONTROL
	// ============ INTERFACE 0: KEYBOARD ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ 0x00,	//bInterfaceNumber: Number of Interface
		/*  4 byte*/ 0x00,	//bAlternateSetting: Alternate setting
		/*  5 byte*/ 0x01,	//bNumEndpoints
		/*  6 byte*/ 0x03,	//bInterfaceClass: HID
		/*  7 byte*/ 0x01,	//bInterfaceSubClass : 1=BOOT, 0=no boot
		/*  8 byte*/ 0x01,	//nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
		/*  9 byte*/ 0x00,	//iInterface: Index of string descriptor

			// HID Descriptor (9 byte)
			/*  1 byte*/ 0x09,	//bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE, // bDescriptorType: HID
			/*3-4 byte*/ 0x11, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_KEYBOARD_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_KEYBOARD_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
	#endif

	#ifdef MOUSE_CONTROL
	// ============ INTERFACE 1: MOUSE ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ 0x01,	//bInterfaceNumber: Number of Interface
		/*  4 byte*/ 0x00,	//bAlternateSetting: Alternate setting
		/*  5 byte*/ 0x01,	//bNumEndpoints
		/*  6 byte*/ 0x03,	//bInterfaceClass: HID
		/*  7 byte*/ 0x01,	//bInterfaceSubClass : 1=BOOT, 0=no boot
		/*  8 byte*/ 0x02,	//nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
		/*  9 byte*/ 0x00,	//iInterface: Index of string descriptor

			// HID Descriptor (9 byte)
			/*  1 byte*/ 0x09,	//bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE,	//bDescriptorType: HID
			/*3-4 byte*/ 0x11, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_MOUSE_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_MOUSE_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
	#endif

	#ifdef CONSUMER_CONTROL
	// ============ INTERFACE 2: CONSUMER CONTROL ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ 0x02,	//bInterfaceNumber: Number of Interface
		/*  4 byte*/ 0x00,	//bAlternateSetting: Alternate setting
		/*  5 byte*/ 0x01,	//bNumEndpoints
		/*  6 byte*/ 0x03,	//bInterfaceClass: HID
		/*  7 byte*/ 0x00,	//bInterfaceSubClass : 1=BOOT, 0=no boot
		/*  8 byte*/ 0x00,	//nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
		/*  9 byte*/ 0x00,	//iInterface: Index of string descriptor

			// HID Descriptor (9 byte)
			/*  1 byte*/ 0x09,	//bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE,	//bDescriptorType: HID
			/*3-4 byte*/ 0x11, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_CONSUMER_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_CONSUMER_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
Адреса такие дал:
#define HID_KEYBOARD_EP        0x81U
#define HID_MOUSE_EP           0x82U
#define HID_CONSUMER_EP        0x83U

#define USB_HID_CONFIG_DESC_SIZ       9U + 25U + 25U + 25U

#define HID_KEYBOARD_EP_SIZE   0x08U
#define HID_MOUSE_EP_SIZE      0x04U
#define HID_CONSUMER_EP_SIZE   0x02U

__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  USB_LEN_DEV_DESC,           // bLength - общая длина дескриптора устройства в байтах
  USB_DESC_TYPE_DEVICE,       // bDescriptorType - показывает, что это за дескриптор. В данном случае - Device descriptor
  0x00, 0x02,                 // bcdUSB - какую версию стандарта USB поддерживает устройство. 2.0
	
	// класс, подкласс устройства и протокол, по стандарту USB. У нас нули, означает каждый интерфейс сам за себя
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
	
  USB_MAX_EP0_SIZE,           // bMaxPacketSize - максимальный размер пакетов для Endpoint 0 (при конфигурировании)
	
	// те самые пресловутые VID и PID,  по которым и определяется, что же это за устройство.
  LOBYTE(USBD_VID), HIBYTE(USBD_VID), /*idVendor*/
  LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS), /*idProduct*/
	
  0x00, 0x02,                 // bcdDevice rel. 2.00 DEVICE_VER_H.DEVICE_VER_L  номер релиза устройства
	
	// дальше идут индексы строк, описывающих производителя, устройство и серийный номер.
	// Отображаются в свойствах устройства в диспетчере устройств
	// А по серийному номеру подключенные устройства с одинаковым VID/PID различаются системой.
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  // bNumConfigurations - количество возможных конфигураций. У нас одна.
};

Устройства клавиатура и мышь - работают хорошо, а третий интерфейс с кодом 10 - запуск устройства невозможен.

### ASSISTANT
Отлично, вы уже почти всё сделали правильно: у вас три интерфейса HID — клавиатура, мышь и **Consumer Control** (для Power/Sleep и т.п.). Проблема в том, что **"третий интерфейс с кодом 10 — запуск устройства невозможен"**, что, скорее всего, означает:

> **Windows не может загрузить драйвер для третьего интерфейса** или **устройство не проходит enumeration**, либо **HID-класс не распознаётся корректно**.

Давайте разберёмся по шагам.

---

## 🔍 1. **Проверка дескриптора конфигурации**

Вы указали:

```c
#define USB_HID_CONFIG_DESC_SIZ       9U + 25U + 25U + 25U
```

Но давайте посчитаем **реальный размер**:

- Configuration Descriptor: **9 байт**
- Interface 0 (Keyboard):  
  - Interface: 9  
  - HID: 9  
  - Endpoint: 7 → **всего 25 байт** ✅
- Interface 1 (Mouse): **25 байт** ✅
- Interface 2 (Consumer): **25 байт** ✅

Итого: **9 + 25×3 = 84 байта** → `0x54`

Убедитесь, что `USB_HID_CONFIG_DESC_SIZ == 84` (0x54). Если вы используете `0x54`, то всё ок.

> ⚠️ **Ошибка часто возникает из-за неправильного `wTotalLength`** — Windows строго проверяет его.

---

## 🔍 2. **Проверка HID-дескриптора для Consumer Control**

Вы объявили интерфейс 2 как:

```c
bInterfaceClass    = 0x03  // HID ✅  
bInterfaceSubClass = 0x00  // не BOOT ✅  
bInterfaceProtocol = 0x00  // no protocol ✅  
```

Это **правильно** для Consumer Control.

Но **главное — содержимое HID Report Descriptor**.

Если вы используете **Consumer Page**, дескриптор должен быть примерно таким:

### ✅ Правильный Consumer Control Report Descriptor (2 байта)

```c
__ALIGN_BEGIN static uint8_t CONSUMER_ReportDesc[CONSUMER_REPORT_DESC_SIZE] __ALIGN_END =
{
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        // Logical Minimum (0)
    0x26, 0xFF, 0x00,  // Logical Maximum (255)
    0x19, 0x00,        // Usage Minimum (0)
    0x2A, 0xFF, 0x00,  // Usage Maximum (255)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x02,        // Report Count (2)
    0x81, 0x00,        // Input (Data, Array, Absolute)
    0xC0               // End Collection
};
```

→ **Размер = 13 байт**

Тогда:

```c
#define CONSUMER_REPORT_DESC_SIZE    13U
```

И в HID Descriptor:

```c
/*8-9 byte*/ LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE)
```

→ должно быть `0x0D, 0x00`

> ❗ Если вы указали неправильный размер (например, 32 или 0), Windows не сможет прочитать дескриптор и откажет в запуске.

---

## 🔍 3. **Размер endpoint-пакета**

Вы задали:

```c
#define HID_CONSUMER_EP_SIZE   0x02U
```

Это **правильно**, так как отчёт — 2 байта.

Но убедитесь, что в функции `USBD_HID_SendReport()` вы **всегда отправляете ровно 2 байта**.

Если вы отправите 1 байт — Windows может отвергнуть пакет.

---

## 🔍 4. **Endpoint address**

Вы используете:

```c
#define HID_CONSUMER_EP        0x83U
```

→ Это **IN endpoint 3**, направление **от устройства к хосту** — **верно**, так как HID-устройство **отправляет** отчёты.

✅ Это правильно.

---

## 🔍 5. **Почему "запуск невозможен"?**

Скорее всего, **Windows не может найти подходящий драйвер**, потому что:

- HID-дескриптор повреждён или имеет неверный размер.
- Отчёт не соответствует ожидаемому формату.
- Устройство не отвечает на запрос дескриптора отчёта.

### Как проверить?

1. Подключите устройство.
2. Откройте **Device Manager** → посмотрите, появилось ли устройство с восклицательным знаком.
3. Кликните ПКМ → **Properties → Details → Hardware IDs**.
4. Если там что-то вроде:
   ```
   USB\VID_XXXX&PID_YYYY&MI_02
   ```
   → значит, интерфейс 2 определился, но драйвер не загрузился.

5. Попробуйте в **Zadig** или **USBTreeView** посмотреть, какие дескрипторы реально возвращает устройство.

---

## ✅ Рекомендуемые исправления

### 1. Убедитесь, что `CONSUMER_REPORT_DESC_SIZE == 13`

```c
#define CONSUMER_REPORT_DESC_SIZE    13U
```

### 2. Убедитесь, что `USB_HID_CONFIG_DESC_SIZ == 84`

```c
#define USB_HID_CONFIG_DESC_SIZ      84U  // 9 + 25*3
```

### 3. Проверьте, что вы **возвращаете правильный дескриптор** в `USBD_HID_GetHSCfgDesc()` или аналогичной функции:

```c
case HID_DESCRIPTOR_TYPE:
    if (req->wIndex == 2) { // Interface 2
        pbuf = CONSUMER_ReportDesc;
        len = CONSUMER_REPORT_DESC_SIZE;
    }
    break;
```

> В STM32 HAL/LL вы должны обрабатывать запрос на **HID Report Descriptor** для каждого интерфейса отдельно!

Если вы просто возвращаете один и тот же дескриптор для всех интерфейсов — **это ошибка**.

---

## 🛠 Пример обработки запроса дескриптора (в `usbd_hid.c`)

Если вы используете кастомный USB-стек (не CubeMX), то в функции обработки стандартных запросов должно быть:

```c
if (req->wValue >> 8 == 0x22) { // Report Descriptor
    switch (req->wIndex & 0xFF) {
        case 0: // Keyboard
            pbuf = KEYBOARD_ReportDesc;
            len = KEYBOARD_REPORT_DESC_SIZE;
            break;
        case 1: // Mouse
            pbuf = MOUSE_ReportDesc;
            len = MOUSE_REPORT_DESC_SIZE;
            break;
        case 2: // Consumer
            pbuf = CONSUMER_ReportDesc;
            len = CONSUMER_REPORT_DESC_SIZE;
            break;
    }
}
```

Если вы этого не сделали — Windows получит **неправильный дескриптор** для интерфейса 2 → **ошибка**.

---

## 🧪 Тест: отправка команды

После исправления дескриптора, отправляйте:

```c
uint8_t consumer_report[2] = {0};

// Power Down
consumer_report[0] = 0x30; // Consumer Power
USBD_HID_SendReport(&hUsbDeviceFS, consumer_report, 2);

// Sleep
consumer_report[0] = 0x31; // Consumer Sleep
USBD_HID_SendReport(&hUsbDeviceFS, consumer_report, 2);
```

> Используйте **Consumer Usage IDs**:
> - `0x30` — Power
> - `0x31` — Sleep
> - `0x32` — Wake Up

---

## ❌ Про "Reset"

Ещё раз: **HID не поддерживает "Reset"**. Вы не сможете перезагрузить ПК через HID. Максимум — выключить (`0x30`) или уснуть (`0x31`).

---

## 📌 Итог

**Причина ошибки**: скорее всего, **неправильный размер HID Report Descriptor** или **некорректная обработка запроса дескриптора для интерфейса 2**.

### Что сделать:

1. Установите `CONSUMER_REPORT_DESC_SIZE = 13`
2. Убедитесь, что `USB_HID_CONFIG_DESC_SIZ = 84`
3. Убедитесь, что при запросе дескриптора отчёта для интерфейса 2 вы возвращаете **именно Consumer-дескриптор**
4. Отправляйте **2 байта** в отчёте
5. Используйте **Consumer Usage ID**: `0x30`, `0x31`, `0x32`

После этого Windows загрузит стандартный `hidusb.sys` драйвер, и интерфейс заработает.

---

Если хотите — пришлите ваш `CONSUMER_ReportDesc` и код обработки запроса дескриптора, я проверю.