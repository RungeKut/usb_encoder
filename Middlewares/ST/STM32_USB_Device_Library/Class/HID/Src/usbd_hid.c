/**
  ******************************************************************************
  * @file    usbd_hid.c
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_ctlreq.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  *USBD_HID_GetCfgDesc(uint16_t *length);
static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

/* Private variables ---------------------------------------------------------*/

USBD_ClassTypeDef  USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /*EP0_TxSent*/
  NULL, /*EP0_RxReady*/
  USBD_HID_DataIn, /*DataIn*/
  NULL, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_HID_GetCfgDesc,
  USBD_HID_GetCfgDesc,
  USBD_HID_GetCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,
};

/* Report Descriptors --------------------------------------------------------*/

__ALIGN_BEGIN static uint8_t HID_KEYBOARD_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x06,        // Usage (Keyboard)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x08,        //   Report Count (8)
  0x19, 0xE0,        //   Usage Minimum (0xE0)
  0x29, 0xE7,        //   Usage Maximum (0xE7)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x03,        //   Report Count (3)
  0x75, 0x08,        //   Report Size (8)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x64,        //   Logical Maximum (100)
  0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
  0x19, 0x00,        //   Usage Minimum (0x00)
  0x29, 0x65,        //   Usage Maximum (0x65)
  0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              // End Collection
  0x05, 0x0C,        // Usage Page (Consumer)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x02,        //   Report ID (2)
  0x05, 0x0C,        //   Usage Page (Consumer)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x08,        //   Report Count (8)
  0x09, 0xB5,        //   Usage (Scan Next Track)
  0x09, 0xB6,        //   Usage (Scan Previous Track)
  0x09, 0xB7,        //   Usage (Stop)
  0x09, 0xB8,        //   Usage (Eject)
  0x09, 0xCD,        //   Usage (Play/Pause)
  0x09, 0xE2,        //   Usage (Mute)
  0x09, 0xE9,        //   Usage (Volume Increment)
  0x09, 0xEA,        //   Usage (Volume Decrement)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              // End Collection
};

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[] __ALIGN_END =
{
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x02, // USAGE (Mouse)
	0xA1, 0x01, // COLLECTION (Application)
		0x09, 0x01, // Report ID (3)
		0xA1, 0x00, // USAGE (Pointer)
		0x05, 0x09, // COLLECTION (Physical)
			0x19, 0x01, // USAGE_PAGE (Button)
			0x29, 0x03, // USAGE_MINIMUM (Button 1)
			0x15, 0x00, // USAGE_MAXIMUM (Button 3)
			0x25, 0x01, // LOGICAL_MINIMUM (0)
			0x95, 0x03, // LOGICAL_MAXIMUM (1)
			0x75, 0x01, // REPORT_COUNT (3)
			0x81, 0x02, // REPORT_SIZE (1)
			0x95, 0x01, // INPUT (Data,Var,Abs)
			0x75, 0x05, // REPORT_COUNT (1)
			0x81, 0x03, // REPORT_SIZE (5)
			0x05, 0x01, // INPUT (Cnst,Var,Abs)
			0x09, 0x30, // USAGE_PAGE (Generic Desktop)
			0x09, 0x31, // USAGE (X)
			0x09, 0x38, // USAGE (Y)
			0x15, 0x81, // LOGICAL_MINIMUM (-127)
			0x25, 0x7F, // LOGICAL_MAXIMUM (127)
			0x75, 0x08, // REPORT_SIZE (8)
			0x95, 0x03, // REPORT_COUNT (2)
			0x81, 0x06, // INPUT (Data,Var,Rel)
		0xC0,       // END_COLLECTION
	0xC0        // END_COLLECTION
};

__ALIGN_BEGIN static uint8_t HID_CONSUMER_ReportDesc[] __ALIGN_END =
{
	0x05, 0x0C,	//Usage Page (Consumer)
	0x09, 0x01,	//Usage (Consumer Control)
	0xA1, 0x01,	//Collection (Application)
	0x85, 0x01,	//Report ID (0x01)
	0x09, 0xE0,	//Usage (Volume)
	0x15, 0xE8,	//Logical Minimum (-24)
	0x25, 0x18,	//Logical Maximum (24)
	0x75, 0x07,	//Report Size (7)
	0x95, 0x01,	//Report Count (1)
	0x81, 0x06,	//Input (Var, Rel)
	0x15, 0x00,	//Logical Minimum (0)
	0x25, 0x01,	//Logical Maximum (1)
	0x75, 0x01,	//Report Size (1)
	0x09, 0xE2,	//Usage (Mute)
	0x81, 0x06,	//Input (Var, Rel)
	0xC0,	//End Collection
	0x06, 0x01, 0x00,	//Usage Page (Generic Desktop Controls)
	0x09, 0x80,	//Usage (System Control)
	0xA1, 0x01,	//Collection (Application)
	0x85, 0x02,	//Report ID (0x02)
	0x25, 0x01,	//Logical Maximum (1)
	0x15, 0x00,	//Logical Minimum (0)
	0x75, 0x01,	//Report Size (1)
	0x0A, 0x81, 0x00,	//Usage (System Power Down)
	0x0A, 0x82, 0x00,	//Usage (System Sleep)
	0x0A, 0x83, 0x00,	//Usage (System Wake Up)
	0x95, 0x03,	//Report Count (3)
	0x81, 0x06,	//Input (Var, Rel)
	0x95, 0x05,	//Report Count (5)
	0x81, 0x01,	//Input (Const)
	0xC0,	//End Collection
	0x06, 0x0C, 0x00,	//Usage Page (Consumer)
	0x09, 0x01,	//Usage (Consumer Control)
	0xA1, 0x01,	//Collection (Application)
	0x85, 0x03,	//Report ID (0x03)
	0x25, 0x01,	//Logical Maximum (1)
	0x15, 0x00,	//Logical Minimum (0)
	0x75, 0x01,	//Report Size (1)
	0x0A, 0xB5, 0x00,	//Usage (Scan Next Track)
	0x0A, 0xB6, 0x00,	//Usage (Scan Previous Track)
	0x0A, 0xB7, 0x00,	//Usage (Stop)
	0x0A, 0xB8, 0x00,	//Usage (Eject)
	0x0A, 0xCD, 0x00,	//Usage (Play/Pause)
	0x0A, 0xE2, 0x00,	//Usage (Mute)
	0x0A, 0xE9, 0x00,	//Usage (Volume Increment)
	0x0A, 0xEA, 0x00,	//Usage (Volume Decrement)
	0x95, 0x08,	//Report Count (8)
	0x81, 0x02,	//Input (Var)
	0x0A, 0x83, 0x01,	//Usage (AL Consumer Control Configuration)
	0x0A, 0x8A, 0x01,	//Usage (AL Email Reader)
	0x0A, 0x92, 0x01,	//Usage (AL Calculator)
	0x0A, 0x94, 0x01,	//Usage (AL Local Machine Browser)
	0x0A, 0x21, 0x02,	//Usage (AC Search)
	0x0A, 0x23, 0x02,	//Usage (AC Home)
	0x0A, 0x24, 0x02,	//Usage (AC Back)
	0x0A, 0x25, 0x02,	//Usage (AC Forward)
	0x95, 0x08,	//Report Count (8)
	0x81, 0x02,	//Input (Var)
	0x0A, 0x26, 0x02,	//Usage (AC Stop)
	0x0A, 0x27, 0x02,	//Usage (AC Refresh)
	0x0A, 0x2A, 0x02,	//Usage (unk)
	0x0A, 0xB3, 0x00,	//Usage (Fast Forward)
	0x0A, 0xB4, 0x00,	//Usage (Rewind)
	0x95, 0x05,	//Report Count (5)
	0x81, 0x02,	//Input (Var)
	0x95, 0x03,	//Report Count (3)
	0x81, 0x01,	//Input (Const)
	0xC0	//End Collection
};

__ALIGN_BEGIN static uint8_t HID_XBOX_ONE_CONTROLLER_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x05,        // Usage (Game Pad)
  0xa1, 0x01,        // Collection (Application)

                     //   # button packet
  0xa1, 0x00,        //   Collection (Physical)
  0x85, 0x20,        //     Report ID (0x20)

                     //     # skip unknown field and counter
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x00,        //     Usage (Undefined)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x03,        //     Input (Constant, Variable, Absolute)

                     //     payload size
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x3b,        //     Usage (Byte Count)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)

                     //     # 16 buttons
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (Button 1)
  0x29, 0x10,        //     Usage Maximum (Button 16)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x75, 0x01,        //     Report Size (1)
  0x95, 0x10,        //     Report Count (16)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)

                     //     # triggers
  0x15, 0x00,        //     Logical Minimum (0)
  0x26, 0xff, 0x03,  //     Logical Maximum (1023)
  0x35, 0x00,        //     Physical Minimum (0)
  0x46, 0xff, 0x03,  //     Physical Maximum (1023)
  0x75, 0x10,        //     Report Size (16)
  0x95, 0x02,        //     Report Count (2)
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x33,        //     Usage (Rx)
  0x09, 0x34,        //     Usage (Ry)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)

                     //     # sticks
  0x75, 0x10,        //     Report Size (16)
  0x16, 0x00, 0x80,  //     Logical Minimum (-32768)
  0x26, 0xff, 0x7f,  //     Logical Maximum (32767)
  0x36, 0x00, 0x80,  //     Physical Minimum (-32768)
  0x46, 0xff, 0x7f,  //     Physical Maximum (32767)
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x01,        //     Usage (Pointer)
  0xa1, 0x00,        //     Collection (Physical)
  0x95, 0x02,        //       Report Count (2)
  0x05, 0x01,        //       Usage Page (Generic Desktop)
  0x09, 0x30,        //       Usage (X)
  0x09, 0x31,        //       Usage (Y)
  0x81, 0x02,        //       Input (Data, Variable, Absolute)
  0xc0,              //     End Collection
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x01,        //     Usage (Pointer)
  0xa1, 0x00,        //     Collection (Physical)
  0x95, 0x02,        //       Report Count (2)
  0x05, 0x01,        //       Usage Page (Generic Desktop)
  0x09, 0x32,        //       Usage (Z)
  0x09, 0x35,        //       Usage (Rz)
  0x81, 0x02,        //       Input (Data, Variable, Absolute)
  0xc0,              //     End Collection
  0xc0,              //   End Collection

                     //   # xbox button packet
  0xa1, 0x00,        //   Collection (Physical)
  0x85, 0x07,        //     Report Id (0x07)

                     //     # skip unknown field and counter
  0x05, 0x01,        //     Usage Page (generic desktop)
  0x09, 0x00,        //     Usage (Undefined)
  0x95, 0x02,        //     Report Count (2)
  0x75, 0x08,        //     Report Size (8)
  0x81, 0x03,        //     Input (Constant, Variable, Absolute)

                     //     # payload size
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x3b,        //     Usage (Byte Count)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x03,        //     Input (Constant, Variable, Absolute)

                     //     # lone button
  0x05, 0x09,        //     Usage Page (Button)
  0x09, 0x10,        //     Usage (Button 17)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)

  0xc0,              //   End Collection
  0xc0               // End Collection
};

__ALIGN_BEGIN static uint8_t HID_CUSTOM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01, // USAGE_PAGE (стандартный рабочий стол)
       0x09, 0x80, // ИСПОЛЬЗОВАНИЕ (управление системой)
       0xa1, 0x01, // КОЛЛЕКЦИЯ (приложение)
           0x19, 0x82, // USAGE_MINIMUM (Режим ожидания системы)
           0x29, 0x83, // USAGE_MAXIMUM (пробуждение системы)
           0x15, 0x00, // LOGICAL_MINIMUM (0) <---------- Добавьте эти три строки
           0x25, 0x01, // LOGICAL_MAXIMUM (1) <----------
           0x75, 0x01, // РАЗМЕР СООБЩЕНИЯ (1) <----------
           0x95, 0x02, // COUNT_REPORTS (2)
           0x81, 0x06, // ВВОД (Данные, Переменная, Связь)
           0x95, 0x06, // ОТЧЕТНОЕ КОЛИЧЕСТВО (6)
           0x81, 0x03, // ВВОД (Const, Var, Abs)
       0xc0                           // КОНЕЦ СБОРКИ
};

/* Размеры дескрипторов ------------------------------------------------------*/
#define KEYBOARD_REPORT_DESC_SIZE  (sizeof(HID_KEYBOARD_ReportDesc))
#define MOUSE_REPORT_DESC_SIZE     (sizeof(HID_MOUSE_ReportDesc))
#define CONSUMER_REPORT_DESC_SIZE  (sizeof(HID_CONSUMER_ReportDesc))
#define CUSTOM_REPORT_DESC_SIZE    (sizeof(HID_CUSTOM_ReportDesc))

/* Configuration Descriptor --------------------------------------------------*/
// Дескриптор конфигурации (описывает возможности устройства)
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
	// Configuration Descriptor (9 byte)
	/*  1 byte*/ 0x09,	//bLength: Configuration Descriptor size - длина дескриптора конфигурации
	/*  2 byte*/ USB_DESC_TYPE_CONFIGURATION,	//bDescriptorType: Configuration - тип дескриптора - конфигурация
	/*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ),	//wTotalLength - общий размер всего дерева под данной конфигурацией в байтах
	/*  5 byte*/ 0x02,	//bNumInterfaces -  в конфигурации три интерфейса
	/*  6 byte*/ 0x01,	//bConfigurationValue - индекс текущей конфигурации
	/*  7 byte*/ 0x00,	//iConfiguration - индекс строки, которая описывает эту конфигурацию
	/*  8 byte*/ 0xA0,	//bmAttributes - признак того, что устройство будет питаться от шины USB
	/*  9 byte*/ 0x2D,	//MaxPower (90 mA)

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
			/*3-4 byte*/ 0x10, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_KEYBOARD_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*5-6 byte*/ LOBYTE(HID_KEYBOARD_EP_SIZE), HIBYTE(HID_KEYBOARD_EP_SIZE),	//wMaxPacketSize
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
			/*3-4 byte*/ 0x10, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_MOUSE_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*5-6 byte*/ LOBYTE(HID_MOUSE_EP_SIZE), HIBYTE(HID_MOUSE_EP_SIZE),	//wMaxPacketSize
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
			/*3-4 byte*/ 0x10, 0x01,	//bcdHID: HID Class Spec release number
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_CONSUMER_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*5-6 byte*/ LOBYTE(HID_CONSUMER_EP_SIZE), HIBYTE(HID_CONSUMER_EP_SIZE),	//wMaxPacketSize
				/*  7 byte*/ 0xFF,	//bInterval: Polling Interval (255 ms)
	#endif

	#ifdef CUSTOM_CONTROL
	// ============ INTERFACE 3: CUSTOM CONTROL ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ 0x03,	//bInterfaceNumber: Number of Interface
		/*  4 byte*/ 0x00,	//bAlternateSetting: Alternate setting
		/*  5 byte*/ 0x02,	//bNumEndpoints
		/*  6 byte*/ 0x03,	//bInterfaceClass: HID
		/*  7 byte*/ 0x00,	//bInterfaceSubClass : 1=BOOT, 0=no boot
		/*  8 byte*/ 0x00,	//nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
		/*  9 byte*/ 0x00,	//iInterface: Index of string descriptor

			// HID Descriptor (9 byte)
			/*  1 byte*/ 0x09,	//bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE,	//bDescriptorType: HID
			/*3-4 byte*/ 0x11, 0x01,	//bcdHID: HID Class Spec release number
			/*  4 byte*/ 0x01, 
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(CUSTOM_REPORT_DESC_SIZE), HIBYTE(CUSTOM_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_CUSTOM_EPIN,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_CUSTOM_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_CUSTOM_EPOUT,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_CUSTOM_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
	#endif
};

/* USB Standard Device Descriptor --------------------------------------------*/
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	//USBD_LL_OpenEP(pdev, HID_MOUSE_EP,    USBD_EP_TYPE_INTR, HID_MOUSE_EP_SIZE);
	USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EP_SIZE);
	USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, USBD_EP_TYPE_INTR, HID_CONSUMER_EP_SIZE);

	//pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 1U;
	pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U;
	pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 1U;
	
	pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));
	if (pdev->pClassData == NULL)
	return USBD_FAIL;
	
	((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;
	return USBD_OK;
}

/**
  * @brief  USBD_HID_Init
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_DeInit(USBD_HandleTypeDef *pdev,
                                uint8_t cfgidx)
{
	//USBD_LL_CloseEP(pdev, HID_MOUSE_EP);
	USBD_LL_CloseEP(pdev, HID_KEYBOARD_EP);
	USBD_LL_CloseEP(pdev, HID_CONSUMER_EP);

	//pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 0U;
	pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 0U;
	pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 0U;

  if (pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;
        case HID_REQ_GET_PROTOCOL:
          USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;
        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;
        case HID_REQ_GET_IDLE:
          USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
            USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
        {
          uint8_t interface = (uint8_t)(req->wIndex & 0xFF);
          if (req->wValue >> 8 == HID_REPORT_DESC)
          {
            if (interface == 0)
            {
              len = MIN(KEYBOARD_REPORT_DESC_SIZE, req->wLength);
              pbuf = HID_KEYBOARD_ReportDesc;
            }
            else if (interface == 1)
            {
              len = MIN(MOUSE_REPORT_DESC_SIZE, req->wLength);
              pbuf = HID_MOUSE_ReportDesc;
            }
            else if (interface == 2)
            {
              len = MIN(CONSUMER_REPORT_DESC_SIZE, req->wLength);
              pbuf = HID_CONSUMER_ReportDesc;
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
              break;
            }
          }
          else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE)
          {
            // Можно вернуть общий HID-дескриптор, но проще игнорировать
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          USBD_CtlSendData(pdev, pbuf, len);
          break;
        }

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
            USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
            hhid->AltSetting = (uint8_t)(req->wValue);
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }
  return ret;
}

/**
  * Стандартная функция — отправка в единственный endpoint (не используется в composite)
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef  *pdev, uint8_t *report, uint16_t len)
{
	return USBD_HID_SendReport_EP(pdev, report, len, HID_KEYBOARD_EP);
}

/**
  НОВАЯ функция — отправка в указанный endpoint
  Эта функция обходит внутреннее состояние hhid->state и позволяет отправлять в любой endpoint. 
  * @brief  USBD_HID_SendReport_EP
  *         Send HID Report to specific endpoint
  * @param  pdev: device instance
  * @param  report: pointer to report
  * @param  len: report length
  * @param  ep_addr: endpoint address (e.g. 0x81, 0x82, 0x83)
  * @retval status
  */
uint8_t USBD_HID_SendReport_EP(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len, uint8_t ep_addr)
{
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    return USBD_LL_Transmit(pdev, ep_addr, report, len);
  }
  return USBD_FAIL;
}

/**
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  return HID_FS_BINTERVAL; // STM32F1 — только Full Speed
}

/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_DataIn(USBD_HandleTypeDef *pdev,
                                uint8_t epnum)
{

  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;
  return USBD_OK;
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

