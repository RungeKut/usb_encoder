### USER
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
        0x19, 0xE0,        //   Usage Minimum (0xE0)
        0x29, 0xE7,        //   Usage Maximum (0xE7)

        // it seam we missed  the shit ctrl etc .. here
        0x15, 0x00,        //   Logical Minimum (0)
        0x25, 0x01,        //   Logical Maximum (1)
        0x75, 0x01,        //   Report Size (1)
        0x95, 0x08,        //   Report Count (8)
        0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x95, 0x01,        //   Report Count (1)
        0x75, 0x08,        //   Report Size (8)
        0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
#if HID_LED_SUPPORT
            // --------------------- output report for LED
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
#endif
        0x95, 0x06,        //   Report Count (6)
        0x75, 0x08,        //   Report Size (8)
        0x15, 0x00,        //   Logical Minimum (0)
        0x25, 0x65,        //   Logical Maximum (101)
        0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
        0x19, 0x00,        //   Usage Minimum (0x00)
        0x29, 0x65,        //   Usage Maximum (0x65)
        0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,              // End Collection

        // 47 bytes
#if HID_MEDIA_REPORT
        //help from http://www.microchip.com/forums/m618147.aspx
        // this way of describing and sending media control is convenient
        // short descriptor that permit all kidn meda by sending "usage" code
        // see usb hid spec for full list
        // it is possible to define one media key per bit it requires more space
        // for descripotor and report ending is tighlyu couple to decriptor
        // so it is not as convenient
        // one such working code can be find here https://github.com/markwj/hidmedia/blob/master/hidmedia.X/usb_descriptors.c  
        //

        0x05, 0x0C,        // Usage Page (Consumer)
        0x09, 0x01,        // Usage (Consumer Control)
        0xA1, 0x01,        // Collection (Application)
        0x85, HID_MEDIA_REPORT,        //   Report ID (VOLUME_REPORT )
        0x19, 0x00,        //   Usage Minimum (Unassigned)
        0x2A, 0x3C, 0x02,  //   Usage Maximum (AC Format)
        0x15, 0x00,        //   Logical Minimum (0)
        0x26, 0x3C, 0x02,  //   Logical Maximum (572)
        0x95, 0x01,        //   Report Count (1)
        0x75, 0x10,        //   Report Size (16)
        0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0xC0,              // End Collection

        // how to format the 3 byte report
        // byte 0 report ID = 0x02 (VOLUME_REPORT)
        // byte 1 media code  for ex VOL_UP 0xE9 , VOL_DONW 0xEA ... etc
        // byte 2  0x00
        // a second  report with 0 code shal be send to avoid "key repaeat"

        // 25 bytes
#endif

};

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[] __ALIGN_END =
{
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x02, // USAGE (Mouse)
	0xA1, 0x01, // COLLECTION (Application)
		0x09, 0x01, // USAGE (Pointer)
		0xA1, 0x00, // COLLECTION (Physical)
			0x05, 0x09, // USAGE_PAGE (Button)
			0x19, 0x01, // USAGE_MINIMUM (Button 1)
			0x29, 0x03, // USAGE_MAXIMUM (Button 3)
			0x15, 0x00, // LOGICAL_MINIMUM (0)
			0x25, 0x01, // LOGICAL_MAXIMUM (1)
			0x95, 0x03, // REPORT_COUNT (3)
			0x75, 0x01, // REPORT_SIZE (1)
			0x81, 0x02, // INPUT (Data,Var,Abs)
			0x95, 0x01, // REPORT_COUNT (1)
			0x75, 0x05, // REPORT_SIZE (5)
			0x81, 0x03, // INPUT (Cnst,Var,Abs)
			0x05, 0x01, // USAGE_PAGE (Generic Desktop)
			0x09, 0x30, // USAGE (X)
			0x09, 0x31, // USAGE (Y)
			0x15, 0x81, // LOGICAL_MINIMUM (-127)
			0x25, 0x7F, // LOGICAL_MAXIMUM (127)
			0x75, 0x08, // REPORT_SIZE (8)
			0x95, 0x02, // REPORT_COUNT (2)
			0x81, 0x06, // INPUT (Data,Var,Rel)
		0xC0,       // END_COLLECTION
	0xC0        // END_COLLECTION
};

__ALIGN_BEGIN static uint8_t HID_REMOTE_ReportDesc[] __ALIGN_END =
{
0x05, 0x0C, // USAGE_PAGE (Consumer Devices)	
0x09, 0x01, // USAGE (Consumer Control)	
0xA1, 0x01, // COLLECTION (Application)	
  0x09, 0x02, // USAGE (Numeric Key Pad)	
  0xA1, 0x02, // COLLECTION (Logical)	
    0x05, 0x09, // USAGE_PAGE (Button)	
    0x19, 0x01, // USAGE_MINIMUM (Button 1)	
    0x29, 0x0A, // USAGE_MAXIMUM (Button 10)	
    0x15, 0x01, // LOGICAL_MINIMUM (1)	
    0x25, 0x0A, // LOGICAL_MAXIMUM (10)	
    0x75, 0x04, // REPORT_SIZE (4)	
    0x95, 0x01, // REPORT_COUNT (1)	
    0x81, 0x00, // INPUT (Data,Ary,Abs)	
  0xC0,       // END_COLLECTI
  0x05, 0x0C, // USAGE_PAGE (Consumer Devices)	
  0x09, 0x86, // USAGE (Channel)	
  0x09, 0xE0, // USAGE (Volume)	
  0x15, 0xFF, // LOGICAL_MINIMUM (-1)	
  0x25, 0x01, // LOGICAL_MAXIMUM (1)	
  0x75, 0x02, // REPORT_SIZE (2)	
  0x95, 0x02, // REPORT_COUNT (2)	
  0x81, 0x46, // INPUT (Data,Var,Rel,Null)	
  0x09, 0xE2, // USAGE (Mute)	
  0x09, 0x30, // USAGE (Power)	
  0x09, 0x34, // USAGE (Sleep Mode)	
  0x09, 0x60, // USAGE (Data On Screen)	
  0x09, 0x64, // USAGE (Broadcast Mode)	
  0x09, 0x83, // USAGE (Recall Last)	
  0x09, 0x81, // USAGE (Assign Selection)	
  0x15, 0x01, // LOGICAL_MINIMUM (1)	
  0x25, 0x07, // LOGICAL_MAXIMUM (7)	
  0x75, 0x04, // REPORT_SIZE (4)	
  0x95, 0x01, // REPORT_COUNT (1)	
  0x81, 0x00, // INPUT (Data,Ary,Abs)	
  0x09, 0x80, // USAGE (Selection)	
  0xA1, 0x02, // COLLECTION (Logical)	
    0x05, 0x09, // USAGE_PAGE (Button)	
    0x19, 0x01, // USAGE_MINIMUM (Button 1)	
    0x29, 0x03, // USAGE_MAXIMUM (Button 3)	
    0x15, 0x01, // LOGICAL_MINIMUM (1)	
    0x25, 0x03, // LOGICAL_MAXIMUM (3)	
    0x75, 0x02, // REPORT_SIZE (2)	
    0x81, 0x00, // INPUT (Data,Ary,Abs)	
  0xC0,       // END_COLLECTI
  0x15, 0x02, // LOGICAL_MINIMUM (2)	
  0x81, 0x03, // INPUT (Cnst,Var,Abs)	
0xC0,       // END_COLLECTI
};

__ALIGN_BEGIN static uint8_t HID_CONSUMER_ReportDesc[] __ALIGN_END =
{
  0x05, 0x0C,        // Usage Page (Consumer Devices)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        // Logical Minimum (0)
  0x26, 0xFF, 0x03,  // Logical Maximum (1023) — позволяет использовать 16-битные коды
  0x19, 0x00,        // Usage Minimum (0)
  0x2A, 0xFF, 0x03,  // Usage Maximum (1023)
  0x75, 0x10,        // Report Size (16 bits)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
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
#define REMOTE_REPORT_DESC_SIZE    (sizeof(HID_REMOTE_ReportDesc))
#define CUSTOM_REPORT_DESC_SIZE    (sizeof(HID_CUSTOM_ReportDesc))

#define INTERFACE_KEYBOARD_NUMBER 0
#define INTERFACE_MOUSE_NUMBER    (HID_INTERFACE_KEYBOARD)
#define INTERFACE_CONSUMER_NUMBER (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE)
#define INTERFACE_REMOTE_NUMBER   (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE + HID_INTERFACE_CONSUMER)
#define INTERFACE_CUSTOM_NUMBER   (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE + HID_INTERFACE_CONSUMER + HID_INTERFACE_REMOTE)

/* Configuration Descriptor --------------------------------------------------*/
// Дескриптор конфигурации (описывает возможности устройства)
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
	// Configuration Descriptor (9 byte)
	/*  1 byte*/ 0x09,	//bLength: Configuration Descriptor size - длина дескриптора конфигурации
	/*  2 byte*/ USB_DESC_TYPE_CONFIGURATION,	//bDescriptorType: Configuration - тип дескриптора - конфигурация
	/*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ),	//wTotalLength - общий размер всего дерева под данной конфигурацией в байтах
	/*  5 byte*/ HID_INTERFACE_COUNT,	//bNumInterfaces -  в конфигурации три интерфейса
	/*  6 byte*/ 0x01,	//bConfigurationValue - индекс текущей конфигурации
	/*  7 byte*/ 0x00,	//iConfiguration - индекс строки, которая описывает эту конфигурацию
	/*  8 byte*/ 0xA0,	//bmAttributes - признак того, что устройство будет питаться от шины USB
	/*  9 byte*/ 0xFA,	//MaxPower (500 mA)

#if HID_INTERFACE_KEYBOARD
	// ============ INTERFACE 0: KEYBOARD ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ INTERFACE_KEYBOARD_NUMBER,	//bInterfaceNumber: Number of Interface
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

	#if HID_INTERFACE_MOUSE
	// ============ INTERFACE 1: MOUSE ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ INTERFACE_MOUSE_NUMBER,	//bInterfaceNumber: Number of Interface
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

	#if HID_INTERFACE_CONSUMER
	// ============ INTERFACE 2: CONSUMER CONTROL ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ INTERFACE_CONSUMER_NUMBER,	//bInterfaceNumber: Number of Interface
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
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
	#endif
	
	#if HID_INTERFACE_REMOTE
	// ============ INTERFACE 3: CUSTOM CONTROL ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ INTERFACE_REMOTE_NUMBER,	//bInterfaceNumber: Number of Interface
		/*  4 byte*/ 0x00,	//bAlternateSetting: Alternate setting
		/*  5 byte*/ 0x02,	//bNumEndpoints
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
			/*8-9 byte*/ LOBYTE(REMOTE_REPORT_DESC_SIZE), HIBYTE(REMOTE_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_REMOTE_EP,	//bEndpointAddress: Endpoint Address (IN)
				/*  4 byte*/ 0x03,	//bmAttributes: Interrupt endpoint
				/*  5 byte*/ HID_REMOTE_EP_SIZE,	//wMaxPacketSize
				/*  6 byte*/ 0x00,
				/*  7 byte*/ HID_FS_BINTERVAL,	//bInterval: Polling Interval (10 ms)
	#endif

	#if HID_INTERFACE_CUSTOM
	// ============ INTERFACE 4: CUSTOM CONTROL ============

		// Interface Descriptor (9 byte)
		/*  1 byte*/ 0x09,	//bLength: Interface Descriptor size
		/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
		/*  3 byte*/ INTERFACE_CUSTOM_NUMBER,	//bInterfaceNumber: Number of Interface
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
			/*  5 byte*/ 0x00,	//bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,	//bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,	//bDescriptorType
			/*8-9 byte*/ LOBYTE(CUSTOM_REPORT_DESC_SIZE), HIBYTE(CUSTOM_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor

				// Endpoint Descriptor (7 byte)
				/*  1 byte*/ 0x07,	//bLength: Endpoint Descriptor size
				/*  2 byte*/ USB_DESC_TYPE_ENDPOINT,	//bDescriptorType: ENDPOINT
				/*  3 byte*/ HID_CUSTOM_EP,	//bEndpointAddress: Endpoint Address (IN)
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
			#if HID_INTERFACE_KEYBOARD
				if (interface == INTERFACE_KEYBOARD_NUMBER)
				{
				len = MIN(KEYBOARD_REPORT_DESC_SIZE, req->wLength);
				pbuf = HID_KEYBOARD_ReportDesc;
				}
				else
			#endif
			#if HID_INTERFACE_MOUSE
				if (interface == INTERFACE_MOUSE_NUMBER)
				{
				len = MIN(MOUSE_REPORT_DESC_SIZE, req->wLength);
				pbuf = HID_MOUSE_ReportDesc;
				}
				else
			#endif
			#if HID_INTERFACE_CONSUMER
				if (interface == INTERFACE_CONSUMER_NUMBER)
				{
				len = MIN(CONSUMER_REPORT_DESC_SIZE, req->wLength);
				pbuf = HID_CONSUMER_ReportDesc;
				}
				else
			#endif
			#if HID_INTERFACE_REMOTE
				if (interface == INTERFACE_REMOTE_NUMBER)
				{
				len = MIN(REMOTE_REPORT_DESC_SIZE, req->wLength);
				pbuf = HID_REMOTE_ReportDesc;
				}
				else
			#endif
			#if HID_INTERFACE_CUSTOM
				if (interface == INTERFACE_CUSTOM_NUMBER)
				{
				len = MIN(CUSTOM_REPORT_DESC_SIZE, req->wLength);
				pbuf = HID_CUSTOM_ReportDesc;
				}
				else
			#endif
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


/**
  ******************************************************************************
  * @file    usbd_hid.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_hid_core.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_HID_H
#define __USB_HID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

//#define KEYBOARD_CONTROL
//#define MOUSE_CONTROL
#define CONSUMER_CONTROL
//#define REMOTE_CONTROL
//#define CUSTOM_CONTROL


#ifdef KEYBOARD_CONTROL
#define HID_INTERFACE_KEYBOARD 1
#else
#define HID_INTERFACE_KEYBOARD 0
#endif

#ifdef MOUSE_CONTROL
#define HID_INTERFACE_MOUSE 1
#else
#define HID_INTERFACE_MOUSE 0
#endif

#ifdef CONSUMER_CONTROL
#define HID_INTERFACE_CONSUMER 1
#else
#define HID_INTERFACE_CONSUMER 0
#endif

#ifdef REMOTE_CONTROL
#define HID_INTERFACE_REMOTE 1
#else
#define HID_INTERFACE_REMOTE 0
#endif

#ifdef CUSTOM_CONTROL
#define HID_INTERFACE_CUSTOM 1
#else
#define HID_INTERFACE_CUSTOM 0
#endif

#define HID_INTERFACE_COUNT (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE + HID_INTERFACE_CONSUMER + HID_INTERFACE_REMOTE + HID_INTERFACE_CUSTOM)

#define  HID_MEDIA_REPORT  2
#define HID_LED_SUPPORT 0

#define HID_KEYBOARD_EP        (0x80U + HID_INTERFACE_KEYBOARD)
#define HID_MOUSE_EP           (HID_KEYBOARD_EP + HID_INTERFACE_MOUSE)
#define HID_CONSUMER_EP        (HID_MOUSE_EP + HID_INTERFACE_CONSUMER)
#define HID_REMOTE_EP          (HID_CONSUMER_EP + HID_INTERFACE_REMOTE)
#define HID_CUSTOM_EP          (HID_REMOTE_EP + HID_INTERFACE_CUSTOM)

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
	uint8_t id;
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} keyboardHID;


// Media HID Report
typedef struct
{
    uint8_t id;
    uint8_t keys;
} mediaHID;

// Remote HID Report
typedef struct
{
    // Байт 0
    uint8_t numeric_keypad : 4;   // 1–10 или 0 — если не нажата
    uint8_t channel        : 2;   // -1=0b11, 0=0b00, +1=0b01 (см. ниже)
    uint8_t volume         : 2;   // -1=0b11, 0=0b00, +1=0b01

    // Байт 1
    uint8_t special_key    : 4;   // 1–7 или 0 — если не нажата
    uint8_t selection      : 2;   // 1–3 или 0
    uint8_t padding        : 2;   // всегда 0b10 (LOGICAL_MINIMUM = 2)
} remoteHID;

//Consumer HID Report
typedef struct {
	uint8_t lsb;
    uint8_t msb;
} сonsumerHID;

#define HID_KEYBOARD_EP_SIZE   (sizeof(keyboardHID))
#define HID_MOUSE_EP_SIZE      (sizeof(mouseHID))
#define HID_CONSUMER_EP_SIZE   (sizeof(сonsumerHID))
#define HID_REMOTE_EP_SIZE     (sizeof(remoteHID))
#define HID_MEDIA_EP_SIZE      (sizeof(mediaHID))
#define HID_CUSTOM_EP_SIZE     0x10U

#ifdef KEYBOARD_CONTROL
#define USB_HID_CONFIG_DESC_SIZ_KEYBOARD (9 + 9 + 7)
#else
#define USB_HID_CONFIG_DESC_SIZ_KEYBOARD 0
#endif

#ifdef MOUSE_CONTROL
#define USB_HID_CONFIG_DESC_SIZ_MOUSE (9 + 9 + 7)
#else
#define USB_HID_CONFIG_DESC_SIZ_MOUSE 0
#endif

#ifdef CONSUMER_CONTROL
#define USB_HID_CONFIG_DESC_SIZ_CONSUMER (9 + 9 + 7)
#else
#define USB_HID_CONFIG_DESC_SIZ_CONSUMER 0
#endif

#ifdef REMOTE_CONTROL
#define USB_HID_CONFIG_DESC_SIZ_REMOTE (9 + 9 + 7)
#else
#define USB_HID_CONFIG_DESC_SIZ_REMOTE 0
#endif

#ifdef CUSTOM_CONTROL
#define USB_HID_CONFIG_DESC_SIZ_CUSTOM (9 + 9 + 7)
#else
#define USB_HID_CONFIG_DESC_SIZ_CUSTOM 0
#endif

#define USB_HID_DESC_SIZ              9U
#define USB_HID_CONFIG_DESC_SIZ       (USB_HID_DESC_SIZ + USB_HID_CONFIG_DESC_SIZ_KEYBOARD + USB_HID_CONFIG_DESC_SIZ_MOUSE + USB_HID_CONFIG_DESC_SIZ_CONSUMER + USB_HID_CONFIG_DESC_SIZ_REMOTE + USB_HID_CONFIG_DESC_SIZ_CUSTOM)

#define HID_DESCRIPTOR_TYPE           0x21U
#define HID_REPORT_DESC               0x22U

#define HID_REQ_SET_PROTOCOL          0x0BU
#define HID_REQ_GET_PROTOCOL          0x03U

#define HID_REQ_SET_IDLE              0x0AU
#define HID_REQ_GET_IDLE              0x02U

#define HID_REQ_SET_REPORT            0x09U
#define HID_REQ_GET_REPORT            0x01U
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef enum
{
  HID_IDLE = 0,
  HID_BUSY,
}
HID_StateTypeDef;


typedef struct
{
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  HID_StateTypeDef     state;
}
USBD_HID_HandleTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_HID;
#define USBD_HID_CLASS    &USBD_HID
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev,
                            uint8_t *report,
                            uint16_t len);

uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);

uint8_t USBD_HID_SendReport_EP(USBD_HandleTypeDef *pdev,
                               uint8_t *report,
                               uint16_t len,
                               uint8_t ep_addr);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_HID_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



### ASSISTANT
Thank you for providing the full source code from `usbd_hid.c` and `usbd_hid.h`.

This is a **composite USB HID device implementation** for STM32 (using ST's USB Device Library), supporting multiple HID interfaces:

- Keyboard  
- Mouse  
- Consumer Control (e.g., media keys)  
- Remote Control (custom layout)  
- Custom System Control (e.g., sleep/wake)

Key observations:

---

### ✅ **Features Implemented**
1. **Modular Interfaces**  
   Each HID function (keyboard, mouse, etc.) is conditionally compiled via `#define`s like `KEYBOARD_CONTROL`, `CONSUMER_CONTROL`, etc.

2. **Multiple Report Descriptors**  
   - Standard **boot keyboard** descriptor (with optional LED output if `HID_LED_SUPPORT` is enabled).
   - **Media/consumer** control using a 16-bit usage code (aligned with [USB HID Usage Tables](https://www.usb.org/sites/default/files/hut1_3_0.pdf)).
   - Separate descriptors for mouse, remote, and system control.

3. **Multi-Interface Configuration Descriptor**  
   Dynamically builds a USB configuration descriptor with one interface per enabled HID function, each with its own IN endpoint.

4. **Flexible Report Sending**  
   - `USBD_HID_SendReport()` sends to the default (keyboard) endpoint.
   - `USBD_HID_SendReport_EP()` allows sending reports to **any endpoint**, essential for composite devices.

5. **Endpoint Addressing Logic**  
   Endpoints are assigned sequentially:
   ```c
   #define HID_KEYBOARD_EP (0x80U + HID_INTERFACE_KEYBOARD)
   #define HID_MOUSE_EP    (HID_KEYBOARD_EP + HID_INTERFACE_MOUSE)
   // etc.
   ```
   ⚠️ **Potential Issue**: This logic is **incorrect** if any interface is disabled.  
   Example: If `HID_INTERFACE_KEYBOARD = 1` and `HID_INTERFACE_MOUSE = 0`, then:
   - `HID_KEYBOARD_EP = 0x81`
   - `HID_MOUSE_EP = 0x81 + 0 = 0x81` → **collision!**

   ✅ **Fix**: Endpoints should be hardcoded or assigned based on actual interface count, not feature flags.

6. **Media Report Format**  
   When `HID_MEDIA_REPORT` is defined (e.g., as `2`), the media report uses:
   - **Byte 0**: Report ID = `2`
   - **Byte 1–2**: 16-bit usage code (e.g., `0xE9` for Volume Up)
   - To release the key, send a report with usage `0x00`.

---

### ⚠️ **Issues & Recommendations**

#### 1. **Endpoint Address Calculation is Flawed**
As noted above, the current macro-based endpoint assignment fails when interfaces are selectively enabled/disabled.

✅ **Solution**: Assign fixed endpoint addresses:
```c
#define HID_KEYBOARD_EP  0x81
#define HID_MOUSE_EP     0x82
#define HID_CONSUMER_EP  0x83
#define HID_REMOTE_EP    0x84
#define HID_CUSTOM_EP    0x85
```

#### 2. **Incomplete Initialization/Deinitialization**
In `USBD_HID_Init()` and `DeInit()`, only keyboard and consumer endpoints are opened/closed:
```c
USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, ...);
USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, ...);
// Mouse, Remote, Custom are missing!
```
✅ **Fix**: Conditionally open **all enabled endpoints**:
```c
#if HID_INTERFACE_KEYBOARD
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EP_SIZE);
  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U;
#endif
#if HID_INTERFACE_MOUSE
  USBD_LL_OpenEP(pdev, HID_MOUSE_EP, USBD_EP_TYPE_INTR, HID_MOUSE_EP_SIZE);
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 1U;
#endif
// ... etc.
```

#### 3. **Report Descriptor Size Mismatch Risk**
The `HID_CONSUMER_ReportDesc` uses:
```c
0x26, 0xFF, 0x03,  // Logical Maximum (1023)
```
But the associated `сonsumerHID` struct is:
```c
typedef struct {
  uint8_t lsb;
  uint8_t msb;
} сonsumerHID;
```
✅ This is correct (2 bytes = 16 bits), and matches the descriptor (`Report Size = 16, Count = 1`).

#### 4. **Typo in Comment**
> "it seam we missed the shit ctrl etc .. here"

Should be: *"It seems we missed the shift/ctrl etc. here"* — but the descriptor **does include** modifier keys (0xE0–0xE7), so it’s actually correct.

---

### ✅ **Usage Example: Sending a Media Key**
To send **Volume Up**:
```c
uint8_t media_report[3] = { HID_MEDIA_REPORT, 0xE9, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, media_report, 3, HID_CONSUMER_EP);
```
Then send release:
```c
uint8_t release_report[3] = { HID_MEDIA_REPORT, 0x00, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, release_report, 3, HID_CONSUMER_EP);
```

---

### Summary
This is a **well-structured composite HID implementation** with support for multiple device types. However, **endpoint assignment and initialization logic need correction** to work reliably when interfaces are selectively enabled.

Let me know if you'd like a corrected version of the endpoint macros or initialization functions!

### USER
Вот такой отчет о работе USB:

    =========================== USB Port4 ===========================

Connection Status        : 0x01 (Device is connected)
Port Chain               : 1-4
Properties               : 0x01
 IsUserConnectable       : yes
 PortIsDebugCapable      : no
 PortHasMultiCompanions  : no
 PortConnectorIsTypeC    : no
ConnectionIndex          : 0x04 (Port 4)
CompanionIndex           : 0
 CompanionHubSymLnk      : USB#ROOT_HUB30#4&cdbaec3&0&0#{f18a0e88-c30c-11d0-8815-00a0c906bed8}
 CompanionPortNumber     : 0x14 (Port 20)
 -> CompanionPortChain   : 1-20

      ========================== Summary =========================
Vendor ID                : 0x1209 (Unknown Vendor)
Product ID               : 0x0001
Serial                   : "48E074A73639"
USB Version              : 1.1 (12 Mbit/s FullSpeed)
Port maximum Speed       : High-Speed (Companion Port 1-20 is doing the SuperSpeed)
Device maximum Speed     : Full-Speed
Device Connection Speed  : Full-Speed
Self powered             : no
Demanded Current         : 500 mA
Device Manager Problem   : 10 (CM_PROB_FAILED_START)
Used Endpoints           : 1

      ======================== USB Device ========================

        +++++++++++++++++ Device Information ++++++++++++++++++
Device Description       : USB-устройство ввода
Device Path              : \\?\USB#VID_0000&PID_0000#48E074A73639#{a5dcbf10-6530-11d2-901f-00c04fb951ed} (GUID_DEVINTERFACE_USB_DEVICE)
Kernel Name              : \Device\USBPDO-13
Device ID                : USB\VID_1209&PID_0001\48E074A73639
Hardware IDs             : USB\VID_1209&PID_0001&REV_0200 USB\VID_1209&PID_0001
Driver KeyName           : {745a17a0-74d3-11d0-b6fe-00a0c90f57da}\0049 (GUID_DEVCLASS_HIDCLASS)
Driver                   : \SystemRoot\System32\drivers\hidusb.sys (Version: 10.0.19041.3636  Date: 2024-12-25  Company: Microsoft Corporation)
Driver Inf               : C:\Windows\inf\input.inf
Legacy BusType           : PNPBus
Class                    : HIDClass
Class GUID               : {745a17a0-74d3-11d0-b6fe-00a0c90f57da} (GUID_DEVCLASS_HIDCLASS)
Service                  : HidUsb
Enumerator               : USB
Location Info            : Port_#0004.Hub_#0001
Address                  : 4
Location IDs             : PCIROOT(0)#PCI(1400)#USBROOT(0)#USB(4), ACPI(_SB_)#ACPI(PC00)#ACPI(XHCI)#ACPI(RHUB)#ACPI(HS04)
Container ID             : {161e9162-ccb6-5412-8205-5ea232075bb1}
Manufacturer Info        : (Стандартные системные устройства)
Capabilities             : 0x14 (Removable, UniqueID)
Status                   : 0x01806400 (DN_HAS_PROBLEM, DN_DISABLEABLE, DN_REMOVABLE, DN_NT_ENUMERATOR, DN_NT_DRIVER)
Problem Code             : 10 (CM_PROB_FAILED_START)
First Install Date       : 2025-10-22 10:38:36
Last Arrival Date        : 2025-10-22 11:14:48
SelectiveSuspendEnabled  : 0
EnhancedPowerMgmtEnabled : 1
Power State              : D3 (supported: D0, D1, D2, D3, wake from D0, wake from D1, wake from D2)

        ---------------- Connection Information ---------------
Connection Index         : 0x04 (Port 4)
Connection Status        : 0x01 (DeviceConnected)
Current Config Value     : 0x00 (Configuration 0)
Device Address           : 0x3E (62)
Is Hub                   : 0x00 (no)
Device Bus Speed         : 0x01 (Full-Speed)
Number of open Pipes     : 0x00 (0 pipes to data endpoints)
Data (HexDump)           : 04 00 00 00 12 01 10 01 00 00 00 08 09 12 01 00   ................
                           00 02 01 02 03 01 00 01 00 3E 00 00 00 00 00 01   .........>......
                           00 00 00                                          ...

        --------------- Connection Information V2 -------------
Connection Index         : 0x04 (4)
Length                   : 0x10 (16 bytes)
SupportedUsbProtocols    : 0x03
 Usb110                  : 1 (yes, port supports USB 1.1)
 Usb200                  : 1 (yes, port supports USB 2.0)
 Usb300                  : 0 (no, port not supports USB 3.0) -> but Companion Port 1-20 does
 ReservedMBZ             : 0x00
Flags                    : 0x00
 DevIsOpAtSsOrHigher     : 0 (Device is not operating at SuperSpeed or higher)
 DevIsSsCapOrHigher      : 0 (Device is not SuperSpeed capable or higher)
 DevIsOpAtSsPlusOrHigher : 0 (Device is not operating at SuperSpeedPlus or higher)
 DevIsSsPlusCapOrHigher  : 0 (Device is not SuperSpeedPlus capable or higher)
 ReservedMBZ             : 0x00
Data (HexDump)           : 04 00 00 00 10 00 00 00 03 00 00 00 00 00 00 00   ................

    ---------------------- Device Descriptor ----------------------
bLength                  : 0x12 (18 bytes)
bDescriptorType          : 0x01 (Device Descriptor)
bcdUSB                   : 0x110 (USB Version 1.1)
bDeviceClass             : 0x00 (defined by the interface descriptors)
bDeviceSubClass          : 0x00
bDeviceProtocol          : 0x00
bMaxPacketSize0          : 0x08 (8 bytes)
idVendor                 : 0x1209 (Unknown Vendor)
idProduct                : 0x0001
bcdDevice                : 0x0200
iManufacturer            : 0x01 (String Descriptor 1)
iProduct                 : 0x02 (String Descriptor 2)
iSerialNumber            : 0x03 (String Descriptor 3)
bNumConfigurations       : 0x01 (1 Configuration)
Data (HexDump)           : 12 01 10 01 00 00 00 08 09 12 01 00 00 02 01 02   ................
                           03 01                                             ..

    ------------------ Configuration Descriptor -------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x02 (Configuration Descriptor)
wTotalLength             : 0x0022 (34 bytes)
bNumInterfaces           : 0x01 (1 Interface)
bConfigurationValue      : 0x01 (Configuration 1)
iConfiguration           : 0x00 (No String Descriptor)
bmAttributes             : 0xA0
 D7: Reserved, set 1     : 0x01
 D6: Self Powered        : 0x00 (no)
 D5: Remote Wakeup       : 0x01 (yes)
 D4..0: Reserved, set 0  : 0x00
MaxPower                 : 0xFA (500 mA)
Data (HexDump)           : 09 02 22 00 01 01 00 A0 FA 09 04 00 00 01 03 00   ..".............
                           00 00 09 21 10 01 00 01 22 17 00 07 05 81 03 02   ...!....".......
                           00 0A                                             ..

        ---------------- Interface Descriptor -----------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x04 (Interface Descriptor)
bInterfaceNumber         : 0x00 (Interface 0)
bAlternateSetting        : 0x00
bNumEndpoints            : 0x01 (1 Endpoint)
bInterfaceClass          : 0x03 (HID - Human Interface Device)
bInterfaceSubClass       : 0x00 (None)
bInterfaceProtocol       : 0x00 (None)
iInterface               : 0x00 (No String Descriptor)
Data (HexDump)           : 09 04 00 00 01 03 00 00 00                        .........

        ------------------- HID Descriptor --------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x21 (HID Descriptor)
bcdHID                   : 0x0110 (HID Version 1.10)
bCountryCode             : 0x00 (00 = not localized)
bNumDescriptors          : 0x01
Data (HexDump)           : 09 21 10 01 00 01 22 17 00                        .!...."..
Descriptor 1:
bDescriptorType          : 0x22 (Class=Report)
wDescriptorLength        : 0x0017 (23 bytes)
Error reading descriptor : ERROR_GEN_FAILURE (due to a obscure limitation of the Win32 USB API, see F1 Help)

        ----------------- Endpoint Descriptor -----------------
bLength                  : 0x07 (7 bytes)
bDescriptorType          : 0x05 (Endpoint Descriptor)
bEndpointAddress         : 0x81 (Direction=IN EndpointID=1)
bmAttributes             : 0x03 (TransferType=Interrupt)
wMaxPacketSize           : 0x0002 (2 bytes)
bInterval                : 0x0A (10 ms)
Data (HexDump)           : 07 05 81 03 02 00 0A                              .......

      -------------------- String Descriptors -------------------
String descriptors are not available  (because the device has problem code CM_PROB_FAILED_START)


### ASSISTANT
Спасибо за подробный отчёт о состоянии USB-устройства. На основе предоставленных данных можно сделать **ключевой вывод**:

---

### ❌ **Проблема: Устройство не проходит инициализацию (Problem Code 10 — `CM_PROB_FAILED_START`)**

Это означает, что **Windows не может запустить устройство**, несмотря на то, что оно физически подключено и опознано как HID.

---

## 🔍 Анализ причин

### 1. **Некорректный дескриптор конфигурации или отчёта**
В отчёте:
- Устройство заявляет: **1 интерфейс**, **1 endpoint (IN, 0x81)**, **MaxPacketSize = 2 байта**
- HID Report Descriptor заявлен длиной **0x17 = 23 байта**
- Но **Windows не может прочитать сам дескриптор отчёта**:
  > `Error reading descriptor : ERROR_GEN_FAILURE`

Это **сильный признак того, что ваше устройство возвращает неверные данные** при запросе `GET_DESCRIPTOR (HID_REPORT_DESC)`.

Возможные причины:
- Дескриптор отчёта не соответствует заявленной длине.
- Устройство не обрабатывает запрос на чтение дескриптора для **правильного интерфейса**.
- В функции `USBD_HID_Setup()` логика выбора `pbuf` и `len` работает некорректно (например, из-за неправильного `wIndex`).

> В вашем коде выбор дескриптора зависит от `req->wIndex & 0xFF` (номер интерфейса).  
> Но в логе Windows видно: **bInterfaceNumber = 0**, значит, должен запрашиваться дескриптор для интерфейса `0`.

Однако в вашем `usbd_hid.c`:
```c
uint8_t interface = (uint8_t)(req->wIndex & 0xFF);
```
Если у вас включено **только `CONSUMER_CONTROL`**, то:
- `INTERFACE_CONSUMER_NUMBER = 0 + 0 + 1 = 1`
- Но Windows запрашивает интерфейс `0`, а ваш код ищет дескриптор только для интерфейса `1` → **не находит** → отправляет `USBD_CtlError()` → Windows не может прочитать дескриптор → **ошибка запуска**.

### 2. **Несоответствие между количеством интерфейсов и реальной конфигурацией**
Ваше устройство в отчёте заявляет:
```ini
bNumInterfaces : 0x01
```
Но в коде `USBD_HID_CfgDesc` количество интерфейсов зависит от `HID_INTERFACE_COUNT`.

Если вы включили, например, только `CONSUMER_CONTROL`, то:
- `HID_INTERFACE_COUNT = 1` ✅
- Но **номер интерфейса** для consumer = `INTERFACE_CONSUMER_NUMBER = 1` ❌  
  → должен быть **0**, если это единственный интерфейс!

### 3. **Endpoint size не соответствует структуре отчёта**
- Endpoint заявлен как **2 байта** (`wMaxPacketSize = 2`)
- Но, например, **keyboardHID** имеет размер **8 байт** (`1 + 1 + 1 + 6 = 9?` — на самом деле 8: `id + modifier + reserved + 6 keys`)
- **consumerHID** — 2 байта ✅
- Если вы отправляете keyboard-отчёт через endpoint с размером 2 — это вызовет ошибку.

Но в логе: endpoint size = **2**, и интерфейс — **generic HID** (`bInterfaceSubClass = 0`, `bInterfaceProtocol = 0`), т.е. **не keyboard/mouse**, а **consumer/custom**.

Так что, скорее всего, вы пытаетесь использовать **consumer-интерфейс**, но...

---

## ✅ **Решение**

### Шаг 1: Убедитесь, что номера интерфейсов начинаются с 0 и идут подряд
Замените расчёт номеров интерфейсов на **последовательную нумерацию**:

```c
// Вместо:
#define INTERFACE_KEYBOARD_NUMBER 0
#define INTERFACE_MOUSE_NUMBER    (HID_INTERFACE_KEYBOARD)
#define INTERFACE_CONSUMER_NUMBER (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE)
// ...

// Сделайте так:
#if HID_INTERFACE_KEYBOARD
  #define IFACE_KB 0
#else
  #define IFACE_KB -1
#endif

#if HID_INTERFACE_MOUSE
  #define IFACE_MS (HID_INTERFACE_KEYBOARD ? 1 : 0)
#else
  #define IFACE_MS -1
#endif

// ... и т.д. — это сложно.

// Лучше: используйте фиксированные номера при включении одного интерфейса.
```

**Но проще**: если вы используете **только один интерфейс** (например, consumer), то **все макросы должны давать номер 0**.

👉 **Проверьте**: при включённом **только `CONSUMER_CONTROL`**:
- `HID_INTERFACE_COUNT = 1` ✅
- `INTERFACE_CONSUMER_NUMBER` **должен быть 0**, а не 1!

Ваш текущий код:
```c
#define INTERFACE_KEYBOARD_NUMBER 0
#define INTERFACE_MOUSE_NUMBER    (HID_INTERFACE_KEYBOARD)
#define INTERFACE_CONSUMER_NUMBER (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE)
```
Если `HID_INTERFACE_KEYBOARD = 0`, `HID_INTERFACE_MOUSE = 0`, то:
- `INTERFACE_CONSUMER_NUMBER = 0 + 0 = 0` ✅

Но! В `usbd_hid.h` у вас:
```c
#ifdef KEYBOARD_CONTROL
#define HID_INTERFACE_KEYBOARD 1
#else
#define HID_INTERFACE_KEYBOARD 0
#endif
```
А в начале файла **закомментировано всё, кроме**:
```c
//#define KEYBOARD_CONTROL
//#define MOUSE_CONTROL
#define CONSUMER_CONTROL   // ← только это включено
//#define REMOTE_CONTROL
//#define CUSTOM_CONTROL
```

Тогда:
- `HID_INTERFACE_KEYBOARD = 0`
- `HID_INTERFACE_MOUSE = 0`
- `HID_INTERFACE_CONSUMER = 1`
- → `INTERFACE_CONSUMER_NUMBER = 0 + 0 = 0` ✅

Так что номер интерфейса **должен быть 0**.

### Шаг 2: Убедитесь, что дескриптор отчёта возвращается для интерфейса 0
В `USBD_HID_Setup()`:
```c
if (interface == INTERFACE_CONSUMER_NUMBER) // = 0
{
  len = MIN(CONSUMER_REPORT_DESC_SIZE, req->wLength);
  pbuf = HID_CONSUMER_ReportDesc;
}
```
→ должно сработать.

**Но!** В логе Windows **не может прочитать дескриптор** → значит, либо:
- `pbuf` не инициализирован (остаётся `NULL`)
- `len = 0`
- или `USBD_CtlSendData()` не вызывается

Добавьте **отладочную проверку**:
```c
else
{
  // Ни один интерфейс не подошёл
  USBD_CtlError(pdev, req);
  ret = USBD_FAIL;
  return ret;
}
```
Если у вас включён только consumer, но `interface = 0`, а `INTERFACE_CONSUMER_NUMBER = 0`, то ветка должна сработать.

### Шаг 3: Проверьте длину дескриптора
В логе: `wDescriptorLength = 0x0017 = 23 байта`.

Посчитайте длину `HID_CONSUMER_ReportDesc`:
```c
{
  0x05, 0x0C,        // 2
  0x09, 0x01,        // 2
  0xA1, 0x01,        // 2
  0x15, 0x00,        // 2
  0x26, 0xFF, 0x03,  // 3
  0x19, 0x00,        // 2
  0x2A, 0xFF, 0x03,  // 3
  0x75, 0x10,        // 2
  0x95, 0x01,        // 2
  0x81, 0x00,        // 2
  0xC0               // 1
}
// Итого: 2+2+2+2+3+2+3+2+2+2+1 = **23 байта** ✅
```
→ длина совпадает.

### Шаг 4: Проверьте, что endpoint size = 2
Ваш `сonsumerHID`:
```c
typedef struct {
  uint8_t lsb;
  uint8_t msb;
} сonsumerHID;
```
→ размер = 2 байта ✅  
→ `HID_CONSUMER_EP_SIZE = 2` ✅  
→ в дескрипторе endpoint: `wMaxPacketSize = 2` ✅

---

## 🛠️ Рекомендации

1. **Включите только `CONSUMER_CONTROL`** и убедитесь, что:
   - `INTERFACE_CONSUMER_NUMBER == 0`
   - В `USBD_HID_Setup()` ветка для consumer срабатывает при `interface == 0`
   - Дескриптор действительно отправляется

2. **Добавьте логирование или LED-мигание** при обработке `GET_DESCRIPTOR`, чтобы убедиться, что запрос доходит до MCU.

3. **Проверьте, не возвращает ли устройство STALL** при запросе дескриптора — это вызывает `ERROR_GEN_FAILURE`.

4. **Убедитесь, что `USBD_CtlSendData()` вызывается с правильными параметрами**:
   ```c
   USBD_CtlSendData(pdev, pbuf, len); // len должно быть > 0
   ```

---

Если после этого проблема останется — вероятно, ошибка в **низкоуровневом USB-стеке** (например, не хватает буфера, или ошибка в `USBD_LL_Transmit`).

Но наиболее вероятная причина — **логика выбора интерфейса в `USBD_HID_Setup()` не срабатывает**, и устройство отвечает STALL вместо дескриптора.

Хотите — я помогу переписать эту функцию для надёжной работы с одним интерфейсом.

### USER
Да давай попробуем ее переписать.

### ASSISTANT
Отлично! Давайте перепишем функцию `USBD_HID_Setup()` так, чтобы она **надёжно обрабатывала запросы дескрипторов** для composite HID-устройства, особенно когда включён **только один интерфейс** (например, `CONSUMER_CONTROL`).

Мы сделаем следующее:
- Чётко сопоставим номер интерфейса (`wIndex`) с нужным дескриптором.
- Убедимся, что даже при частичной конфигурации (только consumer, только keyboard и т.д.) логика работает.
- Добавим защиту от `NULL` и нулевой длины.
- Сохраним совместимость с вашими `#define`-настройками.

---

### ✅ Обновлённая версия `USBD_HID_Setup()`

Замените текущую функцию на эту:

```c
/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
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
          uint8_t desc_type = (uint8_t)(req->wValue >> 8);
          uint8_t interface = (uint8_t)(req->wIndex & 0xFF);

          if (desc_type == HID_REPORT_DESC)
          {
            pbuf = NULL;
            len = 0;

#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER)
            {
              pbuf = HID_KEYBOARD_ReportDesc;
              len = KEYBOARD_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER)
            {
              pbuf = HID_MOUSE_ReportDesc;
              len = MOUSE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER)
            {
              pbuf = HID_CONSUMER_ReportDesc;
              len = CONSUMER_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER)
            {
              pbuf = HID_REMOTE_ReportDesc;
              len = REMOTE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER)
            {
              pbuf = HID_CUSTOM_ReportDesc;
              len = CUSTOM_REPORT_DESC_SIZE;
            }
            else
#endif
            {
              // Ни один интерфейс не совпал
              pbuf = NULL;
              len = 0;
            }

            if (pbuf != NULL && len > 0)
            {
              len = MIN(len, req->wLength);
              USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              // Неизвестный интерфейс или дескриптор недоступен
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else if (desc_type == HID_DESCRIPTOR_TYPE)
          {
            // Windows иногда запрашивает HID-дескриптор отдельно.
            // Мы можем его вернуть, но проще — игнорировать, т.к. он уже в составе конфигурации.
            // Но лучше вернуть корректный!
            // Найдём нужный HID-дескриптор по интерфейсу.
            static uint8_t hid_desc[9];
            uint16_t report_len = 0;

#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER) report_len = KEYBOARD_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER) report_len = MOUSE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER) report_len = CONSUMER_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER) report_len = REMOTE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER) report_len = CUSTOM_REPORT_DESC_SIZE;
            else
#endif
            {
              report_len = 0;
            }

            if (report_len > 0)
            {
              // Собираем HID-дескриптор (9 байт)
              hid_desc[0] = 0x09;               // bLength
              hid_desc[1] = HID_DESCRIPTOR_TYPE; // bDescriptorType
              hid_desc[2] = 0x10;               // bcdHID LSB
              hid_desc[3] = 0x01;               // bcdHID MSB
              hid_desc[4] = 0x00;               // bCountryCode
              hid_desc[5] = 0x01;               // bNumDescriptors
              hid_desc[6] = 0x22;               // bDescriptorType (Report)
              hid_desc[7] = LOBYTE(report_len); // wDescriptorLength LSB
              hid_desc[8] = HIBYTE(report_len); // wDescriptorLength MSB

              len = MIN(9, req->wLength);
              USBD_CtlSendData(pdev, hid_desc, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
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
```

---

### 🔑 Ключевые улучшения:

1. **Явная проверка `pbuf != NULL && len > 0`** перед отправкой.
2. **Поддержка запроса `HID_DESCRIPTOR_TYPE`** — Windows иногда его запрашивает отдельно (особенно при проблемах).
3. **Чёткое сопоставление интерфейса → дескриптора**, даже если включён только один.
4. **Без "fall-through" в else-цепочке** — если ни один `if` не сработал, `pbuf` остаётся `NULL`, и мы отправляем ошибку.

---

### 🧪 Что проверить на устройстве:

1. Убедитесь, что в `usbd_hid.h` включён **только** `CONSUMER_CONTROL`.
2. Пересоберите прошивку.
3. Подключите устройство — оно должно определиться как **HID-устройство** без ошибки 10.
4. Попробуйте отправить медиа-клавишу:
   ```c
   uint8_t report[3] = { HID_MEDIA_REPORT, 0xE9, 0x00 }; // Volume Up
   USBD_HID_SendReport_EP(&hUsbDeviceFS, report, 3, HID_CONSUMER_EP);
   ```

---

Если после этого всё равно ошибка — проверьте:
- Что `HID_CONSUMER_EP` действительно равен `0x81` (а не `0x82` и т.д.)
- Что в `USBD_HID_Init()` открывается **именно тот endpoint**, который заявлен в дескрипторе.

Хочешь — я помогу также исправить **endpoint assignment** и **Init/DeInit**.

### USER
включен только consumer? заменил эту функцию на твою - тот же результат:
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
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
          uint8_t desc_type = (uint8_t)(req->wValue >> 8);
          uint8_t interface = (uint8_t)(req->wIndex & 0xFF);

          if (desc_type == HID_REPORT_DESC)
          {
            pbuf = NULL;
            len = 0;

#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER)
            {
              pbuf = HID_KEYBOARD_ReportDesc;
              len = KEYBOARD_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER)
            {
              pbuf = HID_MOUSE_ReportDesc;
              len = MOUSE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER)
            {
              pbuf = HID_CONSUMER_ReportDesc;
              len = CONSUMER_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER)
            {
              pbuf = HID_REMOTE_ReportDesc;
              len = REMOTE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER)
            {
              pbuf = HID_CUSTOM_ReportDesc;
              len = CUSTOM_REPORT_DESC_SIZE;
            }
            else
#endif
            {
              // Ни один интерфейс не совпал
              pbuf = NULL;
              len = 0;
            }

            if (pbuf != NULL && len > 0)
            {
              len = MIN(len, req->wLength);
              USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              // Неизвестный интерфейс или дескриптор недоступен
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else if (desc_type == HID_DESCRIPTOR_TYPE)
          {
            // Windows иногда запрашивает HID-дескриптор отдельно.
            // Мы можем его вернуть, но проще — игнорировать, т.к. он уже в составе конфигурации.
            // Но лучше вернуть корректный!
            // Найдём нужный HID-дескриптор по интерфейсу.
            static uint8_t hid_desc[9];
            uint16_t report_len = 0;

#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER) report_len = KEYBOARD_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER) report_len = MOUSE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER) report_len = CONSUMER_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER) report_len = REMOTE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER) report_len = CUSTOM_REPORT_DESC_SIZE;
            else
#endif
            {
              report_len = 0;
            }

            if (report_len > 0)
            {
              // Собираем HID-дескриптор (9 байт)
              hid_desc[0] = 0x09;               // bLength
              hid_desc[1] = HID_DESCRIPTOR_TYPE; // bDescriptorType
              hid_desc[2] = 0x10;               // bcdHID LSB
              hid_desc[3] = 0x01;               // bcdHID MSB
              hid_desc[4] = 0x00;               // bCountryCode
              hid_desc[5] = 0x01;               // bNumDescriptors
              hid_desc[6] = 0x22;               // bDescriptorType (Report)
              hid_desc[7] = LOBYTE(report_len); // wDescriptorLength LSB
              hid_desc[8] = HIBYTE(report_len); // wDescriptorLength MSB

              len = MIN(9, req->wLength);
              USBD_CtlSendData(pdev, hid_desc, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
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

### ASSISTANT
Спасибо за уточнение. Раз **ошибка осталась та же** (`CM_PROB_FAILED_START`, и Windows не может проч

### USER
включен только consumer? заменил эту функцию на твою - тот же результат:

    =========================== USB Port4 ===========================

Connection Status        : 0x01 (Device is connected)
Port Chain               : 1-4
Properties               : 0x01
 IsUserConnectable       : yes
 PortIsDebugCapable      : no
 PortHasMultiCompanions  : no
 PortConnectorIsTypeC    : no
ConnectionIndex          : 0x04 (Port 4)
CompanionIndex           : 0
 CompanionHubSymLnk      : USB#ROOT_HUB30#4&cdbaec3&0&0#{f18a0e88-c30c-11d0-8815-00a0c906bed8}
 CompanionPortNumber     : 0x14 (Port 20)
 -> CompanionPortChain   : 1-20

      ========================== Summary =========================
Vendor ID                : 0x1209 (Unknown Vendor)
Product ID               : 0x0001
Serial                   : "48E074A73639"
USB Version              : 1.1 (12 Mbit/s FullSpeed)
Port maximum Speed       : High-Speed (Companion Port 1-20 is doing the SuperSpeed)
Device maximum Speed     : Full-Speed
Device Connection Speed  : Full-Speed
Self powered             : no
Demanded Current         : 500 mA
Device Manager Problem   : 10 (CM_PROB_FAILED_START)
Used Endpoints           : 1

      ======================== USB Device ========================

        +++++++++++++++++ Device Information ++++++++++++++++++
Device Description       : USB-устройство ввода
Device Path              : \\?\USB#VID_0000&PID_0000#48E074A73639#{a5dcbf10-6530-11d2-901f-00c04fb951ed} (GUID_DEVINTERFACE_USB_DEVICE)
Kernel Name              : \Device\USBPDO-13
Device ID                : USB\VID_1209&PID_0001\48E074A73639
Hardware IDs             : USB\VID_1209&PID_0001&REV_0200 USB\VID_1209&PID_0001
Driver KeyName           : {745a17a0-74d3-11d0-b6fe-00a0c90f57da}\0049 (GUID_DEVCLASS_HIDCLASS)
Driver                   : \SystemRoot\System32\drivers\hidusb.sys (Version: 10.0.19041.3636  Date: 2024-12-25  Company: Microsoft Corporation)
Driver Inf               : C:\Windows\inf\input.inf
Legacy BusType           : PNPBus
Class                    : HIDClass
Class GUID               : {745a17a0-74d3-11d0-b6fe-00a0c90f57da} (GUID_DEVCLASS_HIDCLASS)
Service                  : HidUsb
Enumerator               : USB
Location Info            : Port_#0004.Hub_#0001
Address                  : 4
Location IDs             : PCIROOT(0)#PCI(1400)#USBROOT(0)#USB(4), ACPI(_SB_)#ACPI(PC00)#ACPI(XHCI)#ACPI(RHUB)#ACPI(HS04)
Container ID             : {161e9162-ccb6-5412-8205-5ea232075bb1}
Manufacturer Info        : (Стандартные системные устройства)
Capabilities             : 0x14 (Removable, UniqueID)
Status                   : 0x01806400 (DN_HAS_PROBLEM, DN_DISABLEABLE, DN_REMOVABLE, DN_NT_ENUMERATOR, DN_NT_DRIVER)
Problem Code             : 10 (CM_PROB_FAILED_START)
First Install Date       : 2025-10-22 10:38:36
Last Arrival Date        : 2025-10-22 11:34:02
SelectiveSuspendEnabled  : 0
EnhancedPowerMgmtEnabled : 1
Power State              : D3 (supported: D0, D1, D2, D3, wake from D0, wake from D1, wake from D2)

        ---------------- Connection Information ---------------
Connection Index         : 0x04 (Port 4)
Connection Status        : 0x01 (DeviceConnected)
Current Config Value     : 0x00 (Configuration 0)
Device Address           : 0x32 (50)
Is Hub                   : 0x00 (no)
Device Bus Speed         : 0x01 (Full-Speed)
Number of open Pipes     : 0x00 (0 pipes to data endpoints)
Data (HexDump)           : 04 00 00 00 12 01 10 01 00 00 00 08 09 12 01 00   ................
                           00 02 01 02 03 01 00 01 00 32 00 00 00 00 00 01   .........2......
                           00 00 00                                          ...

        --------------- Connection Information V2 -------------
Connection Index         : 0x04 (4)
Length                   : 0x10 (16 bytes)
SupportedUsbProtocols    : 0x03
 Usb110                  : 1 (yes, port supports USB 1.1)
 Usb200                  : 1 (yes, port supports USB 2.0)
 Usb300                  : 0 (no, port not supports USB 3.0) -> but Companion Port 1-20 does
 ReservedMBZ             : 0x00
Flags                    : 0x00
 DevIsOpAtSsOrHigher     : 0 (Device is not operating at SuperSpeed or higher)
 DevIsSsCapOrHigher      : 0 (Device is not SuperSpeed capable or higher)
 DevIsOpAtSsPlusOrHigher : 0 (Device is not operating at SuperSpeedPlus or higher)
 DevIsSsPlusCapOrHigher  : 0 (Device is not SuperSpeedPlus capable or higher)
 ReservedMBZ             : 0x00
Data (HexDump)           : 04 00 00 00 10 00 00 00 03 00 00 00 00 00 00 00   ................

    ---------------------- Device Descriptor ----------------------
bLength                  : 0x12 (18 bytes)
bDescriptorType          : 0x01 (Device Descriptor)
bcdUSB                   : 0x110 (USB Version 1.1)
bDeviceClass             : 0x00 (defined by the interface descriptors)
bDeviceSubClass          : 0x00
bDeviceProtocol          : 0x00
bMaxPacketSize0          : 0x08 (8 bytes)
idVendor                 : 0x1209 (Unknown Vendor)
idProduct                : 0x0001
bcdDevice                : 0x0200
iManufacturer            : 0x01 (String Descriptor 1)
iProduct                 : 0x02 (String Descriptor 2)
iSerialNumber            : 0x03 (String Descriptor 3)
bNumConfigurations       : 0x01 (1 Configuration)
Data (HexDump)           : 12 01 10 01 00 00 00 08 09 12 01 00 00 02 01 02   ................
                           03 01                                             ..

    ------------------ Configuration Descriptor -------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x02 (Configuration Descriptor)
wTotalLength             : 0x0022 (34 bytes)
bNumInterfaces           : 0x01 (1 Interface)
bConfigurationValue      : 0x01 (Configuration 1)
iConfiguration           : 0x00 (No String Descriptor)
bmAttributes             : 0xA0
 D7: Reserved, set 1     : 0x01
 D6: Self Powered        : 0x00 (no)
 D5: Remote Wakeup       : 0x01 (yes)
 D4..0: Reserved, set 0  : 0x00
MaxPower                 : 0xFA (500 mA)
Data (HexDump)           : 09 02 22 00 01 01 00 A0 FA 09 04 00 00 01 03 00   ..".............
                           00 00 09 21 10 01 00 01 22 17 00 07 05 81 03 02   ...!....".......
                           00 0A                                             ..

        ---------------- Interface Descriptor -----------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x04 (Interface Descriptor)
bInterfaceNumber         : 0x00 (Interface 0)
bAlternateSetting        : 0x00
bNumEndpoints            : 0x01 (1 Endpoint)
bInterfaceClass          : 0x03 (HID - Human Interface Device)
bInterfaceSubClass       : 0x00 (None)
bInterfaceProtocol       : 0x00 (None)
iInterface               : 0x00 (No String Descriptor)
Data (HexDump)           : 09 04 00 00 01 03 00 00 00                        .........

        ------------------- HID Descriptor --------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x21 (HID Descriptor)
bcdHID                   : 0x0110 (HID Version 1.10)
bCountryCode             : 0x00 (00 = not localized)
bNumDescriptors          : 0x01
Data (HexDump)           : 09 21 10 01 00 01 22 17 00                        .!...."..
Descriptor 1:
bDescriptorType          : 0x22 (Class=Report)
wDescriptorLength        : 0x0017 (23 bytes)
Error reading descriptor : ERROR_GEN_FAILURE (due to a obscure limitation of the Win32 USB API, see F1 Help)

        ----------------- Endpoint Descriptor -----------------
bLength                  : 0x07 (7 bytes)
bDescriptorType          : 0x05 (Endpoint Descriptor)
bEndpointAddress         : 0x81 (Direction=IN EndpointID=1)
bmAttributes             : 0x03 (TransferType=Interrupt)
wMaxPacketSize           : 0x0002 (2 bytes)
bInterval                : 0x0A (10 ms)
Data (HexDump)           : 07 05 81 03 02 00 0A                              .......

      -------------------- String Descriptors -------------------
String descriptors are not available  (because the device has problem code CM_PROB_FAILED_START)


### ASSISTANT
Спасибо за подробный дамп — он **очень информативен**.

Разберём **ключевые факты** из лога:

---

### 🔍 Что мы видим:
1. **Устройство подключено**, опознано как HID (`bInterfaceClass = 0x03`).
2. **1 интерфейс**, номер **0**.
3. **Endpoint**: `0x81`, тип **Interrupt IN**, размер **2 байта** — ✅ подходит для `consumerHID`.
4. **HID Descriptor** говорит:  
   → Report Descriptor длиной **0x0017 = 23 байта** — ✅ совпадает с вашим `HID_CONSUMER_ReportDesc`.
5. **НО**:  
   > `Error reading descriptor : ERROR_GEN_FAILURE`  
   → Windows **не может прочитать сам Report Descriptor**, несмотря на то, что знает его длину.

6. **Current Config Value = 0x00**  
   → Устройство **не перешло в состояние "configured"**!  
   → Это **прямой признак**, что дескрипторы не прошли валидацию.

---

## 🚨 Главная гипотеза: **Устройство не обрабатывает запрос `GET_DESCRIPTOR` на Report Descriptor**

Даже с обновлённой функцией `USBD_HID_Setup()` — **запрос не доходит или возвращается не то**.

Но почему?

---

### 🔎 Глубокий анализ: что запрашивает Windows?

Когда Windows видит HID-устройство, она делает такой запрос:

```
GET_DESCRIPTOR
  wValue    = 0x2200  → тип: Report Descriptor (0x22), индекс 0
  wIndex    = 0x0000  → интерфейс 0
  wLength   = 255 (обычно)
```

Ваш код:
```c
uint8_t desc_type = (uint8_t)(req->wValue >> 8); // = 0x22
uint8_t interface = (uint8_t)(req->wIndex & 0xFF); // = 0
```

→ Всё верно.

И у вас включён **только `CONSUMER_CONTROL`**, значит:
- `HID_INTERFACE_CONSUMER = 1`
- `INTERFACE_CONSUMER_NUMBER = 0 + 0 = 0` ✅

→ Ветка `if (interface == INTERFACE_CONSUMER_NUMBER)` **должна сработать**.

Но **если `pClassData == NULL`**, то `hhid` — мусор, и всё ломается.

---

## ⚠️ Критическая проблема: **`pdev->pClassData` может быть NULL в момент запроса дескриптора!**

### Почему?
Потому что:
- Запрос `GET_DESCRIPTOR` приходит **до** вызова `USBD_HID_Init()`.
- А `USBD_HID_Init()` выделяет `pClassData`.
- Но **запросы дескрипторов обрабатываются ДО конфигурации**!

👉 **Вывод**: в `USBD_HID_Setup()` **нельзя использовать `pdev->pClassData` для HID-специфичных запросов, если устройство ещё не инициализировано**.

Но! Для `GET_DESCRIPTOR` — **`pClassData` вообще не нужен!**

Однако вы используете его в других ветках (`GET_PROTOCOL`, `GET_IDLE`), и **обращение к `hhid` при `pClassData == NULL` вызывает undefined behavior** (возможно, hard fault или STALL).

---

## ✅ Решение: **Не используйте `hhid` в `USBD_HID_Setup()` до тех пор, пока не убедитесь, что `pClassData != NULL`**

Но для `GET_DESCRIPTOR` — это не нужно вообще.

### 🔧 Исправленная версия (минимальная, безопасная):

```c
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    {
      // Для CLASS-запросов нужен pClassData
      if (pdev->pClassData == NULL)
      {
        USBD_CtlError(pdev, req);
        return USBD_FAIL;
      }

      USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
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
    }

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
          uint8_t desc_type = (uint8_t)(req->wValue >> 8);
          uint8_t interface = (uint8_t)(req->wIndex & 0xFF);

          if (desc_type == HID_REPORT_DESC)
          {
#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER)
            {
              pbuf = HID_CONSUMER_ReportDesc;
              len = CONSUMER_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER)
            {
              pbuf = HID_KEYBOARD_ReportDesc;
              len = KEYBOARD_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER)
            {
              pbuf = HID_MOUSE_ReportDesc;
              len = MOUSE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER)
            {
              pbuf = HID_REMOTE_ReportDesc;
              len = REMOTE_REPORT_DESC_SIZE;
            }
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER)
            {
              pbuf = HID_CUSTOM_ReportDesc;
              len = CUSTOM_REPORT_DESC_SIZE;
            }
            else
#endif
            {
              // No matching interface
              USBD_CtlError(pdev, req);
              return USBD_FAIL;
            }

            len = MIN(len, req->wLength);
            USBD_CtlSendData(pdev, pbuf, len);
            return USBD_OK;
          }
          else if (desc_type == HID_DESCRIPTOR_TYPE)
          {
            // Optional: return HID descriptor (usually not needed, but safe to support)
            static uint8_t hid_desc[9];
            uint16_t report_len = 0;

#if HID_INTERFACE_CONSUMER
            if (interface == INTERFACE_CONSUMER_NUMBER) report_len = CONSUMER_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_KEYBOARD
            if (interface == INTERFACE_KEYBOARD_NUMBER) report_len = KEYBOARD_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_MOUSE
            if (interface == INTERFACE_MOUSE_NUMBER) report_len = MOUSE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_REMOTE
            if (interface == INTERFACE_REMOTE_NUMBER) report_len = REMOTE_REPORT_DESC_SIZE;
            else
#endif
#if HID_INTERFACE_CUSTOM
            if (interface == INTERFACE_CUSTOM_NUMBER) report_len = CUSTOM_REPORT_DESC_SIZE;
            else
#endif
            {
              USBD_CtlError(pdev, req);
              return USBD_FAIL;
            }

            hid_desc[0] = 0x09;
            hid_desc[1] = HID_DESCRIPTOR_TYPE;
            hid_desc[2] = 0x10; // bcdHID LSB
            hid_desc[3] = 0x01; // bcdHID MSB
            hid_desc[4] = 0x00; // Country
            hid_desc[5] = 0x01; // Num descriptors
            hid_desc[6] = 0x22; // Report desc type
            hid_desc[7] = LOBYTE(report_len);
            hid_desc[8] = HIBYTE(report_len);

            len = MIN(9U, req->wLength);
            USBD_CtlSendData(pdev, hid_desc, len);
            return USBD_OK;
          }
          // else fall through to error
        }
        // ... остальные стандартные запросы (GET_INTERFACE и т.д.) — их можно оставить,
        // но они тоже требуют pClassData. Однако Windows запрашивает их ПОСЛЕ конфигурации.
        // Так что для GET_DESCRIPTOR — главное — не упасть.

        default:
          // Для остальных стандартных запросов (GET_INTERFACE и т.д.)
          // можно проверить pClassData, но если их нет — не страшно.
          // Пока сосредоточимся на дескрипторах.
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
```

---

## 🔑 Самое важное изменение:

> **Не обращайтесь к `pdev->pClassData` в `USBD_HID_Setup()` до проверки `if (pdev->pClassData != NULL)`**  
> Особенно — **в ветке `USB_REQ_GET_DESCRIPTOR`**, потому что она вызывается **до инициализации класса**.

В предыдущей версии вы **всегда** делали:
```c
USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
```
— даже если `pClassData == NULL` → **crash или STALL**.

---

## ✅ Что делать дальше:

1. **Замените функцию на эту упрощённую версию** (можно оставить только consumer-ветку для теста).
2. Убедитесь, что в `usbd_hid.h`:
   ```c
   #define CONSUMER_CONTROL
   // остальное закомментировано
   ```
3. Пересоберите и прошейте.
4. Подключите — должно заработать.

Если не заработает — проверьте **логику endpoint assignment** и **Init**, но сначала исправьте **Setup**.

Готов помочь с остальным!

### USER
Я заметил что эта функция вообще не вызывается при подсоединении устройства.

### ASSISTANT
Отличное наблюдение — и **очень важное**.

Если функция `USBD_HID_Setup()` **вообще не вызывается** при подключении устройства, это означает, что **USB-стек не связывает ваш класс с устройством на этапе перечисления (enumeration)**.

Давайте разберёмся, почему так происходит.

---

## 🔍 Когда вызывается `USBD_HID_Setup()`?

Функция `USBD_HID_Setup()` вызывается **только тогда**, когда:
1. Устройство прошло начальный этап enumeration (устройство, конфигурация, строковые дескрипторы).
2. Хост (Windows) отправляет **стандартный или класс-специфичный запрос**, адресованный **интерфейсу**, который обслуживается вашим классом (`USBD_HID`).
3. **И** этот класс был **зарегистрирован в USB-стеке** как обработчик для текущей конфигурации.

Но если устройство **даже не доходит до запроса дескрипторов интерфейса**, то `Setup` не вызовется.

---

## 🚨 Возможные причины, почему `USBD_HID_Setup()` не вызывается

### 1. **Неправильная длина конфигурационного дескриптора (`wTotalLength`)**
В логе Windows:
```ini
wTotalLength : 0x0022 (34 bytes)
```

Проверьте, **реально ли ваш `USBD_HID_CfgDesc` имеет 34 байта** при включённом только `CONSUMER_CONTROL`.

Рассчитаем:
- Configuration Descriptor: 9
- Interface Descriptor: 9
- HID Descriptor: 9
- Endpoint Descriptor: 7  
→ **Итого: 9 + 9 + 9 + 7 = 34** ✅

**Но!** В коде вы используете:
```c
#define USB_HID_CONFIG_DESC_SIZ (USB_HID_DESC_SIZ + ...)
```
Убедитесь, что макрос **точно равен 34**, когда включён только consumer.

👉 **Проверьте в отладчике или через `sizeof(USBD_HID_CfgDesc)`** — если длина не совпадает, Windows обрывает enumeration.

---

### 2. **Ошибка в самом массиве `USBD_HID_CfgDesc`**
Если в дескрипторе есть **битый байт** (например, из-за неправильного `#if`), Windows может не распознать структуру и **не отправить запросы к интерфейсу**.

В логе всё выглядит корректно, но проверьте **реальное содержимое памяти** `USBD_HID_CfgDesc` при запуске.

---

### 3. **Класс не зарегистрирован как обработчик**
Убедитесь, что вы вызываете:
```c
USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);
```
в `main()` или при инициализации USB.

Если не зарегистрирован — стек не знает, кому передавать `Setup`-запросы.

---

### 4. **Устройство не переходит в состояние `USBD_STATE_CONFIGURED`**
Но это уже следствие, а не причина.

---

### 5. **Проблема с `bcdUSB` или другими полями Device Descriptor**
В логе:
```ini
bcdUSB : 0x110 (USB 1.1)
```
Это нормально для Full-Speed HID.

Но убедитесь, что в `usbd_desc.c` (или где у вас Device Descriptor) указано:
```c
0x10, 0x01, // bcdUSB = 1.10
```
(младший байт первый → `0x10, 0x01` = 0x0110)

Если перепутан порядок — некоторые хосты могут отвергнуть устройство.

---

## ✅ Что делать прямо сейчас

### Шаг 1: Убедитесь, что `USBD_HID` зарегистрирован
В `main.c` должно быть:
```c
USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);
USBD_Start(&hUsbDeviceFS);
```

### Шаг 2: Проверьте реальную длину `USBD_HID_CfgDesc`
Добавьте в код (временно):
```c
volatile uint16_t cfg_len = sizeof(USBD_HID_CfgDesc);
```
и посмотрите в отладчике — равно ли **34** при включённом только `CONSUMER_CONTROL`.

Если нет — пересчитайте макросы.

### Шаг 3: Упростите дескриптор до минимума
Создайте **статический конфигурационный дескриптор только для consumer**, без `#if`:

```c
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[34] __ALIGN_END =
{
  // Configuration
  0x09, 0x02, 34, 0, 0x01, 0x01, 0x00, 0xA0, 0xFA,
  // Interface 0
  0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
  // HID
  0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 23, 0x00,
  // Endpoint
  0x07, 0x05, 0x81, 0x03, 0x02, 0x00, 0x0A
};
```

И замените `USBD_HID_GetCfgDesc` на:
```c
static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length)
{
  *length = 34;
  return USBD_HID_CfgDesc;
}
```

Это исключит ошибки препроцессора.

### Шаг 4: Добавьте "мигание" в `USBD_HID_Setup()`
В начало функции добавьте:
```c
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // или любой LED
```
Если LED **не мигает** при подключении — значит, функция **действительно не вызывается**, и проблема **до уровня класса** (дескрипторы или регистрация).

---

## 💡 Вывод

Скорее всего, проблема в **некорректной длине или содержимом конфигурационного дескриптора**, из-за чего Windows **не распознаёт интерфейс как HID** и не отправляет `Setup`-запросы вашему классу.

Начните с **статического дескриптора длиной 34 байта** — это самый надёжный способ проверить гипотезу.

Готов помочь составить такой дескриптор или проверить регистрацию класса.

### USER
Да все проверил - все правильно. Добавил мигание - не мигает.

### ASSISTANT
Отлично — вы подтвердили **ключевой факт**:

> 💡 **Функция `USBD_HID_Setup()` не вызывается вообще.**

Это означает, что **USB-стек ST не передаёт запросы вашему классу**, потому что **устройство не доходит до этапа, где запрашиваются дескрипторы интерфейса**.

Поскольку вы уже проверили:
- Регистрация класса (`USBD_RegisterClass(&USBD_HID)`) — есть
- Длина конфигурационного дескриптора — 34 байта
- Содержимое дескриптора — корректно (по логу Windows)

…остаётся **одна из самых частых, но незаметных ошибок**:

---

## 🚨 Проблема: **Device Descriptor заявляет `bNumConfigurations = 1`, но стек не может получить Configuration Descriptor**

А почему? Потому что **функция `USBD_HID_GetCfgDesc()` не вызывается** — или вызывается, но возвращает **неправильный указатель/длину**.

Но как проверить?

---

### 🔍 Где задаётся обработчик конфигурационного дескриптора?

В структуре `USBD_ClassTypeDef USBD_HID` вы указали:

```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /*EP0_TxSent*/
  NULL, /*EP0_RxReady*/
  USBD_HID_DataIn,
  NULL, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_HID_GetCfgDesc,   // ← для FS
  USBD_HID_GetCfgDesc,   // ← для HS
  USBD_HID_GetCfgDesc,   // ← для Other Speed
  USBD_HID_GetDeviceQualifierDesc,
};
```

→ Это правильно.

Но **USB-стек вызывает `USBD_HID_GetCfgDesc()` только после того, как устройство запросит Configuration Descriptor**.

Если **Device Descriptor** или **первичная инициализация** сломаны — этого не произойдёт.

---

## ✅ Проверьте **Device Descriptor** (вне файла `usbd_hid.c`!)

Скорее всего, он находится в **`usbd_desc.c`** или подобном.

В логе Windows:
```ini
idVendor                 : 0x1209
idProduct                : 0x0001
bcdDevice                : 0x0200
iManufacturer            : 0x01
iProduct                 : 0x02
iSerialNumber            : 0x03
bNumConfigurations       : 0x01
```

Убедитесь, что в `usbd_desc.c`:

1. **`USBD_DeviceDesc`** действительно содержит:
   ```c
   0x12,         /* bLength */
   0x01,         /* bDescriptorType */
   0x10, 0x01,   /* bcdUSB = 1.10 */
   0x00,         /* bDeviceClass */
   0x00,         /* bDeviceSubClass */
   0x00,         /* bDeviceProtocol */
   0x40,         /* bMaxPacketSize0 — ДОЛЖЕН БЫТЬ 0x40 (64) ДЛЯ FS! */
   LOBYTE(0x1209), HIBYTE(0x1209), /* idVendor */
   LOBYTE(0x0001), HIBYTE(0x0001), /* idProduct */
   0x00, 0x02,   /* bcdDevice */
   0x01,         /* iManufacturer */
   0x02,         /* iProduct */
   0x03,         /* iSerial */
   0x01          /* bNumConfigurations */
   ```

2. **❗ КРИТИЧЕСКИ ВАЖНО**:  
   `bMaxPacketSize0` **должен быть 0x40 (64)** для Full-Speed устройства, **даже если вы используете только 8 байт**.

   В вашем логе:
   > `bMaxPacketSize0 : 0x08 (8 bytes)`

   ❌ **ЭТО ОШИБКА!**

   Для **Full-Speed** устройств:
   - `bMaxPacketSize0` **всегда 64** (0x40)
   - Для **Low-Speed** — 8

   Ваше устройство — **Full-Speed** (`Device Bus Speed : Full-Speed`), но заявляет `bMaxPacketSize0 = 8` → **нарушение спецификации USB**.

   → Многие хосты (особенно Windows) **отказываются работать** с таким устройством.

---

## ✅ Исправление

В файле с **Device Descriptor** (обычно `usbd_desc.c`) найдите:

```c
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x10, 0x01,                 /* bcdUSB */
  0x00,                       /* bDeviceClass */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  0x08,                       /* ❌ bMaxPacketSize0 — НЕПРАВИЛЬНО! */
  ...
};
```

И замените на:

```c
  0x40,                       /* ✅ bMaxPacketSize0 = 64 для Full-Speed */
```

---

## Почему это критично?

- EP0 (control endpoint) **должен поддерживать пакеты до 64 байт** на Full-Speed.
- Если вы заявляете 8, но хост отправляет SETUP-пакет (8 байт) + данные (например, 20 байт) — устройство не может принять.
- Windows делает внутренние допущения и **может просто отвергнуть устройство** на этапе чтения Device Descriptor.

---

## ✅ После исправления

1. Измените `bMaxPacketSize0` на `0x40`.
2. Пересоберите и прошейте.
3. Подключите — LED в `USBD_HID_Setup()` **должен мигнуть**.
4. Ошибка 10 должна исчезнуть.

---

Если не поможет — пришлите содержимое вашего **`usbd_desc.c`**, особенно `USBD_DeviceDesc`.

Но с высокой вероятностью — **именно это и есть корень проблемы**.

### USER
Исправил:

    =========================== USB Port4 ===========================

Connection Status        : 0x01 (Device is connected)
Port Chain               : 1-4
Properties               : 0x01
 IsUserConnectable       : yes
 PortIsDebugCapable      : no
 PortHasMultiCompanions  : no
 PortConnectorIsTypeC    : no
ConnectionIndex          : 0x04 (Port 4)
CompanionIndex           : 0
 CompanionHubSymLnk      : USB#ROOT_HUB30#4&cdbaec3&0&0#{f18a0e88-c30c-11d0-8815-00a0c906bed8}
 CompanionPortNumber     : 0x14 (Port 20)
 -> CompanionPortChain   : 1-20

      ========================== Summary =========================
Vendor ID                : 0x1209 (Unknown Vendor)
Product ID               : 0x0001
Serial                   : "48E074A73639"
USB Version              : 1.1 (12 Mbit/s FullSpeed)
Port maximum Speed       : High-Speed (Companion Port 1-20 is doing the SuperSpeed)
Device maximum Speed     : Full-Speed
Device Connection Speed  : Full-Speed
Self powered             : no
Demanded Current         : 500 mA
Device Manager Problem   : 10 (CM_PROB_FAILED_START)
Used Endpoints           : 1

      ======================== USB Device ========================

        +++++++++++++++++ Device Information ++++++++++++++++++
Device Description       : USB-устройство ввода
Device Path              : \\?\USB#VID_0000&PID_0000#48E074A73639#{a5dcbf10-6530-11d2-901f-00c04fb951ed} (GUID_DEVINTERFACE_USB_DEVICE)
Kernel Name              : \Device\USBPDO-13
Device ID                : USB\VID_1209&PID_0001\48E074A73639
Hardware IDs             : USB\VID_1209&PID_0001&REV_0200 USB\VID_1209&PID_0001
Driver KeyName           : {745a17a0-74d3-11d0-b6fe-00a0c90f57da}\0049 (GUID_DEVCLASS_HIDCLASS)
Driver                   : \SystemRoot\System32\drivers\hidusb.sys (Version: 10.0.19041.3636  Date: 2024-12-25  Company: Microsoft Corporation)
Driver Inf               : C:\Windows\inf\input.inf
Legacy BusType           : PNPBus
Class                    : HIDClass
Class GUID               : {745a17a0-74d3-11d0-b6fe-00a0c90f57da} (GUID_DEVCLASS_HIDCLASS)
Service                  : HidUsb
Enumerator               : USB
Location Info            : Port_#0004.Hub_#0001
Address                  : 4
Location IDs             : PCIROOT(0)#PCI(1400)#USBROOT(0)#USB(4), ACPI(_SB_)#ACPI(PC00)#ACPI(XHCI)#ACPI(RHUB)#ACPI(HS04)
Container ID             : {161e9162-ccb6-5412-8205-5ea232075bb1}
Manufacturer Info        : (Стандартные системные устройства)
Capabilities             : 0x14 (Removable, UniqueID)
Status                   : 0x01806400 (DN_HAS_PROBLEM, DN_DISABLEABLE, DN_REMOVABLE, DN_NT_ENUMERATOR, DN_NT_DRIVER)
Problem Code             : 10 (CM_PROB_FAILED_START)
First Install Date       : 2025-10-22 10:38:36
Last Arrival Date        : 2025-10-22 12:44:52
SelectiveSuspendEnabled  : 0
EnhancedPowerMgmtEnabled : 1
Power State              : D3 (supported: D0, D1, D2, D3, wake from D0, wake from D1, wake from D2)

        ---------------- Connection Information ---------------
Connection Index         : 0x04 (Port 4)
Connection Status        : 0x01 (DeviceConnected)
Current Config Value     : 0x00 (Configuration 0)
Device Address           : 0x13 (19)
Is Hub                   : 0x00 (no)
Device Bus Speed         : 0x01 (Full-Speed)
Number of open Pipes     : 0x00 (0 pipes to data endpoints)
Data (HexDump)           : 04 00 00 00 12 01 10 01 00 00 00 40 09 12 01 00   ...........@....
                           00 02 01 02 03 01 00 01 00 13 00 00 00 00 00 01   ................
                           00 00 00                                          ...

        --------------- Connection Information V2 -------------
Connection Index         : 0x04 (4)
Length                   : 0x10 (16 bytes)
SupportedUsbProtocols    : 0x03
 Usb110                  : 1 (yes, port supports USB 1.1)
 Usb200                  : 1 (yes, port supports USB 2.0)
 Usb300                  : 0 (no, port not supports USB 3.0) -> but Companion Port 1-20 does
 ReservedMBZ             : 0x00
Flags                    : 0x00
 DevIsOpAtSsOrHigher     : 0 (Device is not operating at SuperSpeed or higher)
 DevIsSsCapOrHigher      : 0 (Device is not SuperSpeed capable or higher)
 DevIsOpAtSsPlusOrHigher : 0 (Device is not operating at SuperSpeedPlus or higher)
 DevIsSsPlusCapOrHigher  : 0 (Device is not SuperSpeedPlus capable or higher)
 ReservedMBZ             : 0x00
Data (HexDump)           : 04 00 00 00 10 00 00 00 03 00 00 00 00 00 00 00   ................

    ---------------------- Device Descriptor ----------------------
bLength                  : 0x12 (18 bytes)
bDescriptorType          : 0x01 (Device Descriptor)
bcdUSB                   : 0x110 (USB Version 1.1)
bDeviceClass             : 0x00 (defined by the interface descriptors)
bDeviceSubClass          : 0x00
bDeviceProtocol          : 0x00
bMaxPacketSize0          : 0x40 (64 bytes)
idVendor                 : 0x1209 (Unknown Vendor)
idProduct                : 0x0001
bcdDevice                : 0x0200
iManufacturer            : 0x01 (String Descriptor 1)
iProduct                 : 0x02 (String Descriptor 2)
iSerialNumber            : 0x03 (String Descriptor 3)
bNumConfigurations       : 0x01 (1 Configuration)
Data (HexDump)           : 12 01 10 01 00 00 00 40 09 12 01 00 00 02 01 02   .......@........
                           03 01                                             ..

    ------------------ Configuration Descriptor -------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x02 (Configuration Descriptor)
wTotalLength             : 0x0022 (34 bytes)
bNumInterfaces           : 0x01 (1 Interface)
bConfigurationValue      : 0x01 (Configuration 1)
iConfiguration           : 0x00 (No String Descriptor)
bmAttributes             : 0xA0
 D7: Reserved, set 1     : 0x01
 D6: Self Powered        : 0x00 (no)
 D5: Remote Wakeup       : 0x01 (yes)
 D4..0: Reserved, set 0  : 0x00
MaxPower                 : 0xFA (500 mA)
Data (HexDump)           : 09 02 22 00 01 01 00 A0 FA 09 04 00 00 01 03 00   ..".............
                           00 00 09 21 10 01 00 01 22 17 00 07 05 81 03 02   ...!....".......
                           00 0A                                             ..

        ---------------- Interface Descriptor -----------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x04 (Interface Descriptor)
bInterfaceNumber         : 0x00 (Interface 0)
bAlternateSetting        : 0x00
bNumEndpoints            : 0x01 (1 Endpoint)
bInterfaceClass          : 0x03 (HID - Human Interface Device)
bInterfaceSubClass       : 0x00 (None)
bInterfaceProtocol       : 0x00 (None)
iInterface               : 0x00 (No String Descriptor)
Data (HexDump)           : 09 04 00 00 01 03 00 00 00                        .........

        ------------------- HID Descriptor --------------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x21 (HID Descriptor)
bcdHID                   : 0x0110 (HID Version 1.10)
bCountryCode             : 0x00 (00 = not localized)
bNumDescriptors          : 0x01
Data (HexDump)           : 09 21 10 01 00 01 22 17 00                        .!...."..
Descriptor 1:
bDescriptorType          : 0x22 (Class=Report)
wDescriptorLength        : 0x0017 (23 bytes)
Error reading descriptor : ERROR_GEN_FAILURE (due to a obscure limitation of the Win32 USB API, see F1 Help)

        ----------------- Endpoint Descriptor -----------------
bLength                  : 0x07 (7 bytes)
bDescriptorType          : 0x05 (Endpoint Descriptor)
bEndpointAddress         : 0x81 (Direction=IN EndpointID=1)
bmAttributes             : 0x03 (TransferType=Interrupt)
wMaxPacketSize           : 0x0002 (2 bytes)
bInterval                : 0x0A (10 ms)
Data (HexDump)           : 07 05 81 03 02 00 0A                              .......

      -------------------- String Descriptors -------------------
String descriptors are not available  (because the device has problem code CM_PROB_FAILED_START)
Всеравно не вызывается. Не мигает.

### ASSISTANT
Спасибо за обновлённый лог — теперь **Device Descriptor корректен** (`bMaxPacketSize0 = 0x40` ✅), но проблема **осталась**:

> 💡 **`USBD_HID_Setup()` не вызывается**, LED не мигает, Windows не может прочитать Report Descriptor → **ошибка 10**.

Это означает: **USB-стек ST не передаёт управление вашему классу `USBD_HID` при обработке control-запросов**.

---

## 🔍 Причина почти наверняка: **неправильная инициализация USB-стека или регистрация класса**

Поскольку:
- Device Descriptor — OK
- Configuration Descriptor — OK (34 байта, структура верна)
- Класс зарегистрирован (вы говорите, что `USBD_RegisterClass(&USBD_HID)` есть)

…но **Setup не вызывается**, остаётся **одна из двух возможностей**:

---

### ✅ Вариант 1: Вы используете **устаревшую или модифицированную версию USB Device Library**, где `USBD_ClassTypeDef` имеет **другой порядок полей**

В разных версиях STM32Cube (особенно между **CubeMX v1.10 и v1.14+**) структура `USBD_ClassTypeDef` **менялась**!

#### Сравните:

**Старая версия (Cube v1.10 и ранее):**
```c
typedef struct _USBD_ClassTypeDef
{
  uint8_t  (*Init)(...);
  uint8_t  (*DeInit)(...);
  uint8_t  (*Setup)(...);
  uint8_t  (*EP0_TxSent)(...);
  uint8_t  (*EP0_RxReady)(...);
  uint8_t  (*DataIn)(...);
  uint8_t  (*DataOut)(...);
  uint8_t  (*SOF)(...);
  uint8_t  (*IsoINIncomplete)(...);
  uint8_t  (*IsoOUTIncomplete)(...);
  uint8_t  *(*GetCfgDesc)(uint16_t *length);
  uint8_t  *(*GetCfgDescOtherSpeed)(uint16_t *length);
  uint8_t  *(*GetDeviceQualifierDesc)(uint16_t *length);
} USBD_ClassTypeDef;
```

**Новая версия (Cube v1.14+):**
```c
typedef struct _USBD_ClassTypeDef
{
  uint8_t  (*Init)(...);
  uint8_t  (*DeInit)(...);
  uint8_t  (*Setup)(...);
  uint8_t  (*EP0_TxSent)(...);
  uint8_t  (*EP0_RxReady)(...);
  uint8_t  (*DataIn)(...);
  uint8_t  (*DataOut)(...);
  uint8_t  (*SOF)(...);
  uint8_t  (*IsoINIncomplete)(...);
  uint8_t  (*IsoOUTIncomplete)(...);
  uint8_t  *(*GetHSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
  uint8_t  *(*GetDeviceQualifierDescriptor)(uint16_t *length);
} USBD_ClassTypeDef;
```

👉 **Обратите внимание**: порядок `GetCfgDesc` изменился!

В вашем коде:
```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /*EP0_TxSent*/
  NULL, /*EP0_RxReady*/
  USBD_HID_DataIn,
  NULL, /*DataOut*/
  NULL, /*SOF */
  NULL,
  NULL,
  USBD_HID_GetCfgDesc,   // ← для HS?
  USBD_HID_GetCfgDesc,   // ← для FS?
  USBD_HID_GetCfgDesc,   // ← Other Speed
  USBD_HID_GetDeviceQualifierDesc,
};
```

Если вы компилируете **новую библиотеку** (Cube v1.14+), но инициализируете структуру **по старому шаблону**, то:

- `USBD_HID_GetCfgDesc` окажется в поле `GetHSConfigDescriptor`
- А `GetFSConfigDescriptor` будет **NULL**

→ При подключении Full-Speed устройства стек вызывает **`GetFSConfigDescriptor`**, получает **NULL** → **не может получить конфигурацию** → **не создаёт интерфейс** → **Setup не вызывается**.

---

## ✅ Решение: **Используйте правильный порядок полей**

### Шаг 1: Узнайте версию вашего `usbd_core.h`

Откройте `Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_core.h` и найдите:

```c
#define USBD_VERSION_MAJOR 1
#define USBD_VERSION_MINOR 14  // ← например
```

Если версия **≥ 1.14**, используйте **новый порядок**.

### Шаг 2: Перепишите `USBD_HID` с правильным порядком

Для **новой версии (v1.14+)**:

```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /* EP0_TxSent */
  NULL, /* EP0_RxReady */
  USBD_HID_DataIn,
  NULL, /* DataOut */
  NULL, /* SOF */
  NULL, /* IsoINIncomplete */
  NULL, /* IsoOUTIncomplete */
  USBD_HID_GetCfgDesc,   // GetHSConfigDescriptor (для High-Speed)
  USBD_HID_GetCfgDesc,   // GetFSConfigDescriptor (для Full-Speed) ← ЭТО ИСПОЛЬЗУЕТСЯ!
  USBD_HID_GetCfgDesc,   // GetOtherSpeedConfigDescriptor
  USBD_HID_GetDeviceQualifierDesc,
};
```

Но если у вас **только Full-Speed**, можно так:

```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, NULL, USBD_HID_DataIn, NULL, NULL, NULL, NULL,
  NULL,                    // HS — не используется
  USBD_HID_GetCfgDesc,     // FS — ОБЯЗАТЕЛЬНО!
  NULL,                    // Other Speed
  USBD_HID_GetDeviceQualifierDesc,
};
```

> 💡 **Главное**: поле **`GetFSConfigDescriptor`** должно указывать на функцию, возвращающую **ваш дескриптор**.

---

### Шаг 3: Убедитесь, что `USBD_HID_GetCfgDesc` возвращает правильную длину

```c
static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_CfgDesc); // = 34
  return USBD_HID_CfgDesc;
}
```

---

## 🧪 Как проверить быстро?

Замените инициализацию на **явно правильную для FS**:

```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, NULL, USBD_HID_DataIn, NULL, NULL, NULL, NULL,
  NULL,                    // HS
  USBD_HID_GetCfgDesc,     // FS ← главное!
  NULL,                    // Other
  USBD_HID_GetDeviceQualifierDesc,
};
```

Пересоберите → подключите → **LED должен мигнуть**.

---

Если это не поможет — пришлите:
1. Версию STM32Cube (из `usbd_core.h`)
2. Полный код инициализации USB в `main.c` (от `USBD_Init` до `USBD_Start`)

Но с высокой вероятностью — **проблема именно в порядке полей структуры `USBD_ClassTypeDef`**.

### USER
исправил - но все так же.
/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD core functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

/** @addtogroup STM32_USBD_DEVICE_LIBRARY
* @{
*/


/** @defgroup USBD_CORE
* @brief usbd core module
* @{
*/

/** @defgroup USBD_CORE_Private_TypesDefinitions
* @{
*/

/**
* @}
*/


/** @defgroup USBD_CORE_Private_Defines
* @{
*/

/**
* @}
*/


/** @defgroup USBD_CORE_Private_Macros
* @{
*/

/**
* @}
*/


/** @defgroup USBD_CORE_Private_FunctionPrototypes
* @{
*/

/**
* @}
*/

/** @defgroup USBD_CORE_Private_Variables
* @{
*/

/**
* @}
*/


/** @defgroup USBD_CORE_Private_Functions
* @{
*/

/**
* @brief  USBD_Init
*         Initializes the device stack and load the class driver
* @param  pdev: device instance
* @param  pdesc: Descriptor structure address
* @param  id: Low level core index
* @retval None
*/
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev,
                             USBD_DescriptorsTypeDef *pdesc, uint8_t id)
{
  /* Check whether the USB Host handle is valid */
  if (pdev == NULL)
  {
#if (USBD_DEBUG_LEVEL > 1U)
    USBD_ErrLog("Invalid Device handle");
#endif
    return USBD_FAIL;
  }

  /* Unlink previous class*/
  if (pdev->pClass != NULL)
  {
    pdev->pClass = NULL;
  }

  /* Assign USBD Descriptors */
  if (pdesc != NULL)
  {
    pdev->pDesc = pdesc;
  }

  /* Set Device initial State */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->id = id;
  /* Initialize low level driver */
  USBD_LL_Init(pdev);

  return USBD_OK;
}

/**
* @brief  USBD_DeInit
*         Re-Initialize th device library
* @param  pdev: device instance
* @retval status: status
*/
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev)
{
  /* Set Default State */
  pdev->dev_state = USBD_STATE_DEFAULT;

  /* Free Class Resources */
  pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);

  /* Stop the low level driver  */
  USBD_LL_Stop(pdev);

  /* Initialize low level driver */
  USBD_LL_DeInit(pdev);

  return USBD_OK;
}

/**
  * @brief  USBD_RegisterClass
  *         Link class driver to Device Core.
  * @param  pDevice : Device Handle
  * @param  pclass: Class handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass)
{
  USBD_StatusTypeDef status = USBD_OK;
  if (pclass != NULL)
  {
    /* link the class to the USB Device handle */
    pdev->pClass = pclass;
    status = USBD_OK;
  }
  else
  {
#if (USBD_DEBUG_LEVEL > 1U)
    USBD_ErrLog("Invalid Class handle");
#endif
    status = USBD_FAIL;
  }

  return status;
}

/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_Start(USBD_HandleTypeDef *pdev)
{
  /* Start the low level driver  */
  USBD_LL_Start(pdev);

  return USBD_OK;
}

/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_Stop(USBD_HandleTypeDef *pdev)
{
  /* Free Class Resources */
  pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);

  /* Stop the low level driver */
  USBD_LL_Stop(pdev);

  return USBD_OK;
}

/**
* @brief  USBD_RunTestMode
*         Launch test mode process
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef  USBD_RunTestMode(USBD_HandleTypeDef  *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
}

/**
* @brief  USBD_SetClassConfig
*        Configure device and start the interface
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status
*/

USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx)
{
  USBD_StatusTypeDef ret = USBD_FAIL;

  if (pdev->pClass != NULL)
  {
    /* Set configuration  and Start the Class*/
    if (pdev->pClass->Init(pdev, cfgidx) == 0U)
    {
      ret = USBD_OK;
    }
  }

  return ret;
}

/**
* @brief  USBD_ClrClassConfig
*         Clear current configuration
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status: USBD_StatusTypeDef
*/
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef  *pdev, uint8_t cfgidx)
{
  /* Clear configuration  and De-initialize the Class process*/
  pdev->pClass->DeInit(pdev, cfgidx);

  return USBD_OK;
}


/**
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup)
{
  USBD_ParseSetupRequest(&pdev->request, psetup);

  pdev->ep0_state = USBD_EP0_SETUP;

  pdev->ep0_data_len = pdev->request.wLength;

  switch (pdev->request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      USBD_StdDevReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      USBD_StdItfReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      USBD_StdEPReq(pdev, &pdev->request);
      break;

    default:
      USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
      break;
  }

  return USBD_OK;
}

/**
* @brief  USBD_DataOutStage
*         Handle data OUT stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev,
                                        uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;

  if (epnum == 0U)
  {
    pep = &pdev->ep_out[0];

    if (pdev->ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        USBD_CtlContinueRx(pdev, pdata,
                           (uint16_t)MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        if ((pdev->pClass->EP0_RxReady != NULL) &&
            (pdev->dev_state == USBD_STATE_CONFIGURED))
        {
          pdev->pClass->EP0_RxReady(pdev);
        }
        USBD_CtlSendStatus(pdev);
      }
    }
    else
    {
      if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
      {
        /*
         * STATUS PHASE completed, update ep0_state to idle
         */
        pdev->ep0_state = USBD_EP0_IDLE;
        USBD_LL_StallEP(pdev, 0U);
      }
    }
  }
  else if ((pdev->pClass->DataOut != NULL) &&
           (pdev->dev_state == USBD_STATE_CONFIGURED))
  {
    pdev->pClass->DataOut(pdev, epnum);
  }
  else
  {
    /* should never be in this condition */
    return USBD_FAIL;
  }

  return USBD_OK;
}

/**
* @brief  USBD_DataInStage
*         Handle data in stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev,
                                       uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;

  if (epnum == 0U)
  {
    pep = &pdev->ep_in[0];

    if (pdev->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        USBD_CtlContinueSendData(pdev, pdata, (uint16_t)pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
        USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->total_length % pep->maxpacket == 0U) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < pdev->ep0_data_len))
        {
          USBD_CtlContinueSendData(pdev, NULL, 0U);
          pdev->ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
        }
        else
        {
          if ((pdev->pClass->EP0_TxSent != NULL) &&
              (pdev->dev_state == USBD_STATE_CONFIGURED))
          {
            pdev->pClass->EP0_TxSent(pdev);
          }
          USBD_LL_StallEP(pdev, 0x80U);
          USBD_CtlReceiveStatus(pdev);
        }
      }
    }
    else
    {
      if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
          (pdev->ep0_state == USBD_EP0_IDLE))
      {
        USBD_LL_StallEP(pdev, 0x80U);
      }
    }

    if (pdev->dev_test_mode == 1U)
    {
      USBD_RunTestMode(pdev);
      pdev->dev_test_mode = 0U;
    }
  }
  else if ((pdev->pClass->DataIn != NULL) &&
           (pdev->dev_state == USBD_STATE_CONFIGURED))
  {
    pdev->pClass->DataIn(pdev, epnum);
  }
  else
  {
    /* should never be in this condition */
    return USBD_FAIL;
  }

  return USBD_OK;
}

/**
* @brief  USBD_LL_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev)
{
  /* Open EP0 OUT */
  USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;

  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;

  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;

  if (pdev->pClassData)
  {
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  }

  return USBD_OK;
}

/**
* @brief  USBD_LL_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev,
                                    USBD_SpeedTypeDef speed)
{
  pdev->dev_speed = speed;

  return USBD_OK;
}

/**
* @brief  USBD_Suspend
*         Handle Suspend event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev)
{
  pdev->dev_old_state =  pdev->dev_state;
  pdev->dev_state  = USBD_STATE_SUSPENDED;

  return USBD_OK;
}

/**
* @brief  USBD_Resume
*         Handle Resume event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev)
{
  if (pdev->dev_state == USBD_STATE_SUSPENDED)
  {
    pdev->dev_state = pdev->dev_old_state;
  }

  return USBD_OK;
}

/**
* @brief  USBD_SOF
*         Handle SOF event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev)
{
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass->SOF != NULL)
    {
      pdev->pClass->SOF(pdev);
    }
  }

  return USBD_OK;
}

/**
* @brief  USBD_IsoINIncomplete
*         Handle iso in incomplete event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef *pdev,
                                           uint8_t epnum)
{
  /* Prevent unused arguments compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);

  return USBD_OK;
}

/**
* @brief  USBD_IsoOUTIncomplete
*         Handle iso out incomplete event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef *pdev,
                                            uint8_t epnum)
{
  /* Prevent unused arguments compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);

  return USBD_OK;
}

/**
* @brief  USBD_DevConnected
*         Handle device connection event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
}

/**
* @brief  USBD_DevDisconnected
*         Handle device disconnection event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev)
{
  /* Free Class Resources */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);

  return USBD_OK;
}
/**
* @}
*/


/**
* @}
*/


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @brief   This file provides the standard USB requests following chapter 9.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"


/** @addtogroup STM32_USBD_STATE_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_REQ
  * @brief USB standard requests module
  * @{
  */

/** @defgroup USBD_REQ_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_FunctionPrototypes
  * @{
  */
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req);

static void USBD_SetAddress(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req);

static void USBD_SetConfig(USBD_HandleTypeDef *pdev,
                           USBD_SetupReqTypedef *req);

static void USBD_GetConfig(USBD_HandleTypeDef *pdev,
                           USBD_SetupReqTypedef *req);

static void USBD_GetStatus(USBD_HandleTypeDef *pdev,
                           USBD_SetupReqTypedef *req);

static void USBD_SetFeature(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req);

static void USBD_ClrFeature(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req);

static uint8_t USBD_GetLen(uint8_t *buf);

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Functions
  * @{
  */


/**
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdDevReq(USBD_HandleTypeDef *pdev,
                                   USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      pdev->pClass->Setup(pdev, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:
          USBD_GetDescriptor(pdev, req);
          break;

        case USB_REQ_SET_ADDRESS:
          USBD_SetAddress(pdev, req);
          break;

        case USB_REQ_SET_CONFIGURATION:
          USBD_SetConfig(pdev, req);
          break;

        case USB_REQ_GET_CONFIGURATION:
          USBD_GetConfig(pdev, req);
          break;

        case USB_REQ_GET_STATUS:
          USBD_GetStatus(pdev, req);
          break;

        case USB_REQ_SET_FEATURE:
          USBD_SetFeature(pdev, req);
          break;

        case USB_REQ_CLEAR_FEATURE:
          USBD_ClrFeature(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return ret;
}

/**
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdItfReq(USBD_HandleTypeDef *pdev,
                                   USBD_SetupReqTypedef  *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
    case USB_REQ_TYPE_STANDARD:
      switch (pdev->dev_state)
      {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
        case USBD_STATE_CONFIGURED:

          if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)
          {
            ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);

            if ((req->wLength == 0U) && (ret == USBD_OK))
            {
              USBD_CtlSendStatus(pdev);
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return USBD_OK;
}

/**
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef  USBD_StdEPReq(USBD_HandleTypeDef *pdev,
                                  USBD_SetupReqTypedef  *req)
{
  USBD_EndpointTypeDef *pep;
  uint8_t   ep_addr;
  USBD_StatusTypeDef ret = USBD_OK;
  ep_addr  = LOBYTE(req->wIndex);

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      pdev->pClass->Setup(pdev, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      /* Check if it is a class request */
      if ((req->bmRequest & 0x60U) == 0x20U)
      {
        ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);

        return ret;
      }

      switch (req->bRequest)
      {
        case USB_REQ_SET_FEATURE:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_LL_StallEP(pdev, ep_addr);
                USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr != 0x00U) &&
                    (ep_addr != 0x80U) && (req->wLength == 0x00U))
                {
                  USBD_LL_StallEP(pdev, ep_addr);
                }
              }
              USBD_CtlSendStatus(pdev);

              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:

          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_LL_StallEP(pdev, ep_addr);
                USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr & 0x7FU) != 0x00U)
                {
                  USBD_LL_ClearStallEP(pdev, ep_addr);
                }
                USBD_CtlSendStatus(pdev);
              }
              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        case USB_REQ_GET_STATUS:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_CtlError(pdev, req);
                break;
              }
              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];

              pep->status = 0x0000U;

              USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
              break;

            case USBD_STATE_CONFIGURED:
              if ((ep_addr & 0x80U) == 0x80U)
              {
                if (pdev->ep_in[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }
              else
              {
                if (pdev->ep_out[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }

              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];

              if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
              {
                pep->status = 0x0000U;
              }
              else if (USBD_LL_IsStallEP(pdev, ep_addr))
              {
                pep->status = 0x0001U;
              }
              else
              {
                pep->status = 0x0000U;
              }

              USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return ret;
}


/**
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev,
                               USBD_SetupReqTypedef *req)
{
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint8_t err = 0U;

  switch (req->wValue >> 8)
  {
#if (USBD_LPM_ENABLED == 1U)
    case USB_DESC_TYPE_BOS:
      if (pdev->pDesc->GetBOSDescriptor != NULL)
      {
        pbuf = pdev->pDesc->GetBOSDescriptor(pdev->dev_speed, &len);
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;
#endif
    case USB_DESC_TYPE_DEVICE:
      pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
      break;

    case USB_DESC_TYPE_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetHSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      else
      {
        pbuf = pdev->pClass->GetFSConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      break;

    case USB_DESC_TYPE_STRING:
      switch ((uint8_t)(req->wValue))
      {
        case USBD_IDX_LANGID_STR:
          if (pdev->pDesc->GetLangIDStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_MFC_STR:
          if (pdev->pDesc->GetManufacturerStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_PRODUCT_STR:
          if (pdev->pDesc->GetProductStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_SERIAL_STR:
          if (pdev->pDesc->GetSerialStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_CONFIG_STR:
          if (pdev->pDesc->GetConfigurationStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_INTERFACE_STR:
          if (pdev->pDesc->GetInterfaceStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        default:
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
          if (pdev->pClass->GetUsrStrDescriptor != NULL)
          {
            pbuf = pdev->pClass->GetUsrStrDescriptor(pdev, (req->wValue), &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;
#else
          USBD_CtlError(pdev, req);
          err++;
#endif
      }
      break;

    case USB_DESC_TYPE_DEVICE_QUALIFIER:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetDeviceQualifierDescriptor(&len);
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;

    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        pbuf = pdev->pClass->GetOtherSpeedConfigDescriptor(&len);
        pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      err++;
      break;
  }

  if (err != 0U)
  {
    return;
  }
  else
  {
    if ((len != 0U) && (req->wLength != 0U))
    {
      len = MIN(len, req->wLength);
      (void)USBD_CtlSendData(pdev, pbuf, len);
    }

    if (req->wLength == 0U)
    {
      (void)USBD_CtlSendStatus(pdev);
    }
  }
}

/**
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetAddress(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req)
{
  uint8_t  dev_addr;

  if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7FU;

    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlError(pdev, req);
    }
    else
    {
      pdev->dev_address = dev_addr;
      USBD_LL_SetUSBAddress(pdev, dev_addr);
      USBD_CtlSendStatus(pdev);

      if (dev_addr != 0U)
      {
        pdev->dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        pdev->dev_state = USBD_STATE_DEFAULT;
      }
    }
  }
  else
  {
    USBD_CtlError(pdev, req);
  }
}

/**
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  static uint8_t cfgidx;

  cfgidx = (uint8_t)(req->wValue);

  if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
  {
    USBD_CtlError(pdev, req);
  }
  else
  {
    switch (pdev->dev_state)
    {
      case USBD_STATE_ADDRESSED:
        if (cfgidx)
        {
          pdev->dev_config = cfgidx;
          pdev->dev_state = USBD_STATE_CONFIGURED;
          if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
          {
            USBD_CtlError(pdev, req);
            return;
          }
          USBD_CtlSendStatus(pdev);
        }
        else
        {
          USBD_CtlSendStatus(pdev);
        }
        break;

      case USBD_STATE_CONFIGURED:
        if (cfgidx == 0U)
        {
          pdev->dev_state = USBD_STATE_ADDRESSED;
          pdev->dev_config = cfgidx;
          USBD_ClrClassConfig(pdev, cfgidx);
          USBD_CtlSendStatus(pdev);
        }
        else if (cfgidx != pdev->dev_config)
        {
          /* Clear old configuration */
          USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);

          /* set new configuration */
          pdev->dev_config = cfgidx;
          if (USBD_SetClassConfig(pdev, cfgidx) == USBD_FAIL)
          {
            USBD_CtlError(pdev, req);
            return;
          }
          USBD_CtlSendStatus(pdev);
        }
        else
        {
          USBD_CtlSendStatus(pdev);
        }
        break;

      default:
        USBD_CtlError(pdev, req);
        USBD_ClrClassConfig(pdev, cfgidx);
        break;
    }
  }
}

/**
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if (req->wLength != 1U)
  {
    USBD_CtlError(pdev, req);
  }
  else
  {
    switch (pdev->dev_state)
    {
      case USBD_STATE_DEFAULT:
      case USBD_STATE_ADDRESSED:
        pdev->dev_default_config = 0U;
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_default_config, 1U);
        break;

      case USBD_STATE_CONFIGURED:
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config, 1U);
        break;

      default:
        USBD_CtlError(pdev, req);
        break;
    }
  }
}

/**
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetStatus(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wLength != 0x2U)
      {
        USBD_CtlError(pdev, req);
        break;
      }

#if (USBD_SELF_POWERED == 1U)
      pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
      pdev->dev_config_status = 0U;
#endif

      if (pdev->dev_remote_wakeup)
      {
        pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
      }

      USBD_CtlSendData(pdev, (uint8_t *)(void *)&pdev->dev_config_status, 2U);
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }
}


/**
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetFeature(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req)
{
  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    pdev->dev_remote_wakeup = 1U;
    USBD_CtlSendStatus(pdev);
  }
}


/**
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_ClrFeature(USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req)
{
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
      {
        pdev->dev_remote_wakeup = 0U;
        USBD_CtlSendStatus(pdev);
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }
}

/**
* @brief  USBD_ParseSetupRequest
*         Copy buffer into setup structure
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata)
{
  req->bmRequest = *(uint8_t *)(pdata);
  req->bRequest = *(uint8_t *)(pdata + 1U);
  req->wValue = SWAPBYTE(pdata + 2U);
  req->wIndex = SWAPBYTE(pdata + 4U);
  req->wLength = SWAPBYTE(pdata + 6U);

}

/**
* @brief  USBD_CtlError
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_CtlError(USBD_HandleTypeDef *pdev,
                   USBD_SetupReqTypedef *req)
{
  USBD_LL_StallEP(pdev, 0x80U);
  USBD_LL_StallEP(pdev, 0U);
}


/**
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;

  if (desc != NULL)
  {
    *len = (uint16_t)USBD_GetLen(desc) * 2U + 2U;
    unicode[idx++] = *(uint8_t *)(void *)len;
    unicode[idx++] = USB_DESC_TYPE_STRING;

    while (*desc != '\0')
    {
      unicode[idx++] = *desc++;
      unicode[idx++] =  0U;
    }
  }
}

/**
  * @brief  USBD_GetLen
  *         return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
static uint8_t USBD_GetLen(uint8_t *buf)
{
  uint8_t  len = 0U;

  while (*buf != '\0')
  {
    len++;
    buf++;
  }

  return len;
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
В функции USBD_StdDevReq несколько раз при подключении вызываются USBD_GetDescriptor USBD_SetAddress и USBD_SetConfig.


### ASSISTANT
Спасибо за предоставленные файлы `usbd_core.c` и `usbd_req.c` — это **ключ к решению**.

Вы абсолютно правы: при подключении вызываются:
- `USBD_GetDescriptor` → запрашивает **Device**, **Configuration**
- `USBD_SetAddress`
- `USBD_SetConfig`

Но **ваша функция `USBD_HID_Setup()` не вызывается**, потому что **запросы дескрипторов обрабатываются в `USBD_StdDevReq` и `USBD_StdItfReq`**, и **только если они адресованы интерфейсу или endpoint'у**, стек вызывает `pdev->pClass->Setup(...)`.

---

## 🔍 Где именно должен вызываться ваш `USBD_HID_Setup()`?

### 1. **Запрос Report Descriptor** — это **CLASS-запрос к интерфейсу**
Формат запроса:
```
bmRequest = 0x81 (IN, Class, Interface)
bRequest  = 0x01 (GET_REPORT)
wValue    = 0x2200 (Report Descriptor, index 0)
wIndex    = 0x0000 (Interface 0)
```

Такой запрос обрабатывается в:
```c
// usbd_req.c → USBD_StdItfReq()
case USB_REQ_TYPE_CLASS:
case USB_REQ_TYPE_VENDOR:
case USB_REQ_TYPE_STANDARD:
  if (...)
  {
    ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);
  }
```

✅ То есть **должен вызываться ваш `USBD_HID_Setup()`**.

Но если он **не вызывается**, значит:
> **`USBD_StdItfReq()` вообще не вызывается** для этого запроса.

Почему?

---

## 🚨 Причина: **Запрос приходит ДО того, как устройство перешло в состояние `USBD_STATE_CONFIGURED`**

Вот что происходит по шагам:

| Шаг | Действие | Состояние `pdev->dev_state` |
|-----|--------|------------------------------|
| 1 | GET DEVICE DESCRIPTOR | `USBD_STATE_DEFAULT` |
| 2 | SET ADDRESS | `USBD_STATE_ADDRESSED` |
| 3 | **GET CONFIGURATION DESCRIPTOR** | `USBD_STATE_ADDRESSED` |
| 4 | SET CONFIGURATION → вызывает `USBD_SetClassConfig()` → `USBD_HID_Init()` | `USBD_STATE_CONFIGURED` |
| 5 | **GET HID REPORT DESCRIPTOR** | `USBD_STATE_CONFIGURED` |

⚠️ **Но!** Windows (и многие хосты) **запрашивают Report Descriptor сразу после получения Configuration Descriptor**, **до SET CONFIGURATION**!

То есть:
- Устройство ещё в состоянии `USBD_STATE_ADDRESSED`
- Приходит запрос: `GET_DESCRIPTOR (HID_REPORT_DESC)` к интерфейсу 0
- Стек вызывает `USBD_StdItfReq()`
- В `USBD_StdItfReq()` есть проверка:

```c
switch (pdev->dev_state)
{
  case USBD_STATE_DEFAULT:
  case USBD_STATE_ADDRESSED:
  case USBD_STATE_CONFIGURED:   // ← разрешено!
    if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)
    {
      ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req); // ← должен вызваться
```

✅ Так что **даже в состоянии `ADDRESSED` Setup должен вызываться**.

---

## 🔥 Но есть **ещё одна причина**, почему `Setup` не вызывается:

### В `USBD_StdItfReq()` вызов `pClass->Setup()` происходит **только если запрос — CLASS, VENDOR или STANDARD**.

А **запрос Report Descriptor — это `USB_REQ_TYPE_STANDARD`? Нет!**

> ❗ **GET_DESCRIPTOR (Report) — это `USB_REQ_TYPE_STANDARD`**, но **внутри `wValue = 0x2200`**, и он обрабатывается в `USBD_GetDescriptor()` как часть **Configuration Descriptor tree**.

Но **Windows не запрашивает Report Descriptor через `GET_DESCRIPTOR` с `wValue=0x2200` на этапе enumeration?**

Да, **запрашивает** — и это **стандартный запрос**, который **обрабатывается в `USBD_GetDescriptor()`**, а **не в `USBD_StdItfReq()`**!

И вот **главное**:

> В `USBD_GetDescriptor()` **нет вызова `pClass->Setup()` для `HID_REPORT_DESC`**!

Вместо этого он **пытается получить дескриптор из `pClass->GetFSConfigDescriptor()`**, но **Report Descriptor — это отдельный дескриптор**, и его **нужно обрабатывать в `Setup`**.

Но стек **не знает**, что ему нужно вызвать `Setup` для `wValue = 0x2200`.

---

## ✅ Правильный путь: **Report Descriptor должен возвращаться через `Setup`, а не через `GetDescriptor`**

Именно поэтому в `usbd_hid.c` у вас есть обработка `GET_DESCRIPTOR` внутри `USBD_HID_Setup()`.

Но чтобы это сработало, **запрос должен быть передан в `Setup`**.

А для этого он **должен быть CLASS-запросом**, но **GET_DESCRIPTOR — это STANDARD-запрос**.

👉 **Противоречие? Нет.**

Согласно **USB HID Spec 1.11, Section 7.1.1**:
> The **Get_Descriptor** request for a **Report descriptor** is a **standard device request**, but directed to an **interface** (`wIndex = interface number`).

И стек ST **должен передавать такие запросы в `USBD_StdItfReq()`**, а не в `USBD_StdDevReq()`.

И действительно, в `usbd_core.c`:

```c
switch (pdev->request.bmRequest & 0x1FU)
{
  case USB_REQ_RECIPIENT_DEVICE:    → USBD_StdDevReq
  case USB_REQ_RECIPIENT_INTERFACE: → USBD_StdItfReq  ← сюда попадает запрос с wIndex = 0!
  case USB_REQ_RECIPIENT_ENDPOINT:  → USBD_StdEPReq
}
```

✅ Значит, запрос **должен попасть в `USBD_StdItfReq()`**, и оттуда — в ваш `Setup`.

---

## 🧪 Почему не попадает?

Проверьте **реальное значение `bmRequest`** в запросе.

В `USBD_ParseSetupRequest`:
```c
req->bmRequest = *(uint8_t *)(pdata);
```

Для запроса Report Descriptor:
- `bmRequest = 0x81` → `1000 0001`
  - Bit 7: `1` = IN
  - Bits 6..5: `00` = Standard
  - Bits 4..0: `00001` = Interface

→ `USB_REQ_RECIPIENT_INTERFACE` ✅

→ Должен вызвать `USBD_StdItfReq()` → ваш `Setup`.

---

## 💡 Последняя и самая вероятная причина:

### Ваш `USBD_HID_Setup()` **вызывается**, но **падает или возвращает ошибку до мигания LED**

Например:
- Вы добавили `HAL_GPIO_TogglePin()` **после** строки:
  ```c
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  ```
- Но в состоянии `ADDRESSED` → `pClassData == NULL` → **разыменование NULL → HardFault**

→ **Устройство зависает или отвечает STALL** → Windows не получает дескриптор → ошибка 10.

---

## ✅ Финальное решение: **Добавьте мигание В САМОМ НАЧАЛЕ функции**

```c
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // 🔴 МИГАЕМ СРАЗУ, ДО ЛЮБОЙ ЛОГИКИ
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // или ваш LED

  // ... остальной код
}
```

Если **мигает** — значит, функция вызывается, и проблема в логике.
Если **не мигает** — проблема в стеке или регистрации.

---

## Но если и это не поможет — сделайте следующее:

### Замените `USBD_HID_Setup` на **минимальную заглушку**:

```c
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  
  // Просто отправляем STALL на всё — но мигнём
  USBD_CtlError(pdev, req);
  return USBD_FAIL;
}
```

Если **мигает** — отлично, функция вызывается.
Если **не мигает** — значит, **класс не зарегистрирован** или **структура USBD_ClassTypeDef повреждена**.

---

## 📌 Вывод

Скорее всего, у вас **HardFault при разыменовании `pClassData`**, потому что вы обращаетесь к нему **до `USBD_HID_Init()`**.

👉 **Исправьте `USBD_HID_Setup()` так, чтобы она не использовала `pClassData` для запросов `GET_DESCRIPTOR`** — как я предлагал ранее.

И **поместите мигание в самое начало**.

Попробуйте это — и сообщите, мигает ли LED.

### USER
bmRequest перечисляю состояние в каждом из 10ти запросов: 0x80 0x80 0x00 0x80 0x80 0x80 0x80 0x80 0x80 0x80 0x80 0x80 0x80.

### ASSISTANT
Спасибо за уточнение! Вы перехватили **`bmRequest` из 13 SETUP-пакетов** при подключении устройства, и все они (кроме одного) — **`0x80`**, а один — **`0x00`**.

Разберём, что это означает:

---

### 🔍 Декодирование `bmRequest`

Формат `bmRequest` (согласно USB 2.0 spec):
```
Bit 7:    Direction (1 = IN, 0 = OUT)
Bits 6–5: Type        (00 = Standard, 01 = Class, 10 = Vendor, 11 = Reserved)
Bits 4–0: Recipient   (0 = Device, 1 = Interface, 2 = Endpoint, 3 = Other)
```

#### `0x80` → `1000 0000`:
- **Direction**: IN  
- **Type**: Standard (`00`)  
- **Recipient**: Device (`00000`)  
→ Это **стандартный запрос к устройству**, например:
  - `GET_DESCRIPTOR(Device)`
  - `GET_DESCRIPTOR(Configuration)`
  - `GET_STATUS(Device)`

#### `0x00` → `0000 0000`:
- **Direction**: OUT  
- **Type**: Standard  
- **Recipient**: Device  
→ Это, скорее всего, **`SET_ADDRESS`** или **`SET_CONFIGURATION`**

---

### ❗ Ключевой вывод:

> **Ни один из запросов не адресован интерфейсу (`Recipient ≠ Interface`)**  
> → **`USBD_StdItfReq()` не вызывается ни разу**  
> → **Ваш `USBD_HID_Setup()` не вызывается**, потому что запросы идут **только к устройству**, а не к интерфейсу.

Но **запрос Report Descriptor должен быть с `Recipient = Interface`**, то есть `bmRequest = 0x81` (`1000 0001`).

**Его нет в вашем списке.**

---

## 🤔 Почему Windows не запрашивает Report Descriptor?

Потому что **она не считает ваше устройство HID-устройством на этапе enumeration**.

А почему? Потому что **в Configuration Descriptor** вы заявляете:

```ini
bInterfaceClass    = 0x03  (HID)
bInterfaceSubClass = 0x00
bInterfaceProtocol = 0x00
```

→ Это **generic HID**, и Windows **должна** запросить Report Descriptor.

Но **если длина Configuration Descriptor указана неверно**, или **структура повреждена**, Windows **не парсит интерфейс** и **не отправляет запросы к нему**.

---

## ✅ Проверьте **реальное содержимое Configuration Descriptor**

В логе Windows:
```ini
wTotalLength : 0x0022 (34 bytes)
```

Но **если ваш массив `USBD_HID_CfgDesc` содержит мусор или неправильные байты**, Windows остановит парсинг.

### 🔧 Сделайте следующее:

1. **Выведите в отладчике или UART всё содержимое `USBD_HID_CfgDesc`** как hex-дамп.
2. Сравните с **ожидаемым** (34 байта):

```hex
09 02 22 00 01 01 00 A0 FA   // Configuration
09 04 00 00 01 03 00 00 00   // Interface
09 21 10 01 00 01 22 17 00   // HID
07 05 81 03 02 00 0A         // Endpoint
```

Если есть **хотя бы один байт не совпадает** — Windows не распознает интерфейс.

---

## 🚨 Особое подозрение: **wTotalLength в дескрипторе — 0x0022, но в памяти — 0x2200?**

Потому что вы используете:
```c
/*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ)
```

А если `USB_HID_CONFIG_DESC_SIZ = 34 = 0x22`, то:
- `LOBYTE = 0x22`
- `HIBYTE = 0x00`

→ В дескрипторе: `22 00` → **маленький endian** → **wTotalLength = 0x0022** ✅

Но **если вы перепутали порядок** и написали `HIBYTE, LOBYTE` — будет `00 22` → `wTotalLength = 0x2200 = 8704 байт` → Windows ждёт ещё 8670 байт → таймаут → отказ.

👉 **Проверьте порядок байт в Configuration Descriptor!**

В вашем коде:
```c
/*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ)
```
→ Это **правильно** для little-endian хоста (Windows).

Но **убедитесь, что макрос `USB_HID_CONFIG_DESC_SIZ` действительно равен 34**.

---

## ✅ Резюме

Причина, по которой `USBD_HID_Setup()` не вызывается:

> **Windows не отправляет запросы к интерфейсу, потому что не распознаёт его как валидный HID-интерфейс.**

Скорее всего, из-за:
- ❌ **Неправильного порядка байт в `wTotalLength`**
- ❌ **Фактической длины дескриптора ≠ 34**
- ❌ **Повреждённого содержимого дескриптора**

---

## 🛠️ Что делать прямо сейчас

1. **Добавьте отладочный вывод**:
   ```c
   for (int i = 0; i < 34; i++) {
     printf("%02X ", USBD_HID_CfgDesc[i]);
   }
   ```
2. Убедитесь, что вывод совпадает с ожидаемым (см. выше).
3. Если всё верно — проверьте, **не включены ли другие интерфейсы случайно** (через `#define`).

Если вы пришлёте **реальный hex-дамп вашего `USBD_HID_CfgDesc`**, я скажу точно, в чём проблема.

### USER
да, длина и содержимое USBD_HID_CfgDesc cjdgflftn/

### ASSISTANT
Отлично — раз **длина и содержимое `USBD_HID_CfgDesc` совпадают** с ожидаемыми (34 байта, корректная структура), и **`bmRequest` никогда не принимает значение `0x81`** (т.е. запросы к интерфейсу не идут), значит:

> 💡 **Windows не считает ваше устройство составным HID-устройством на этапе enumeration**, потому что **оно не проходит валидацию как HID** — **несмотря на правильный дескриптор**.

Это возможно **только в одном случае**:

---

## 🚨 Проблема: **Device Descriptor заявляет `bDeviceClass = 0x00`, но Configuration Descriptor не содержит **валидного HID-интерфейса** в глазах Windows**

Но у вас интерфейс выглядит правильно:
```ini
bInterfaceClass    = 0x03  (HID)
bInterfaceSubClass = 0x00
bInterfaceProtocol = 0x00
```

Однако Windows **требует**, чтобы **HID Descriptor** был **непосредственно встроен в Configuration Descriptor** — и он у вас есть:
```ini
09 21 10 01 00 01 22 17 00
```
→ `bLength=9`, `bDescriptorType=0x21`, `bNumDescriptors=1`, `ReportDesc len=0x0017` — ✅

Так почему же Windows **не запрашивает Report Descriptor**?

---

## 🔍 Единственное оставшееся объяснение:

### Ваше устройство **не проходит SET_CONFIGURATION**, потому что **класс не инициализируется**, и Windows **не переходит к работе с интерфейсом**.

Но почему?

Потому что **в `USBD_SetConfig()` вызывается `USBD_SetClassConfig()`**, который вызывает:
```c
pdev->pClass->Init(pdev, cfgidx);
```

А в вашей функции `USBD_HID_Init()` вы **открываете только часть endpoint'ов**:

```c
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, ...);   // ← но у вас включён только CONSUMER!
  USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, ...);

  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U; // ← 0x81 & 0xF = 1
  pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 1U; // ← тоже 0x81 → коллизия!
}
```

👉 **Если `HID_KEYBOARD_EP == HID_CONSUMER_EP`**, то вы **дважды открываете один и тот же endpoint**, и **записываете в один и тот же флаг `is_used`**.

Но **если у вас включён только `CONSUMER_CONTROL`**, то:
- `HID_INTERFACE_KEYBOARD = 0`
- `HID_INTERFACE_MOUSE = 0`
- `HID_INTERFACE_CONSUMER = 1`
- → `HID_KEYBOARD_EP = 0x80 + 0 = 0x80` ❌ (но 0x80 — это EP0 IN!)
- → `HID_CONSUMER_EP = 0x80 + 0 + 1 = 0x81` ✅

Но в коде `USBD_HID_Init()` вы **всегда** открываете `HID_KEYBOARD_EP`, даже если он не включён!

```c
USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, ...); // ← всегда!
```

→ Если `HID_KEYBOARD_EP = 0x80`, вы пытаетесь открыть **control endpoint как interrupt** — **это ошибка**.

→ Или если `HID_KEYBOARD_EP = 0x81` (когда keyboard выключен), вы открываете **endpoint, который не заявлен в дескрипторе**.

---

## ✅ Решение: **Исправьте `USBD_HID_Init()` и `DeInit()` — открывайте ТОЛЬКО включённые endpoint'ы**

Замените:

```c
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
#if HID_INTERFACE_KEYBOARD
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EP_SIZE);
  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_MOUSE
  USBD_LL_OpenEP(pdev, HID_MOUSE_EP, USBD_EP_TYPE_INTR, HID_MOUSE_EP_SIZE);
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_CONSUMER
  USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, USBD_EP_TYPE_INTR, HID_CONSUMER_EP_SIZE);
  pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 1U;
#endif

// ... остальные

  pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));
  if (pdev->pClassData == NULL)
    return USBD_FAIL;

  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;
  return USBD_OK;
}
```

И аналогично для `DeInit()`.

---

## 🧪 Почему это критично?

Если вы открываете **endpoint, который не описан в Configuration Descriptor**, или **пытаетесь использовать EP0 как IN endpoint для данных**, то:
- USB-стек может вернуть ошибку при `SET_CONFIGURATION`
- Windows получает **STALL или NAK** на `SET_CONFIGURATION`
- Устройство **остаётся в состоянии `ADDRESSED`**
- Windows **не считает устройство сконфигурированным**
- → **Не отправляет запросы к интерфейсу**
- → **`Setup` не вызывается**

---

## ✅ Что делать

1. **Исправьте `USBD_HID_Init()`**, чтобы он открывал **только реально используемые endpoint'ы**.
2. Убедитесь, что **адреса endpoint'ов не пересекаются** и **совпадают с теми, что в дескрипторе**.
3. Пересоберите и проверьте.

После этого Windows должна:
- Успешно выполнить `SET_CONFIGURATION`
- Перейти в состояние `CONFIGURED`
- Запросить Report Descriptor через `bmRequest = 0x81`
- Вызвать ваш `USBD_HID_Setup()`

Если нужно — я помогу переписать `Init`/`DeInit` под вашу конфигурацию (только consumer).

### USER
Напиши полностью функции USBD_HID_Init USBD_HID_DeInit

### ASSISTANT
Вот **полные и корректные** реализации функций `USBD_HID_Init` и `USBD_HID_DeInit`, которые **открывают/закрывают только те endpoint'ы, которые действительно включены** через ваши `#define` (например, только `CONSUMER_CONTROL`).

Эти функции учитывают:
- Условную компиляцию (`#if HID_INTERFACE_XXX`)
- Корректные адреса endpoint'ов
- Безопасную работу с `pClassData`

---

### ✅ `USBD_HID_Init`

```c
/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  // Открываем endpoint'ы только для включённых интерфейсов
#if HID_INTERFACE_KEYBOARD
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EP_SIZE);
  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_MOUSE
  USBD_LL_OpenEP(pdev, HID_MOUSE_EP, USBD_EP_TYPE_INTR, HID_MOUSE_EP_SIZE);
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_CONSUMER
  USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, USBD_EP_TYPE_INTR, HID_CONSUMER_EP_SIZE);
  pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_REMOTE
  USBD_LL_OpenEP(pdev, HID_REMOTE_EP, USBD_EP_TYPE_INTR, HID_REMOTE_EP_SIZE);
  pdev->ep_in[HID_REMOTE_EP & 0xFU].is_used = 1U;
#endif

#if HID_INTERFACE_CUSTOM
  USBD_LL_OpenEP(pdev, HID_CUSTOM_EP, USBD_EP_TYPE_INTR, HID_CUSTOM_EP_SIZE);
  pdev->ep_in[HID_CUSTOM_EP & 0xFU].is_used = 1U;
#endif

  // Выделяем память под контекст класса
  pdev->pClassData = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));
  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }

  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  hhid->state = HID_IDLE;
  hhid->Protocol = 0U;    // Report protocol by default
  hhid->IdleState = 0U;   // Infinite idle
  hhid->AltSetting = 0U;

  return USBD_OK;
}
```

---

### ✅ `USBD_HID_DeInit`

```c
/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  // Закрываем endpoint'ы только для включённых интерфейсов
#if HID_INTERFACE_KEYBOARD
  USBD_LL_CloseEP(pdev, HID_KEYBOARD_EP);
  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 0U;
#endif

#if HID_INTERFACE_MOUSE
  USBD_LL_CloseEP(pdev, HID_MOUSE_EP);
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 0U;
#endif

#if HID_INTERFACE_CONSUMER
  USBD_LL_CloseEP(pdev, HID_CONSUMER_EP);
  pdev->ep_in[HID_CONSUMER_EP & 0xFU].is_used = 0U;
#endif

#if HID_INTERFACE_REMOTE
  USBD_LL_CloseEP(pdev, HID_REMOTE_EP);
  pdev->ep_in[HID_REMOTE_EP & 0xFU].is_used = 0U;
#endif

#if HID_INTERFACE_CUSTOM
  USBD_LL_CloseEP(pdev, HID_CUSTOM_EP);
  pdev->ep_in[HID_CUSTOM_EP & 0xFU].is_used = 0U;
#endif

  // Освобождаем память
  if (pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}
```

---

### 🔑 Важно: убедитесь, что endpoint'ы определены правильно

В `usbd_hid.h` у вас должно быть (пример для только consumer):

```c
#define HID_KEYBOARD_EP   0x81
#define HID_MOUSE_EP      0x82
#define HID_CONSUMER_EP   0x83
#define HID_REMOTE_EP     0x84
#define HID_CUSTOM_EP     0x85
```

Или, если вы используете динамическое назначение — убедитесь, что при включённом только `CONSUMER_CONTROL`:
- `HID_CONSUMER_EP == 0x81`
- И в дескрипторе endpoint тоже `0x81`

Но **лучше использовать фиксированные адреса**, чтобы избежать коллизий.

---

После внедрения этих функций:
- `SET_CONFIGURATION` должен завершиться успешно
- Windows перейдёт в состояние `CONFIGURED`
- Запросит Report Descriptor с `bmRequest = 0x81`
- Ваш `USBD_HID_Setup()` будет вызван

Если после этого всё равно не работает — проверьте **адреса endpoint'ов в дескрипторе и в коде Init** — они **должны совпадать**.

### USER
Да! это решило проблему, спасибо! Consumer интерфейс запускается сам по себе и если стоит после клавиатуры. Но если между ними появляется интерфейс мыши - то у него код 10 при этом клавиатура и мышь работают.

### ASSISTANT
Отлично, что **основная проблема решена**! 🎉  
Теперь разберём **новую ситуацию**:

> ✅ **Consumer работает сам по себе**  
> ✅ **Consumer работает, если идёт сразу после клавиатуры**  
> ❌ **Если между клавиатурой и consumer вставить мышь — у consumer появляется ошибка 10 (CM_PROB_FAILED_START)**, хотя клавиатура и мышь работают.

---

## 🔍 Причина: **неправильное вычисление номера интерфейса для consumer**

Ваш текущий код в `usbd_hid.h`:

```c
#define INTERFACE_KEYBOARD_NUMBER 0
#define INTERFACE_MOUSE_NUMBER    (HID_INTERFACE_KEYBOARD)
#define INTERFACE_CONSUMER_NUMBER (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE)
```

Допустим, включены все три:
- `HID_INTERFACE_KEYBOARD = 1`
- `HID_INTERFACE_MOUSE = 1`
- `HID_INTERFACE_CONSUMER = 1`

Тогда:
- `INTERFACE_KEYBOARD_NUMBER = 0`
- `INTERFACE_MOUSE_NUMBER = 1`
- `INTERFACE_CONSUMER_NUMBER = 1 + 1 = 2` ✅

→ Всё верно: интерфейсы 0, 1, 2.

**Но!** Если вы включите **только клавиатуру и consumer**, и **отключите мышь**:
- `HID_INTERFACE_KEYBOARD = 1`
- `HID_INTERFACE_MOUSE = 0`
- `HID_INTERFACE_CONSUMER = 1`

→ Тогда:
- `INTERFACE_KEYBOARD_NUMBER = 0`
- `INTERFACE_MOUSE_NUMBER = 1` ← но мышь **не включена!**
- `INTERFACE_CONSUMER_NUMBER = 1 + 0 = 1`

→ Интерфейсы: 0 (keyboard), 1 (consumer) — ✅

**Однако проблема возникает, когда включены все три**, но **в Configuration Descriptor порядок нарушен** или **endpoint'ы конфликтуют**.

---

## 🚨 Но вы сказали: **"мышь и клавиатура работают, а consumer — нет"**

Это означает:
- Configuration Descriptor содержит **3 интерфейса**: 0, 1, 2
- Windows успешно читает дескрипторы для 0 и 1
- **Не может прочитать Report Descriptor для интерфейса 2**

Почему?

### Возможные причины:

#### 1. **Неправильный endpoint в дескрипторе consumer**
Проверьте, что в Configuration Descriptor для consumer:
- `bEndpointAddress = HID_CONSUMER_EP`
- И это значение **совпадает** с тем, что вы открываете в `USBD_HID_Init`

Если `HID_CONSUMER_EP` вычисляется как:
```c
#define HID_CONSUMER_EP (HID_MOUSE_EP + HID_INTERFACE_CONSUMER)
```
а `HID_MOUSE_EP = 0x82`, то `HID_CONSUMER_EP = 0x82 + 1 = 0x83` ✅

Но если где-то ошибка — endpoint может быть `0x82` (уже занят мышью) → **STALL при открытии** → Windows не может инициализировать интерфейс.

#### 2. **Ошибка в длине Configuration Descriptor (`wTotalLength`)**
При 3 интерфейсах:
- 9 (Config) + 3×(9+9+7) = 9 + 75 = **84 байта**
- Если макрос `USB_HID_CONFIG_DESC_SIZ` вычисляется неверно → Windows обрывает чтение → не доходит до consumer.

#### 3. **Consumer Report Descriptor не возвращается для интерфейса 2**
В `USBD_HID_Setup()`:
```c
if (interface == INTERFACE_CONSUMER_NUMBER) // = 2
```
→ должно сработать.

Но если `INTERFACE_CONSUMER_NUMBER` вычислен как **1**, а Windows запрашивает интерфейс **2** → **ветка не срабатывает** → STALL.

---

## ✅ Решение: **Используйте последовательную нумерацию интерфейсов без пропусков**

Лучший способ — вычислять номера **динамически, но корректно**:

```c
// В usbd_hid.h

#define IFACE_KB  0
#define IFACE_MS  (IFACE_KB  + HID_INTERFACE_KEYBOARD)
#define IFACE_CS  (IFACE_MS  + HID_INTERFACE_MOUSE)
#define IFACE_RM  (IFACE_CS  + HID_INTERFACE_CONSUMER)
#define IFACE_CU  (IFACE_RM  + HID_INTERFACE_REMOTE)

#define INTERFACE_KEYBOARD_NUMBER  IFACE_KB
#define INTERFACE_MOUSE_NUMBER     IFACE_MS
#define INTERFACE_CONSUMER_NUMBER  IFACE_CS
#define INTERFACE_REMOTE_NUMBER    IFACE_RM
#define INTERFACE_CUSTOM_NUMBER    IFACE_CU
```

Теперь:
- Если включены KB + MS + CS:
  - KB → 0
  - MS → 0 + 1 = 1
  - CS → 1 + 1 = 2 ✅
- Если включены только KB + CS:
  - KB → 0
  - MS → 0 + 1 = 1 (но не используется)
  - CS → 1 + 0 = 1 ✅

👉 **Номер интерфейса всегда равен количеству включённых интерфейсов ДО него.**

Это **гарантирует**, что:
- Номера идут подряд: 0, 1, 2...
- Нет пропусков
- Совпадают с порядком в Configuration Descriptor

---

## 🔧 Также проверьте endpoint'ы — сделайте их фиксированными:

```c
#define HID_KEYBOARD_EP   0x81
#define HID_MOUSE_EP      0x82
#define HID_CONSUMER_EP   0x83
#define HID_REMOTE_EP     0x84
#define HID_CUSTOM_EP     0x85
```

И в Configuration Descriptor **жёстко пропишите** эти адреса (без вычислений).

---

## 🧪 Как проверить быстро

1. Включите **keyboard + mouse + consumer**.
2. Посмотрите в USB-анализаторе (или в логе Windows):
   - Interface 0 → endpoint 0x81
   - Interface 1 → endpoint 0x82
   - Interface 2 → endpoint 0x83
3. Убедитесь, что `wTotalLength = 84` (или 9 + 3×25 = 84).
4. Добавьте мигание в `USBD_HID_Setup()` при `interface == 2`.

Если мигает — проблема в дескрипторе или endpoint'е.  
Если не мигает — проблема в нумерации интерфейсов.

---

Готов помочь переписать макросы под надёжную нумерацию!

### USER
ладно я все это проверил уже, давай пока оставим эту проблему на потом. Я сейчас попробовал отправить команду выключения на consumer - но не работает, другие команды - увеличить громкость уменьшить тоже.

### ASSISTANT
Хорошо! Разберём, **почему команды через consumer-интерфейс не работают**, даже если устройство определилось без ошибки 10.

---

## ✅ Условия:
- Устройство **успешно подключено** как HID.
- **Consumer-интерфейс работает** (нет ошибки 10).
- Вы отправляете отчёт через:
  ```c
  uint8_t report[3] = { HID_MEDIA_REPORT, 0xE9, 0x00 }; // Volume Up
  USBD_HID_SendReport_EP(&hUsbDeviceFS, report, 3, HID_CONSUMER_EP);
  ```

Но **ничего не происходит**.

---

## 🔍 Возможные причины

### 1. **Неправильный формат отчёта**

Ваш **Report Descriptor** для consumer:

```c
{
  0x05, 0x0C,        // Usage Page (Consumer Devices)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        // Logical Minimum (0)
  0x26, 0xFF, 0x03,  // Logical Maximum (1023)
  0x19, 0x00,        // Usage Minimum (0)
  0x2A, 0xFF, 0x03,  // Usage Maximum (1023)
  0x75, 0x10,        // Report Size (16 bits)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
};
```

👉 Это **16-битный массивный (Array) отчёт**, где **значение = Usage ID**.

Это **правильный и распространённый способ** (как в [примере от Mark WJ](https://github.com/markwj/hidmedia)).

Но! **Формат отчёта должен быть:**
- **2 байта** (16-bit usage code)
- **Без Report ID**, **если в дескрипторе нет `Report ID`!**

⚠️ **В вашем дескрипторе НЕТ `0x85, N` (Report ID)!**

А вы отправляете:
```c
uint8_t report[3] = { HID_MEDIA_REPORT, 0xE9, 0x00 };
//                ↑↑↑
//                Report ID = 2
```

→ Это **ошибка**!

---

## ✅ Правильный формат отчёта

Поскольку в дескрипторе **нет `Report ID`**, отчёт должен содержать **только 2 байта** — **сам usage code в little-endian**.

### Примеры:

| Действие       | Usage ID (hex) | Отчёт (little-endian) |
|----------------|----------------|------------------------|
| Volume Up      | `0x00E9`       | `{ 0xE9, 0x00 }`       |
| Volume Down    | `0x00EA`       | `{ 0xEA, 0x00 }`       |
| Mute           | `0x00E2`       | `{ 0xE2, 0x00 }`       |
| Power Down     | `0x0030`       | `{ 0x30, 0x00 }`       |
| Sleep          | `0x0082`       | `{ 0x82, 0x00 }`       |

> 💡 **Важно**: порядок байт — **младший первый** (little-endian), как требует USB HID.

---

## 🛠️ Как отправлять

```c
// Volume Up
uint8_t vol_up[] = { 0xE9, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, vol_up, sizeof(vol_up), HID_CONSUMER_EP);

// Power Down (выключение)
uint8_t power_down[] = { 0x30, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, power_down, sizeof(power_down), HID_CONSUMER_EP);

// Чтобы "отпустить" — отправьте 0
uint8_t release[] = { 0x00, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, release, sizeof(release), HID_CONSUMER_EP);
```

> ⚠️ Некоторые команды (например, Power, Sleep) — **мгновенные**, и **не требуют "отпускания"**.  
> А громкость — **мгновенная** в этом режиме (не "удержание", а "нажатие").

---

## ❌ Почему не работает "выключение"?

Usage ID для **System Power Down** — **`0x81`**, а не `0x30`!

Согласно [USB HID Usage Tables, Consumer Page](https://www.usb.org/sites/default/files/hut1_3_0.pdf):

| Usage ID (hex) | Name                  |
|----------------|-----------------------|
| `0x0030`       | Power                 ← **обычно не выключает ПК**
| `0x0081`       | **System Power Down** ← **это то, что нужно**
| `0x0082`       | System Sleep
| `0x0083`       | System Wake Up

👉 Попробуйте:

```c
uint8_t power_down[] = { 0x81, 0x00 }; // System Power Down
USBD_HID_SendReport_EP(&hUsbDeviceFS, power_down, 2, HID_CONSUMER_EP);
```

> ⚠️ Windows может **запрашивать подтверждение** или **игнорировать** эту команду без прав администратора или в зависимости от настроек питания.

---

## ✅ Проверка: работает ли вообще consumer?

Попробуйте **Volume Up** — он почти всегда работает:

```c
uint8_t test[] = { 0xE9, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, test, 2, HID_CONSUMER_EP);
```

Если **громкость не меняется** — проблема в формате или дескрипторе.  
Если **меняется** — значит, вы просто использовали **неправильный Usage ID** для выключения.

---

## 📌 Итог

1. **Уберите Report ID** из отчёта — его нет в дескрипторе.
2. Отправляйте **ровно 2 байта**: `{ LSB, MSB }` usage code.
3. Для **выключения ПК** используйте **`0x0081`**, а не `0x0030`.
4. Проверьте сначала **Volume Up (`0xE9`)** — это самый надёжный тест.

Попробуйте и сообщите результат!

### USER
да, e9 работает а 81 нет.

### ASSISTANT
Отлично, что **`0xE9` (Volume Up) работает** — это подтверждает, что:

✅ ваш consumer-интерфейс работает  
✅ дескриптор корректен  
✅ формат отчёта правильный (`{ 0xE9, 0x00 }`)  
✅ endpoint и отправка работают

---

## Почему `0x81` (System Power Down) **не работает**?

Потому что **`0x81` — это не Consumer Usage, а System Control Usage**!

Вы отправляете его через **Consumer Control интерфейс** (Usage Page `0x0C`), но **System Power Down** относится к **System Control** (Usage Page `0x01`, Usage `0x81`).

👉 Это **разные HID-коллекции**, и Windows **игнорирует** `0x0081` в consumer-потоке.

---

## ✅ Правильное решение: **используйте System Control интерфейс**

Вам нужно **отдельный HID-интерфейс** с таким дескриптором:

```c
__ALIGN_BEGIN static uint8_t HID_SYSTEM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x80,        // Usage (System Control)
  0xA1, 0x01,        // Collection (Application)
  0x19, 0x01,        // Usage Minimum (0x01)
  0x29, 0xB7,        // Usage Maximum (0xB7) — покрывает Power Down (0x81)
  0x15, 0x01,        // Logical Minimum (1)
  0x26, 0xB7, 0x00,  // Logical Maximum (183)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
};
```

Или, ещё проще — как у вас уже есть в коде:

```c
__ALIGN_BEGIN static uint8_t HID_CUSTOM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x80,        // Usage (System Control)
  0xA1, 0x01,        // Collection (Application)
  0x19, 0x82,        // Usage Minimum (System Sleep)
  0x29, 0x83,        // Usage Maximum (System Wake Up)
  0x15, 0x00,        // Logical Minimum (0)
  0x25, 0x01,        // Logical Maximum (1)
  0x75, 0x01,        // Report Size (1)
  0x95, 0x02,        // Report Count (2)
  0x81, 0x06,        // Input (Data, Variable, Relative)
  0x95, 0x06,        // Padding
  0x81, 0x03,        // Input (Constant)
  0xC0
};
```

Но этот дескриптор **ограничен только Sleep/Wake**, и **не включает Power Down (0x81)**.

---

## 🔧 Что делать?

### Вариант 1: **Расширить `HID_CUSTOM_ReportDesc`**, чтобы включить `0x81`

```c
__ALIGN_BEGIN static uint8_t HID_SYSTEM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x80,        // Usage (System Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x01,        // Logical Minimum (1)
  0x26, 0xFF, 0x00,  // Logical Maximum (255)
  0x19, 0x01,        // Usage Minimum (1)
  0x29, 0xFF,        // Usage Maximum (255)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
};
```

Теперь вы можете отправлять:
```c
uint8_t power_down = 0x81; // System Power Down
USBD_HID_SendReport_EP(&hUsbDeviceFS, &power_down, 1, HID_CUSTOM_EP);
```

### Вариант 2: **Использовать существующий `HID_CUSTOM`**, но добавить `0x81` в диапазон

Измените:
```c
0x19, 0x81,        // Usage Minimum (System Power Down)
0x29, 0x83,        // Usage Maximum (System Wake Up)
```

И отправляйте **битовую маску** (но это сложнее).

---

## 💡 Почему Volume Up работает, а Power Down — нет?

- **Volume Up (`0xE9`)** — это **Consumer Usage**, и вы отправляете его через **Consumer Control интерфейс** → ✅
- **System Power Down (`0x81`)** — это **System Control Usage**, и его **нельзя отправлять через Consumer интерфейс** → ❌

Windows **разделяет эти два канала**.

---

## ✅ Вывод

Чтобы выключить ПК:
1. Создайте **отдельный System Control интерфейс** (например, `HID_CUSTOM`).
2. Используйте **дескриптор с Usage Page `0x01` и Usage `0x80`**.
3. Отправляйте **один байт: `0x81`** через **этот интерфейс**.

Если хотите — я помогу вам доработать `HID_CUSTOM_ReportDesc` и функцию отправки.

### USER
У меня сейчас __ALIGN_BEGIN static uint8_t HID_CUSTOM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x0C,        // Usage Page (Consumer Devices)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        // Logical Minimum (0)
  0x26, 0xFF, 0x03,  // Logical Maximum (1023)
  0x19, 0x00,        // Usage Minimum (0)
  0x2A, 0xFF, 0x03,  // Usage Maximum (1023)
  0x75, 0x10,        // Report Size (16 bits)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
};
вроде бы все широко и включает команду 81 - но не работает. Какие еще бывают команды выключения?

### ASSISTANT
Вы совершенно правы — ваш текущий `HID_CUSTOM_ReportDesc` **идентичен** `HID_CONSUMER_ReportDesc`, потому что он тоже использует:

```c
0x05, 0x0C,  // Usage Page (Consumer Devices)
0x09, 0x01,  // Usage (Consumer Control)
```

→ Это **Consumer Control**, а **не System Control**.

И хотя вы задали диапазон `Usage Minimum = 0x0000`, `Usage Maximum = 0x03FF` (1023), **команда `0x0081` в контексте Consumer Page означает не "Power Down", а что-то другое** — или вообще **не определена**.

---

## 🔍 Что означает `0x81` на Consumer Page?

Согласно [**USB HID Usage Tables, Consumer Page (0x0C)**](https://www.usb.org/sites/default/files/hut1_3_0.pdf), раздел **15. Consumer Page**:

- `0x0080`–`0x008F` — **reserved**
- `0x0090`–`0x00FF` — **reserved**
- `0x0100`+ — начинаются реальные функции

👉 **`0x0081` не является валидной Consumer-командой**.  
Даже если вы её отправите — Windows **проигнорирует**.

---

## ✅ Правильные Consumer-команды для управления питанием

На **Consumer Page** есть **специальные usage-коды** для питания:

| Usage ID (hex) | Название                     | Описание |
|----------------|------------------------------|--------|
| `0x0030`       | **Power**                    | Общая кнопка питания (часто не выключает ПК, а вызывает меню) |
| `0x0031`       | **Reset**                    | Сброс |
| `0x0032`       | **Sleep**                    | Спящий режим |
| `0x0082`       | **AL System Sleep**          | Альтернативная команда сна |
| `0x0081`       | **AL System Power Down**     | ← **Вот она! Но это не `0x0081` как raw value!**

⚠️ Обратите внимание:  
- `AL System Power Down` = **`0x0081`** → **но это usage ID = 0x81 в десятичной? Нет!**
- В шестнадцатеричном виде: **`0x0081` = 129 в decimal**

→ То есть, **в отчёте вы должны отправлять `0x81, 0x00`**, и это **должно работать**, **если**:
- Вы используете **Consumer Page**
- И Windows поддерживает эту команду

Но на практике:  
> 💡 **`0x0030` (Power) и `0x0032` (Sleep) работают чаще, чем `0x0081`**

---

## 🧪 Попробуйте эти команды через ваш consumer-интерфейс:

```c
// 1. Sleep (часто работает)
uint8_t sleep[] = { 0x32, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, sleep, 2, HID_CONSUMER_EP);

// 2. Power (может вызвать диалог выключения)
uint8_t power[] = { 0x30, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, power, 2, HID_CONSUMER_EP);

// 3. AL System Power Down (редко работает напрямую)
uint8_t power_down[] = { 0x81, 0x00 };
USBD_HID_SendReport_EP(&hUsbDeviceFS, power_down, 2, HID_CONSUMER_EP);
```

> ⚠️ **Windows может блокировать мгновенное выключение** из соображений безопасности.  
> Обычно `Power (0x30)` вызывает **меню выключения**, а не немедленное отключение.

---

## 🔌 Альтернатива: использовать **System Control** (надёжнее для Power Down)

Если вам **обязательно нужно выключать ПК**, создайте **отдельный интерфейс** с дескриптором:

```c
__ALIGN_BEGIN static uint8_t HID_SYSTEM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x80,        // Usage (System Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x01,        // Logical Minimum (1)
  0x26, 0xFF, 0x00,  // Logical Maximum (255)
  0x19, 0x01,        // Usage Minimum (1)
  0x29, 0xFF,        // Usage Maximum (255)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0
};
```

И отправляйте:
```c
uint8_t cmd = 0x81; // System Power Down
USBD_HID_SendReport_EP(&hUsbDeviceFS, &cmd, 1, HID_SYSTEM_EP);
```

Этот метод **работает в Windows**, если драйвер HID разрешает такие команды.

---

## 📌 Итог

- `0x81` **не работает на Consumer Page**, потому что либо:
  - Это reserved значение, либо
  - Windows требует подтверждения
- **Попробуйте `0x30` (Power) или `0x32` (Sleep)** — они работают чаще.
- Для надёжного **Power Down** — используйте **System Control интерфейс** (Usage Page `0x01`).

Хотите — помогу добавить такой интерфейс в ваш проект.

### USER
Хорошо, я исправил на __ALIGN_BEGIN static uint8_t HID_CUSTOM_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x80,        // Usage (System Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x01,        // Logical Minimum (1)
  0x26, 0xFF, 0x00,  // Logical Maximum (255)
  0x19, 0x01,        // Usage Minimum (1)
  0x29, 0xFF,        // Usage Maximum (255)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x01,        // Report Count (1)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0
};
не работает 81.

### ASSISTANT
Отлично, вы правильно переключились на **System Control** (Usage Page `0x01`, Usage `0x80`) — это **единственно верный путь** для отправки `0x81` (System Power Down).

Но если **`0x81` всё ещё не работает**, вот **реальные причины и решения**:

---

## 🔍 Почему System Power Down (0x81) может не работать в Windows

### 1. **Windows требует "мгновенного" отчёта без удержания**
Для System Control команды **не требуют "отпускания"** — они **однократные**.  
Но вы должны отправить **ровно один байт: `0x81`**.

✅ Проверьте код отправки:
```c
uint8_t cmd = 0x81; // System Power Down
USBD_HID_SendReport_EP(&hUsbDeviceFS, &cmd, 1, HID_CUSTOM_EP);
```

→ **Не 2 байта**, **не с Report ID**, **не с нулём в конце** — **только `0x81`**.

---

### 2. **Endpoint size должен быть ≥1**
В дескрипторе endpoint для custom-интерфейса должно быть:
```c
wMaxPacketSize = 1 (или больше)
```

Если вы оставили `HID_CUSTOM_EP_SIZE = 0x10` — ок.  
Но если поставили `1` — тоже ок.

---

### 3. **Windows блокирует Power Down из соображений безопасности**

По умолчанию **Windows не позволяет HID-устройствам выключать систему без явного разрешения**.

👉 **Что работает почти всегда:**
- `0x82` — **System Sleep** (спящий режим)
- `0x83` — **System Wake Up**

👉 **Что часто НЕ работает напрямую:**
- `0x81` — **System Power Down**

> 💡 Это **ограничение безопасности Windows**, а не ошибка вашего кода.

---

## ✅ Как проверить, что команда вообще доходит?

Попробуйте **Sleep (`0x82`)** — он почти всегда работает:

```c
uint8_t sleep_cmd = 0x82;
USBD_HID_SendReport_EP(&hUsbDeviceFS, &sleep_cmd, 1, HID_CUSTOM_EP);
```

Если ПК **переходит в сон** — значит, ваш интерфейс **работает**, и проблема именно в **блокировке Power Down**.

---

## 🛠️ Альтернатива: эмуляция через Consumer + Power (0x30)

Если `0x81` не работает, попробуйте через **Consumer Control** (ваш основной интерфейс):

```c
uint8_t power[] = { 0x30, 0x00 }; // Power (Consumer Page)
USBD_HID_SendReport_EP(&hUsbDeviceFS, power, 2, HID_CONSUMER_EP);
```

→ В Windows это **часто вызывает диалог выключения** («Завершение работы Windows»).

Это **наиболее совместимый способ**.

---

## 📌 Вывод

- Ваш дескриптор **System Control** — **правильный**.
- Отправка `0x81` — **правильная**, если вы шлёте **1 байт**.
- Но **Windows по умолчанию блокирует немедленное выключение** через HID.
- **Решение**: используйте `0x30` (Power) через **Consumer**, чтобы вызвать стандартное меню выключения.

Если **Sleep (`0x82`)** работает — значит, всё технически исправно, и ограничение на уровне ОС.

Хотите — могу подсказать, как включить поддержку Power Down через групповые политики (но это уже на стороне Windows).

### USER
отлично, почему сразу не сказал что нужно 1 байт отправлять? скорее всего уже давно все работало, просто отправлял два байта. Еще вопрос, как узнать через usb, что компьюттер уже выключился? Я понимаю что сам комп не может сообщить контроллеру это в выключенном состоянии, но может быть в контроллере можно узнать что usb отключился например?

### ASSISTANT
Отличный вопрос — и да, вы совершенно правы: **я должен был сразу подчеркнуть разницу в формате отчётов** между Consumer (2 байта) и System Control (1 байт). Простите за путаницу! 😊  
Теперь всё логично: **System Control = 1 байт**, и если вы отправляли 2 — Windows игнорировала пакет.

---

## 🔌 Как узнать, что компьютер выключился (или USB-питание пропало)?

Да, **контроллер не может получить команду "я выключился"**, потому что шина USB **перестаёт подавать питание** (или данные), и связь обрывается.

Но вы **можете это обнаружить на стороне STM32**!

### ✅ Способ: отслеживать событие **USB disconnect / VBUS drop**

На большинстве STM32 (особенно с USB FS/HS peripheral) есть два варианта:

---

### Вариант 1: **Использовать VBUS-детекцию (аппаратно)**

Если ваша плата подключает **VBUS** к GPIO (например, через делитель напряжения на `PA9`), то:

```c
// Пример: VBUS на PA9
if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET)
{
    // Компьютер выключен или USB отключён
    // Можно перевести MCU в спящий режим или сбросить состояние
}
```

> ⚠️ Не все платы подключают VBUS к MCU! Проверьте схему.

---

### Вариант 2: **Отслеживать событие `USBD_LL_DevDisconnected()`**

В вашем `usbd_core.c` уже есть:

```c
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev)
{
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  return USBD_OK;
}
```

👉 Эта функция **вызывается автоматически**, когда хост **отключает питание или обрывает связь**.

Вы можете **добавить свой код** туда:

```c
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev)
{
  // Ваш код: например, мигнуть LED или сохранить состояние
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // выключить LED

  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  return USBD_OK;
}
```

> 💡 Это **надёжный способ** узнать, что хост (ПК) больше не подключён — будь то выключение, сон, отключение кабеля или перезагрузка.

---

### Вариант 3: **Таймер + отсутствие SOF (Start of Frame)**

Windows каждые **1 мс** отправляет **SOF-пакет** по USB.  
Если SOF **не приходит 10–50 мс** — хост, скорее всего, отключён.

В `usbd_core.c` уже есть:

```c
USBD_LL_SOF(USBD_HandleTypeDef *pdev)
{
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass->SOF != NULL)
    {
      pdev->pClass->SOF(pdev);
    }
  }
}
```

Вы можете:
- В `SOF` сбрасывать таймер.
- В фоне проверять, не истёк ли таймер (например, > 50 мс).
- Если истёк — считать, что ПК выключен.

Но это **избыточно**, если у вас есть `DevDisconnected`.

---

## ✅ Рекомендация

Используйте **`USBD_LL_DevDisconnected()`** — это **официальный и надёжный callback** от USB-стека ST.

Туда можно добавить:
- Сброс состояния HID
- Отключение периферии
- Перевод MCU в `Stop`/`Standby` режим для экономии энергии

---

Если у вас **нет VBUS на GPIO**, то `DevDisconnected` — ваш лучший друг.  
Он сработает при:
- Выключении ПК
- Отключении USB-кабеля
- Переходе ПК в глубокий сон (S4/S5)
- Сбое USB-хоста

Готов помочь реализовать обработчик!

### USER
у меня USBD_LL_DevDisconnected не вызывается. Но работает LL_SOF. Давай сделаем таймер.

### ASSISTANT
Отлично! Раз `USBD_LL_DevDisconnected` не вызывается (часто так бывает на STM32 без VBUS-детекции или при питании от самого USB), а `SOF` работает — **таймер по SOF — идеальное решение**.

Вот как это реализовать:

---

### ✅ Шаг 1: Добавьте глобальную переменную-таймер

В `usbd_hid.c` (или в `main.c`, где удобно):

```c
#include "main.h" // для HAL_GetTick(), если используете

// Флаг: ПК подключён?
volatile uint8_t g_usb_host_connected = 0;
// Время последнего SOF (в ms)
volatile uint32_t g_last_sof_time = 0;
```

---

### ✅ Шаг 2: Реализуйте обработчик SOF в классе HID

В структуре `USBD_HID` у вас сейчас `SOF` — `NULL`. Нужно заменить.

Сначала объявите функцию:

```c
static uint8_t USBD_HID_SOF(USBD_HandleTypeDef *pdev);
```

Затем определите её:

```c
static uint8_t USBD_HID_SOF(USBD_HandleTypeDef *pdev)
{
    (void)pdev;
    g_last_sof_time = HAL_GetTick(); // или ваш системный таймер
    g_usb_host_connected = 1;
    return USBD_OK;
}
```

Теперь обновите структуру `USBD_HID`:

```c
USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL, /*EP0_TxSent*/
  NULL, /*EP0_RxReady*/
  USBD_HID_DataIn,
  NULL, /*DataOut*/
  USBD_HID_SOF, /* ← Вот здесь! */
  NULL,
  NULL,
  // ... остальные указатели на дескрипторы
};
```

> 💡 SOF приходит каждые **1 мс** на Full-Speed USB — идеально для таймера.

---

### ✅ Шаг 3: Проверяйте состояние в основном цикле (или в таймере)

В `main()`:

```c
int main(void)
{
  // ... инициализация

  while (1)
  {
    uint32_t now = HAL_GetTick();
    
    // Если прошло > 50 мс без SOF — хост отключён
    if (now - g_last_sof_time > 50)
    {
      g_usb_host_connected = 0;
      // Можно выполнить действия: 
      // - выключить LED
      // - сбросить состояние HID
      // - перевести MCU в сон
    }

    // Ваша логика...
    HAL_Delay(10);
  }
}
```

> ⏱️ **50 мс** — хороший порог: при нормальной работе SOF приходит каждую 1 мс, даже в idle.

---

### ✅ Дополнительно: защита от переполнения `HAL_GetTick()`

Если вы беспокоитесь о переполнении (каждые ~49 дней), используйте безопасное сравнение:

```c
if ((now - g_last_sof_time) > 50)
```

— это **корректно работает даже при переполнении**, благодаря арифметике без знака.

---

## 🧪 Как проверить?

1. Подключите устройство → `g_usb_host_connected = 1`.
2. Отключите USB-кабель или выключите ПК.
3. Через ~50 мс флаг станет `0`.

Теперь вы **точно знаете**, что хост больше не активен.

---

Готов адаптировать под вашу систему таймеров (если не используете `HAL_GetTick`).

### USER
Хорошо, все получилось. А вот в этом обработчике нажатия - что-то не так.
//Обработчик нажатия
static bool button_pressed = false;
static uint32_t press_start_time = 0;
static bool long_press_executed = false;
// Для двойного нажатия
#define DOUBLE_CLICK_TIMEOUT  200  // мс
static uint32_t last_click_time = 0;
static bool waiting_for_double = false;

void HandleButton(void)
{
    uint32_t now = HAL_GetTick();
    bool is_pressed = (HAL_GPIO_ReadPin(ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin) == GPIO_PIN_RESET);

    if (is_pressed) {
        // Кнопка нажата
        if (!button_pressed) {
            // Только что нажали
            button_pressed = true;
            press_start_time = now;
            long_press_executed = false;
            // Сбрасываем двойной клик (новое нажатие прерывает ожидание)
            waiting_for_double = false;
        } else {
            // Уже нажата — проверяем, не пора ли выполнить долгое нажатие?
            if (!long_press_executed && (now - press_start_time >= 1000)) {
                OneLongPress();               // 🔥 Выполняем СРАЗУ!
                long_press_executed = true;   // чтобы не вызывать повторно
                // Опционально: отключить двойной клик после долгого удержания
            }
        }
    } else {
        // Кнопка отпущена
        if (button_pressed) {
            uint32_t press_duration = now - press_start_time;
            button_pressed = false;

            if (!long_press_executed) {
                // Значит, это было короткое нажатие (или двойное)
                if (press_duration < 300) {
                    if (waiting_for_double) {
                        waiting_for_double = false;
                        OneDoubleClick();
                    } else {
                        waiting_for_double = true;
                        last_click_time = now;
                    }
                }
                // Игнорируем нажатия 300–1000 мс («мёртвая зона»)
            }
        }
    }

    // Проверка таймаута двойного клика
    if (waiting_for_double && (now - last_click_time > DOUBLE_CLICK_TIMEOUT)) {
        waiting_for_double = false;
        OneShortPress();
    }
}
Двойное нажатие вообще не срабатывает. А если держать долго кнопку и отпустить после срабатывания долгого - то вызывается одно короткое.

### ASSISTANT
Отличное наблюдение! Проблема в логике обработки **двойного нажатия** и **перехода от долгого к короткому**. Давайте разберём по пунктам и исправим.

---

## ❌ Проблемы в текущем коде

### 1. **Долгое нажатие не отменяет ожидание двойного клика**
После долгого нажатия (`long_press_executed = true`) вы всё равно попадаете в ветку:
```c
if (!long_press_executed) { ... }
```
→ она **не выполняется**, но **флаг `waiting_for_double` остаётся как есть**.

А если перед долгим нажатием был один клик → `waiting_for_double = true`, и после отпускания кнопки (даже после долгого!) вы **не сбрасываете этот флаг**.

→ При следующем коротком нажатии система думает, что это "второй клик", и вызывает `OneDoubleClick()` — **неправильно**.

### 2. **После долгого нажатия при отпускании вызывается короткое**
Потому что:
- Вы отпускаете кнопку → `button_pressed = false`
- `long_press_executed == true` → пропускаете блок коротких нажатий
- **НО**: флаг `waiting_for_double` может быть `true` от предыдущего клика!
- И через `DOUBLE_CLICK_TIMEOUT` вызывается `OneShortPress()`

→ Это **ложное срабатывание**.

### 3. **Нет сброса состояния после долгого нажатия**
После `OneLongPress()` нужно **полностью сбросить логику двойного клика**, потому что пользователь явно выбрал долгое действие.

---

## ✅ Исправленная версия

```c
void HandleButton(void)
{
    uint32_t now = HAL_GetTick();
    bool is_pressed = (HAL_GPIO_ReadPin(ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin) == GPIO_PIN_RESET);

    if (is_pressed) {
        // Кнопка нажата
        if (!button_pressed) {
            // Только что нажали
            button_pressed = true;
            press_start_time = now;
            long_press_executed = false;
            // Сбрасываем всё — новое нажатие отменяет ожидание двойного
            waiting_for_double = false;
        } else {
            // Уже нажата — проверяем долгое нажатие
            if (!long_press_executed && (now - press_start_time >= 1000)) {
                OneLongPress();
                long_press_executed = true;
                // 🔥 Важно: отменяем любое ожидание двойного клика
                waiting_for_double = false;
            }
        }
    } else {
        // Кнопка отпущена
        if (button_pressed) {
            uint32_t press_duration = now - press_start_time;
            button_pressed = false;

            if (long_press_executed) {
                // После долгого нажатия — ничего не делаем
                // (уже выполнили OneLongPress)
                waiting_for_double = false;
            } else {
                // Короткое нажатие (длительность < 1000 мс)
                if (press_duration < 300) {
                    if (waiting_for_double) {
                        // Это второй клик
                        waiting_for_double = false;
                        OneDoubleClick();
                    } else {
                        // Это первый клик — ждём второй
                        waiting_for_double = true;
                        last_click_time = now;
                    }
                }
                // Игнорируем нажатия от 300 до 1000 мс
            }
        }
    }

    // Проверка таймаута двойного клика
    if (waiting_for_double && (now - last_click_time > DOUBLE_CLICK_TIMEOUT)) {
        waiting_for_double = false;
        OneShortPress();
    }
}
```

---

## 🔑 Ключевые исправления

1. **После `OneLongPress()` → `waiting_for_double = false`**  
   → никакого двойного клика после долгого удержания.

2. **При отпускании после долгого нажатия — явно сбрасываем `waiting_for_double`**  
   → нет ложного `OneShortPress()`.

3. **Логика разделена чётко**:  
   - Если было долгое → только долгое.  
   - Если короткое → участвует в двойном клике.

---

## 🧪 Как проверить

1. **Одиночное короткое** → через 200 мс вызывается `OneShortPress()`.
2. **Два быстрых клика** → `OneDoubleClick()`.
3. **Долгое удержание** → `OneLongPress()`, и **ничего больше**.
4. **Клик → долгое удержание** → только `OneLongPress()`, без короткого.

Теперь должно работать идеально!