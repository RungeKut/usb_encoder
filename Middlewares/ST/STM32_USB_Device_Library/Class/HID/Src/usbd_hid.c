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
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x06,        // Usage (Keyboard)
  0xA1, 0x01,        // Collection (Application)
  0x05, 0x07,        // Usage Page (Key Codes)
  0x19, 0xE0,        // Usage Minimum (224) - Modifiers
  0x29, 0xE7,        // Usage Maximum (231)
  0x15, 0x00,        // Logical Minimum (0)
  0x25, 0x01,        // Logical Maximum (1)
  0x75, 0x01,        // Report Size (1)
  0x95, 0x08,        // Report Count (8)
  0x81, 0x02,        // Input (Data, Variable, Absolute)
  0x95, 0x01,        // Report Count (1)
  0x75, 0x08,        // Report Size (8)
  0x81, 0x03,        // Input (Constant)
  0x95, 0x06,        // Report Count (6)
  0x75, 0x08,        // Report Size (8)
  0x15, 0x00,        // Logical Minimum (0)
  0x25, 0x65,        // Logical Maximum (101)
  0x05, 0x07,        // Usage Page (Key Codes)
  0x19, 0x00,        // Usage Minimum (0)
  0x29, 0x65,        // Usage Maximum (101)
  0x81, 0x00,        // Input (Data, Array)
  0xC0               // End Collection
};

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[] __ALIGN_END =
{
  0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,        // USAGE (Mouse)
  0xA1, 0x01,        // COLLECTION (Application)
  0x09, 0x01,        //   Report ID (3)
  0xA1, 0x00,        //   USAGE (Pointer)
  0x05, 0x09,        //   COLLECTION (Physical)
  0x19, 0x01,        //     USAGE_PAGE (Button)
  0x29, 0x03,        //     USAGE_MINIMUM (Button 1)
  0x15, 0x00,        //     USAGE_MAXIMUM (Button 3)
  0x25, 0x01,        //     LOGICAL_MINIMUM (0)
  0x95, 0x03,        //     LOGICAL_MAXIMUM (1)
  0x75, 0x01,        //     REPORT_COUNT (3)
  0x81, 0x02,        //     REPORT_SIZE (1)
  0x95, 0x01,        //     INPUT (Data,Var,Abs)
  0x75, 0x05,        //     REPORT_COUNT (1)
  0x81, 0x03,        //     REPORT_SIZE (5)
  0x05, 0x01,        //     INPUT (Cnst,Var,Abs)
  0x09, 0x30,        //     USAGE_PAGE (Generic Desktop)
  0x09, 0x31,        //     USAGE (X)
  0x09, 0x38,        //     USAGE (Y)
  0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
  0x25, 0x7F,        //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,        //     REPORT_SIZE (8)
  0x95, 0x03,        //     REPORT_COUNT (2)
  0x81, 0x06,        //     INPUT (Data,Var,Rel)
  0xC0,              //   END_COLLECTION
  0xC0               // END_COLLECTION
};

__ALIGN_BEGIN static uint8_t HID_CONSUMER_ReportDesc[] __ALIGN_END =
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

/* Размеры дескрипторов ------------------------------------------------------*/
#define KEYBOARD_REPORT_DESC_SIZE  (sizeof(HID_KEYBOARD_ReportDesc))
#define MOUSE_REPORT_DESC_SIZE     (sizeof(HID_MOUSE_ReportDesc))
#define CONSUMER_REPORT_DESC_SIZE  (sizeof(HID_CONSUMER_ReportDesc))

/* Configuration Descriptor --------------------------------------------------*/
// Дескриптор конфигурации (описывает возможности устройства)
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  // Configuration Descriptor (9 byte)
  /*  1 byte*/ 0x09,      // bLength: Configuration Descriptor size - длина дескриптора конфигурации
  /*  2 byte*/ USB_DESC_TYPE_CONFIGURATION, // bDescriptorType: Configuration - тип дескриптора - конфигурация
  /*3-4 byte*/ LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ), // wTotalLength - общий размер всего дерева под данной конфигурацией в байтах
  /*  5 byte*/ 0x03,      // bNumInterfaces -  в конфигурации три интерфейса
  /*  6 byte*/ 0x01,      // bConfigurationValue - индекс текущей конфигурации
  /*  7 byte*/ 0x00,      // iConfiguration - индекс строки, которая описывает эту конфигурацию
  /*  8 byte*/ 0x40,      // bmAttributes - признак того, что устройство будет питаться от шины USB
  /*  9 byte*/ 0x32,      // MaxPower (100 mA)

  // ============ INTERFACE 0: KEYBOARD ============
	
	// Interface Descriptor (9 byte)
  /*  1 byte*/ 0x09,      // bLength: Interface Descriptor size
	/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
	/*  3 byte*/ 0x00,      // bInterfaceNumber: Number of Interface
	/*  4 byte*/ 0x00,      // bAlternateSetting: Alternate setting
	/*  5 byte*/ 0x01,      // bNumEndpoints
	/*  6 byte*/ 0x03,      // bInterfaceClass: HID
	/*  7 byte*/ 0x01,      // bInterfaceSubClass : 1=BOOT, 0=no boot
	/*  8 byte*/ 0x01,      // nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
	/*  9 byte*/ 0x00,      // iInterface: Index of string descriptor
	
	    // HID Descriptor (9 byte)
      /*  1 byte*/ 0x09,      // bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE, // bDescriptorType: HID
			/*  3 byte*/ 0x11,      // bcdHID: HID Class Spec release number
			/*  4 byte*/ 0x01, 
			/*  5 byte*/ 0x00,      // bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,      // bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,      // bDescriptorType
			/*8-9 byte*/ LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor
	
	        // Endpoint Descriptor (7 byte)
          /*  1 byte*/ 0x07,      // bLength: Endpoint Descriptor size
					/*  2 byte*/ USB_DESC_TYPE_ENDPOINT, // bDescriptorType: ENDPOINT
					/*  3 byte*/ HID_KEYBOARD_EP, // bEndpointAddress: Endpoint Address (IN)
					/*  4 byte*/ 0x03,      // bmAttributes: Interrupt endpoint
					/*  5 byte*/ HID_KEYBOARD_EP_SIZE, // wMaxPacketSize
					/*  6 byte*/ 0x00,
					/*  7 byte*/ HID_FS_BINTERVAL, // bInterval: Polling Interval (10 ms)

  // ============ INTERFACE 1: MOUSE ============
	
	// Interface Descriptor (9 byte)
  /*  1 byte*/ 0x09,      // bLength: Interface Descriptor size
	/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
	/*  3 byte*/ 0x01,      // bInterfaceNumber: Number of Interface
	/*  4 byte*/ 0x00,      // bAlternateSetting: Alternate setting
	/*  5 byte*/ 0x01,      // bNumEndpoints
	/*  6 byte*/ 0x03,      // bInterfaceClass: HID
	/*  7 byte*/ 0x01,      // bInterfaceSubClass : 1=BOOT, 0=no boot
	/*  8 byte*/ 0x02,      // nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
	/*  9 byte*/ 0x00,      // iInterface: Index of string descriptor
	
	    // HID Descriptor (9 byte)
      /*  1 byte*/ 0x09,      // bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE, // bDescriptorType: HID
			/*  3 byte*/ 0x11,      // bcdHID: HID Class Spec release number
			/*  4 byte*/ 0x01, 
			/*  5 byte*/ 0x00,      // bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,      // bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,      // bDescriptorType
			/*8-9 byte*/ LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor
			
	        // Endpoint Descriptor (7 byte)
          /*  1 byte*/ 0x07,      // bLength: Endpoint Descriptor size
					/*  2 byte*/ USB_DESC_TYPE_ENDPOINT, // bDescriptorType: ENDPOINT
					/*  3 byte*/ HID_MOUSE_EP, // bEndpointAddress: Endpoint Address (IN)
					/*  4 byte*/ 0x03,      // bmAttributes: Interrupt endpoint
					/*  5 byte*/ HID_MOUSE_EP_SIZE, // wMaxPacketSize
					/*  6 byte*/ 0x00,
					/*  7 byte*/ HID_FS_BINTERVAL, // bInterval: Polling Interval (10 ms)

  // ============ INTERFACE 2: CONSUMER CONTROL ============
	
	// Interface Descriptor (9 byte)
  /*  1 byte*/ 0x09,      // bLength: Interface Descriptor size
	/*  2 byte*/ USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface descriptor type
	/*  3 byte*/ 0x02,      // bInterfaceNumber: Number of Interface
	/*  4 byte*/ 0x00,      // bAlternateSetting: Alternate setting
	/*  5 byte*/ 0x01,      // bNumEndpoints
	/*  6 byte*/ 0x03,      // bInterfaceClass: HID
	/*  7 byte*/ 0x01,      // bInterfaceSubClass : 1=BOOT, 0=no boot
	/*  8 byte*/ 0x00,      // nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
	/*  9 byte*/ 0x00,      // iInterface: Index of string descriptor
	
	    // HID Descriptor (9 byte)
      /*  1 byte*/ 0x09,      // bLength: HID Descriptor size
			/*  2 byte*/ HID_DESCRIPTOR_TYPE, // bDescriptorType: HID
			/*  3 byte*/ 0x11,      // bcdHID: HID Class Spec release number
			/*  4 byte*/ 0x01, 
			/*  5 byte*/ 0x00,      // bCountryCode: Hardware target country
			/*  6 byte*/ 0x01,      // bNumDescriptors: Number of HID class descriptors to follow
			/*  7 byte*/ 0x22,      // bDescriptorType
			/*8-9 byte*/ LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE), // wItemLength: Total length of Report descriptor
			
	        // Endpoint Descriptor (7 byte)
          /*  1 byte*/ 0x07,      // bLength: Endpoint Descriptor size
					/*  2 byte*/ USB_DESC_TYPE_ENDPOINT, // bDescriptorType: ENDPOINT
					/*  3 byte*/ HID_CONSUMER_EP, // bEndpointAddress: Endpoint Address (IN)
					/*  4 byte*/ 0x03,      // bmAttributes: Interrupt endpoint
					/*  5 byte*/ HID_CONSUMER_EP_SIZE, // wMaxPacketSize
					/*  6 byte*/ 0x00,
					/*  7 byte*/ HID_FS_BINTERVAL, // bInterval: Polling Interval (10 ms)
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
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EP, USBD_EP_TYPE_INTR, HID_KEYBOARD_EP_SIZE);
  USBD_LL_OpenEP(pdev, HID_MOUSE_EP,    USBD_EP_TYPE_INTR, HID_MOUSE_EP_SIZE);
  USBD_LL_OpenEP(pdev, HID_CONSUMER_EP, USBD_EP_TYPE_INTR, HID_CONSUMER_EP_SIZE);

  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 1U;
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 1U;
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
  USBD_LL_CloseEP(pdev, HID_KEYBOARD_EP);
  USBD_LL_CloseEP(pdev, HID_MOUSE_EP);
  USBD_LL_CloseEP(pdev, HID_CONSUMER_EP);

  pdev->ep_in[HID_KEYBOARD_EP & 0xFU].is_used = 0U;
  pdev->ep_in[HID_MOUSE_EP & 0xFU].is_used = 0U;
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

