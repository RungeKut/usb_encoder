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
static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t  *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t  *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
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
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
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
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        // Usage (Pointer)
  0xA1, 0x00,        // Collection (Physical)
  0x05, 0x09,        // Usage Page (Buttons)
  0x19, 0x01,        // Usage Minimum (Button 1)
  0x29, 0x03,        // Usage Maximum (Button 3)
  0x15, 0x00,        // Logical Minimum (0)
  0x25, 0x01,        // Logical Maximum (1)
  0x95, 0x03,        // Report Count (3)
  0x75, 0x01,        // Report Size (1)
  0x81, 0x02,        // Input (Data, Variable, Absolute)
  0x95, 0x01,        // Report Count (1)
  0x75, 0x05,        // Report Size (5)
  0x81, 0x03,        // Input (Constant)
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x30,        // Usage (X)
  0x09, 0x31,        // Usage (Y)
  0x09, 0x38,        // Usage (Wheel)
  0x15, 0x81,        // Logical Minimum (-127)
  0x25, 0x7F,        // Logical Maximum (127)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x03,        // Report Count (3)
  0x81, 0x06,        // Input (Data, Variable, Relative)
  0xC0,              // End Collection
  0xC0               // End Collection
};

__ALIGN_BEGIN static uint8_t HID_CONSUMER_ReportDesc[] __ALIGN_END =
{
  0x05, 0x0C,        // Usage Page (Consumer)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        // Logical Minimum (0)
  0x26, 0xFF, 0x03,  // Logical Maximum (1023)
  0x19, 0x00,        // Usage Minimum (0)
  0x2A, 0xFF, 0x03,  // Usage Maximum (1023)
  0x81, 0x00,        // Input (Data, Array, Absolute)
  0xC0               // End Collection
};

/* Размеры дескрипторов ------------------------------------------------------*/
#define KEYBOARD_REPORT_DESC_SIZE  (sizeof(HID_KEYBOARD_ReportDesc))
#define MOUSE_REPORT_DESC_SIZE     (sizeof(HID_MOUSE_ReportDesc))
#define CONSUMER_REPORT_DESC_SIZE  (sizeof(HID_CONSUMER_ReportDesc))

/* Configuration Descriptor (FS) — total size = 84 bytes ---------------------*/
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  // Configuration Descriptor (9 byte)
  0x09, 0x02, // bLength, bDescriptorType
  LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ), // wTotalLength = 84
  0x03,       // bNumInterfaces
  0x01,       // bConfigurationValue
  0x00,       // iConfiguration
  0xC0,       // bmAttributes
  0x32,       // MaxPower (100 mA)

  // ============ INTERFACE 0: KEYBOARD ============
  /*9 byte*/ 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_KEYBOARD_EP, 0x03, HID_KEYBOARD_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 1: MOUSE ============
  /*9 byte*/ 0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_MOUSE_EP, 0x03, HID_MOUSE_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 2: CONSUMER CONTROL ============
  /*9 byte*/ 0x09, 0x04, 0x02, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_CONSUMER_EP, 0x03, HID_CONSUMER_EP_SIZE, 0x00, 0x10,
};

/* USB HID device HS Configuration Descriptor --------------------------------*/
// Для STM32F1 (Full Speed only) можно просто скопировать FS
// Но формально HS не используется на F1
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  // Configuration Descriptor (9 byte)
  0x09, 0x02, // bLength, bDescriptorType
  LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ), // wTotalLength = 84
  0x03,       // bNumInterfaces
  0x01,       // bConfigurationValue
  0x00,       // iConfiguration
  0xC0,       // bmAttributes
  0x32,       // MaxPower (100 mA)

  // ============ INTERFACE 0: KEYBOARD ============
  /*9 byte*/ 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_KEYBOARD_EP, 0x03, HID_KEYBOARD_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 1: MOUSE ============
  /*9 byte*/ 0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_MOUSE_EP, 0x03, HID_MOUSE_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 2: CONSUMER CONTROL ============
  /*9 byte*/ 0x09, 0x04, 0x02, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_CONSUMER_EP, 0x03, HID_CONSUMER_EP_SIZE, 0x00, 0x10,
};

/* USB HID device Other Speed Configuration Descriptor -----------------------*/
// Так же просто скопирован FS
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  // Configuration Descriptor (9 byte)
  0x09, 0x02, // bLength, bDescriptorType
  LOBYTE(USB_HID_CONFIG_DESC_SIZ), HIBYTE(USB_HID_CONFIG_DESC_SIZ), // wTotalLength = 84
  0x03,       // bNumInterfaces
  0x01,       // bConfigurationValue
  0x00,       // iConfiguration
  0xC0,       // bmAttributes
  0x32,       // MaxPower (100 mA)

  // ============ INTERFACE 0: KEYBOARD ============
  /*9 byte*/ 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(KEYBOARD_REPORT_DESC_SIZE), HIBYTE(KEYBOARD_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_KEYBOARD_EP, 0x03, HID_KEYBOARD_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 1: MOUSE ============
  /*9 byte*/ 0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(MOUSE_REPORT_DESC_SIZE), HIBYTE(MOUSE_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_MOUSE_EP, 0x03, HID_MOUSE_EP_SIZE, 0x00, 0x0A,

  // ============ INTERFACE 2: CONSUMER CONTROL ============
  /*9 byte*/ 0x09, 0x04, 0x02, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
  /*9 byte*/ 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, LOBYTE(CONSUMER_REPORT_DESC_SIZE), HIBYTE(CONSUMER_REPORT_DESC_SIZE),
  /*7 byte*/ 0x07, 0x05, HID_CONSUMER_EP, 0x03, HID_CONSUMER_EP_SIZE, 0x00, 0x10,
};

/* USB HID device Configuration Descriptor -----------------------------------*/
//Не используется на FS-only
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ]  __ALIGN_END  =
{
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
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
//  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

//  if (pdev->dev_state == USBD_STATE_CONFIGURED)
//  {
//    if (hhid->state == HID_IDLE)
//    {
//      hhid->state = HID_BUSY;
//      USBD_LL_Transmit(pdev,
//                       HID_EPIN_ADDR,
//                       report,
//                       len);
//    }
//  }
//  return USBD_OK;
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
//  uint32_t polling_interval = 0U;

//  /* HIGH-speed endpoints */
//  if (pdev->dev_speed == USBD_SPEED_HIGH)
//  {
//    /* Sets the data transfer polling interval for high speed transfers.
//     Values between 1..16 are allowed. Values correspond to interval
//     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
//    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
//  }
//  else   /* LOW and FULL-speed endpoints */
//  {
//    /* Sets the data transfer polling interval for low and full
//    speed transfers */
//    polling_interval =  HID_FS_BINTERVAL;
//  }

//  return ((uint32_t)(polling_interval));
}

/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_CfgFSDesc);
  return USBD_HID_CfgFSDesc;
}

/**
  * @brief  USBD_HID_GetCfgHSDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_CfgHSDesc);
  return USBD_HID_CfgHSDesc;
}

/**
  * @brief  USBD_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_HID_OtherSpeedCfgDesc);
  return USBD_HID_OtherSpeedCfgDesc;
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

