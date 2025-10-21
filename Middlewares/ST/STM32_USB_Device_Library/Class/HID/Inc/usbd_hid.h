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

#define KEYBOARD_CONTROL
//#define MOUSE_CONTROL
//#define CONSUMER_CONTROL
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

#ifdef CUSTOM_CONTROL
#define HID_INTERFACE_CUSTOM 1
#else
#define HID_INTERFACE_CUSTOM 0
#endif

#define HID_INTERFACE_COUNT (HID_INTERFACE_KEYBOARD + HID_INTERFACE_MOUSE + HID_INTERFACE_CONSUMER + HID_INTERFACE_CUSTOM)

#define  HID_MEDIA_REPORT  2
#define HID_LED_SUPPORT 0

#define HID_KEYBOARD_EP        0x81U
#define HID_MOUSE_EP           0x82U
#define HID_CONSUMER_EP        0x83U
#define HID_CUSTOM_EPIN        0x84U
#define HID_CUSTOM_EPOUT       0x01U

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

//Consumer HID Report
typedef struct {
	uint8_t lobyte;
	uint8_t hibyte;
	uint8_t lobyte2;
	uint8_t hibyte2;
} сonsumerHID;

#define HID_KEYBOARD_EP_SIZE   (sizeof(keyboardHID))
#define HID_MOUSE_EP_SIZE      (sizeof(mouseHID))
#define HID_CONSUMER_EP_SIZE   (sizeof(сonsumerHID))
#define HID_MEDIA_EP_SIZE      (sizeof(mediaHID))
#define HID_CUSTOM_EP_SIZE     0x10U

#define USB_HID_CONFIG_DESC_SIZ       9U + 25U //+ 25U //+ 25U //+ 25U // (9+9+7=25)
#define USB_HID_DESC_SIZ              9U

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
