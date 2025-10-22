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
#define MOUSE_CONTROL
//#define CONSUMER_CONTROL
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
	uint8_t id;
    uint8_t keys1;
	uint8_t keys2;
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
