/******************************************************************************
  Filename:       wechatapp.h
  Revised:        $Date: 2016-04-26 10:36:34 -0700 (Mon, 26 Apr 2016) $
  Revision:       $Revision: 1.0.4 $

  Description:    This file contains the BloodPressure service definitions and
                  prototypes.

 Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#ifndef WECHATAPP_H
#define WECHATAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "epb.h"

/*********************************************************************
 * CONSTANTS
 */
#define DEVICE_KEY {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};


#define ARGS_ITEM_SET(ARGS_TYPE, ARGS_POINTER, ITEM_NAME, ITEM_VALUE)	\
do {	\
	ARGS_TYPE *tmp = (ARGS_TYPE *)ARGS_POINTER;	\
	tmp->ITEM_NAME = ITEM_VALUE;	\
} while(0)

//#define EAM_md5AndNoEnrypt 1     //ÈÏÖ¤·½Ê½Ö»ÄÜ¶¨ÒåÆäÖÐµÄÒ»ÖÖ
//#define EAM_md5AndAesEnrypt 1
#define EAM_macNoEncrypt 2 

#define DEVICE_TYPE "gh_1bafe245c2cb"
#define DEVICE_ID "WeChatBluetoothDevice"

#define CHALLENAGE_LENGTH 4


/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
	unsigned char bMagicNumber;
	unsigned char bVer;
	unsigned short nLength;
	unsigned short nCmdId;
	unsigned short nSeq;
}BpFixHead;

typedef struct
{
	uint8  m_magicCode[2];
	uint16 m_version;
	uint16 m_totalLength;
	uint16 m_cmdid;
	uint16 m_seq;
	uint16 m_errorCode;
}WechatBlueDemoHead;

typedef struct 
{
	int cmd;
	CString send_msg;
}wechat_info;

typedef struct 
{
	bool wechats_switch_state; //¹«ÖÚÕËºÅÇÐ»»µ½Ç°Ì¨µÄ×´Ì¬
	bool indication_state;
	bool auth_state;
	bool init_state;
	bool auth_send;
	bool init_send;
	unsigned short send_data_seq;
	unsigned short push_data_seq;
	unsigned short seq; 
}wechat_state;

typedef struct
{
	uint8 *data;
	uint16 len;
	uint16 offset;
} data_info;


typedef enum
{
	errorCodeUnpackAuthResp = 0x9990,
	errorCodeUnpackInitResp = 0x9991,
	errorCodeUnpackSendDataResp = 0x9992,
	errorCodeUnpackCtlCmdResp = 0x9993,
	errorCodeUnpackRecvDataPush = 0x9994,
	errorCodeUnpackSwitchViewPush = 0x9995,
	errorCodeUnpackSwitchBackgroundPush = 0x9996,
	errorCodeUnpackErrorDecode = 0x9997,
}wechatUnpackErrorCode;

typedef enum
{
	sendTextReq = 0x01,
	sendTextResp = 0x1001,
	openLightPush = 0x2001,
	closeLightPush = 0x2002,
}wechatCmdID;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL
 */

extern uint16 gapConnHandle;

extern wechat_state wechatSta;

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * @fn      Wechat_AddService
 *
 * @brief   Initializes the Wecaht service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
extern uint8 Wechat_Init(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* WECHATAPP_H */
