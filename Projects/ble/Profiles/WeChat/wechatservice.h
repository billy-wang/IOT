/*******************************************************************************
  Filename:       wechatService.h
  Revised:        $Date: 2014-04-21 10:36:34 -0700 (Mon, 21 Apr 2014) $
  Revision:       $Revision: 38229 $

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

#ifndef WECHATSERVICE_H
#define WECHATSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
//#include "wechatapp.h"

/*********************************************************************
 * CONSTANTS
 */
 
// WeChat Service UUID
#define WECHAT_SERV_UUID                        0xFEE7

// Characteristics  UUID
#define WECHAT_WRITE_CHAR_UUID                  0xFEC7
#define WECHAT_INDICATE_CHAR_UUID               0xFEC8
#define WECHAT_READ_CHAR_UUID                   0xFEC9

// Wechat Service bit fields
#define WECHAT_SERVICE                   0x00000001

// Callback events
#define WECHAT_INDICATE_ENABLED         1
#define WECHAT_INDICATE_DISABLED        2
#define WECHAT_CMD_NULL									3
#define WECHAT_CMD_AUTH									4
#define WECHAT_CMD_INIT									5
#define WECHAT_CMD_TEST_SENDDAT					6
#define WECHAT_ON_WRITE									7

#define BLE_WECHAT_MAX_DATA_LEN            (ATT_MTU_SIZE- 3)

/*********************************************************************
 * TYPEDEFS
 */

// Wechat Service callback function
typedef void (*wechatServiceCB_t)(uint8 event, uint8 *pValue, uint8 len, uint16 offset);

/*********************************************************************
 * MACROS
 */

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
extern bStatus_t Wechat_AddService(uint32 services);

/*
 * @fn      Wechat_Register
 *
 * @brief   Register a callback function with the BloodPressure Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Wechat_Register(wechatServiceCB_t pfnServiceCB);

/*
 * Wechat_SetParameter - Set a Wechat parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len   - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Wechat_SetParameter(uint8 param, uint8 len, 
                                            void *value);
  
/*
 * Wechat_GetParameter - Get a Wechat parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Wechat_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn          Wechat_Indicate
 *
 * @brief       Send a notification containing a wechat.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 * @param       taskId     - calling task's Id.
 *
 * @return      Success or Failure
 */
extern bStatus_t Wechat_Indicate(uint16 connHandle, 
                                            attHandleValueInd_t *pNoti,
                                            uint8 taskId);

extern void Wechat_on_write(uint8 *pValue, uint8 len, uint16 offset);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* WECHATSERVICE_H */
