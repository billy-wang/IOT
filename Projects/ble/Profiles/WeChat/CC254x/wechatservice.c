/**************************************************************************************************
  Filename:       wechatservice.c
  Revised:        $Date: 2015-03-24 09:19:15 -0700 (Tue, 24 Mar 2015) $
  Revision:       $Revision: 43274 $

  Description:    This file contains the BloodPressure sample service 
                  for use with the BloodPressure   sample application.

 Copyright 2011 - 2015 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "wechatservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of bloodPressure measurement value in attribute array
#define WECHAT_WRITE_POS      					3
#define WECHAT_INDICATE_VALUE_POS       5
#define WECHAT_INDICATE_CONFIG_POS      6
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// WeChat service
CONST uint8 wechatServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(WECHAT_SERV_UUID), HI_UINT16(WECHAT_SERV_UUID)
};

// Wechat write characteristic
CONST uint8 wechatWriteUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(WECHAT_WRITE_CHAR_UUID), HI_UINT16(WECHAT_WRITE_CHAR_UUID)
};

// Wechat indicate characteristic
CONST uint8 wechatIndicateUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(WECHAT_INDICATE_CHAR_UUID), HI_UINT16(WECHAT_INDICATE_CHAR_UUID)
};

// Wechat read characteristic
CONST uint8 wechatReadUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(WECHAT_READ_CHAR_UUID), HI_UINT16(WECHAT_READ_CHAR_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static wechatServiceCB_t wechatServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Wechat Service attribute
static CONST gattAttrType_t wechatService = { ATT_BT_UUID_SIZE, wechatServUUID };

// Wechat Write Characteristic
static uint8 wechatWriteProps = GATT_PROP_WRITE;
static uint8 wechatWrite = 0;
static uint8 wechatWriteUserDesp[17] = "WechatWrite";

// Wechat Indicate Characteristic
static uint8 wechatIndicateProps = GATT_PROP_INDICATE;
static gattCharCfg_t *wechatIndicateConfig;
static uint8 wechatIndicate[20] = { 0 };
static uint8 wechatIndicateUserDesp[17] = "WechatIndicate";


// Wechat Read Characteristic
static uint8  wechateReadProps = GATT_PROP_READ;
static uint8  wechateRead=0;
static uint8 wechatReadUserDesp[17] = "WechatRead";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t wechatAttrTbl[] = 
{
  // Wechat Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&wechatService            /* pValue */
  },

    //////////////////////////////////////////////
    // WECHAT WRITE 
    //////////////////////////////////////////////
    
    // 1. Write Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &wechatWriteProps 
    },

    // 2. Write Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, wechatWriteUUID },
      GATT_PERMIT_WRITE,
      0, 
      &wechatWrite 
    },

    // 3.Write Characteristic User Description
    { 
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ, 
      0, 
      wechatWriteUserDesp
    }, 
 
    //////////////////////////////////////////////
    // WECHAT INDICATE
    //////////////////////////////////////////////
    
    // 4.Indicate Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &wechatIndicateProps 
    },

    // 5.Indicate Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, wechatIndicateUUID },
      0,
      0, 
      wechatIndicate 
    },

    // 6.Indicate Characteristic Configuration
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&wechatIndicateConfig
    },

		// 7.Indicate Characteristic User Description
		{ 
			{ ATT_BT_UUID_SIZE, charUserDescUUID },
			GATT_PERMIT_READ, 
			0, 
			wechatIndicateUserDesp
		}, 

    //////////////////////////////////////////////
    // WECHAT READ
    //////////////////////////////////////////////
    
    // 8.Read Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &wechateReadProps 
    },

    // 9.Read Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, wechatReadUUID },
      GATT_PERMIT_READ,
      0, 
      &wechateRead 
    },
    
		// 10.Read Characteristic User Description
		{ 
			{ ATT_BT_UUID_SIZE, charUserDescUUID },
			GATT_PERMIT_READ, 
			0, 
			wechatReadUserDesp
		}, 
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t wechat_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method);
static bStatus_t wechat_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Wechat Service Callbacks
CONST gattServiceCBs_t wechatCBs =
{
  wechat_ReadAttrCB,  // Read callback function pointer
  wechat_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Wechat_AddService
 *
 * @brief   Initializes the Wechat  service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Wechat_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  wechatIndicateConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                             linkDBNumConns );
  if ( wechatIndicateConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, wechatIndicateConfig );
  
  if ( services & WECHAT_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( wechatAttrTbl, 
                                          GATT_NUM_ATTRS( wechatAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &wechatCBs );  
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      Wechat_Register
 *
 * @brief   Register a callback function with the Wechat Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void Wechat_Register( wechatServiceCB_t pfnServiceCB )
{
  wechatServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      Wechat_SetParameter
 *
 * @brief   Set a thermomter parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Wechat_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
	NPI_Printf("set param %d %d\r\n", param, len);
  switch ( param )
  {
  	case WECHAT_CMD_AUTH:
			if ( len <= 20 ) 
				osal_memcmp(wechatIndicate, value, len);
			else
				ret = bleInvalidRange;
			break;
			
		case WECHAT_CMD_INIT:
			break;
			
		case WECHAT_CMD_TEST_SENDDAT:
			break;
			
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      Wechat_GetParameter
 *
 * @brief   Get a Wechat  parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Wechat_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
	NPI_Printf("get param %d\r\n", param);
  switch ( param )
  {
   	case WECHAT_CMD_AUTH:
			osal_memcmp(value, wechatIndicate, 20);
			break;
		case WECHAT_CMD_INIT:
			break;
		case WECHAT_CMD_TEST_SENDDAT:
			break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          Wechat_Indicate
 *
 * @brief       Send a indication containing a wechat.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t Wechat_Indicate( uint16 connHandle, attHandleValueInd_t *pNoti,uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, wechatIndicateConfig );

  // If indications enabled
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle
    pNoti->handle = wechatAttrTbl[WECHAT_INDICATE_VALUE_POS].handle;
  
    // Send the Indication
    return GATT_Indication( connHandle, pNoti, FALSE, taskId );
  }

  return bleIncorrectMode;
}

/*********************************************************************
 * @fn          wechat_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t wechat_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                          uint8 *pValue, uint8 *pLen, uint16 offset,
                                          uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
	NPI_Printf("Read %d %d %d\r\n", *pLen, pAttr->type.len, offset);

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
  	NPI_Printf("not permissions read\r\n");
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case WECHAT_READ_CHAR_UUID:
        {
#if 0					
					NPI_Printf("Read %d", *pLen);
					for(uint8 i=0; i<*pLen; i++)
					{
						NPI_Printf(" %x", *pLen);
					}
					NPI_Printf("\r\n");
#endif					
					osal_memcpy( pValue, pAttr->pValue, pAttr->type.len );
        }
        break;         
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  return ( status );
}

/*********************************************************************
 * @fn     wechat_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t wechat_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
	
  switch ( uuid )
  {
    case GATT_CLIENT_CHAR_CFG_UUID :
		{
			NPI_Printf("indicate config \r\n");
      if ( pAttr->handle == wechatAttrTbl[WECHAT_INDICATE_CONFIG_POS].handle )
      {
        // BloodPressure Indications
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
          uint8 value = BUILD_UINT16( pValue[0], pValue[1] );
					
					value = (value == GATT_CFG_NO_OPERATION) ? WECHAT_INDICATE_DISABLED : WECHAT_INDICATE_ENABLED ;
          (*wechatServiceCB)( value, NULL, NULL, NULL);
        }
      }
      else
      {
      	NPI_Printf("err: cfg invalid handle\r\n");
        status = ATT_ERR_INVALID_HANDLE;
      }
      break;     
		}

		case WECHAT_WRITE_CHAR_UUID:
		{
			NPI_Printf("write \r\n");
#if 1
			(*wechatServiceCB)( WECHAT_ON_WRITE, pValue, len, offset);
#else
      if ( pAttr->handle == wechatAttrTbl[WECHAT_WRITE_POS].handle )
      {
      	(*wechatServiceCB)( WECHAT_ON_WRITE, pValue, len, offset);
			}
			else
			{
				NPI_Printf("err: invalid handle\r\n");
				status = ATT_ERR_INVALID_HANDLE;
			}
#endif
			break;
		}
		
    default:
			NPI_Printf("err: attr not found!\r\n");
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  
  return ( status );

}


/*********************************************************************
*********************************************************************/
