/**************************************************************************************************
  Filename:       wechatapp.c
  Revised:        $Date: 2016-04-26 08:56:11 -0700 (Fri, 26 Aug 2016) $
  Revision:       $Revision: 1.0.4 $

  Description:    This file contains the wechat sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2014 Texas Instruments Incorporated. All rights reserved.

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
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "Osal_snv.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "npi.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "flybirdBLEPeripheral.h"


#include "linkdb.h"
#include "OSAL_Clock.h"
#include "wechatservice.h"
#include "epb.h"
#include "epb_MmBp.h"
#include "wechatapp.h"
#include "wechat_util.h"
#include "md5.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//#define BP_MEAS_LEN                           19
#define PROTO_VERSION 0x010004
#define AUTH_PROTO 1

#define MAC_ADDRESS_LENGTH 6

#ifdef EAM_macNoEncrypt
	#define AUTH_METHOD EAM_macNoEncrypt
	#define MD5_TYPE_AND_ID_LENGTH 0
	#define CIPHER_TEXT_LENGTH 0
#endif

#ifdef EAM_md5AndAesEnrypt
	#define AUTH_METHOD EAM_md5AndAesEnrypt
	#define MD5_TYPE_AND_ID_LENGTH 16
	#define CIPHER_TEXT_LENGTH 16
#endif

#ifdef EAM_md5AndNoEnrypt
	#define AUTH_METHOD EAM_md5AndNoEnrypt
	#define MD5_TYPE_AND_ID_LENGTH 16
	#define CIPHER_TEXT_LENGTH 0
#endif

// Max wechat storage count
#define WECHAT_AUTH_STORE_MAX                         	4

#define SEND_HELLO_WECHAT "Hello, WeChat!"

#define WECHAT_MAGICCODE_H 0xfe
#define WECHAT_MAGICCODE_L 0xcf
#define WECHAT_VERSION 0x01

/*********************************************************************
 * TYPEDEFS
 */
 
typedef enum
{
	errorCodeProduce = 0x9980,
}wechatPackErrorCode;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task ID for internal task/event processing
//uint8 simpleBLEPeripheral_TaskID;

data_info g_rcv_data = {NULL, 0, 0};

wechat_state wechatSta = {false, false, false, false, false, false,0,0,0};

uint8 challeange[CHALLENAGE_LENGTH] = {0x11,0x22,0x33,0x44}; //ÎªÁË·½±ãÕâÀïÌîÁËÒ»×éÈ·¶¨µÄÊý×é£¬Ê¹ÓÃ¹ý³ÌÖÐÇë×ÔÐÐÉú³ÉËæ»úÊý

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 simpleBLEPeripheral_TaskID;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void* memcpy(void *dest, const void *src, size_t len);
extern char *bdAddr2Str( uint8 *pAddr );

extern uint32 crc32(uint32 crc, const uint8 *buf, int len);



/*********************************************************************
 * LOCAL VARIABLES
 */
//static gaprole_States_t gapProfileState = GAPROLE_INIT;
wechat_info wechatinfo = {WECHAT_CMD_NULL, {NULL, 0}};

#if (defined EAM_md5AndNoEnrypt) || (defined EAM_md5AndAesEnrypt)
	uint8 md5_type_and_id[16];
#endif

const uint8 key[16] = DEVICE_KEY
uint8 session_key[16] = {0};

static attHandleValueInd_t wechatStoreAuth[WECHAT_AUTH_STORE_MAX];
static uint8 wechatStoreStartIndex = 0;
static uint8 wechatStoreIndex = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static void Wechat_state_reset(void);
static uint16 Wechat_get_md5(void);
static int8 Wechat_data_produce(void *args, uint8 **r_data, uint32 *r_len);

static void wechat_error_chack(int error_code);
int wechat_data_consume(data_info g_rcv_data);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Callbacks
//static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
//{
//  simpleProfileChangeCB    // Charactersitic value change callback
//};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


static void Wechat_state_reset(void)
{
	wechatSta.auth_send = false;
	wechatSta.auth_state = false;
	wechatSta.indication_state = false;
	wechatSta.init_send = false;
	wechatSta.init_state = false;
	wechatSta.send_data_seq = 0;
	wechatSta.push_data_seq = 0;
	wechatSta.seq = 0;
}

/**@brief   Function for the light initialization.
 *
 * @details Initializes all lights used by this application.
 */

static uint16 Wechat_get_md5(void)
{
 	uint8 stat = SUCCESS;

	#if (defined EAM_md5AndNoEnrypt ) || ( defined EAM_md5AndAesEnrypt )
	char device_type[] = DEVICE_TYPE;
	char device_id[] = DEVICE_ID;
	char argv[sizeof(DEVICE_TYPE) + sizeof(DEVICE_ID) - 1];
	
	memcpy(argv,device_type,sizeof(DEVICE_TYPE));
	/*when add the DEVICE_ID to DEVICE_TYPE, the offset shuld -1 to overwrite '\0'  at the end of DEVICE_TYPE */
	memcpy(argv + sizeof(DEVICE_TYPE)-1,device_id,sizeof(DEVICE_ID));
	
	
	stat = md5(argv, md5_type_and_id);

	NPI_Printf( "\r\nDEVICE_TYPE and DEVICE_ID: %s\r\n",argv);
	NPI_PrintString ( "\r\nMD5:");
	for ( uint8 i = 0; i < 16; i++ )
		NPI_Printf ( " %02x", md5_type_and_id[i] );
	NPI_PrintString( "\r\n" );

	#endif

	return stat;
}

/*********************************************************************
 * @fn      Wechat_Init
 *
 * @brief   Initialization function for the BLE wechat App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   none
 *
 * @return  none
 */
uint8 Wechat_Init( void )
{
 	uint8 stat = SUCCESS;
  Wechat_state_reset();
	stat = Wechat_get_md5();
	return stat;
}



/**@brief Function for error handling, which is called when an error has occurred. 
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] p_data_handler where Error come from
 */
static void wechat_error_chack(int error_code)
{
	switch(error_code)
	{
		case EEC_system:
			NPI_Printf("error: system error\r\n");
			break ;
		case EEC_needAuth:
			NPI_Printf("error: needAuth\r\n");
			break ;
		case EEC_sessionTimeout:
			NPI_Printf("error: sessionTimeout\r\n");
			break ;
		case EEC_decode:
			NPI_Printf("error: decode\r\n");
			break ;
		case EEC_deviceIsBlock:
			NPI_Printf("error: deviceIsBlock\r\n");
			break ;
		case EEC_serviceUnAvalibleInBackground:
			NPI_Printf("error: serviceUnAvalibleInBackground\r\n");
			break ;
		case EEC_deviceProtoVersionNeedUpdate:
			NPI_Printf("error: deviceProtoVersionNeedUpdate\r\n");
			break ;
		case EEC_phoneProtoVersionNeedUpdate:
			NPI_Printf("error: phoneProtoVersionNeedUpdate\r\n");
			break ;
		case EEC_maxReqInQueue:
			NPI_Printf("error: maxReqInQueue\r\n");
			break ;
		case EEC_userExitWxAccount:
			NPI_Printf("error: userExitWxAccount\r\n");
			break ;
		default:
			break ;
	}
	//please reset  ble device
}

/*********************************************************************
* @fn 		 Wechat_device_write
*
* @brief	 Perform a periodic application task. This function gets
* 				 called every five seconds as a result of the SBP_PERIODIC_EVT
* 				 OSAL event. In this example, the value of the third
* 				 characteristic in the SimpleGATTProfile service is retrieved
* 				 from the profile, and then copied into the value of the
* 				 the fourth characteristic.
*
* @param	 none
*
* @return  none
*/
void Wechat_on_write(uint8 *pValue, uint8 len, uint16 offset)
{
	int error_code;	
	int chunk_size = 0;

	//if ( len <= BLE_WECHAT_MAX_DATA_LEN ) 
	{
		if (g_rcv_data.len == 0) 
		{
			BpFixHead *fix_head = (BpFixHead *) pValue;
			g_rcv_data.len = ntohs(fix_head->nLength);
			g_rcv_data.offset = 0;
			g_rcv_data.data = (uint8 *)osal_mem_alloc(g_rcv_data.len);
		} 

		chunk_size = g_rcv_data.len - g_rcv_data.offset;
		chunk_size = chunk_size < len ? chunk_size : len;
		memcpy(g_rcv_data.data+g_rcv_data.offset, pValue, chunk_size);
		g_rcv_data.offset += chunk_size;
		if (g_rcv_data.len <= g_rcv_data.offset) 
		{
			error_code = wechat_data_consume(g_rcv_data);
			osal_mem_free(g_rcv_data.data);
			g_rcv_data.data=0;		
			g_rcv_data.len = 0;
			g_rcv_data.offset = 0;
			wechat_error_chack(error_code);
			//error_code = p_data_handler->m_data_consume_func(g_rcv_data.data, g_rcv_data.len);
			//p_data_handler->m_data_free_func(g_rcv_data.data,g_rcv_data.len);
			//wechat_error_chack(p_wcs, p_data_handler, error_code);
			//g_rcv_data.len = 0;
			//g_rcv_data.offset = 0;
		} 
	}
}

int wechat_data_consume(data_info g_rcv_data)
{
	BpFixHead *fix_head = (BpFixHead *)g_rcv_data.data;
	uint8 fix_head_len = sizeof(BpFixHead);

	NPI_Printf("##Received data: ");
	for(uint8 i=0;i<g_rcv_data.len;++i)
	{
		NPI_Printf(" %x",g_rcv_data.data[i]);
	}
	NPI_Printf("\r\n CMDID: %d \r\n", ntohs(fix_head->nCmdId));
	NPI_Printf("len: %d \r\n", ntohs(fix_head->nLength));
	NPI_Printf("Seq: %d \r\n",ntohs(fix_head->nSeq));

	switch(ntohs(fix_head->nCmdId))
	{
		case ECI_none:
			break;

		case ECI_resp_auth:
		{
			AuthResponse* authResp;
			authResp = epb_unpack_auth_response(g_rcv_data.data+fix_head_len,g_rcv_data.len-fix_head_len);
			NPI_Printf("\r\n@@Received 'authResp'\r\n");

			if(!authResp)
			{
				return errorCodeUnpackAuthResp;
			}
			NPI_Printf("\r\n unpack 'authResp' success!\r\n");

			if(authResp->base_response)
			{
				if(authResp->base_response->err_code == 0)
				{
							wechatSta.auth_state = true;
				}
				else
				{
					NPI_Printf("error code:%d \r\n",authResp->base_response->err_code);
					if(authResp->base_response->has_err_msg)
					{
						NPI_Printf("base_response error msg:%s\r\n",authResp->base_response->err_msg.str);	
					}

					epb_unpack_auth_response_free(authResp);
					return authResp->base_response->err_code;
				}
			}

			#if defined EAM_md5AndAesEnrypt// get sessionkey
			if(authResp->aes_session_key.len)
			{
				NPI_Printf("session_key:\r\n");
				//AES_Init(key);
				//AES_Decrypt(session_key,authResp->aes_session_key.data,authResp->aes_session_key.len,key);
				decrypt_data(authResp->aes_session_key.data, authResp->aes_session_key.len, session_key, key);

			}
			#endif

			epb_unpack_auth_response_free(authResp);
			break;
		}
		case ECI_resp_sendData:
		{
			NPI_Printf("\r\n@@Received 'sendDataResp'\r\n");

			#if defined EAM_md5AndAesEnrypt		
				uint32 length = g_rcv_data.len- fix_head_len;//¼ÓÃÜºóÊý¾Ý³¤¶È
				uint8 *p = osal_mem_alloc (length);
				if(!p)
				{
					NPI_Printf("Not enough memory! \r\n"); 
					if(g_rcv_data.data)
						osal_mem_free(g_rcv_data.data);
					g_rcv_data.data = NULL; 
					return 0;
				}

				//AES_Init(session_key);
				//AES_Decrypt(p,g_rcv_data.data+fix_head_len,g_rcv_data.len- fix_head_len,session_key);
				decrypt_data(g_rcv_data.data+fix_head_len, length, p, session_key);
					
				uint8 temp;
				temp = p[length - 1];//Ëã³öÌî³ä³¤¶È
				g_rcv_data.len = g_rcv_data.len - temp;//È¡¼ÓÃÜÇ°Êý¾Ý×Ü³¤¶È
				memcpy(g_rcv_data.data + fix_head_len, p ,length -temp);//°ÑÃ÷ÎÄ·Å»Ø
				if(p)
				{
					osal_mem_free(p);
					p = NULL;
				}
			#endif
			
			SendDataResponse *sendDataResp;
			sendDataResp = epb_unpack_send_data_response(g_rcv_data.data+fix_head_len,g_rcv_data.len-fix_head_len);
			if(!sendDataResp)
			{
				return errorCodeUnpackSendDataResp;
			}

			WechatBlueDemoHead *bledemohead = (WechatBlueDemoHead*)sendDataResp->data.data;
			if(ntohs(bledemohead->m_cmdid) == sendTextResp)
			{
				NPI_Printf("\r\n received msg: %s\r\n",sendDataResp->data.data+sizeof(WechatBlueDemoHead));
			}

			if(sendDataResp->base_response->err_code)
			{
				epb_unpack_send_data_response_free(sendDataResp);
				return sendDataResp->base_response->err_code;
			}

			epb_unpack_send_data_response_free(sendDataResp);
			break;
		}
		case ECI_resp_init:
		{
			NPI_Printf("\r\n@@Received 'initResp'\r\n");
			#if defined EAM_md5AndAesEnrypt		
			uint32 length = g_rcv_data.len- fix_head_len;//¼ÓÃÜºóÊý¾Ý³¤¶È
			uint8 *p = osal_mem_alloc (length);
			if(!p)
			{
				NPI_Printf("\r\nNot enough memory!");
				if(g_rcv_data.data)
					osal_mem_free(g_rcv_data.data);
				g_rcv_data.data = NULL; 
				return 0;
			}

			//AES_Init(session_key);
			//AES_Decrypt(p,g_rcv_data.data+fix_head_len,g_rcv_data.len- fix_head_len,session_key);
			decrypt_data(g_rcv_data.data+fix_head_len, length, p, session_key);
					
			uint8 temp;
			temp = p[length - 1];//Ëã³öÌî³ä³¤¶È
			g_rcv_data.len = g_rcv_data.len - temp;//È¡¼ÓÃÜÇ°Êý¾Ý×Ü³¤¶È
			memcpy(g_rcv_data.data + fix_head_len, p ,length -temp);//°ÑÃ÷ÎÄ·Å»Ø
			if(p)
			{
				osal_mem_free(p);
				p = NULL;
			}
			#endif
			
			InitResponse *initResp = epb_unpack_init_response(g_rcv_data.data+fix_head_len, g_rcv_data.len-fix_head_len);
			if(!initResp)
			{
				return errorCodeUnpackInitResp;
			}

			NPI_Printf("\r\n unpack 'initResp' success!");
			if(initResp->base_response)
			{
				if(initResp->base_response->err_code == 0)
				{
					if(initResp->has_challeange_answer)
					{
						if(crc32(0,challeange,CHALLENAGE_LENGTH) == initResp->challeange_answer)
						{
							wechatSta.init_state = true;
						}
					}
					else 
						wechatSta.init_state = true;

					wechatSta.wechats_switch_state = true;
				}
				else
				{
					NPI_Printf("\r\n error code:%d",initResp->base_response->err_code);

					if(initResp->base_response->has_err_msg)
					{
						NPI_Printf("\r\n base_response error msg:%s",initResp->base_response->err_msg.str);
					}

					epb_unpack_init_response_free(initResp);
					return initResp->base_response->err_code;
				}
			}

			epb_unpack_init_response_free(initResp);
			break;
		}
		case ECI_push_recvData:
		{
			#if defined EAM_md5AndAesEnrypt
			uint32 length = g_rcv_data.len- fix_head_len;//¼ÓÃÜºóÊý¾Ý³¤¶È
			uint8 *p = osal_mem_alloc (length);
			if(!p)
			{
				NPI_Printf("\r\nNot enough memory!");
				if(g_rcv_data.data)
					osal_mem_free(g_rcv_data.data); 
				g_rcv_data.data =NULL; 
				return 0;
			}

			//AES_Init(session_key);
			//AES_Decrypt(p,g_rcv_data.data+fix_head_len,g_rcv_data.len- fix_head_len,session_key);
			decrypt_data(g_rcv_data.data+fix_head_len, length, p, session_key);
					
			uint8 temp;
			temp = p[length - 1];//Ëã³öÌî³ä³¤¶È
			g_rcv_data.len = g_rcv_data.len - temp;//È¡¼ÓÃÜÇ°Êý¾Ý×Ü³¤¶È
			memcpy(g_rcv_data.data + fix_head_len, p ,length -temp);//°ÑÃ÷ÎÄ·Å»Ø
			if(p)
			{
				osal_mem_free(p);
				p = NULL;
			}
			#endif
			
			RecvDataPush *recvDatPush;
			recvDatPush = epb_unpack_recv_data_push(g_rcv_data.data+fix_head_len, g_rcv_data.len-fix_head_len);

			NPI_Printf("\r\n@@Received 'recvDataPush'\r\n");

			if(!recvDatPush)
			{
				return errorCodeUnpackRecvDataPush;
			}

			NPI_Printf("\r\n unpack the 'recvDataPush' successfully! \r\n");
			if(recvDatPush->base_push == NULL)
			{
				NPI_Printf("\r\n recvDatPush->base_push is NULL! \r\n");
			}
			else 
			{
				NPI_Printf("\r\n recvDatPush->base_push is not NULL! \r\n");
			}
			NPI_Printf("\r\n recvDatPush->data.len: %x \r\n",recvDatPush->data.len);
			NPI_Printf("\r\n recvDatPush->data.data:  \r\n");
			
			const uint8 *d = recvDatPush->data.data;
			for(uint8 i=0;i<recvDatPush->data.len;++i)
			{
				NPI_Printf(" %x",d[i]);
			}
			if(recvDatPush->has_type)
			{
				NPI_Printf("\r\n recvDatPush has type! \r\n");
				NPI_Printf("\r\n type: %d\r\n",recvDatPush->type);
			}

			WechatBlueDemoHead *bledemohead = (WechatBlueDemoHead*)recvDatPush->data.data;
			NPI_Printf("\r\n magicCode: %x",bledemohead->m_magicCode[0]);
			NPI_Printf(" %x",bledemohead->m_magicCode[1]);
			NPI_Printf("\r\n version: %x",ntohs(bledemohead->m_version));
			NPI_Printf("\r\n totalLength: %x",ntohs(bledemohead->m_totalLength));
			NPI_Printf("\r\n cmdid: %x",ntohs(bledemohead->m_cmdid ));
			NPI_Printf("\r\n errorCode: %x",ntohs(bledemohead->m_errorCode));

			if(ntohs(bledemohead->m_cmdid ) == openLightPush)
			{
				NPI_Printf("\r\n light on!! ");				
				HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
			}
			else if(ntohs(bledemohead->m_cmdid )  == closeLightPush)
			{
					NPI_Printf("\r\n light off!! ");
					HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
			}
			epb_unpack_recv_data_push_free(recvDatPush);
			wechatSta.push_data_seq++;
			break;
		}
		case ECI_push_switchView:
		{
			wechatSta.wechats_switch_state = !wechatSta.wechats_switch_state;
			NPI_Printf("\r\n@@Received 'switchViewPush'\r\n");

			#if defined EAM_md5AndAesEnrypt		
			uint32 length = g_rcv_data.len- fix_head_len;//¼ÓÃÜºóÊý¾Ý³¤¶È
			uint8 *p = osal_mem_alloc (length);
			if(!p)
			{
				NPI_Printf("\r\nNot enough memory!");
				if(g_rcv_data.data)
					osal_mem_free(g_rcv_data.data);
				g_rcv_data.data = NULL; 
				return 0;
			}

			//AES_Init(session_key);
			//AES_Decrypt(p,g_rcv_data.data+fix_head_len,g_rcv_data.len- fix_head_len,session_key);
			decrypt_data(g_rcv_data.data+fix_head_len, length, p, session_key);
			
			uint8 temp;
			temp = p[length - 1];//Ëã³öÌî³ä³¤¶È
			g_rcv_data.len = g_rcv_data.len - temp;//È¡¼ÓÃÜÇ°Êý¾Ý×Ü³¤¶È
			memcpy(g_rcv_data.data + fix_head_len, p ,length -temp);//°ÑÃ÷ÎÄ·Å»Ø
			if(p)
			{
				osal_mem_free(p);
				p = NULL;
			}
			#endif	
			
			SwitchViewPush *swichViewPush;
			swichViewPush = epb_unpack_switch_view_push(g_rcv_data.data+fix_head_len,g_rcv_data.len-fix_head_len);
			if(!swichViewPush)
			{
				return errorCodeUnpackSwitchViewPush;
			}

			epb_unpack_switch_view_push_free(swichViewPush);
			break;
		}
		case ECI_push_switchBackgroud:
		{
			NPI_Printf("\r\n@@Received 'switchBackgroudPush'\r\n");
			#if defined EAM_md5AndAesEnrypt
			uint32 length = g_rcv_data.len- fix_head_len;//¼ÓÃÜºóÊý¾Ý³¤¶È
			uint8 *p = osal_mem_alloc (length);
			if(!p)
			{
				NPI_Printf("\r\nNot enough memory!");
				if(g_rcv_data.data)
					osal_mem_free(g_rcv_data.data);
				g_rcv_data.data = NULL;  
				return 0;
			}

			//AES_Init(session_key);
			//AES_Decrypt(p,g_rcv_data.data+fix_head_len,g_rcv_data.len- fix_head_len,session_key);
			decrypt_data(g_rcv_data.data+fix_head_len, length, p, session_key);

			uint8 temp;
			temp = p[length - 1];//Ëã³öÌî³ä³¤¶È
			g_rcv_data.len = g_rcv_data.len - temp;//È¡¼ÓÃÜÇ°Êý¾Ý×Ü³¤¶È
			memcpy(g_rcv_data.data + fix_head_len, p ,length -temp);//°ÑÃ÷ÎÄ·Å»Ø
			if(g_rcv_data.data)
			{
				osal_mem_free(p);
				p = NULL;
			}
			#endif
			
			SwitchBackgroudPush *switchBackgroundPush = epb_unpack_switch_backgroud_push(g_rcv_data.data+fix_head_len,g_rcv_data.len-fix_head_len);
			if(! switchBackgroundPush)
			{
				return errorCodeUnpackSwitchBackgroundPush;
			}	

			epb_unpack_switch_backgroud_push_free(switchBackgroundPush);
			break;
		}
		case ECI_err_decode:
				break;

		default:
			NPI_Printf("\r\n !!ERROR CMDID:%d",ntohs(fix_head->nCmdId));
			break;
	}
	return 0;
}

int8 Wechat_data_produce(void *args, uint8 **r_data, uint32 *r_len)
{
	static uint16 bleDemoHeadLen = sizeof(WechatBlueDemoHead);
	wechat_info *info = (wechat_info *)args;
	BaseRequest basReq = {NULL};
	static uint8 fix_head_len = sizeof(BpFixHead);
	BpFixHead fix_head = {0xFE, 1, 0, htons(ECI_req_auth), 0};
	wechatSta.seq++;
	int ret;

	switch (info->cmd)
	{
		case WECHAT_CMD_AUTH:
		{
			#if defined EAM_md5AndAesEnrypt
			uint8 deviceid[] = DEVICE_ID;
			static uint32 seq = 0x00000001;//
			uint32 ran = 0x11223344;//ÎªÁË·½±ãÆð¼ûÕâÀï·ÅÁËÒ»¸ö¹Ì¶¨Öµ×öÎªËæ»úÊý£¬ÔÚÊ¹ÓÃÊ±Çë×ÔÐÐÉú³ÉËæ»úÊý¡£
			ran = t_htonl(ran);
			seq = t_htonl(seq);
			uint8 id_len = osal_strlen(DEVICE_ID);
			uint8* data = osal_mem_alloc(id_len+8);
			if(!data)
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}

			memcpy(data,deviceid,id_len);
			memcpy(data+id_len,(uint8*)&ran,4);
			memcpy(data+id_len+4,(uint8*)&seq,4);
			uint32 crc = crc32(0, data, id_len+8);
			crc = t_htonl(crc);
			osal_memset(data,0x00,id_len+8);
			memcpy(data,(uint8*)&ran,4);
			memcpy(data+4,(uint8*)&seq,4);
			memcpy(data+8,(uint8*)&crc,4);	
			//uint8 CipherText[16];
			//AES_Init(key);
			//AES_Encrypt_PKCS7 (data, CipherText, 12, key);
			uint8 CipherText[128];
			encrypt_data(data, CipherText, key);
			if(data)
			{
				osal_mem_free(data);
				data = NULL;
			}
			AuthRequest authReq = {
				&basReq, 
				true,
				{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, 
				PROTO_VERSION, 
				AUTH_PROTO, 
				(EmAuthMethod)AUTH_METHOD, 
				true ,
				{CipherText, CIPHER_TEXT_LENGTH}, 
				false, 
				{NULL, 0}, 
				false, 
				{NULL, 0}, 
				false, 
				{NULL, 0},
				true,
				{DEVICE_ID,sizeof(DEVICE_ID)}
			};
			seq++;
			#endif
				
			#if defined EAM_macNoEncrypt
			static uint8 mac_address[MAC_ADDRESS_LENGTH];
			//get_mac_addr(mac_address);
			bdAddr2Str( mac_address );
			AuthRequest authReq = {
				&basReq, 
				false,
				{NULL, 0}, 
				PROTO_VERSION, 
				AUTH_PROTO, 
				(EmAuthMethod)AUTH_METHOD, 
				false,
				{NULL, 0}, 
				true, 
				{mac_address, MAC_ADDRESS_LENGTH}, 
				false, 
				{NULL, 0}, 
				false, 
				{NULL, 0},
				true,
				{DEVICE_ID,sizeof(DEVICE_ID)}
			};
			#endif
				
			#if defined EAM_md5AndNoEnrypt
				AuthRequest authReq = {
					&basReq, 
					true,
					{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, 
					PROTO_VERSION, 
					(EmAuthMethod)AUTH_PROTO, 
					(EmAuthMethod)AUTH_METHOD, 
					false ,
					{NULL, 0}, 
					false, 
					{NULL, 0}, 
					false, 
					{NULL, 0}, 
					false, 
					{NULL, 0},
					true,
					{DEVICE_ID,sizeof(DEVICE_ID)}
				};
			#endif

			*r_len = epb_auth_request_pack_size(&authReq) + fix_head_len;
			*r_data = (uint8 *)osal_mem_alloc(*r_len);
			if(!(*r_data))
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}
			ret = epb_pack_auth_request(&authReq, *r_data+fix_head_len, *r_len-fix_head_len);
			if( ret <0)
			{
				osal_mem_free(*r_data);
				*r_data = NULL;
				NPI_Printf("epb pack auth request failed!\r\n");
				return ret;
			}

			fix_head.nCmdId = htons(ECI_req_auth);
			fix_head.nLength = htons(*r_len);
			fix_head.nSeq = htons(wechatSta.seq);
			memcpy(*r_data, &fix_head, fix_head_len);
			break;
		}

		case WECHAT_CMD_INIT:
		{
			//has challeange
			InitRequest initReq = {&basReq,false, {NULL, 0},true, {challeange, CHALLENAGE_LENGTH}};
			*r_len = epb_init_request_pack_size(&initReq) + fix_head_len;
			#if defined EAM_md5AndAesEnrypt
			uint8 length = *r_len;				
			uint8 *p = osal_mem_alloc(AES_get_length( *r_len-fix_head_len));
			if(!p)
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}
			*r_len = AES_get_length( *r_len-fix_head_len)+fix_head_len;
			#endif
			
			//pack data
			*r_data = (uint8 *)osal_mem_alloc(*r_len);
			if(!(*r_data))
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}

			ret = epb_pack_init_request(&initReq, *r_data+fix_head_len, *r_len-fix_head_len);
			if( ret < 0 )
			{
				osal_mem_free(*r_data);
				*r_data = NULL;
				return ret;
			}

			//encrypt body
			#if defined EAM_md5AndAesEnrypt
			//AES_Init(session_key);
			//AES_Encrypt_PKCS7(*r_data+fix_head_len,p,length-fix_head_len,session_key);//Ô­Ê¼Êý¾Ý³¤¶È
			//uint8 CipherText[128];
			encrypt_data(*r_data+fix_head_len,p, session_key);

			memcpy(*r_data + fix_head_len, p, *r_len-fix_head_len);
			if(p)
				osal_mem_free(p);
			#endif
			
			fix_head.nCmdId = htons(ECI_req_init);
			fix_head.nLength = htons(*r_len);
			fix_head.nSeq = htons(wechatSta.seq);
			memcpy(*r_data, &fix_head, fix_head_len);
			break;
		}
		case WECHAT_CMD_TEST_SENDDAT:
		{
			NPI_Printf("test msg to send : %s \r\n",(uint8*)info->send_msg.str);
			
			WechatBlueDemoHead  *bleDemoHead = (WechatBlueDemoHead*)osal_mem_alloc(bleDemoHeadLen+info->send_msg.len);
			if(!bleDemoHead)
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}

			bleDemoHead->m_magicCode[0] = WECHAT_MAGICCODE_H;
			bleDemoHead->m_magicCode[1] = WECHAT_MAGICCODE_L;
			bleDemoHead->m_version = htons( WECHAT_VERSION);
			bleDemoHead->m_totalLength = htons(bleDemoHeadLen + info->send_msg.len);
			bleDemoHead->m_cmdid = htons(sendTextReq);
			bleDemoHead->m_seq = htons(wechatSta.seq);
			bleDemoHead->m_errorCode = 0;	

			/*connect body and head.*/
			/*turn to uint8* befort offset.*/
			memcpy((uint8*)bleDemoHead+bleDemoHeadLen, info->send_msg.str, info->send_msg.len);			
			SendDataRequest sendDatReq = {&basReq, {(uint8*) bleDemoHead, (bleDemoHeadLen + info->send_msg.len)}, false, (EmDeviceDataType)NULL};
			*r_len = epb_send_data_request_pack_size(&sendDatReq) + fix_head_len;

			#if defined EAM_md5AndAesEnrypt
			uint16 length = *r_len;
			uint8 *p = osal_mem_alloc(AES_get_length( *r_len-fix_head_len));
			if(!p)
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}
			*r_len = AES_get_length( *r_len-fix_head_len)+fix_head_len;
			#endif
			
			*r_data = (uint8 *)osal_mem_alloc(*r_len);
			if(!(*r_data))
			{
				NPI_Printf("Not enough memory!\r\n");
				return ( bleMemAllocError );
			}

			ret = epb_pack_send_data_request(&sendDatReq, *r_data+fix_head_len, *r_len-fix_head_len);
			if( ret < 0 )
			{
				*r_data = NULL;

				#if defined EAM_md5AndAesEnrypt
				if(p)
				{
					osal_mem_free(p);
					p = NULL;
				}
				#endif

				NPI_Printf("\r\nepb_pack_send_data_request error!");
				return ret;
			}

			#if defined EAM_md5AndAesEnrypt
			//encrypt body
			//AES_Init(session_key);
			//AES_Encrypt_PKCS7(*r_data+fix_head_len,p,length-fix_head_len,session_key);//Ô­Ê¼Êý¾Ý³¤¶È
			encrypt_data(*r_data+fix_head_len,p, session_key);
			memcpy(*r_data + fix_head_len, p, *r_len-fix_head_len);
			if(p)
			{
				osal_mem_free(p); 
				p = NULL;
			}
			#endif
			fix_head.nCmdId = htons(ECI_req_sendData);
			fix_head.nLength = htons(*r_len);
			fix_head.nSeq = htons(wechatSta.seq);
			memcpy(*r_data, &fix_head, fix_head_len);
			if(bleDemoHead)
			{
				osal_mem_free(bleDemoHead);
				bleDemoHead = NULL;
			}

#if 0
			NPI_Printf("##send data: ");
			uint8 *d = *r_data;
			for(uint8 i=0;i<*r_len;++i)
			{
				NPI_Printf(" %x",d[i]);
			}
			BpFixHead *fix_head = (BpFixHead *)*r_data;
			NPI_Printf("\r\n CMDID: %d\r\n",ntohs(fix_head->nCmdId));
			NPI_Printf("len: %d\r\n", ntohs(fix_head->nLength ));
			NPI_Printf("Seq: %d\r\n", ntohs(fix_head->nSeq));
#endif
			wechatSta.send_data_seq++;
			break;
		}
	}	
	return	SUCCESS;
}

/*********************************************************************
 * @fn      WechatSendStoredAuth
 *
 * @brief   Send a stored measurement indication. An incoming indication
 *          confirmation will trigger the next pending stored measurement.
 *
 * @return  none
 */
static void WechatSendStoredAuth(void)
{
  // We connected to this peer before so send any stored measurements
  if (wechatStoreStartIndex != wechatStoreIndex)
  {
    attHandleValueInd_t *pStoreInd = &wechatStoreAuth[wechatStoreStartIndex];

    // Send wechat auth - can fail if not connected or CCC not enabled
    bStatus_t status = Wechat_Indicate( gapConnHandle, pStoreInd,
                                                   simpleBLEPeripheral_TaskID );
    // If sucess, increment the counters and the indication confirmation
    // will trigger the next indication if there are more pending.
    if (status == SUCCESS)
    {
      wechatStoreStartIndex = wechatStoreStartIndex + 1;

      // Wrap around buffer
      if (wechatStoreStartIndex > WECHAT_AUTH_STORE_MAX)
      {
        wechatStoreStartIndex = 0;
      }

      // Clear out this Meas indication.
      VOID osal_memset( pStoreInd, 0, sizeof(attHandleValueInd_t) );

			NPI_Printf("WechatSendStoredAuth StartIndex %d indicate is seccess\r\n", wechatStoreStartIndex-1);
    }
		else
		{
			NPI_Printf("wechatSendStoredAuth StartIndex %d indicate is failed\r\n", wechatStoreStartIndex);
		}
  }
}

/*********************************************************************
 * @fn      wechatStoreIndications
 *
 * @brief   Queue indications
 *
 * @return  none
 */
static void wechatStoreIndications(attHandleValueInd_t *pInd)
{
  attHandleValueInd_t *pStoreInd = &wechatStoreAuth[wechatStoreIndex];

  if (pStoreInd->pValue != NULL)
  {
    // Free old indication's payload.
    GATT_bm_free((gattMsg_t *)pStoreInd, ATT_HANDLE_VALUE_IND);
  }

  // Store measurement
  VOID osal_memcpy(&wechatStoreAuth[wechatStoreIndex], pInd, sizeof(attHandleValueInd_t));

  // Store index
  wechatStoreIndex = wechatStoreIndex + 1;
  if (wechatStoreIndex > WECHAT_AUTH_STORE_MAX)
  {
    wechatStoreIndex = 0;
  }

  if (wechatStoreIndex == wechatStoreStartIndex)
  {
    wechatStoreStartIndex = wechatStoreStartIndex + 1;
    if(wechatStoreStartIndex > WECHAT_AUTH_STORE_MAX)
    {
      wechatStoreStartIndex = 0;
    }
  }
}

 
/*********************************************************************
* @fn 		 Wechat_device_auth
*
* @brief	 Perform a periodic application task. This function gets
* 				 called every five seconds as a result of the SBP_PERIODIC_EVT
* 				 OSAL event. In this example, the value of the third
* 				 characteristic in the SimpleGATTProfile service is retrieved
* 				 from the profile, and then copied into the value of the
* 				 the fourth characteristic.
*
* @param	 none
*
* @return  none
*/
int32 Wechat_device_auth(void)
{ 
	//if (m_mpbledemo2_handler == NULL) {
	//			m_mpbledemo2_handler = get_handler_by_type(PRODUCT_TYPE_MPBLEDEMO2);
	//		}

	// BloodPressure measurement value stored in this structure.
  attHandleValueInd_t  wechatauth;

  wechatauth.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_IND,
                                           BLE_WECHAT_MAX_DATA_LEN, NULL);
  if (wechatauth.pValue != NULL)
  {
		wechatauth.pValue = NULL;
		wechatauth.len = 0;
				
		ARGS_ITEM_SET(wechat_info, &wechatinfo, cmd, WECHAT_CMD_AUTH); 
		Wechat_data_produce(&wechatinfo, &wechatauth.pValue, (uint32 *)&wechatauth.len);
		if(wechatauth.pValue == NULL)
		{
			NPI_Printf("wechat Pack Error Code\r\n");
			return (errorCodeProduce);
		}

		//sent data	
		NPI_Printf("\r\n##send %d auth data: ", wechatauth.len);
		for(uint8 i=0;i<wechatauth.len;++i)
		{
			NPI_Printf(" %x",wechatauth.pValue[i]);
		}
		NPI_Printf("\r\n");
				
		BpFixHead *fix_head = (BpFixHead *)wechatauth.pValue;
		NPI_Printf("CMDID: %d \r\n", ntohs(fix_head->nCmdId));
		NPI_Printf("len: %d \r\n", ntohs(fix_head->nLength ));
		NPI_Printf("Seq: %d \r\n", ntohs(fix_head->nSeq));

		//ble_wechat_indicate_data(p_wcs, m_mpbledemo2_handler, data, len);
		//m_mpbledemo2_handler->m_data_free_func(data,len);
		wechatStoreIndications(&wechatauth);
		WechatSendStoredAuth();
  }
	
	return 0;
}


 
/*********************************************************************
* @fn 		 Wechat_device_init
*
* @brief	 Perform a periodic application task. This function gets
* 				 called every five seconds as a result of the SBP_PERIODIC_EVT
* 				 OSAL event. In this example, the value of the third
* 				 characteristic in the SimpleGATTProfile service is retrieved
* 				 from the profile, and then copied into the value of the
* 				 the fourth characteristic.
*
* @param	 none
*
* @return  none
*/
int32 Wechat_device_init(void)
{ 
	//if (m_mpbledemo2_handler == NULL) {
	//			m_mpbledemo2_handler = get_handler_by_type(PRODUCT_TYPE_MPBLEDEMO2);
	//		}

	// BloodPressure measurement value stored in this structure.
  attHandleValueInd_t  wechatinit;

  wechatinit.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_IND,
                                           BLE_WECHAT_MAX_DATA_LEN, NULL);
  if (wechatinit.pValue != NULL)
  {
		//uint8 *data = NULL;
		//uint32 len = 0;
				
		ARGS_ITEM_SET(wechat_info, &wechatinfo, cmd, WECHAT_CMD_INIT); 
		Wechat_data_produce(&wechatinfo, &wechatinit.pValue, (uint32 *)&wechatinit.len);
		if(wechatinit.pValue == NULL)
		{
			NPI_Printf("wechat Pack Error Code\r\n");
			return (errorCodeProduce);
		}

		//sent data	
		NPI_Printf("\r\n##send %d init data: ", wechatinit.len);
		for(uint8 i=0;i<wechatinit.len;++i)
		{
			NPI_Printf(" %x",wechatinit.pValue[i]);
		}
		NPI_Printf("\r\n");
				
		BpFixHead *fix_head = (BpFixHead *)wechatinit.pValue;
		NPI_Printf("CMDID: %d \r\n", ntohs(fix_head->nCmdId));
		NPI_Printf("len: %d \r\n", ntohs(fix_head->nLength ));
		NPI_Printf("Seq: %d \r\n", ntohs(fix_head->nSeq));

		//ble_wechat_indicate_data(p_wcs, m_mpbledemo2_handler, data, len);
		//m_mpbledemo2_handler->m_data_free_func(data,len);
		wechatStoreIndications(&wechatinit);
		WechatSendStoredAuth();
  }
	
	return 0;
}


 
/*********************************************************************
* @fn 		 Wechat_device_test_senddat
*
* @brief	 Perform a periodic application task. This function gets
* 				 called every five seconds as a result of the SBP_PERIODIC_EVT
* 				 OSAL event. In this example, the value of the third
* 				 characteristic in the SimpleGATTProfile service is retrieved
* 				 from the profile, and then copied into the value of the
* 				 the fourth characteristic.
*
* @param	 none
*
* @return  none
*/
uint8 Wechat_device_test_senddat(void)
{ 
	//if (m_mpbledemo2_handler == NULL) {
	//			m_mpbledemo2_handler = get_handler_by_type(PRODUCT_TYPE_MPBLEDEMO2);
	//		}

	// BloodPressure measurement value stored in this structure.
  attHandleValueInd_t  wechatauth;

  wechatauth.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_IND,
                                           BLE_WECHAT_MAX_DATA_LEN, NULL);
  if (wechatauth.pValue != NULL)
  {
		wechatauth.pValue = NULL;
		wechatauth.len = 0;
				
		ARGS_ITEM_SET(wechat_info, &wechatinfo, cmd, WECHAT_CMD_TEST_SENDDAT);	
		ARGS_ITEM_SET(wechat_info, &wechatinfo, send_msg.len, sizeof(SEND_HELLO_WECHAT));
		ARGS_ITEM_SET(wechat_info, &wechatinfo, send_msg.str, SEND_HELLO_WECHAT); 
		Wechat_data_produce(&wechatinfo, &wechatauth.pValue, (uint32 *)&wechatauth.len);
		if(wechatauth.pValue == NULL)
		{
			NPI_Printf("wechat Pack Error Code\r\n");
			return (errorCodeProduce);
		}

		//sent data	
		NPI_Printf("\r\n##send %d data: ", wechatauth.len);
		for(uint8 i=0;i<wechatauth.len;++i)
		{
			NPI_Printf(" %x",wechatauth.pValue[i]);
		}
		NPI_Printf("\r\n");
				
		BpFixHead *fix_head = (BpFixHead *)wechatauth.pValue;
		NPI_Printf("CMDID: %d \r\n", ntohs(fix_head->nCmdId));
		NPI_Printf("len: %d \r\n", ntohs(fix_head->nLength ));
		NPI_Printf("Seq: %d \r\n", ntohs(fix_head->nSeq));

		//ble_wechat_indicate_data(p_wcs, m_mpbledemo2_handler, data, len);
		//m_mpbledemo2_handler->m_data_free_func(data,len);
		wechatStoreIndications(&wechatauth);
		WechatSendStoredAuth();
  }
	
	return 0;
}

void Wechat_main_process(void)
{
	int error_code;
	if ( (wechatSta.indication_state) && (!wechatSta.auth_state) && (!wechatSta.auth_send) )
	{
		error_code = Wechat_device_auth();
		//APP_ERROR_CHECK(error_code);
		if (error_code != errorCodeProduce)		
			wechatSta.auth_send = true;	
	}
	if( (wechatSta.auth_state) && (!wechatSta.init_state) && (!wechatSta.init_send) )
	{
		error_code = Wechat_device_init();
		//APP_ERROR_CHECK(error_code);
		if (error_code != errorCodeProduce)
			wechatSta.init_send = true;
	}
	//return ;
}

/*********************************************************************
*********************************************************************/
