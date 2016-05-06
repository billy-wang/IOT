/**************************************************************************************************
  Filename:       aes.c
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

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "linkdb.h"
#include "OSAL_Clock.h"
#include "wechatservice.h"
#include "wechatapp.h"
#include "wechat_util.h"
#include "md5.h"
#include "aes_crypt.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//#define BP_MEAS_LEN                           19

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task ID for internal task/event processing
//uint8 simpleBLEPeripheral_TaskID;


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
//extern void* memcpy(void *dest, const void *src, size_t len);

/*********************************************************************
 * LOCAL VARIABLES
 */
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

#if 0
uint8 data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

uint8 key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

uint8 encrypt_buf[128];
int encrypt_len = 0;
uint8 decrypt_buf[128];
int decrypt_len = 0;
#endif



/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );


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

unsigned int AES_get_length(unsigned int length)
{
	return ((length>>4) + 1)<<4;
}

/**@brief   Function for the light initialization.
 *
 * @details Initializes all lights used by this application.
 */

void encrypt_data(uint8* data, uint8 encrypt_buf[128], uint8 key[16])
{
	int encrypt_len =0;
	int len = sizeof(data);
	int offset = 0;

	AES_CTX c;
	aes_cbc_encrypt_init(&c, key);

	while(len > 16) {
		aes_cbc_encrypt_update(&c, data+offset, encrypt_buf+offset);
		offset += 16;
		len -= 16;
	}

	int out_len = 0;
	aes_cbc_encrypt_final(&c, data+offset, len, encrypt_buf+offset, &out_len);
	encrypt_len = offset + out_len;
	
	NPI_Printf("encrypt len: %d\r\n", encrypt_len); //48
	for(int i=0; i<encrypt_len; i++) {
		NPI_Printf("%02x", encrypt_buf[i]);
	} //c6a13b37878f5b826f4f8162a1c8d879b58a1064d8aca99bd9b0405b8545f5bbe0db23614cac28000000000088ac2800
	NPI_Printf("\r\n");
}

void decrypt_data(uint8 encrypt_buf[128], int encrypt_len, uint8 decrypt_buf[128], uint8 key[16])
{
	int decrypt_len =0;
	int len = encrypt_len;
	int offset = 0;

	AES_CTX c;
	aes_cbc_decrypt_init(&c, key);

	while(len > 16) {
		aes_cbc_decrypt_update(&c, encrypt_buf+offset, decrypt_buf+offset);
		offset += 16;
		len -= 16;
	}

	int out_len = 0;
	aes_cbc_decrypt_final(&c, encrypt_buf+offset, decrypt_buf+offset, &out_len);
	decrypt_len = offset + out_len;
	
	NPI_Printf("decrypt len: %d\r\n", decrypt_len); //40
	for(int i=0; i<decrypt_len; i++) {
		NPI_Printf("%02x", decrypt_buf[i]);
	} //000102030405060708090a0b0c0d0e0f000102030405060708090a0b0c0d0e0f0001020304050607
	NPI_Printf("\r\n");
}


/*********************************************************************
*********************************************************************/


