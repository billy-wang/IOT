/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
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
#include "bpservice.h"
#include "timeapp.h"
#include "OSAL_Clock.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   9000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // #if (defined HAL_KEY) && (HAL_KEY == TRUE)


// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Some values used to simulate measurements
#define FLAGS_IDX_MAX                         7      //3 flags c/f -- timestamp -- site

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Delay to begin discovery from start of connection in ms
#define DEFAULT_DISCOVERY_DELAY               1000

#define BP_DISCONNECT_PERIOD                  6000

#define CUFF_MAX                              40

#define TIMER_CUFF_PERIOD                     500

// Max measurement storage count
#define BP_STORE_MAX                         	4

#define BP_CUFF_MEAS_LEN                      12
#define BP_MEAS_LEN                           19

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task ID for internal task/event processing
uint8 simpleBLEPeripheral_TaskID;

// Connection handle
uint16 gapConnHandle;

// Time stamp read from server
uint8 timeConfigDone;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Service discovery state
static uint8 timeAppDiscState = DISC_IDLE;

// Service discovery complete
static uint8 timeAppDiscoveryCmpl = FALSE;

// Characteristic configuration state
static uint8 timeAppConfigState = TIMEAPP_CONFIG_START;

// TRUE if pairing started
static uint8 timeAppPairingStarted = FALSE;

// TRUE if discovery postponed due to pairing
static uint8 timeAppDiscPostponed = FALSE;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	0x46, 	// 'F'
	0x6c, 	// 'l'
	0x79, 	// 'y'
	0x62, 	// 'b'
	0x69, 	// 'i'
	0x72, 	// 'r'
	0x64, 	// 'd'
	0x4c, 	// 'L'
	0x61, 	// 'a'
	0x62, 	// 'b'
	0x6f, 	// 'o'
	0x72, 	// 'r'
	0x61, 	// 'a'
	0x74, 	// 't'
	0x6f, 	// 'o'
	0x72, 	// 'r'
	0x79, 	// 'y'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Flybird Peripheral";

// Bonded state
static bool timeAppBonded = FALSE;

// Bonded peer address
static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Last connection address
static uint8 lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};;

static bool connectedToLastAddress = false;

// GAP connection handle
//uint16 gapConnHandle;

static uint16 bpSystolic = 120; //mmg
static uint16 bpDiastolic = 80; //mmg
static uint16 bpMAP = 90; //70-110mmg
static uint16 bpPulseRate = 60; //pulseRate
static uint8  bpUserId = 1;
static uint16 bpMeasStatus = 0;

// flags for simulated measurements
static const uint8 bloodPressureFlags[FLAGS_IDX_MAX] =
{
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP |
    BLOODPRESSURE_FLAGS_PULSE | BLOODPRESSURE_FLAGS_USER |
    BLOODPRESSURE_FLAGS_STATUS,
  BLOODPRESSURE_FLAGS_MMHG | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_MMHG,
  BLOODPRESSURE_FLAGS_KPA,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP,
  BLOODPRESSURE_FLAGS_KPA | BLOODPRESSURE_FLAGS_TIMESTAMP |
    BLOODPRESSURE_FLAGS_PULSE,
  0x00
};

// Program State
enum
{
  BPM_STATE_IDLE,            
  BPM_STATE_ADVERTISING,              
  BPM_STATE_CONNECTED             
};

// Measurement State
enum
{
  BPM_MEAS_STATE_IDLE,            
  BPM_MEAS_STATE_ACTIVE,             
  BPM_MEAS_STATE_READY            
};

// initial value of flags
static uint8 bloodPressureFlagsIdx = 0;

// cuff count, when we reach max, send final BP meas
static uint8 cuffCount = 0;

static attHandleValueInd_t bpStoreMeas[BP_STORE_MAX];
static uint8 bpStoreStartIndex = 0;
static uint8 bpStoreIndex = 0;

static uint8 lastChar3Value = 0;

// Structure of NV data for the connected device's address information
typedef struct
{
  uint8   publicAddr[B_ADDR_LEN];     // Master's address
  uint8   reconnectAddr[B_ADDR_LEN];  // Privacy Reconnection Address
  uint16  stateFlags;                 // State flags: SM_AUTH_STATE_AUTHENTICATED & SM_AUTH_STATE_BONDING
} BondRec_t;

static uint8 gPairStatus=0;
static uint8 PrivateStatus=0;

// Local RAM shadowed bond records
static BondRec_t BondRecords[10] = {0};
static uint8 ConnectedWhiteListDevAddr[B_ADDR_LEN] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void timeAppDisconnected( void );
static void timeAppDiscpostponed( void );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );
static void simpleProfileSendIndicate(void);

static void bpServiceCB(uint8 event);
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
																						uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void bpFinalMeas(void);
static void bpStoreIndications(attHandleValueInd_t* pInd);
static void bpSendStoredMeas();
static void cuffMeas(void);
static void simulateMeas( void );

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

static char *bdAddr2Str ( uint8 *pAddr );

static void DevPrivateCheck( void );
static uint8 StorePrivateBDadd(void);
static uint8 BondFindAddr( uint8 *pDevAddr );
static uint8 RWFlashTest( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static const gapBondCBs_t timeApp_BondMgrCBs =
{
  timeAppPasscodeCB,                     // Passcode callback (not used by application)
  timeAppPairStateCB                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
 
#if (defined HAL_UART) && (HAL_UART == TRUE)
// Npi Serial Callbacks
static void NpiSerialPeripheralCB( uint8 port, uint8 events )
{
	if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))
	{
		uint8 numBytes = 0;

		numBytes = NPI_RxBufLen();

		if(numBytes == 0)
		{
			return;
		}
		else
		{
			uint8 *buffer = osal_mem_alloc(numBytes);
			if(buffer)
			{
				NPI_ReadTransport(buffer,numBytes);
	
				//The received data is sent to a serial port to realize the loopback
				NPI_WriteTransport(buffer, numBytes);

				osal_mem_free(buffer);
			}
		}
	}
}
#endif


/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
		#if (defined HAL_KEY) && (HAL_KEY == TRUE)
      // device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE; //GAPBOND_PAIRING_MODE_INITIATE; //GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;	//GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Stop config reads when done
  timeConfigDone = FALSE;

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLEPeripheral_TaskID );

  // Register for BloodPressure service callback
  BloodPressure_Register( bpServiceCB );

  // Initialize measurement storage table
  VOID osal_memset( bpStoreMeas, 0, (sizeof(attHandleValueInd_t) * BP_STORE_MAX) );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  BloodPressure_AddService( GATT_ALL_SERVICES );
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 
																									11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
  }


#if (defined HAL_KEY) && (HAL_KEY == TRUE)

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

#if 0
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low
#endif
#endif // #if (defined HAL_KEY) && (HAL_KEY == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

#if (defined HAL_UART) && (HAL_UART == TRUE)
	// Initialize uart
	NPI_InitTransport(NpiSerialPeripheralCB);
	NPI_WriteTransport("FlybirdBLECentral_Init\r\n", 23);
	NPI_PrintString("FlybirdBLECentral_Init2\r\n");

	NPI_PrintValue("Serial print value decimal= ", 168, 10);
	NPI_PrintString("\r\n");
	NPI_PrintValue("Serial print value hexadecimal = 0x", 0x88, 16);
	NPI_PrintString("\r\n");
#endif

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
#if 0
	NPI_PrintValue("ProcessEvent ", events, 16);
	NPI_PrintString("\r\n");
#endif

  VOID task_id; // OSAL required parameter that isn't used in this function

  ////////////////////////////////////////////////////////////////////
  // 		 	EVENT    			 MSG 				DONE
  ////////////////////////////////////////////////////////////////////
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  ////////////////////////////////////////////////////////////////////
  // 			START    		BOND    	 EVENT     	DONE
  ////////////////////////////////////////////////////////////////////
	if ( events & SMP_BOND_EVT )
	{
		DevPrivateCheck();

		return ( events ^ SMP_BOND_EVT );
	}

  ////////////////////////////////////////////////////////////////////
  // 			START    		DEVICE    	 EVENT     	DONE
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    GAPBondMgr_Register( (gapBondCBs_t *) &timeApp_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

		#if (defined HAL_LED) && (HAL_LED == TRUE)
		// Set LED1 on to give feedback that the power is on, and a timer to turn off
		HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
		osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // To keep the LED on continuously.
		osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_POWERON_LED_TIMEOUT_EVT, 1000 );
		#endif

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  ////////////////////////////////////////////////////////////////////
  // 				POWERON		LED 		TIMEOUT 	EVENT		 DONE
  ////////////////////////////////////////////////////////////////////
	#if (defined HAL_LED) && (HAL_LED == TRUE)
	if ( events & SBP_POWERON_LED_TIMEOUT_EVT )
	{
		osal_pwrmgr_device( PWRMGR_BATTERY ); // Revert to battery mode after LED off
		HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
		return ( events ^ SBP_POWERON_LED_TIMEOUT_EVT );
	}
	#endif

  ////////////////////////////////////////////////////////////////////
  // 				PERIODIC 					EVENT 						DONE
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  ////////////////////////////////////////////////////////////////////
  // 			START				DISCOVERY 			EVENT 				DONE
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_START_DISCOVERY_EVT )
  {
    if ( timeAppPairingStarted )
    {
      // Postpone discovery until pairing completes
      timeAppDiscPostponed = TRUE;
    }
    else
    {
      timeAppDiscState = timeAppDiscStart();
    }
    return ( events ^ SBP_START_DISCOVERY_EVT );
  }

  ////////////////////////////////////////////////////////////////////
  // 				BloodPressure 		MEAS 		EVENT		DONE
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_TIMER_BPMEAS_EVT )
  {
    // Perform final measurement
    bpFinalMeas();

    return (events ^ SBP_TIMER_BPMEAS_EVT);
  }

  ////////////////////////////////////////////////////////////////////
  // 				TIMER			CUFF 			EVENT
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_TIMER_CUFF_EVT )
  {
		// Perform a Cutoff Measurement
		cuffMeas();

		cuffCount++;

		// If cuff count not met, keep sending cuff measurements
		if(cuffCount < CUFF_MAX)
		{
			// Start interval timer to send BP, just for simulation
			osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_CUFF_EVT, TIMER_CUFF_PERIOD );
		}
		// Get ready to send final measurement
		else
		{
			// Start timer to send final BP meas
			osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_BPMEAS_EVT, TIMER_CUFF_PERIOD );
		}

    return (events ^ SBP_TIMER_CUFF_EVT);
  }

  ////////////////////////////////////////////////////////////////////
  //			 Enable 			BloodPressure 			CCC
  ////////////////////////////////////////////////////////////////////
  if ( events & SBP_CCC_UPDATE_EVT )
  {
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      //if previously connected and measurements are active send stored
      if( connectedToLastAddress == true)
      {
        //send stored measurements
        bpSendStoredMeas();
      }
    }

    return (events ^ SBP_CCC_UPDATE_EVT);
  }

	////////////////////////////////////////////////////////////////////
  // 	Disconnect 		after 		sending 		measurement
	////////////////////////////////////////////////////////////////////
  if ( events & SBP_DISCONNECT_EVT )
  {
    uint8 advEnable = FALSE;

    //disable advertising on disconnect
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );

		#if (defined HAL_LED) && (HAL_LED == TRUE)
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
		#endif

    // Terminate Connection
    GAPRole_TerminateConnection();

    return (events ^ SBP_DISCONNECT_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {     
		#if (defined HAL_KEY) && (HAL_KEY == TRUE)
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, 
                                      ((keyChange_t *)pMsg)->keys );
      break;
		#endif

    case GATT_MSG_EVENT:
      // Process GATT message
      simpleBLEPeripheral_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;

    default:
      // do nothing
      break;
  }
}

#if (defined HAL_KEY) && (HAL_KEY == TRUE)
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
		SK_Keys |= HAL_KEY_UP;

		HalLcdWriteString( "JOY UP Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "JOY UP Press\r\n");

		// set simulated measurement flag index
    if (++bloodPressureFlagsIdx == FLAGS_IDX_MAX)
    {
      bloodPressureFlagsIdx = 0;
    }
  }

  if ( keys & HAL_KEY_CENTER )
  {
		SK_Keys |= HAL_KEY_CENTER;

		HalLcdWriteString( "JOY CENTER Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "JOY CENTER Press\r\n");

		RWFlashTest();
  }

  if ( keys & HAL_KEY_LEFT )
  {
		SK_Keys |= HAL_KEY_LEFT;

		HalLcdWriteString( "JOY LEFT Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "JOY LEFT Press\r\n");

		simpleProfileSendIndicate();
  }

  if ( keys & HAL_KEY_DOWN )
  {
		SK_Keys |= HAL_KEY_DOWN;

		HalLcdWriteString( "JOY DOWN Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "JOY DOWN Press\r\n");
  }

  if ( keys & HAL_KEY_SW_7 )
  {
		SK_Keys |= HAL_KEY_SW_7;

		HalLcdWriteString( "Button 2 Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "Button 2 Press\r\n");
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    SK_Keys |= SK_KEY_RIGHT;

		HalLcdWriteString( "JOY RIGHT Press", HAL_LCD_LINE_3 );
		NPI_PrintString( "JOY RIGHT Press\r\n");

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );

#if 0
			//start simulation timer (start --> cuff -->measurement ready)
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_CUFF_EVT, TIMER_CUFF_PERIOD );

      //reset cuff count
      cuffCount = 0;
#endif

#ifndef PLUS_BROADCASTER
    }
		else //connected mode, simulate some measurements
    {
      simulateMeas();
    }
#endif // PLUS_BROADCASTER
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );

}
#endif // #if (defined HAL_KEY) && (HAL_KEY == TRUE)


/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{
	//Measurement Indication Confirmation
	switch (pMsg->method)
	{
		case ATT_ERROR_RSP :
			HalLcdWriteString("Error Resp",  HAL_LCD_LINE_3 );
			NPI_PrintString("Error Resp\r\n");
			break;
		case ATT_EXCHANGE_MTU_REQ :
			HalLcdWriteString("Exch MTU Req",  HAL_LCD_LINE_3 );
			NPI_PrintString("Exch MTU Req\r\n");
			break;
		case ATT_EXCHANGE_MTU_RSP :
			HalLcdWriteString("Exch MTU Resp",  HAL_LCD_LINE_3 );
			NPI_PrintString("Exch MTU Resp\r\n");
			break;
		case ATT_FIND_INFO_REQ :
			HalLcdWriteString("Find Info Req",	HAL_LCD_LINE_3 );
			NPI_PrintString("Find Info Req\r\n");
			break;
		case ATT_FIND_INFO_RSP :
			HalLcdWriteString("Find Info Resp",	HAL_LCD_LINE_3 );
			NPI_PrintString("Find Info Resp\r\n");
			break;
		case ATT_FIND_BY_TYPE_VALUE_REQ :
			HalLcdWriteString("Find vaue Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Find vaue Req\r\n");
			break;
		case ATT_FIND_BY_TYPE_VALUE_RSP :
			HalLcdWriteString("Find vaue Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Find vaue Resp\r\n");
			break;
		case ATT_READ_BY_TYPE_REQ :
			HalLcdWriteString("Read Type Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Type Req\r\n");
			break;
		case ATT_READ_BY_TYPE_RSP :
			HalLcdWriteString("Read Type Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Type Resp\r\n");
			break;
		case ATT_READ_REQ :
			HalLcdWriteString("Read Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Req\r\n");
			break;
		case ATT_READ_RSP :
			HalLcdWriteString("Read Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Resp\r\n");
			break;
		case ATT_READ_BLOB_REQ :
			HalLcdWriteString("Read Blob Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Blob Req\r\n");
			break;
		case ATT_READ_BLOB_RSP :
			HalLcdWriteString("Read Blob Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Blob Resp\r\n");
			break;
		case ATT_READ_MULTI_REQ :
			HalLcdWriteString("Read Multi Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Multi Req\r\n");
			break;
		case ATT_READ_MULTI_RSP :
			HalLcdWriteString("Read Multi Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Error Resp\r\n");
			break;
		case ATT_READ_BY_GRP_TYPE_REQ :
			HalLcdWriteString("Read Type Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Type Req\r\n");
			break;
		case ATT_READ_BY_GRP_TYPE_RSP :
			HalLcdWriteString("Read Type Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Read Type Resp\r\n");
			break;
		case ATT_WRITE_REQ :
			HalLcdWriteString("Write Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Write Req\r\n");
			break;
		case ATT_WRITE_RSP :
			HalLcdWriteString("Write Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Write Resp\r\n");
			break;
		case ATT_PREPARE_WRITE_REQ :
			HalLcdWriteString("Prep Write Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Prep Write Req\r\n");
			break;
		case ATT_PREPARE_WRITE_RSP :
			HalLcdWriteString("Prep Write Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Prep Write Resp\r\n");
			break;
		case ATT_EXECUTE_WRITE_REQ :
			HalLcdWriteString("Exec Write Req", HAL_LCD_LINE_3 );
			NPI_PrintString("Exec Write Req\r\n");
			break;
		case ATT_EXECUTE_WRITE_RSP :
			HalLcdWriteString("Exec Write Resp", HAL_LCD_LINE_3 );
			NPI_PrintString("Exec Write Resp\r\n");
			break;
		case ATT_HANDLE_VALUE_NOTI :
			HalLcdWriteString("Handle Noti", HAL_LCD_LINE_3 );
			NPI_PrintString("Handle Noti\r\n");
			break;
		case ATT_HANDLE_VALUE_IND :
			HalLcdWriteString("Handle Ind", HAL_LCD_LINE_3 );
			NPI_PrintString("Handle Ind\r\n");
			break;
		case ATT_HANDLE_VALUE_CFM :
			HalLcdWriteString("Handle Cfm", HAL_LCD_LINE_3 );
			NPI_PrintString("Handle Cfm\r\n");
			break;
		case ATT_WRITE_CMD :
			HalLcdWriteString("Write Command", HAL_LCD_LINE_3 );
			NPI_PrintString("Write Command\r\n");
			break;
		case ATT_SIGNED_WRITE_CMD :
			HalLcdWriteString("Signed W Command", HAL_LCD_LINE_3 );
			NPI_PrintString("Signed W Command\r\n");
			break;
		default:
			HalLcdWriteString("Unknown", HAL_LCD_LINE_3 );
			NPI_PrintString("Unknown\r\n");
			break;
	}

  // Measurement Indication Confirmation
  if( pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
      bpSendStoredMeas();
  }

  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    timeAppIndGattMsg( pMsg );
  }
  else if ( pMsg->method == ATT_READ_RSP ||
            pMsg->method == ATT_WRITE_RSP )
  {
    timeAppConfigState = timeAppConfigGattMsg ( timeAppConfigState, pMsg );
    if ( timeAppConfigState == TIMEAPP_CONFIG_CMPL )
    {
      timeAppDiscoveryCmpl = TRUE;
    }
  }
  else
  {
    timeAppDiscState = timeAppDiscGattMsg( timeAppDiscState, pMsg );
    if ( timeAppDiscState == DISC_IDLE )
    {
      // Start characteristic configuration
      timeAppConfigState = timeAppConfigNext( TIMEAPP_CONFIG_START );
    }
  }

  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      timeAppDisconnected
 *
 * @brief   Handle disconnect.
 *
 * @return  none
 */
static void timeAppDisconnected( void )
{
  // Initialize state variables
  timeAppDiscState = DISC_IDLE;
  timeAppPairingStarted = FALSE;
  timeAppDiscPostponed = FALSE;

  // stop periodic measurement
  osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_CUFF_EVT );

  // reset bloodPressure measurement client configuration
  uint16 param = 0;
  BloodPressure_SetParameter(BLOODPRESSURE_MEAS_CHAR_CFG, sizeof(uint16), (uint8 *) &param);

  // reset bloodPressure intermediate measurement client configuration
  BloodPressure_SetParameter(BLOODPRESSURE_IMEAS_CHAR_CFG, sizeof(uint16), (uint8 *) &param);

  uint8 advEnable = FALSE;

  //disable advertising on disconnect
  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
}

/*********************************************************************
 * @fn      timeAppDiscpostponed
 *
 * @brief  discovery was postponed start discovery.
 *
 * @param  none
 *
 * @return  none
 */
static void timeAppDiscpostponed( void )
{
	linkDBItem_t	*pItem;

	if ( (pItem = linkDB_Find( gapConnHandle )) != NULL )
	{
		// Store bonding state of pairing
		timeAppBonded = ( (pItem->stateFlags & LINK_BOUND) == LINK_BOUND );

		if ( timeAppBonded )
		{
			osal_memcpy( timeAppBondedAddr, pItem->addr, B_ADDR_LEN );
			NPI_PrintString("Found ");
			NPI_WriteTransport(bdAddr2Str( timeAppBondedAddr ), (uint8)osal_strlen( (char*)bdAddr2Str( timeAppBondedAddr )));
			NPI_PrintString(" BondedAddr\r\n");
		}
	}

	// If discovery was postponed start discovery
	if ( timeAppDiscPostponed && timeAppDiscoveryCmpl == FALSE )
	{
		timeAppDiscPostponed = FALSE;
		osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DISCOVERY_EVT );
	}
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

				// Display device address
				HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
				HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
				NPI_PrintString("Initialized\r\n");
      }
      break;

    case GAPROLE_ADVERTISING:
      {
				uint8 ownAddress[B_ADDR_LEN];
				GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
				// Display device address
				HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
				HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
				NPI_PrintString("Advertising\r\n");

				#if (defined HAL_LED) && (HAL_LED == TRUE)
				HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
				#endif
      }
			break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8 advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                           &advertEnabled);
        
        // Reset flag for next connection.
        first_conn_flag = 0;
      }
      break;
#endif //PLUS_BROADCASTER         
      
    case GAPROLE_CONNECTED:
      {        
        HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
				NPI_PrintString("Connected\r\n");

				#if (defined HAL_LED) && (HAL_LED == TRUE)
				HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
				#endif

				linkDBItem_t	*pItem;

				// Get connection handle
				GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );

				// Get peer bd address
				if ( (pItem = linkDB_Find( gapConnHandle )) != NULL)
				{
					// If connected to device without bond do service discovery
					if ( !osal_memcmp( pItem->addr, timeAppBondedAddr, B_ADDR_LEN ) )
					{
						timeAppDiscoveryCmpl = FALSE;
					}
					else
					{
						timeAppDiscoveryCmpl = TRUE;
					}

					// if this was last connection address don't do discovery
					if(osal_memcmp( pItem->addr, lastConnAddr, B_ADDR_LEN ))
					{
						timeAppDiscoveryCmpl = TRUE;
						connectedToLastAddress = true;
					}
					else
					{
						//save the last connected address
						osal_memcpy(lastConnAddr, pItem->addr, B_ADDR_LEN );
					}

					// Initiate service discovery if necessary
					if ( timeAppDiscoveryCmpl == FALSE )
					{
					 // osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
					}

				}

#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
            uint8 advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;
            
            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
				NPI_PrintString("Connected Advertising\r\n");
      }
      break;

    case GAPROLE_WAITING:
      {
        HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
				NPI_PrintString("Disconnected\r\n");

				#if (defined HAL_LED) && (HAL_LED == TRUE)
					// Turn off LED that shows we're advertising
					HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
				#endif
          
#ifdef PLUS_BROADCASTER                
        uint8 advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                             &advertEnabled);
#endif
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
				NPI_PrintString("Timed Out\r\n");

#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif 
      }
      break;

    case GAPROLE_ERROR:
      {
        HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
				NPI_PrintString("Error\r\n");
      }
      break;

    default:
      {
        HalLcdWriteString( "",  HAL_LCD_LINE_3 );
				NPI_PrintString("unknow state!!\r\n");
      }
      break;

  }

		// if disconnected
	 if ( gapProfileState == GAPROLE_CONNECTED &&
						 newState != GAPROLE_CONNECTED )
	 {
		 timeAppDisconnected();

		 //always stop intermediate timer
		 osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_CUFF_EVT );

		 // stop disconnect timer
		 osal_stop_timerEx( simpleBLEPeripheral_TaskID, BP_DISCONNECT_PERIOD );

	 }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  uint8 valueToCopy;
  uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

  if( (stat == SUCCESS) && (lastChar3Value != valueToCopy))
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
		lastChar3Value = valueToCopy;
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 stat;

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
			if( stat == SUCCESS )
			{
	      HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
				NPI_PrintValue("Char 1: ", (uint16)(newValue), 16);
				NPI_PrintString("\r\n");
			}
			else
			{
				NPI_PrintString("Char 1: faild \r\n");
			}

      break;

    case SIMPLEPROFILE_CHAR3:
      stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
			if( stat == SUCCESS )
			{
	      HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
				NPI_PrintValue("Char 3: ", (uint16)(newValue), 16);
				NPI_PrintString("\r\n");
			}
			else
			{
				NPI_PrintString("Char 3: faild \r\n");
			}

      break;

		case SIMPLEPROFILE_NOTI_DISABLED:
			NPI_PrintString("Char 4 notify disabled !\r\n");
			break;

		case SIMPLEPROFILE_NOTI_ENABLED:
			NPI_PrintString("Char 4 notify enabled !\r\n");
			break;

		case SIMPLEPROFILE_INDI_DISABLED:
			NPI_PrintString("Char 6 indicate disabled !\r\n");
			break;

		case SIMPLEPROFILE_INDI_ENABLED:
			NPI_PrintString("Char 6 indicate enabled !\r\n");
			break;

    default:
      // should not reach here!
			NPI_PrintString("unknow Callback indicating\r\n");
      break;
  }
}

/*********************************************************************
 * @fn      simpleProfileSendIndicate
 *
 * @brief   Send a simple indication. An incoming indication
 *          confirmation will trigger the next pending stored measurement.
 *
 * @return  none
 */
static void simpleProfileSendIndicate(void)
{
	uint16 notify_Handle;
	//uint8 buf[20]={0};
	uint8 *p;
	uint8 status;

	GAPRole_GetParameter( GAPROLE_CONNHANDLE, &notify_Handle);

	for(uint8 i = 0; i < 20; i++)
	{
		*(p+i) = i;
	}

	status = SimpleProfile_Char6_Indicate(notify_Handle, p, 20, simpleBLEPeripheral_TaskID);
	if(status == SUCCESS)
	{
		NPI_PrintString("indicate is seccess to send!\r\n");
	}
	else
	{
		NPI_PrintString("indicate is fail to send!\r\n");
	}
}

/*********************************************************************
 * @fn      BP Service Callback
 *
 * @brief   Callback function for bloodpressure service.
 *
 * @param   event - service event
 *
 *   Need to know if BP Meas enabled so we can send stored meas. 
 *
 *    #define BLOODPRESSURE_MEAS_NOTI_ENABLED         1
 *    #define BLOODPRESSURE_MEAS_NOTI_DISABLED        2
 *
 * @return  none
 */
static void bpServiceCB(uint8 event)
{
  switch (event)
  {
    case BLOODPRESSURE_MEAS_NOTI_ENABLED:
      osal_set_event( simpleBLEPeripheral_TaskID, SBP_CCC_UPDATE_EVT );
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      timeAppStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
	NPI_PrintValue("PairState", state, 16);
	NPI_PrintString("\r\n");

	switch (state)
	{
		case GAPBOND_PAIRING_STATE_STARTED:
			timeAppPairingStarted = TRUE;

			NPI_PrintString("Pairing started\r\n");
			break;
		case GAPBOND_PAIRING_STATE_COMPLETE:
			timeAppPairingStarted = FALSE;
			timeAppDiscPostponed = TRUE;
			switch (status)
			{
				case SUCCESS :
					NPI_PrintString("Pairing success\r\n");
					if( StorePrivateBDadd() == SUCCESS )
					{
						osal_start_timerEx( simpleBLEPeripheral_TaskID, SMP_BOND_EVT, 50);
					}
					else
					{
						GAPRole_TerminateConnection();
					}
					break;
				case SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED	:	// 0x01
				case SMP_PAIRING_FAILED_OOB_NOT_AVAIL					:	// 0x02
				case SMP_PAIRING_FAILED_AUTH_REQ							:	// 0x03
				case SMP_PAIRING_FAILED_CONFIRM_VALUE					:	// 0x04
				case SMP_PAIRING_FAILED_NOT_SUPPORTED					:	// 0x05
				case SMP_PAIRING_FAILED_ENC_KEY_SIZE					:	// 0x06
				case SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED			:	// 0x07
				case SMP_PAIRING_FAILED_UNSPECIFIED						:	// 0x08
				case SMP_PAIRING_FAILED_REPEATED_ATTEMPTS			:	// 0x09
				default :
					NPI_PrintValue(" Pairing fail ", status, 10);
					NPI_PrintString("!!\r\n");
					GAPRole_TerminateConnection();
					break;
			}
			break;
		case GAPBOND_PAIRING_STATE_BONDED :
			switch (status)
			{
				case SUCCESS :
					NPI_PrintString("Bonding success\r\n");
					osal_start_timerEx( simpleBLEPeripheral_TaskID, SMP_BOND_EVT, 50);
					break;
				default :
					NPI_PrintValue("Bonding fail ", status, 10);
					NPI_PrintString("!!\r\n");
					GAPRole_TerminateConnection();
					break;
			}
		case GAPBOND_PAIRING_STATE_BOND_SAVED :
			NPI_PrintString("Bonding record saved in NV\r\n");
			timeAppDiscpostponed();
			break;
		default:
			NPI_PrintValue("Unknow state ", status, 10);
			NPI_PrintString("\r\n");
			GAPRole_TerminateConnection();
			break;
	}
}

/*********************************************************************
 * @fn      timeAppPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
	#if (defined HAL_UART) && (HAL_UART== TRUE)
  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;

  // Display passcode to user
  if ( uiOutputs != 0 )
  {
		NPI_PrintString("Passcode:\r\n");
		NPI_PrintString( (uint8 *) _ltoa(passcode, str, 10));
		NPI_PrintString("\r\n");
  }
	#endif

  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, 0 );
}

/*********************************************************************
 * @fn      bpFinalMeas
 *
 * @brief   Prepare and send a bloodPressure measurement indication
 *
 * @return  none
 */
static void bpFinalMeas(void)
{
  // BloodPressure measurement value stored in this structure.
  attHandleValueInd_t  bloodPressureMeas;

  bloodPressureMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_IND,
                                           BP_MEAS_LEN, NULL);
  if (bloodPressureMeas.pValue != NULL)
  {
    // att value notification structure
    uint8 *p = bloodPressureMeas.pValue;

    //flags
    uint8 flags = bloodPressureFlags[bloodPressureFlagsIdx];

    // flags 1 byte long
    *p++ = flags;

    // Bloodpressure components.
    *p++ = LO_UINT16(bpSystolic);
    *p++ = HI_UINT16(bpSystolic);
    *p++ = LO_UINT16(bpDiastolic);
    *p++ = HI_UINT16(bpDiastolic);
    *p++ = LO_UINT16(bpMAP);
    *p++ = HI_UINT16(bpMAP);

    //timestamp
    if (flags & BLOODPRESSURE_FLAGS_TIMESTAMP)
    {
      UTCTimeStruct time;

      // Get time structure from OSAL
      osal_ConvertUTCTime( &time, osal_getClock() );

      *p++ = 0x07;
      *p++ = 0xdb;
      *p++ = time.month;
      *p++ = time.day;
      *p++ = time.hour;
      *p++ = time.minutes;
      *p++ = time.seconds;

			#if 0
			NPI_PrintValue("", time.year, 10);
			NPI_PrintValue("-", time.month, 10);
			NPI_PrintValue("-", time.day, 10);
			NPI_PrintValue(" ", time.hour, 10);
			NPI_PrintValue(":", time.minutes, 10);
			NPI_PrintValue(":", time.seconds, 10);
			NPI_PrintString("\r\n");
			#endif
    }

    if (flags & BLOODPRESSURE_FLAGS_PULSE)
    {
      *p++ = LO_UINT16(bpPulseRate);
      *p++ = HI_UINT16(bpPulseRate);
    }

    if (flags & BLOODPRESSURE_FLAGS_USER)
    {
      *p++ = bpUserId;
    }

    if (flags & BLOODPRESSURE_FLAGS_STATUS)
    {
      *p++ = LO_UINT16(bpMeasStatus);
      *p++ = HI_UINT16(bpMeasStatus);
    }

    bloodPressureMeas.len = (uint8) (p - bloodPressureMeas.pValue);

    // store measurment
    bpStoreIndications(&bloodPressureMeas);

    // send stored measurements
    bpSendStoredMeas();

    // start disconnect timer
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DISCONNECT_EVT, BP_DISCONNECT_PERIOD );
  }
}

/*********************************************************************
 * @fn      bpStoreIndications
 *
 * @brief   Queue indications
 *
 * @return  none
 */
static void bpStoreIndications(attHandleValueInd_t *pInd)
{
	NPI_PrintValue("bpStoreIndications ", (uint16) pInd->pValue, 16);
	NPI_PrintString("\r\n");

  attHandleValueInd_t *pStoreInd = &bpStoreMeas[bpStoreIndex];

  if (pStoreInd->pValue != NULL)
  {
    // Free old indication's payload.
    GATT_bm_free((gattMsg_t *)pStoreInd, ATT_HANDLE_VALUE_IND);
  }

  // Store measurement
  VOID osal_memcpy(&bpStoreMeas[bpStoreIndex], pInd, sizeof(attHandleValueInd_t));

  // Store index
  bpStoreIndex = bpStoreIndex + 1;
  if (bpStoreIndex > BP_STORE_MAX)
  {
    bpStoreIndex = 0;
  }

  if (bpStoreIndex == bpStoreStartIndex)
  {
    bpStoreStartIndex = bpStoreStartIndex + 1;
    if(bpStoreStartIndex > BP_STORE_MAX)
    {
      bpStoreStartIndex = 0;
    }
  }
}

/*********************************************************************
 * @fn      bpSendStoredMeas
 *
 * @brief   Send a stored measurement indication. An incoming indication
 *          confirmation will trigger the next pending stored measurement.
 *
 * @return  none
 */
static void bpSendStoredMeas(void)
{
  // We connected to this peer before so send any stored measurements
  if (bpStoreStartIndex != bpStoreIndex)
  {
    attHandleValueInd_t *pStoreInd = &bpStoreMeas[bpStoreStartIndex];

    // Send Measurement - can fail if not connected or CCC not enabled
    bStatus_t status = BloodPressure_MeasIndicate( gapConnHandle, pStoreInd,
                                                   simpleBLEPeripheral_TaskID );
    // If sucess, increment the counters and the indication confirmation
    // will trigger the next indication if there are more pending.
    if (status == SUCCESS)
    {
      bpStoreStartIndex = bpStoreStartIndex + 1;

      // Wrap around buffer
      if (bpStoreStartIndex > BP_STORE_MAX)
      {
        bpStoreStartIndex = 0;
      }

      // Clear out this Meas indication.
      VOID osal_memset( pStoreInd, 0, sizeof(attHandleValueInd_t) );

			NPI_PrintValue("bpSendStoredMeas StartIndex", bpStoreStartIndex-1, 10);
			NPI_PrintString(" indicate is seccess\r\n");
    }
		else
		{
			NPI_PrintValue("bpSendStoredMeas StartIndex", bpStoreStartIndex, 10);
			NPI_PrintString(" indicate is failed\r\n");
		}
  }
}

/*********************************************************************
 * @fn      cuffMeas
 *
 * @brief   Prepare and send a bloodPressure measurement notification
 *
 * @return  none
 */
static void cuffMeas(void)
{
  attHandleValueNoti_t bloodPressureIMeas;

  HalLedSet ( HAL_LED_2, HAL_LED_MODE_BLINK );
  HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );
  HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );

  bloodPressureIMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,
                                            BP_CUFF_MEAS_LEN, NULL);
  if (bloodPressureIMeas.pValue != NULL)
  {
    // ATT value notification structure.
    uint8 *p = bloodPressureIMeas.pValue;

    //flags
    uint8 flags = BLOODPRESSURE_FLAGS_MMHG  | BLOODPRESSURE_FLAGS_TIMESTAMP |
                  BLOODPRESSURE_FLAGS_PULSE | BLOODPRESSURE_FLAGS_USER      |
                  BLOODPRESSURE_FLAGS_STATUS;

    // flags 1 byte long
    *p++ = flags;

    //bloodpressure components
    // IEEE The 16–bit value contains a 4-bit exponent to base 10,
    // followed by a 12-bit mantissa. Each is in twoscomplementform.
    *p++ = LO_UINT16(bpSystolic); //120 = 0x0078  SFloat little endian = 0x7800
    *p++ = HI_UINT16(bpSystolic);
    *p++ = 0xFF;   //not used in cuff  IEEE NaN =0x07FF
    *p++ = 0x07;
    *p++ = 0xFF;   //not used in cuff IEEE NaN =0x07FF
    *p++ = 0x07;

    // Timestamp.
    if (flags & BLOODPRESSURE_FLAGS_TIMESTAMP)
    {
      UTCTimeStruct time;

      // Get time structure from OSAL
      osal_ConvertUTCTime( &time, osal_getClock() );

      *p++ = 0x07;
      *p++ = 0xdb;
      *p++ = time.month;
      *p++ = time.day;
      *p++ = time.hour;
      *p++ = time.minutes;
      *p++ = time.seconds;

			#if 0
			NPI_PrintValue(" ", time.year, 10);
			NPI_PrintValue("-", time.month, 10);
			NPI_PrintValue("-", time.day, 10);
			NPI_PrintValue(" ", time.hour, 10);
			NPI_PrintValue(":", time.minutes, 10);
			NPI_PrintValue(":", time.seconds, 10);
			NPI_PrintString("\r\n");
			#endif
    }

    if(flags & BLOODPRESSURE_FLAGS_PULSE)
    {
      *p++ = LO_UINT16(bpPulseRate);
      *p++ = HI_UINT16(bpPulseRate);
    }

    if(flags & BLOODPRESSURE_FLAGS_USER)
    {
      *p++ =  bpUserId;
    }

    if(flags & BLOODPRESSURE_FLAGS_STATUS)
    {
      *p++ = LO_UINT16(bpMeasStatus);
      *p++ = HI_UINT16(bpMeasStatus);
    }

    bloodPressureIMeas.len = (uint8) (p - bloodPressureIMeas.pValue);

    // Cuff measurements are not stored, only sent.
    if ( BloodPressure_IMeasNotify( gapConnHandle, &bloodPressureIMeas,
                                    simpleBLEPeripheral_TaskID ) != SUCCESS )
    {
			NPI_PrintValue("send", (uint16)bloodPressureIMeas.pValue, 16);		
			NPI_PrintValue(" len", (uint16)bloodPressureIMeas.len, 16);
			NPI_PrintString(" bp notify is failed\r\n");
			
      GATT_bm_free( (gattMsg_t *)&bloodPressureIMeas, ATT_HANDLE_VALUE_NOTI );
    }
		else
		{
			NPI_PrintValue("send", (uint16)bloodPressureIMeas.pValue, 16);		
			NPI_PrintValue(" len", (uint16)bloodPressureIMeas.len, 16);
			NPI_PrintString(" bp notify is seccess\r\n");
		}
  }
}

/*********************************************************************
 * @fn      simulateMeas
 *
 * @brief   Modify Measurement Values
 *
 * @param   none
 *
 * @return  none
 */
static void simulateMeas( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    if(bpSystolic < 150)
      bpSystolic +=1;
    else
      bpSystolic = 80;

    if(bpDiastolic < 110)
      bpDiastolic +=1;
    else
      bpDiastolic = 90;

    if(bpMAP < 110)
      bpMAP +=1;
    else
      bpMAP =70;

    if(bpPulseRate < 140)
      bpPulseRate +=1;
    else
      bpPulseRate =40;

    if(bpUserId < 5)
      bpUserId +=1;
    else
      bpUserId =1;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}

/*********************************************************************
 * @fn      DevPrivateCheck
 *
 * @brief   Check the idot device is private.

 * @param   None.
 *
 * @return  None.
 */
static void DevPrivateCheck( void )
{
	uint8 idx;													// NV Index
	uint8 publicAddr[B_ADDR_LEN]				// Place to put the public address
					= {0, 0, 0, 0, 0, 0};
	uint8 ret;

	uint8 bondAddr[B_ADDR_LEN]				// Place to put the public address
					= {0, 0, 0, 0, 0, 0};

	ret=GAPRole_GetParameter( GAPROLE_CONN_BD_ADDR, publicAddr);
	if (ret == SUCCESS )
	{
		// Read in NV Main Bond Record and compare public address
		ret=osal_snv_read( BLE_NVID_CUST_START, sizeof( bondAddr), bondAddr);
		if (ret == SUCCESS )
		{
			if ( osal_memcmp( bondAddr, publicAddr, B_ADDR_LEN ) )
			{
					// TCL P728M 			E5:D1:B7:6D:E5:D8
					// Lenovo X2-T0		2D:95:A6:8C:70:88
					// Found it
					NPI_PrintString("Found ");
					NPI_WriteTransport(bdAddr2Str( bondAddr ), (uint8)osal_strlen( (char*)bdAddr2Str( bondAddr )));
					NPI_PrintString(" central device\r\n");

					//start simulation timer (start --> cuff -->measurement ready)
					osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_CUFF_EVT, TIMER_CUFF_PERIOD );

					//reset cuff count
					cuffCount = 0;
			}
			else
			{
				GAPRole_TerminateConnection();
			}
		}
		else
		{
			idx = BondFindAddr(publicAddr);
			if ( idx == GAP_BONDINGS_MAX )
			{
				GAPRole_TerminateConnection();
			}
		}
	}
	else
	{
		NPI_PrintString("can't get bond addr from flash\r\n");
	}
}

/*********************************************************************
 * @fn      StorePrivateBDadd
 *
 * @brief   Check the device is private.

 * @param   None.
 *
 * @return  None.
 */
static uint8 StorePrivateBDadd (void)
{
	uint8 ret;
	uint8 pDevAddr[B_ADDR_LEN]= {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	GAPRole_GetParameter( GAPROLE_CONN_BD_ADDR, pDevAddr);

	ret=osal_snv_write( BLE_NVID_CUST_START, sizeof( pDevAddr), pDevAddr);
	if ( ret == SUCCESS )
	{
		osal_memcpy( ConnectedWhiteListDevAddr, pDevAddr, B_ADDR_LEN );
		gPairStatus = true;
		PrivateStatus = true;
		return SUCCESS;
	}
	else
	{
		PrivateStatus = false;
		return FAILURE;
	}

}

/*********************************************************************
 * @fn      BondFindAddr
 *
 * @brief   Look through the bonding entries to find an address.
 *
 * @param   pDevAddr - device address to look for
 *
 * @return  index to empty bonding (0 - (GAP_BONDINGS_MAX-1),
 *          GAP_BONDINGS_MAX if no empty entries
 */
static uint8 BondFindAddr( uint8 *pDevAddr )
{
  // Item doesn't exist, so create all the items
  for ( uint8 idx = 0; idx < GAP_BONDINGS_MAX; idx++ )
  {
    // Read in NV Main Bond Record and compare public address
    if ( osal_memcmp( ConnectedWhiteListDevAddr, pDevAddr, B_ADDR_LEN ) )
    {
			switch(pDevAddr[0])
			{
				case 0xE5 :
					NPI_PrintString("Find TCL P728M\r\n");
					break;
				case 0x02 :
					NPI_PrintString("Find Lenovo X2-T0\r\n");
					break;
				default :
					NPI_PrintString("Find Unkown ble device\r\n");
					break;
			}
      return ( idx ); // Found it
    }
  }

  return ( GAP_BONDINGS_MAX );
}

/*********************************************************************
 * @fn      RWFlashTest
 *
 * @brief   Start test read or write flash
 *
 * @return  None.
 */
static uint8 RWFlashTest( void )
{
	uint8 idx;
	uint8 publicAddr[B_ADDR_LEN]				// Place to put the public address
					= {0, 0, 0, 0, 0, 0};

	uint8 pDevAddr[B_ADDR_LEN]= {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	//uint8 pDevAddr[B_ADDR_LEN]= {"1, 2, 3, 4, 5, 6"};
	//uint8 pDevAddr[B_ADDR_LEN]= "123456";

  HalLedSet( (HAL_LED_ALL), HAL_LED_MODE_ON );

	osal_memcpy( ConnectedWhiteListDevAddr, pDevAddr, B_ADDR_LEN );

	idx=osal_snv_write( BLE_NVID_CUST_START,
				sizeof( ConnectedWhiteListDevAddr), ConnectedWhiteListDevAddr);
	if (idx == SUCCESS )
	{
		HalLedBlink ( HAL_LED_1, 1, 50, 1000);
		NPI_PrintValue("Write BD [", (uint16)(ConnectedWhiteListDevAddr), 16);
		NPI_PrintString(" ] to snv flash Success\r\n");

		idx=osal_snv_read( BLE_NVID_CUST_START, sizeof( publicAddr), publicAddr);
		if (idx == SUCCESS )
		{
			idx = BondFindAddr(publicAddr);
			if ( idx < GAP_BONDINGS_MAX )
			{
				HalLedBlink ( HAL_LED_2, 1, 50, 1000);
				return SUCCESS;
			}
		}
		else
		{
			HalLedSet( HAL_LED_2 , HAL_LED_MODE_OFF );
			NPI_PrintString("snv read failed\r\n");

			return NV_OPER_FAILED;
		}
	}
	else
	{
		NPI_PrintValue("Write BD [", (uint16)(ConnectedWhiteListDevAddr), 16);
		NPI_PrintString(" ] to snv flash Failed\r\n");
		HalLedSet( HAL_LED_1 , HAL_LED_MODE_OFF );

		return NV_OPER_FAILED;
	}
}

/*********************************************************************
*********************************************************************/
