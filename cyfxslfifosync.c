#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyfxslfifosync.h"
#include "cyu3usb.h"
#include "cyu3gpif.h"
#include <cyu3gpio.h>
#include <cyu3lpp.h>
#include <cyu3sib.h>
#include "cyu3pib.h"
#include "pib_regs.h"
#include "gpif2_config.h"
#include "host_commands.h"

#define PROJECT_VERSION ( 0x180517C1 )


#define EVT_GPIF_INTR_T0   ( 1 << 1 )
#define EVT_GPIF_INTR_T1   ( 1 << 2 )
#define EVT_SD0_COMPLETE   ( 1 << 3 )
#define EVT_SD1_COMPLETE   ( 1 << 4 )
#define EVT_USB_INTR       ( 1 << 5 )
#define EVT_NEED_RESET     ( 1 << 6 )


#define SD_SECTOR ( 512 )

#define DMA_BUF_SIZE ( 50 * 1024 )
#define DMA_BUF_CNT  ( 2 )
#define SD_INTR_SECTORS ( 16 * 2048 )

uint8_t glEp0Buffer[4*32];
uint16_t glRecvdLen;
CyU3PThread     bulkSrcSinkAppThread;	 /* Application thread structure */
static CyU3PEvent  appEvent;                          /* Application Event group */
CyU3PDmaChannel glDmaChans[ 2 ];

CyU3PSibDevInfo_t devInfos[ 2 ];
SystemState_t state;


void InitSystemStates() {

	CyU3PMemSet( (uint8_t*)&state, 0, sizeof(SystemState_t) );

	state.can_write 		  = 0;
	state.intr_count[ 0 ] = 0;
	state.intr_count[ 1 ] = 0;
	state.last_intr_count = 0;
	state.first_error     = 0;
	state.last_error      = 0;

	for ( int i = 0; i < 2; i++ ) {
		state.sds[ i ].sector    = 0;
		state.sds[ i ].starts    = 0;
		state.sds[ i ].completes = 0;
		state.sds[ i ].max_sector = 40000000;
	}
	state.sds[ 0 ].port   = 0;
	state.sds[ 0 ].socket = CY_U3P_SIB_SOCKET_0;

	state.sds[ 1 ].port   = 1;
	state.sds[ 1 ].socket = CY_U3P_SIB_SOCKET_1;
}

void GetSdDeviceInfo( int port, void* buf ) {
    CyU3PSibDevInfo_t* devInfo = (CyU3PSibDevInfo_t*) buf;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint32_t* buf32 = ( uint32_t*) buf;

    apiRetStatus = CyU3PSibQueryDevice (port, devInfo);

    if (apiRetStatus != CY_U3P_SUCCESS) {
    	buf32[ 0 ] = 1111;
    	buf32[ 1 ] = (uint32_t) apiRetStatus;
    	return;
    }
}


/* Callback to handle the USB setup requests. */
CyBool_t
HandleUsbRequest (
		uint32_t setupdat0, /* SETUP Data 0 */
		uint32_t setupdat1  /* SETUP Data 1 */
)
{
	uint8_t  bRequest;
	uint16_t wLength;

	bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
	wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

	if( bRequest == 0x05) {
		return CyTrue;
	} else if ( bRequest == CMD_GET_VERSION ) {

		FirmwareDescription_t fw_desc;
		fw_desc.version = PROJECT_VERSION;
		CyU3PUsbSendEP0Data( (uint16_t)sizeof( FirmwareDescription_t ), (uint8_t*)&fw_desc);
		return CyTrue;

	} else if (bRequest == CMD_CYPRESS_RESET) {

		CyU3PUsbGetEP0Data( wLength, glEp0Buffer, NULL );
		CyU3PEventSet( &appEvent, EVT_NEED_RESET, CYU3P_EVENT_OR );
		return CyTrue;

	} else if (bRequest == CMD_READ_DEBUG_INFO) {

		CyU3PMemSet ((uint8_t *)&glEp0Buffer[0], 0, sizeof (glEp0Buffer));
		unsigned int* Ep0Buffer = (unsigned int*)&glEp0Buffer[0];
		uint32_t ints_both = state.intr_count[ 0 ] + state.intr_count[ 1 ];
		uint32_t delta_intrs = ints_both - state.last_intr_count;
		Ep0Buffer[0] = delta_intrs * DMA_BUF_SIZE;
		Ep0Buffer[1] = state.intr_count[0];
		Ep0Buffer[2] = state.sds[0].starts;
		Ep0Buffer[3] = state.sds[0].completes;
		Ep0Buffer[4] = DMA_BUF_SIZE;
		Ep0Buffer[5] = state.intr_count[1];
		Ep0Buffer[6] = state.sds[1].starts;
		Ep0Buffer[7] = state.sds[1].completes;

		Ep0Buffer[ 8] = state.first_error;
		Ep0Buffer[ 9] = state.last_error;

		Ep0Buffer[10] = SD_SECTOR;
		Ep0Buffer[11] = SD_INTR_SECTORS;
		Ep0Buffer[12] = state.intr_count[0];
		Ep0Buffer[13] = state.sds[0].sector;
		Ep0Buffer[14] = state.sds[0].max_sector;

		Ep0Buffer[15] = state.intr_count[1];
		Ep0Buffer[16] = state.sds[1].sector;
		Ep0Buffer[17] = state.sds[1].max_sector;

//		CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
//		uint32_t dma_status = 0, tx_prod = 0, tx_cons = 0;
//		apiRetStatus = CyU3PDmaChannelGetStatus( &glDmaChans[0], (CyU3PDmaState_t*)&dma_status, &tx_prod, &tx_cons );
//		Ep0Buffer[10] = apiRetStatus;
//		Ep0Buffer[11] = dma_status;
//		Ep0Buffer[12] = tx_prod;
//		Ep0Buffer[13] = tx_cons;
//
//		apiRetStatus = CyU3PDmaChannelGetStatus( &glDmaChans[1], (CyU3PDmaState_t*)&dma_status, &tx_prod, &tx_cons );
//		Ep0Buffer[14] = apiRetStatus;
//		Ep0Buffer[15] = dma_status;
//		Ep0Buffer[16] = tx_prod;
//		Ep0Buffer[17] = tx_cons;

		state.last_intr_count = ints_both;

		CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
		return CyTrue;

	} else if (bRequest == CMD_SDDEV0_INFO || bRequest == CMD_SDDEV1_INFO ) {

		CyU3PMemSet( (uint8_t *)&glEp0Buffer[0], 0, sizeof (glEp0Buffer));
		int port = bRequest == CMD_SDDEV0_INFO ? 0 : 1;
		CyU3PMemCopy( (uint8_t *)&glEp0Buffer[0], (uint8_t*)&devInfos[ port ], sizeof(CyU3PSibDevInfo_t) );
		CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
		return CyTrue;

	}

	return CyFalse;
}


void HandleGpifIntr( int thread ) {
//	state.intr_count[ thread ] += 1;
}

void StartNextSdTx( SdCardState_t* sds ) {
	if ( !state.can_write ) {
		return;
	}
	CyU3PReturnStatus_t apiRetStatus =
	CyU3PSibReadWriteRequest(
			CyFalse,
			sds->port,
			0, /* unit */
			SD_INTR_SECTORS,
			sds->sector,
			sds->socket );

	if (apiRetStatus != CY_U3P_SUCCESS)	{
		if ( state.first_error == 0 ) {
			state.first_error = apiRetStatus | 0x200;
		}
		state.last_error = apiRetStatus | 0x200;
	} else {
		sds->starts += 1;
		sds->sector += SD_INTR_SECTORS;
		if ( sds->sector >= sds->max_sector ) {
			state.can_write = CyFalse;
		}
	}
}

void HandleSdComplete( int port ) {
	if ( port != 0 && port != 1 ) {
		state.last_error = 0xEE0001;
	}
	SdCardState_t* sds = &state.sds[ port ];

	sds->completes += 1;
	StartNextSdTx( sds );
}

/* This is a callback function to handle gpif events */
void
CallbackGpif (
		CyU3PGpifEventType event,               /* Event type that is being notified. */
		uint8_t            smState         /* Current state of the State Machine. */
)
{
	switch (event)
	{
		case CYU3P_GPIF_EVT_SM_INTERRUPT:
			if ( smState == T0_DMA_BUSY ) {
				//CyU3PEventSet( &appEvent, EVT_GPIF_INTR_T0, CYU3P_EVENT_OR );
				state.intr_count[ 0 ] += 1;
			} else
			if ( smState == T1_DMA_BUSY ) {
				//CyU3PEventSet( &appEvent, EVT_GPIF_INTR_T1, CYU3P_EVENT_OR );
				state.intr_count[ 1 ] += 1;
			} else {
				state.last_error = 0xEE0002;
			}
			break;
		default:
			break;
	}
}

void
CallbackSib (
        uint8_t             portId,
        CyU3PSibEventType   evt,
        CyU3PReturnStatus_t status)
{
    if (evt == CY_U3P_SIB_EVENT_XFER_CPLT) {
    	if ( portId == 0 ) {
    		CyU3PEventSet( &appEvent, EVT_SD0_COMPLETE, CYU3P_EVENT_OR );
		} else
		if ( portId == 1 ) {
    		CyU3PEventSet( &appEvent, EVT_SD1_COMPLETE, CYU3P_EVENT_OR );
		}
    } else
    if (evt == CY_U3P_SIB_EVENT_INSERT) {
    } else
    if (evt == CY_U3P_SIB_EVENT_REMOVE) {
    } else
    if (evt == CY_U3P_SIB_EVENT_DATA_ERROR) {
    	state.last_error = 0x301;
    } else
    if (evt == CY_U3P_SIB_EVENT_ABORT) {
    	state.last_error = 0x302;
    }
}

/* This is the callback function to handle the USB events. */
void
CallbackUsb (
		CyU3PUsbEventType_t evtype, /* Event type */
		uint16_t            evdata  /* Event data */
)
{
	switch (evtype)
	{
	case CY_U3P_USB_EVENT_SETCONF:
	case CY_U3P_USB_EVENT_RESET:
	case CY_U3P_USB_EVENT_DISCONNECT:
	default:
		break;
	}
}

void
InitUsb()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Start the USB functionality. */
	apiRetStatus = CyU3PUsbStart();
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* The fast enumeration is the easiest way to setup a USB connection,
	 * where all enumeration phase is handled by the library. Only the
	 * class / vendor requests need to be handled by the application. */
	CyU3PUsbRegisterSetupCallback(HandleUsbRequest, CyTrue);

	/* Setup the callback to handle the USB events. */
	CyU3PUsbRegisterEventCallback(CallbackUsb);

	/* Set the USB Enumeration descriptors */

	/* Super speed device descriptor. */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* High speed device descriptor. */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* BOS descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* Device qualifier descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* Super speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* High speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* Full speed configuration descriptor */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* String descriptor 0 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* String descriptor 1 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* String descriptor 2 */
	apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	/* Connect the USB Pins with super speed operation enabled. */
	apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

}

void
InitGpif()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Load the GPIF configuration for Slave FIFO sync mode. */
	apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	CyU3PGpifRegisterCallback(CallbackGpif);
}

void
StartGpif()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Set FW_TRG to false. */
	CyU3PGpifControlSWInput( CyFalse );

	/* Start the state machine. */
	apiRetStatus = CyU3PGpifSMStart (RESET, ALPHA_RESET);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}
}

void
InitGpio()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PGpioClock_t gpioClock;

    /* GPIO module needs to be initialized before SIB is initialized. This is required because
       GPIOs are used in the SIB code.
     */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 16;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK;
    gpioClock.halfDiv    = 0;
    apiRetStatus = CyU3PGpioInit (&gpioClock, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }
}

void
InitPib()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PPibClock_t pibClock;

	/* Initialize the p-port block. */
	pibClock.clkDiv = 2;
	pibClock.clkSrc = CY_U3P_SYS_CLK;
	pibClock.isHalfDiv = CyFalse;
	/* Disable DLL for sync GPIF */
	pibClock.isDllEnable = CyFalse;
	apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}
}

void
InitSib()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PSibIntfParams_t intfParams;

    /* Populate the structure with data from the mailbox request. */
    intfParams.resetGpio       = 0xFF;                          /* No GPIO control on SD/MMC power. */
    intfParams.rstActHigh      = CyTrue;                        /* Don't care as no GPIO is selected. */
    intfParams.cardDetType     = CY_U3P_SIB_DETECT_NONE;        /* Hotplug is not supported. */
    intfParams.writeProtEnable = CyFalse;                       /* Write protect handling enabled. */
    intfParams.lowVoltage      = CyTrue;                        /* Low voltage operation enabled. */
    intfParams.voltageSwGpio   = 45;                            /* Use GPIO_45 for voltage switch on S0 port. */
    intfParams.lvGpioState     = CyFalse;                       /* Driving GPIO low selects 1.8 V on SxVDDQ. */
    intfParams.useDdr          = CyTrue;                        /* DDR clocking enabled. */
    intfParams.maxFreq         = CY_U3P_SIB_FREQ_104MHZ;        /* No S port clock limitation. */
    intfParams.cardInitDelay   = 0;                             /* No delay required between SD card insertion
                                                                   before initialization. */
    apiRetStatus = CyU3PSibSetIntfParams (0, &intfParams);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

    intfParams.voltageSwGpio = 57;                          /* Use GPIO_57 for voltage switch on S1 port. */
    apiRetStatus = CyU3PSibSetIntfParams (1, &intfParams);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }


    /* SIB start request. No parameters are used for this request. */
	apiRetStatus = CyU3PSibStart ();
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

    /* Register a callback for SIB events. */
    CyU3PSibRegisterCbk(CallbackSib);


    apiRetStatus = CyU3PSibInit(0);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

    apiRetStatus = CyU3PSibInit(1);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

    /* Set write commit size for both devices to 8 MB. */
    apiRetStatus = CyU3PSibSetWriteCommitSize (0, 16384);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

    apiRetStatus = CyU3PSibSetWriteCommitSize (1, 16384);
    if (apiRetStatus != CY_U3P_SUCCESS) {
    	HandleError(apiRetStatus);
    }

}

void
InitDma()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PDmaChannelConfig_t dmaCfg;

	CyU3PMemSet ((uint8_t*)&dmaCfg, 0, sizeof (dmaCfg));
	dmaCfg.size           = DMA_BUF_SIZE;
	dmaCfg.count          = DMA_BUF_CNT;
	dmaCfg.prodSckId      = CY_U3P_PIB_SOCKET_0;
	dmaCfg.consSckId      = CY_U3P_SIB_SOCKET_0;
	dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
	dmaCfg.notification   = 0;
	dmaCfg.cb             = 0;
	dmaCfg.prodHeader     = 0;
	dmaCfg.prodFooter     = 0;
	dmaCfg.consHeader     = 0;
	dmaCfg.prodAvailCount = 0;
	apiRetStatus = CyU3PDmaChannelCreate (&glDmaChans[ 0 ], CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	CyU3PMemSet ((uint8_t*)&dmaCfg, 0, sizeof (dmaCfg));
	dmaCfg.size           = DMA_BUF_SIZE;
	dmaCfg.count          = DMA_BUF_CNT;
	dmaCfg.prodSckId      = CY_U3P_PIB_SOCKET_1;
	dmaCfg.consSckId      = CY_U3P_SIB_SOCKET_1;
	dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
	dmaCfg.notification   = 0;
	dmaCfg.cb             = 0;
	dmaCfg.prodHeader     = 0;
	dmaCfg.prodFooter     = 0;
	dmaCfg.consHeader     = 0;
	dmaCfg.prodAvailCount = 0;
	apiRetStatus = CyU3PDmaChannelCreate (&glDmaChans[ 1 ], CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}
}

void
DestroyDma()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	state.can_write = CyFalse;

	apiRetStatus = CyU3PDmaChannelDestroy (&glDmaChans[0]);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	apiRetStatus = CyU3PDmaChannelDestroy (&glDmaChans[1]);
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

}

void
StartInfDma()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	apiRetStatus = CyU3PDmaChannelSetXfer(&glDmaChans[ 0 ], 0 );
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}

	apiRetStatus = CyU3PDmaChannelSetXfer(&glDmaChans[ 1 ], 0 );
	if (apiRetStatus != CY_U3P_SUCCESS)	{
		HandleError(apiRetStatus);
	}
}

/* Entry function for the BulkSrcSinkAppThread. */
void
AppThread_Entry (
		uint32_t input)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	InitSystemStates();

	InitGpio();
	InitPib();
	InitGpif();
	StartGpif();

	InitSib();
	InitDma();
	StartInfDma();

	InitUsb();

	GetSdDeviceInfo( 0, &devInfos[ 0 ] );
	GetSdDeviceInfo( 1, &devInfos[ 1 ] );
	state.sds[ 0 ].max_sector = devInfos[ 0 ].numBlks - 20480;
	state.sds[ 1 ].max_sector = devInfos[ 0 ].numBlks - 20480;

	state.can_write = CyTrue;

	StartNextSdTx( &state.sds[ 0 ] );
	StartNextSdTx( &state.sds[ 1 ] );
	/* Set FW_TRG for state machine (really starts the SM) */
	CyU3PGpifControlSWInput( CyTrue );

	uint32_t evMask = EVT_GPIF_INTR_T0 | EVT_GPIF_INTR_T1 |
					  EVT_SD0_COMPLETE | EVT_SD1_COMPLETE |
					  EVT_USB_INTR     | EVT_NEED_RESET;
	uint32_t evt;

	for (;;)
	{
		apiRetStatus = CyU3PEventGet (&appEvent, evMask, CYU3P_EVENT_OR_CLEAR, &evt, CYU3P_WAIT_FOREVER);
		if ( apiRetStatus == CY_U3P_SUCCESS ) {
			if ( evt & EVT_GPIF_INTR_T0 ) {
				HandleGpifIntr( 0 );
			} else
			if ( evt & EVT_GPIF_INTR_T1 ) {
				HandleGpifIntr( 1 );
			} else
			if ( evt & EVT_SD0_COMPLETE ) {
				HandleSdComplete( 0 );
			} else
			if ( evt & EVT_SD1_COMPLETE ) {
				HandleSdComplete( 1 );
			} else
			if ( evt & EVT_USB_INTR ) {

			} else
			if ( evt & EVT_NEED_RESET ) {
				state.can_write = CyFalse;
				CyU3PThreadSleep(500);
				CyU3PSibStop();
				CyU3PThreadSleep(10);
				DestroyDma();
				CyU3PThreadSleep(100);
				CyU3PDeviceReset(CyFalse);
			}
		}
	}
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
		void)
{
	void *ptr = NULL;
	uint32_t retThrdCreate = CY_U3P_SUCCESS;

	/* Allocate the memory for the threads */
	ptr = CyU3PMemAlloc (CY_FX_BULKSRCSINK_THREAD_STACK);

	if ( CyU3PEventCreate (&appEvent) != 0 ) {
		while(1);
	}

	/* Create the thread for the application */
	retThrdCreate = CyU3PThreadCreate (&bulkSrcSinkAppThread,      /* App thread structure */
			"21:Bulk_src_sink",                      /* Thread ID and thread name */
			AppThread_Entry,              /* App thread entry function */
			0,                                       /* No input parameter to thread */
			ptr,                                     /* Pointer to the allocated thread stack */
			CY_FX_BULKSRCSINK_THREAD_STACK,          /* App thread stack size */
			CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
			CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
			CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
			CYU3P_AUTO_START                         /* Start the thread immediately */
	);

	if (retThrdCreate != 0)
	{
		/* Thread Creation failed with the error code retThrdCreate */
		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}
}

int
main (void)
{
	CyU3PIoMatrixConfig_t io_cfg;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	/* Initialize the device */
	status = CyU3PDeviceInit (NULL);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}

	/* Initialize the caches. Enable instruction cache and keep data cache disabled.
	 * The data cache is useful only when there is a large amount of CPU based memory
	 * accesses. When used in simple cases, it can decrease performance due to large
	 * number of cache flushes and cleans and also it adds to the complexity of the
	 * code. */
	status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}


	/* Configure the IO matrix for the device. */

    io_cfg.isDQ32Bit        = CyFalse;
    io_cfg.s0Mode           = CY_U3P_SPORT_8BIT;
    io_cfg.s1Mode           = CY_U3P_SPORT_8BIT;
    io_cfg.gpioSimpleEn[0]  = 0;
    //io_cfg.gpioSimpleEn[1]  = 0x02102800; /* IOs 43, 45, 52 and 57 are chosen as GPIO. */
    io_cfg.gpioSimpleEn[1]  = 0x02002000; /* IOs 45, 57 are chosen as GPIO. */
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    io_cfg.useUart          = CyFalse;
    io_cfg.useI2C           = CyFalse;
    io_cfg.useI2S           = CyFalse;
    io_cfg.useSpi           = CyFalse;
    io_cfg.lppMode          = CY_U3P_IO_MATRIX_LPP_NONE;

	status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
	if (status != CY_U3P_SUCCESS)
	{
		goto handle_fatal_error;
	}

	/* This is a non returnable call for initializing the RTOS kernel */
	CyU3PKernelEntry ();

	/* Dummy return to make the compiler happy */
	return 0;

	handle_fatal_error:
	/* Cannot recover from this error. */
	CyU3PDeviceReset(CyFalse);
	while(1){

	}
	return 0;
}

/* Application Error Handler */
void
HandleError (
		CyU3PReturnStatus_t apiRetStatus
)
{
	CyU3PThreadSleep(100);
	CyU3PDeviceReset(CyFalse);
}


