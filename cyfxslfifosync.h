#ifndef _INCLUDED_CYFXBULKSRCSINK_H_
#define _INCLUDED_CYFXBULKSRCSINK_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define CY_FX_BULKSRCSINK_THREAD_STACK       (0x1000)                  /* Bulk loop application thread stack size */
#define CY_FX_BULKSRCSINK_THREAD_PRIORITY    (8)                       /* Bulk loop application thread priority */
#define CY_FX_BULKSRCSINK_PATTERN            (0xAA)                    /* 8-bit pattern to be loaded to the source buffers. */
#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */
#define CY_FX_EP_BURST_LENGTH          (16)


typedef struct SdCardState_t {
	uint32_t port;
	uint32_t sector;
	uint32_t starts;
	uint32_t completes;
	uint32_t socket;
	uint32_t max_sector;
} SdCardState_t;


typedef struct SystemState_t {
	CyBool_t can_write;
	uint32_t intr_count[ 2 ];
	uint32_t last_intr_count;
	uint32_t first_error;
	uint32_t last_error;
	SdCardState_t sds[ 2 ];
} SystemState_t;

void GetSdDeviceInfo( int port, void* buf );

void InitSystemStates();
void StartNextSdTx( SdCardState_t* sds );
void HandleSdComplete( int port );
void HandleGpifIntr( int thread );
CyBool_t HandleUsbRequest( uint32_t setupdat0, uint32_t setupdat1 );

void HandleError( CyU3PReturnStatus_t apiRetStatus );
void StartInfDma();
void InitDma();
void InitSib();
void InitGpio();
void InitPib();
void StartGpif();
void InitGpif();
void InitUsb();

/* External definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];

#include <cyu3externcend.h>

#endif /* _INCLUDED_CYFXBULKSRCSINK_H_ */

/*[]*/
