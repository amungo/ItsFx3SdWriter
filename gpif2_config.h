/*
 * Project Name: itsfx3_common.cyfx
 * Time : 05/18/2017 16:31:15
 * Device Type: FX3
 * Project Type: GPIF2
 *
 *
 *
 *
 * This is a generated file and should not be modified
 * This file need to be included only once in the firmware
 * This file is generated by Gpif2 designer tool version - 1.0.1198.2
 * 
 */

#ifndef _INCLUDED__
#define _INCLUDED__
#include "cyu3types.h"
#include "cyu3gpif.h"

/* Summary
   Number of states in the state machine
 */
#define CY_NUMBER_OF_STATES 16

/* Summary
   Mapping of user defined state names to state indices
 */
#define RESET 0
#define T0_FIRST_DONE 2
#define T0_READ 4
#define PRE_START 1
#define T0_DMA_BUSY 7
#define T1_READ 9
#define T0_DONE 5
#define T0_FIRST_RD 6
#define T1_DONE 10
#define T1_FIRST_RD 3
#define T1_FIRST_DONE 12
#define T1_DMA_BUSY 11
#define T0_CHECK_DMA 14
#define T0_SKIP_BUF 8
#define T1_SKIP_BUF 13
#define T1_CHECK_DMA 15


/* Summary
   Initial value of early outputs from the state machine.
 */
#define ALPHA_RESET 0x0


/* Summary
   Transition function values used in the state machine.
 */
uint16_t CyFxGpifTransition[]  = {
    0x0000, 0x5555, 0xFFFF, 0xAAAA
};

/* Summary
   Table containing the transition information for various states. 
   This table has to be stored in the WAVEFORM Registers.
   This array consists of non-replicated waveform descriptors and acts as a 
   waveform table. 
 */
CyU3PGpifWaveData CyFxGpifWavedata[]  = {
    {{0x3E739E01,0x00000100,0x80000080},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739406,0x20000100,0x80400040},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739403,0x24000100,0x80400040},{0x00000000,0x00000000,0x00000000}},
    {{0x1E739A0C,0x20010006,0xC0400040},{0x00000000,0x00000000,0x00000000}},
    {{0x1E739A05,0x24010006,0xC0400040},{0x00000000,0x00000000,0x00000000}},
    {{0x2E739C0B,0x00000100,0xC0400100},{0x3E739409,0x24000100,0x80400040}},
    {{0x2E739C02,0x24000100,0xC0400040},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739408,0x00000100,0x80000040},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739A0E,0x00000102,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739A0A,0x20000102,0xC0400040},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739404,0x20000100,0x80400040},{0x2E739C07,0x00000100,0xC0400100}},
    {{0x3E73940D,0x04000100,0x80000040},{0x00000000,0x00000000,0x00000000}},
    {{0x2E739C07,0x00000100,0xC0400100},{0x3E739404,0x20000100,0x80400040}},
    {{0x3E739A0F,0x04000102,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x3E739409,0x24000100,0x80400040},{0x2E739C0B,0x00000100,0xC0400100}}
};

/* Summary
   Table that maps state indices to the descriptor table indices.
 */
uint8_t CyFxGpifWavedataPosition[]  = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,10,14
};

/* Summary
   GPIF II configuration register values.
 */
uint32_t CyFxGpifRegValue[]  = {
    0x80008300,  /*  CY_U3P_PIB_GPIF_CONFIG */
    0x00000003,  /*  CY_U3P_PIB_GPIF_BUS_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_BUS_CONFIG2 */
    0x00000046,  /*  CY_U3P_PIB_GPIF_AD_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_STATUS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INTR */
    0x00000002,  /*  CY_U3P_PIB_GPIF_INTR_MASK */
    0x00000082,  /*  CY_U3P_PIB_GPIF_SERIAL_IN_CONFIG */
    0x00000782,  /*  CY_U3P_PIB_GPIF_SERIAL_OUT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_DIRECTION */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_DEFAULT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_POLARITY */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_TOGGLE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_BUS_SELECT */
    0x00000006,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_RESET */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_CTRL_COUNT_LIMIT */
    0x0000010A,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_RESET */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_ADDR_COUNT_LIMIT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_STATE_COUNT_CONFIG */
    0x0000FFFF,  /*  CY_U3P_PIB_GPIF_STATE_COUNT_LIMIT */
    0x0000010B,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_RESET */
    0x0000C800,  /*  CY_U3P_PIB_GPIF_DATA_COUNT_LIMIT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CTRL_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COMP_VALUE */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ADDR_COMP_MASK */
    0x00000000,  /*  CY_U3P_PIB_GPIF_DATA_CTRL */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  CY_U3P_PIB_GPIF_EGRESS_ADDRESS */
    0x80010400,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010401,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010402,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x80010403,  /*  CY_U3P_PIB_GPIF_THREAD_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_LAMBDA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_ALPHA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_BETA_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_CTRL_STAT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_SWITCH */
    0x00000000,  /*  CY_U3P_PIB_GPIF_WAVEFORM_SWITCH_TIMEOUT */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CRC_CONFIG */
    0x00000000,  /*  CY_U3P_PIB_GPIF_CRC_DATA */
    0xFFFFFFC1  /*  CY_U3P_PIB_GPIF_BETA_DEASSERT */
};

/* Summary
   This structure holds all the configuration inputs for the GPIF II. 
 */
const CyU3PGpifConfig_t CyFxGpifConfig  = {
    (uint16_t)(sizeof(CyFxGpifWavedataPosition)/sizeof(uint8_t)),
    CyFxGpifWavedata,
    CyFxGpifWavedataPosition,
    (uint16_t)(sizeof(CyFxGpifTransition)/sizeof(uint16_t)),
    CyFxGpifTransition,
    (uint16_t)(sizeof(CyFxGpifRegValue)/sizeof(uint32_t)),
    CyFxGpifRegValue
};

#endif   /* _INCLUDED__ */