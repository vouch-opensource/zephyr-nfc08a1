/**
  ******************************************************************************
  * @file    demo_ce.h
  * @brief   Header for card emulation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEMO_CE_H
#define DEMO_CE_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "platform.h"

    /** @addtogroup X-CUBE-NFC6_Applications
     *  @brief Sample applications for X-NUCLEO-NFC06A1 STM32 expansion boards.
     *  @{
     */

    /** @addtogroup CardEmulation
     *  @{
     */

    /** @defgroup CE_CardEmul
     *  @brief Card Emulation management functions
     * @{
     */


    /* Exported constants --------------------------------------------------------*/
    /* T3T Information Block Bytes offset */
#define T3T_INFBLK_VER_OFFSET       0
#define T3T_INFBLK_NBR_OFFSET       1
#define T3T_INFBLK_NBW_OFFSET       2
#define T3T_INFBLK_NMAXB_OFFSET     3
#define T3T_INFBLK_WRITEFLAG_OFFSET 9
#define T3T_INFBLK_RWFLAG_OFFSET    10
#define T3T_INFBLK_LN_OFFSET        11
#define T3T_INFBCK_CHECKSUM_OFFSET  14

    /* T3T Information Block WriteFlag values */
#define T3T_WRITEFLAG_OFF 0x00
#define T3T_WRITEFLAG_ON  0x0F

    /* T3T COMMAND OFFSET */
#define T3T_CHECK_RESP_CMD_OFFSET    0
#define T3T_CHECK_RESP_NFCID2_OFFSET 1
#define T3T_CHECK_RESP_SF1_OFFSET    9
#define T3T_CHECK_RESP_SF2_OFFSET    10
#define T3T_CHECK_RESP_NOB_OFFSET    11
#define T3T_CHECK_RESP_DATA_OFFSET   12
#define T3T_UPDATE_RESP_CMD_OFFSET    0
#define T3T_UPDATE_RESP_NFCID2_OFFSET 1
#define T3T_UPDATE_RESP_SF1_OFFSET    9
#define T3T_UPDATE_RESP_SF2_OFFSET    10

    /* External variables --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
    /* Exported functions ------------------------------------------------------- */
    /** @defgroup CE_CardEmul_Exported_functions
     *  @{
     */
    void     demoCeInit(uint8_t* nfcfNfcid);
    uint16_t demoCeT3T(uint8_t *rxData, uint16_t rxDataLen, uint8_t *txBuf, uint16_t txBufLen );
    uint16_t demoCeT4T(uint8_t *rxData, uint16_t rxDataLen, uint8_t *txBuf, uint16_t txBufLen );



    /**
      * @}
      */

    /**
      * @}
      */

    /**
      * @}
      */

    /**
      * @}
      */

#ifdef __cplusplus
}
#endif

#endif /* DEMO_CE_H */

