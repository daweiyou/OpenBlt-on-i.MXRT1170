/************************************************************************************//**
* \file         Source/ARMCM7_STM32F7/rs232.c
* \brief        Bootloader RS232 communication interface source file.
* \ingroup      Target_ARMCM7_STM32F7
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2018  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_COM_RS232_ENABLE > 0)
//#include "stm32f7xx.h"                           /* STM32 CPU and HAL header           */
//#include "stm32f7xx_ll_usart.h"                  /* STM32 LL USART header              */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_lpuart.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Timeout time for the reception of a CTO packet. The timer is started upon
 *         reception of the first packet byte.
 */
#define RS232_CTO_RX_PACKET_TIMEOUT_MS (100u)
/** \brief Timeout for transmitting a byte in milliseconds. */
#define RS232_BYTE_TX_TIMEOUT_MS       (10u)

#if (BOOT_COM_RS232_CHANNEL_INDEX == 0)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 1)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 2)
#define USART_CHANNEL   LPUART2 //use in RT1170 LPUART2, GPIO_DISP_B2_11/GPIO_DISP_B2_10, J9 - p2 & 4
#define USART_CLK_FREQ  CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart2)
#elif (BOOT_COM_RS232_CHANNEL_INDEX == 3)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 4)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 5)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 6)

#elif (BOOT_COM_RS232_CHANNEL_INDEX == 7)

#endif


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static blt_bool Rs232ReceiveByte(blt_int8u *data);
static void     Rs232TransmitByte(blt_int8u data);



/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: M15, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AD_25, software_input_on: Disable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: L13, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AD_24, software_input_on: Disable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: High, slew_rate: Slow}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void InitUsartPin(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_11_LPUART2_RXD,      
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_DISP_B2_10_LPUART2_TXD,      
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_DISP_B2_11_LPUART2_RXD,          
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_DISP_B2_10_LPUART2_TXD,          
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}

/************************************************************************************//**
** \brief     Initializes the RS232 communication interface.
** \return    none.
**
****************************************************************************************/
void Rs232Init(void)
{
  ASSERT_CT((BOOT_COM_RS232_CHANNEL_INDEX == 0) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 1) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 2) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 3) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 4) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 5) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 6) ||
            (BOOT_COM_RS232_CHANNEL_INDEX == 7));
  
    lpuart_config_t config;

    InitUsartPin();
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOOT_COM_RS232_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(USART_CHANNEL, &config, USART_CLK_FREQ);
} /*** end of Rs232Init ***/

void Rs232_Deinit(void)
{
    LPUART_Deinit(USART_CHANNEL);
} /*** end of Rs232Init ***/
/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
void Rs232TransmitPacket(blt_int8u *data, blt_int8u len)
{
  blt_int16u data_index;

  /* verify validity of the len-paramenter */
  ASSERT_RT(len <= BOOT_COM_RS232_TX_MAX_DATA);

  /* first transmit the length of the packet */
  Rs232TransmitByte(len);

  /* transmit all the packet bytes one-by-one */
  for (data_index = 0; data_index < len; data_index++)
  {
    /* keep the watchdog happy */
    CopService();
    /* write byte */
    Rs232TransmitByte(data[data_index]);
  }
} /*** end of Rs232TransmitPacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \param     len Pointer where the length of the packet is to be stored.
** \return    BLT_TRUE if a packet was received, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool Rs232ReceivePacket(blt_int8u *data, blt_int8u *len)
{
  static blt_int8u xcpCtoReqPacket[BOOT_COM_RS232_RX_MAX_DATA+1];  /* one extra for length */
  static blt_int8u xcpCtoRxLength;
  static blt_bool  xcpCtoRxInProgress = BLT_FALSE;
  static blt_int32u xcpCtoRxStartTime = 0;

  /* start of cto packet received? */
  if (xcpCtoRxInProgress == BLT_FALSE)
  {
    /* store the message length when received */
    if (Rs232ReceiveByte(&xcpCtoReqPacket[0]) == BLT_TRUE)
    {
      if ( (xcpCtoReqPacket[0] > 0) &&
           (xcpCtoReqPacket[0] <= BOOT_COM_RS232_RX_MAX_DATA) )
      {
        /* store the start time */
        xcpCtoRxStartTime = TimerGet();
        /* reset packet data count */
        xcpCtoRxLength = 0;
        /* indicate that a cto packet is being received */
        xcpCtoRxInProgress = BLT_TRUE;
      }
    }
  }
  else
  {
    /* store the next packet byte */
    if (Rs232ReceiveByte(&xcpCtoReqPacket[xcpCtoRxLength+1]) == BLT_TRUE)
    {
      /* increment the packet data count */
      xcpCtoRxLength++;

      /* check to see if the entire packet was received */
      if (xcpCtoRxLength == xcpCtoReqPacket[0])
      {
        /* copy the packet data */
        CpuMemCopy((blt_int32u)data, (blt_int32u)&xcpCtoReqPacket[1], xcpCtoRxLength);
        /* done with cto packet reception */
        xcpCtoRxInProgress = BLT_FALSE;
        /* set the packet length */
        *len = xcpCtoRxLength;
        /* packet reception complete */
        return BLT_TRUE;
      }
    }
    else
    {
      /* check packet reception timeout */
      if (TimerGet() > (xcpCtoRxStartTime + RS232_CTO_RX_PACKET_TIMEOUT_MS))
      {
        /* cancel cto packet reception due to timeout. note that that automaticaly
         * discards the already received packet bytes, allowing the host to retry.
         */
        xcpCtoRxInProgress = BLT_FALSE;
      }
    }
  }
  /* packet reception not yet complete */
  return BLT_FALSE;
} /*** end of Rs232ReceivePacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface byte if one is present.
** \param     data Pointer to byte where the data is to be stored.
** \return    BLT_TRUE if a byte was received, BLT_FALSE otherwise.
**
****************************************************************************************/
static blt_bool Rs232ReceiveByte(blt_int8u *data)
{
//  if (LL_USART_IsActiveFlag_RXNE(USART_CHANNEL) != 0)
//  {
//    /* retrieve and store the newly received byte */
//    *data = LL_USART_ReceiveData8(USART_CHANNEL);
//    /* all done */
//    return BLT_TRUE;
//  }
  
  if(kStatus_Success == LPUART_ReadBlocking(USART_CHANNEL, data, 1))
  {
      /* all done */
      return BLT_TRUE;
  }
  /* still here to no new byte received */
  return BLT_FALSE;
} /*** end of Rs232ReceiveByte ***/


/************************************************************************************//**
** \brief     Transmits a communication interface byte.
** \param     data Value of byte that is to be transmitted.
** \return    none.
**
****************************************************************************************/
static void Rs232TransmitByte(blt_int8u data)
{
  blt_int32u timeout;

  /* write byte to transmit holding register */
  //LL_USART_TransmitData8(USART_CHANNEL, data);
  LPUART_WriteBlocking(USART_CHANNEL, &data, 1);
  
  /* set timeout time to wait for transmit completion. */
  timeout = TimerGet() + RS232_BYTE_TX_TIMEOUT_MS;
  /* wait for tx holding register to be empty */
//  while (LL_USART_IsActiveFlag_TXE(USART_CHANNEL) == 0)
//  {
//    /* keep the watchdog happy */
//    CopService();
//    /* break loop upon timeout. this would indicate a hardware failure. */
//    if (TimerGet() > timeout)
//    {
//      break;
//    }
//  }
} /*** end of Rs232TransmitByte ***/
#endif /* BOOT_COM_RS232_ENABLE > 0 */


/*********************************** end of rs232.c ************************************/
