/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: netdev.c,v 1.8 2006/06/07 08:39:58 adam Exp $
 */


/*---------------------------------------------------------------------------*/
#include <string.h>
#include "uip.h"
#include "uip_arp.h"
#include "boot.h"
//#include "stm32f7xx.h"                           /* STM32 CPU and HAL header           */
#include "fsl_common.h"
#include "fsl_enet.h"
#include "fsl_phy.h"
#if defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET
#include "fsl_memory.h"
#endif
#include "fsl_enet_mdio.h"
#include "fsl_phyksz8081.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
/*---------------------------------------------------------------------------*/
#define NETDEV_DEFAULT_MACADDR0           (0x08)
#define NETDEV_DEFAULT_MACADDR1           (0x00)
#define NETDEV_DEFAULT_MACADDR2           (0x27)
#define NETDEV_DEFAULT_MACADDR3           (0x69)
#define NETDEV_DEFAULT_MACADDR4           (0x5B)
#define NETDEV_DEFAULT_MACADDR5           (0x45)

/* Timeout time for transmitting a packet. */
#define NETDEV_TX_PACKET_TIMEOUT_MS       (250u)


/*---------------------------------------------------------------------------*/
//ETH_HandleTypeDef heth;


//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */
//
//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */
//
//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
//
//#if defined ( __ICCARM__ ) /*!< IAR Compiler */
//  #pragma data_alignment=4
//#endif
//__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */
//

/*---------------------------------------------------------------------------*/
static struct uip_eth_addr macAddress;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_ENET        ENET
#define EXAMPLE_PHY_ADDRESS 0x02U

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops
/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops
/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Bus)
#define ENET_RXBD_NUM          (4)
#define ENET_TXBD_NUM          (4)
#define ENET_RXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_DATA_LENGTH       (1000)
#define ENET_TRANSMIT_DATA_NUM (20)
#ifndef APP_ENET_BUFF_ALIGNMENT
#define APP_ENET_BUFF_ALIGNMENT ENET_BUFF_ALIGNMENT
#endif
#ifndef PHY_AUTONEGO_TIMEOUT_COUNT
#define PHY_AUTONEGO_TIMEOUT_COUNT (100000)
#endif
#ifndef PHY_STABILITY_DELAY_US
#define PHY_STABILITY_DELAY_US (0U)
#endif

#ifndef MAC_ADDRESS
#define MAC_ADDRESS {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x60}
#endif

#define BROADCAST_ADDRESS {0xff,0xff,0xff,0xff,0xff,0xff}
const uint8_t g_broadcastAddr[6] = BROADCAST_ADDRESS;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Build ENET broadcast frame. */
static void ENET_BuildBroadCastFrame(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Buffer descriptors should be in non-cacheable region and should be align to "ENET_BUFF_ALIGNMENT". */
AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);
/*! @brief The data buffers can be in cacheable region or in non-cacheable region.
 * If use cacheable region, the alignment size should be the maximum size of "CACHE LINE SIZE" and "ENET_BUFF_ALIGNMENT"
 * If use non-cache region, the alignment size is the "ENET_BUFF_ALIGNMENT".
 */
SDK_ALIGN(uint8_t g_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(uint8_t g_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);

enet_handle_t g_handle;
uint8_t g_frame[ENET_DATA_LENGTH + 14];

/*! @brief The MAC address for ENET device. */
uint8_t g_macAddr[6] = MAC_ADDRESS;

/*! @brief Enet PHY and MDIO interface handler. */
static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

/*---------------------------------------------------------------------------*/
//void HAL_ETH_MspInit(ETH_HandleTypeDef* heth)
//{
//  GPIO_InitTypeDef GPIO_InitStruct;
//
//  if (heth->Instance == ETH)
//  {
//    /* Ethernet clock enable */
//    __HAL_RCC_ETH_CLK_ENABLE();
//
//    /* ETH GPIO Configuration:
//     * PC1      ------> ETH_MDC
//     * PA1      ------> ETH_REF_CLK
//     * PA2      ------> ETH_MDIO
//     * PA7      ------> ETH_CRS_DV
//     * PC4      ------> ETH_RXD0
//     * PC5      ------> ETH_RXD1
//     * PB13     ------> ETH_TXD1
//     * PG11     ------> ETH_TX_EN
//     * PG13     ------> ETH_TXD0
//     */
//    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_13;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
//    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//  }
//}


/*---------------------------------------------------------------------------*/
//void HAL_ETH_MspDeInit(ETH_HandleTypeDef* heth)
//{
//  if (heth->Instance == ETH)
//  {
//    /* Ethernet clock disable */
//    __HAL_RCC_ETH_CLK_DISABLE();
//
//    /* Reset Ethernet GPIO pin configuration. */
//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
//    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_11 | GPIO_PIN_13);
//  }
//}


void BOARD_InitModuleClock(void)
{
    const clock_sys_pll1_config_t sysPll1Config = {
        .pllDiv2En = true,
    };
    CLOCK_InitSysPll1(&sysPll1Config);
    clock_root_config_t rootCfg = {.mux = 4, .div = 10}; /* Generate 50M root clock. */
    CLOCK_SetRootClock(kCLOCK_Root_Enet1, &rootCfg);
}

static void ENET_BuildBroadCastFrame(void)
{
    uint32_t count  = 0;
    uint32_t length = ENET_DATA_LENGTH - 14;

    for (count = 0; count < 6U; count++)
    {
        g_frame[count] = 0xFFU;
    }
    memcpy(&g_frame[6], &g_macAddr[0], 6U);
    g_frame[12] = (length >> 8) & 0xFFU;
    g_frame[13] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        g_frame[count + 14] = count % 0xFFU;
    }
}

/*---------------------------------------------------------------------------*/
void netdev_init(void)
{
    enet_config_t config;
    phy_config_t phyConfig = {0};
    uint32_t length        = 0;
    bool link              = false;
    bool autonego          = false;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t testTxNum = 0;
    status_t status;
    enet_data_error_stats_t eErrStatic;
    volatile uint32_t count = 0;
    
        /* Hardware Initialization. */
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

  /* Store the default MAC address. */
//  macAddress.addr[0] = NETDEV_DEFAULT_MACADDR0;
//  macAddress.addr[1] = NETDEV_DEFAULT_MACADDR1;
//  macAddress.addr[2] = NETDEV_DEFAULT_MACADDR2;
//  macAddress.addr[3] = NETDEV_DEFAULT_MACADDR3;
//  macAddress.addr[4] = NETDEV_DEFAULT_MACADDR4;
//  macAddress.addr[5] = NETDEV_DEFAULT_MACADDR5;
    
    
//  /* Initialize Ethernet. */
//  heth.Instance = ETH;
//  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
//  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
//  heth.Init.MACAddr = &(macAddress.addr)[0];
//  heth.Init.RxMode = ETH_RXPOLLING_MODE;
//  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
//  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
//  (void)HAL_ETH_Init(&heth);
//
//  /* Initialize Tx Descriptors list: Chain Mode */
//  HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
//  /* Initialize Rx Descriptors list: Chain Mode  */
//  HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
//  /* Enable MAC and DMA transmission and reception */
//  HAL_ETH_Start(&heth);
    
    
      /* Configure the MAC address */
  macAddress.addr[0] = g_macAddr[0];
  macAddress.addr[1] = g_macAddr[1];
  macAddress.addr[2] = g_macAddr[2];
  macAddress.addr[3] = g_macAddr[3];
  macAddress.addr[4] = g_macAddr[4];
  macAddress.addr[5] = g_macAddr[5];
  
  
    BOARD_InitModuleClock();
    /* 50M ENET_REF_CLOCK output to PHY and ENET module. */
    IOMUXC_GPR->GPR4 |= IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK;
    /* Errata ERR050396: For some bus masters, writing access to CM7 TCM is handled by a NIC301 block which does
     * not support sparse write conversion. It results in a data corruption when sparse writing to CM7 TCM happens.
     * For ENET: If CM7 TCM is the destination for writing, IOMUXC_GPR_GPR28[CACHE_ENET] should be cleared. If
     * IOMUXC_GPR_GPR28[CACHE_ENET] is set, write buffers must be placed in OCRAM or external RAM. */
    IOMUXC_GPR->GPR28 &= (~IOMUXC_GPR_GPR28_CACHE_ENET_MASK);
    GPIO_PinInit(GPIO9, 11, &gpio_config);
    GPIO_PinInit(GPIO12, 12, &gpio_config);
    /* Pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO9, 11, 1);
    GPIO_WritePinOutput(GPIO12, 12, 0);
    SDK_DelayAtLeastUs(10000, CLOCK_GetFreq(kCLOCK_CpuClk));
    GPIO_WritePinOutput(GPIO12, 12, 1);
    SDK_DelayAtLeastUs(6, CLOCK_GetFreq(kCLOCK_CpuClk));
    PRINTF("\r\nENET OpenBLT start.\r\n");

    /* Prepare the buffer configuration. */
    enet_buffer_config_t buffConfig[] = {{
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        &g_rxBuffDescrip[0],
        &g_txBuffDescrip[0],
        &g_rxDataBuff[0][0],
        &g_txDataBuff[0][0],
        true,
        true,
        NULL,
    }};
    
    /* Get default configuration. */
    /*
     * config.miiMode = kENET_RmiiMode;
     * config.miiSpeed = kENET_MiiSpeed100M;
     * config.miiDuplex = kENET_MiiFullDuplex;
     * config.rxMaxFrameLen = ENET_FRAME_MAX_FRAMELEN;
     */
    ENET_GetDefaultConfig(&config);

    /* The miiMode should be set according to the different PHY interfaces. */
#ifdef EXAMPLE_PHY_INTERFACE_RGMII
    config.miiMode = kENET_RgmiiMode;
#else
    config.miiMode = kENET_RmiiMode;
#endif
    phyConfig.phyAddr               = EXAMPLE_PHY_ADDRESS;
    phyConfig.autoNeg               = true;
    mdioHandle.resource.base        = EXAMPLE_ENET;
    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;
    /* Initialize PHY and wait auto-negotiation over. */
    PRINTF("Wait for PHY init...\r\n");
    do
    {
        status = PHY_Init(&phyHandle, &phyConfig);
        if (status == kStatus_Success)
        {
            PRINTF("Wait for PHY link up...\r\n");
            /* Wait for auto-negotiation success and link up */
            count = PHY_AUTONEGO_TIMEOUT_COUNT;
            do
            {
                PHY_GetAutoNegotiationStatus(&phyHandle, &autonego);
                PHY_GetLinkStatus(&phyHandle, &link);
                if (autonego && link)
                {
                    PRINTF("PHY Auto-negotiation success\r\n");
                    break;
                }
            } while (--count);
            if (!autonego)
            {
                PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
            }
        }
    } while (!(link && autonego));
#if PHY_STABILITY_DELAY_US
    /* Wait a moment for PHY status to be stable. */
    SDK_DelayAtLeastUs(PHY_STABILITY_DELAY_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
#endif

    /* Get the actual PHY link speed. */
    PHY_GetLinkSpeedDuplex(&phyHandle, &speed, &duplex);
    /* Change the MII speed and duplex for actual link status. */
    config.miiSpeed  = (enet_mii_speed_t)speed;
    config.miiDuplex = (enet_mii_duplex_t)duplex;

    ENET_Init(EXAMPLE_ENET, &g_handle, &config, &buffConfig[0], &g_macAddr[0], EXAMPLE_CLOCK_FREQ);
    ENET_ActiveRead(EXAMPLE_ENET);
}


/*---------------------------------------------------------------------------*/
void netdev_init_mac(void)
{
  uip_setethaddr(macAddress);
}


/*---------------------------------------------------------------------------*/
void netdev_get_mac(unsigned char * mac_addr)
{
  mac_addr[0] = macAddress.addr[0];
  mac_addr[1] = macAddress.addr[1];
  mac_addr[2] = macAddress.addr[2];
  mac_addr[3] = macAddress.addr[3];
  mac_addr[4] = macAddress.addr[4];
  mac_addr[5] = macAddress.addr[5];
}


/*---------------------------------------------------------------------------*/
unsigned int netdev_read(void)
{
    enet_config_t config;
    phy_config_t phyConfig = {0};
    uint32_t length        = 0;
    bool link              = false;
    bool autonego          = false;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t testTxNum = 0;
    status_t status;
    enet_data_error_stats_t eErrStatic;
    volatile uint32_t count = 0;
    
  unsigned int result = 0;
//  uint16_t  len = 0;
//  uint8_t * buffer;
//  __IO ETH_DMADescTypeDef *dmarxdesc;
//
//  /* Check if a new frame was received. */
//  if (HAL_ETH_GetReceivedFrame(&heth) == HAL_OK)
//  {
//    /* Obtain the size of the packet and a pointer to the buffer with packet data. */
//    len = heth.RxFrameInfos.length;
//    buffer = (uint8_t *)heth.RxFrameInfos.buffer;
//    /* Copy the received packet data into the uip buffer. */
//    memcpy(uip_buf, buffer, len);
//    /* Obtain pointer to the reception DMA descriptor. */
//    dmarxdesc = heth.RxFrameInfos.FSRxDesc;
//    /* Give the buffer back to DMA by setting the Own-bit in the reception descriptor. */
//    dmarxdesc->Status |= ETH_DMARXDESC_OWN;
//    /* Clear the segment count. */
//    heth.RxFrameInfos.SegCount =0;
//    /* When reception buffer unavailable flag is set, clear it and resume reception. */
//    if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
//    {
//      /* Clear RBUS ETHERNET DMA flag */
//      heth.Instance->DMASR = ETH_DMASR_RBUS;
//      /* Resume DMA reception */
//      heth.Instance->DMARPDR = 0;
//    }
//    /* Update the result. */
//    result = len;
//  }
  
          /* Get the Frame size */
        status = ENET_GetRxFrameSize(&g_handle, &length, 0);
        /* Call ENET_ReadFrame when there is a received frame. */
        if (length != 0)
        {
            
            /* Received valid frame. Deliver the rx buffer with the size equal to length. */
            uint8_t *data = (uint8_t *)malloc(length);
            status        = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, data, length, 0, NULL);
            if (status == kStatus_Success)
            {
#if 0
                if(memcmp(g_broadcastAddr, data, 6) != 0)
                {
                  
                    PRINTF(" A frame received. the length %d ", length);
                    PRINTF(" Dest Address %02x:%02x:%02x:%02x:%02x:%02x Src Address %02x:%02x:%02x:%02x:%02x:%02x \r\n",
                       data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
                       data[10], data[11]);

                }
                else
                {
                  //¹ã²¥°ü
                    /*
                    PRINTF(" A frame received. the length %d ", length);
                    PRINTF(" Dest Address %02x:%02x:%02x:%02x:%02x:%02x Src Address %02x:%02x:%02x:%02x:%02x:%02x \r\n",
                       data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
                       data[10], data[11]);
                    */
                }
#endif
                /* Copy the received packet data into the uip buffer. */
                memcpy(uip_buf, data, length);
                /* Update the result. */
                result = length;
            }
            free(data);
        }
        else if (status == kStatus_ENET_RxFrameError)
        {
            /* Update the received buffer when error happened. */
            /* Get the error information of the received g_frame. */
            ENET_GetRxErrBeforeReadFrame(&g_handle, &eErrStatic, 0);
            /* update the receive buffer. */
            ENET_ReadFrame(EXAMPLE_ENET, &g_handle, NULL, 0, 0, NULL);
            result = 0;
        }
        
  /* Give the result back to the caller. */
  return result;
}


/*---------------------------------------------------------------------------*/
void netdev_send(void)
{
//  uint8_t * buffer;
//  __IO ETH_DMADescTypeDef *DmaTxDesc;
//  uint32_t framelength;
//  uint32_t timeout;
//  ErrorStatus errorStatus = SUCCESS;

    enet_config_t config;
    phy_config_t phyConfig = {0};
    uint32_t length        = 0;
    bool link              = false;
    bool autonego          = false;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t testTxNum = 0;
    status_t status;
    enet_data_error_stats_t eErrStatic;
    volatile uint32_t count = 0;
    
    uint32_t framelength;
    uint8_t * buffer;
    
    static uint32_t TxNum = 0;
    
//  /* Obtain pointer to the transmission DMA descriptor. */
//  DmaTxDesc = heth.TxDesc;
//  /* Set timeout time to wait for the DMA buffer to become available. */
//  timeout = TimerGet() + NETDEV_TX_PACKET_TIMEOUT_MS;
//  /* Only continue with packet transmission of the buffer is available. */
//  while ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
//  {
//    CopService();
//    /* Break loop upon timeout. This would indicate a hardware failure. */
//    if (TimerGet() > timeout)
//    {
//      /* Update the error status. */
//      errorStatus = ERROR;
//      break;
//    }
//  }
  /* Only continue with transmission if not error was detected. */
//  if (errorStatus == SUCCESS)
//  {
//    /* Store the framelength. */
//    framelength = uip_len;
//    /* Obtain pointer to write the packet data to. */
//    buffer = (uint8_t *)(heth.TxDesc->Buffer1Addr);
//    /* Copy the packet data to the buffer. */
//    memcpy(buffer, uip_buf, framelength);
//    /* Prepare transmit descriptors to give to DMA. */
//    HAL_ETH_TransmitFrame(&heth, framelength);
//  }

  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to
   * resume transmission.
   */
//  if ((heth.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
//  {
//    /* Clear TUS ETHERNET DMA flag. */
//    heth.Instance->DMASR = ETH_DMASR_TUS;
//    /* Resume DMA transmission. */
//    heth.Instance->DMATPDR = 0;
//  }
  
            if (kStatus_Success == PHY_GetLinkStatus(&phyHandle, &link))
            {
                if (link)
                {
                    /* Store the framelength. */
                    framelength = uip_len;

                   /* Copy the packet data to the buffer. */
                   memcpy(&g_frame[0], uip_buf, framelength);
                    
                    if (kStatus_Success ==
                        ENET_SendFrame(EXAMPLE_ENET, &g_handle, &g_frame[0], framelength, 0, false, NULL))
                    {
                        //PRINTF("The %d frame transmitted success!\r\n", TxNum++);
                    }
                    else
                    {
                        //PRINTF(" \r\nTransmit frame failed!\r\n");
                    }
                }
            }
}


