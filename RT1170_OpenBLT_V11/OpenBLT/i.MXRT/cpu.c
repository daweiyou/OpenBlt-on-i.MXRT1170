/************************************************************************************//**
* \file         Source/ARMCM7_STM32F7/cpu.c
* \brief        Bootloader cpu module source file.
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
//#include "stm32f7xx.h"                           /* STM32 CPU and HAL header           */
#include "fsl_common.h"
#include "fsl_debug_console.h"

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define CPU_USER_APP_START_ADDR (0x30010000) //64KB offset
/** \brief Pointer to the user program's reset vector. */
#define CPU_USER_PROGRAM_STARTADDR_PTR    ((blt_addr)(CPU_USER_APP_START_ADDR+0x00000004))//((blt_addr)(NvmGetUserProgBaseAddress() + 0x00000004))
/** \brief Pointer to the user program's vector table. */
#define CPU_USER_PROGRAM_VECTABLE_OFFSET  ((blt_addr)(CPU_USER_APP_START_ADDR))//((blt_int32u)NvmGetUserProgBaseAddress())


/****************************************************************************************
* Hook functions
****************************************************************************************/
#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
extern blt_bool CpuUserProgramStartHook(void);
#endif


/************************************************************************************//**
** \brief     Initializes the CPU module.
** \return    none.
**
****************************************************************************************/
void CpuInit(void)
{
  /* bootloader runs in polling mode so disable the global interrupts. this is done for
   * safety reasons. if the bootloader was started from a running user program, it could 
   * be that the user program did not properly disable the interrupt generation of 
   * peripherals.
   */
  CpuIrqDisable();
} /*** end of CpuInit ***/


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
typedef enum _shutdown_types
{
    kShutdownType_Shutdown = 0,
    kShutdownType_Cleanup = 1,
    kShutdownType_Reset = 2,
} shutdown_type_t;
// See bl_shutdown_cleanup.h for documentation of this function.
void shutdown_cleanup(shutdown_type_t shutdown)
{
//    if (shutdown != kShutdownType_Reset)
//    {
//        // Clear (flush) the flash cache.
//#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
//#if !BL_DEVICE_IS_LPC_SERIES
//        //flash_cache_clear(NULL);
//    FTFx_CACHE_ClearCachePrefetchSpeculation(g_bootloaderContext.allFlashCacheState, true);
//    FTFx_CACHE_ClearCachePrefetchSpeculation(g_bootloaderContext.allFlashCacheState, false);
//#endif // !BL_DEVICE_IS_LPC_SERIES
//#endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
//    }
//
//    if (shutdown != kShutdownType_Cleanup)
//    {
//        // Shutdown all peripherals because they could be active
//        uint32_t i;
//        for (i = 0; g_peripherals[i].typeMask != 0; i++)
//        {
//            if (g_peripherals[i].controlInterface->shutdown)
//            {
//                g_peripherals[i].controlInterface->shutdown(&g_peripherals[i]);
//            }
//        }
//    }
//
//    // If we are permanently exiting the bootloader, there are a few extra things to do.
//    if (shutdown == kShutdownType_Shutdown)
//    {
//        // Turn off global interrupt
//        lock_acquire();
//
//        // Shutdown microseconds driver.
//        microseconds_shutdown();
//
//        // Disable force ROM.
//#if defined(RCM_FM_FORCEROM_MASK)
//        RCM->FM = ((~RCM_FM_FORCEROM_MASK) & RCM->FM) | RCM_FM_FORCEROM(0);
//#elif defined(SMC_FM_FORCECFG_MASK)
//#if defined(SMC0)
//        SMC0->FM = ((~SMC_FM_FORCECFG_MASK) & SMC0->FM) | SMC_FM_FORCECFG(0);
//#else
//        SMC->FM = ((~SMC_FM_FORCECFG_MASK) & SMC->FM) | SMC_FM_FORCECFG(0);
//#endif
//#endif // defined(RCM_FM_FORCEROM_MASK)
//
//        // Clear status register (bits are w1c).
//#if defined(RCM_MR_BOOTROM_MASK)
//        RCM->MR = ((~RCM_MR_BOOTROM_MASK) & RCM->MR) | RCM_MR_BOOTROM(3);
//#elif defined(SMC_MR_BOOTCFG_MASK)
//#if defined(SMC0)
//        SMC0->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC0->MR) | SMC_MR_BOOTCFG(3);
//#else
//        SMC->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC->MR) | SMC_MR_BOOTCFG(3);
//#endif
//#endif // defined(RCM_MR_BOOTROM_MASK)
//
//        init_interrupts();
//
//        // Set the VTOR to default.
//        SCB->VTOR = kDefaultVectorTableAddress;
//
//        // Restore clock to default before leaving bootloader.
//        configure_clocks(kClockOption_ExitBootloader);
//
//        // De-initialize hardware such as disabling port clock gate
//        deinit_hardware();
//
//        // Restore global interrupt.
//        __enable_irq();
//
//#if BL_FEATURE_BYPASS_WATCHDOG
//        // De-initialize watchdog
//        bootloader_watchdog_deinit();
//#endif // BL_FEATURE_BYPASS_WATCHDOG
//    }
//
//    // Memory barriers for good measure.
//    __ISB();
//    __DSB();
}

//! @brief Exits bootloader and jumps to the user application.
//static void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer)
//{
  
//#if BL_FEATURE_OTFAD_MODULE
//    quadspi_cache_clear();
//    oftfad_resume_as_needed();
//#endif
//
//    shutdown_cleanup(kShutdownType_Shutdown);
//
//    // Create the function call to the user application.
//    // Static variables are needed since changed the stack pointer out from under the compiler
//    // we need to ensure the values we are using are not stored on the previous stack
//    static uint32_t s_stackPointer = 0;
//    s_stackPointer = stackPointer;
//    static void (*farewellBootloader)(void) = 0;
//    farewellBootloader = (void (*)(void))applicationAddress;
//
//    // Set the VTOR to the application vector table address.
//    SCB->VTOR = (uint32_t)APP_VECTOR_TABLE;
//
//    // Set stack pointers to the application stack pointer.
//    __set_MSP(s_stackPointer);
//    __set_PSP(s_stackPointer);
//
//    // Jump to the application.
//    farewellBootloader();
//    // Dummy fcuntion call, should never go to this fcuntion call
//    shutdown_cleanup(kShutdownType_Shutdown);
//}

/************************************************************************************//**
** \brief     Starts the user program, if one is present. In this case this function
**            does not return.
** \return    none.
**
****************************************************************************************/
void CpuStartUserProgram(void)
{
  void (*pProgResetHandler)(void);

  PRINTF("%s@%d\r\n", __func__, __LINE__);

  /* check if a user program is present by verifying the checksum */
  if (NvmVerifyChecksum() == BLT_FALSE)
  {
#if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
    /* bootloader will stay active so perform deferred initialization to make sure
     * the communication interface that were not yet initialized are now initialized.
     * this is needed to make sure firmware updates via these communication interfaces
     * will be possible.
     */
    ComDeferredInit();
#endif
    /* not a valid user program so it cannot be started */
    return;
  }
#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
  /* invoke callback */
  if (CpuUserProgramStartHook() == BLT_FALSE)
  {
  #if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
    /* bootloader will stay active so perform deferred initialization to make sure
     * the communication interface that were not yet initialized are now initialized.
     * this is needed to make sure firmware updates via these communication interfaces
     * will be possible.
     */
    ComDeferredInit();
  #endif
    /* callback requests the user program to not be started */
    return;
  }
#endif
#if (BOOT_COM_ENABLE > 0)
  /* release the communication interface */
  ComFree();
#endif
  /* reset the HAL */
  Deinit();
  /* reset the timer */
  TimerReset();
  /* remap user program's vector table */
  SCB->VTOR = CPU_USER_PROGRAM_VECTABLE_OFFSET;// & (blt_int32u)0x1FFFFF80;
  /* set the address where the bootloader needs to jump to. this is the address of
   * the 2nd entry in the user program's vector table. this address points to the
   * user program's reset handler.
   */
  pProgResetHandler = (void(*)(void))(*((blt_addr *)CPU_USER_PROGRAM_STARTADDR_PTR));
  /* The Cortex-M4 core has interrupts enabled out of reset. the bootloader
   * explicitly disables these for security reasons. Enable them here again, so it does 
   * not have to be done by the user program.
   */
  CpuIrqEnable();
  /* start the user program by activating its reset interrupt service routine */
  pProgResetHandler();
#if (BOOT_COM_DEFERRED_INIT_ENABLE > 0) && (BOOT_COM_ENABLE > 0)
  /* theoretically, the code never gets here because the user program should now be
   * running and the previous function call should not return. In case it did return
   * for whatever reason, make sure all communication interfaces are initialized so that
   * firmware updates can be started.
   */
  ComDeferredInit();
#endif
} /*** end of CpuStartUserProgram ***/


/************************************************************************************//**
** \brief     Copies data from the source to the destination address.
** \param     dest Destination address for the data.
** \param     src  Source address of the data.
** \param     len  length of the data in bytes.
** \return    none.
**
****************************************************************************************/
void CpuMemCopy(blt_addr dest, blt_addr src, blt_int16u len)
{
  blt_int8u *from, *to;

  /* set casted pointers */
  from = (blt_int8u *)src;
  to = (blt_int8u *)dest;

  /* copy all bytes from source address to destination address */
  while (len-- > 0)
  {
    /* store byte value from source to destination */
    *to++ = *from++;
    /* keep the watchdog happy */
    CopService();
  }
} /*** end of CpuMemCopy ***/


/************************************************************************************//**
** \brief     Sets the bytes at the destination address to the specified value.
** \param     dest Destination address for the data.
** \param     value Value to write.
** \param     len  Number of bytes to write.
** \return    none.
**
****************************************************************************************/
void CpuMemSet(blt_addr dest, blt_int8u value, blt_int16u len)
{
  blt_int8u *to;

  /* set casted pointer */
  to = (blt_int8u *)dest;

  /* set all bytes at the destination address to the specified value */
  while (len-- > 0)
  {
    /* set byte value */
    *to++ = value;
    /* keep the watchdog happy */
    CopService();
  }
} /*** end of CpuMemSet ***/


/*********************************** end of cpu.c **************************************/
