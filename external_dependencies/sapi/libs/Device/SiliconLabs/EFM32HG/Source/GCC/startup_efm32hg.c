/*
 * @file startup_efm32hg.c
 * @brief CMSIS Compatible EFM32HG startup file in C.
 *        Should be used with GCC 'GNU Tools ARM Embedded'
 * @version 5.0.0
 * Date:    12 June 2014
 *
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#include <stdint.h>

#include "../../Include/system_efm32hg.h"
/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern int main() __attribute__((noreturn));    /* Pre Main (C library entry point) */
extern void SystemInit (void);            /* CMSIS System Initialization      */


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);                          /* Default empty handler */
void Reset_Handler(void);                            /* Reset Handler */

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M Processor Exceptions */
void NMI_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));

/* Part Specific Interrupts */
void DMA_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_EVEN_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void ACMP0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_ODD_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_RX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void LEUART0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void PCNT0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void CMU_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void VCMP_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void MSC_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void AES_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void USART0_RX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void USART0_TX_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));



/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
  /* Cortex-M Exception Handlers */
  (pFunc)&__StackTop,                       /*      Initial Stack Pointer     */
  Reset_Handler,                            /*      Reset Handler             */
  NMI_Handler,                              /*      NMI Handler               */
  HardFault_Handler,                        /*      Hard Fault Handler        */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  SVC_Handler,                              /*      SVCall Handler            */
  Default_Handler,                          /*      Reserved                  */
  Default_Handler,                          /*      Reserved                  */
  PendSV_Handler,                           /*      PendSV Handler            */
  SysTick_Handler,                          /*      SysTick Handler           */

  /* External interrupts */

  DMA_IRQHandler,                       /*  0 - DMA       */
  GPIO_EVEN_IRQHandler,                       /*  1 - GPIO_EVEN       */
  TIMER0_IRQHandler,                       /*  2 - TIMER0       */
  ACMP0_IRQHandler,                       /*  3 - ACMP0       */
  ADC0_IRQHandler,                       /*  4 - ADC0       */
  I2C0_IRQHandler,                       /*  5 - I2C0       */
  GPIO_ODD_IRQHandler,                       /*  6 - GPIO_ODD       */
  TIMER1_IRQHandler,                       /*  7 - TIMER1       */
  USART1_RX_IRQHandler,                       /*  8 - USART1_RX       */
  USART1_TX_IRQHandler,                       /*  9 - USART1_TX       */
  LEUART0_IRQHandler,                       /*  10 - LEUART0       */
  PCNT0_IRQHandler,                       /*  11 - PCNT0       */
  RTC_IRQHandler,                       /*  12 - RTC       */
  CMU_IRQHandler,                       /*  13 - CMU       */
  VCMP_IRQHandler,                       /*  14 - VCMP       */
  MSC_IRQHandler,                       /*  15 - MSC       */
  AES_IRQHandler,                       /*  16 - AES       */
  USART0_RX_IRQHandler,                       /*  17 - USART0_RX       */
  USART0_TX_IRQHandler,                       /*  18 - USART0_TX       */
  USB_IRQHandler,                       /*  19 - USB       */
  TIMER2_IRQHandler,                       /*  20 - TIMER2       */

};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) {
  uint32_t *pSrc, *pDest;
  uint32_t *pTable __attribute__((unused));

  SystemInit();

  pSrc  = &__etext;
  pDest = &__data_start__;

  for ( ; pDest < &__data_end__ ; )
  {
    *pDest++ = *pSrc++;
  }

  pDest = &__bss_start__;

  for ( ; pDest < &__bss_end__ ; )
  {
    *pDest++ = 0ul;
  }

  main();
}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
  while(1);
}

