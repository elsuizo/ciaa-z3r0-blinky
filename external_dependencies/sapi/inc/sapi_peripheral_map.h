/* Copyright 2015, Eric Pernia.
 * Copyright 2016, Ian Olivieri.
 * Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2015-09-23 */

#ifndef _SAPI_PERIPHERALMAP_H_
#define _SAPI_PERIPHERALMAP_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/* ----- Begin Pin Config Structs NXP LPC4337 ----- */

typedef struct{
   int8_t port;
   int8_t pin;
} pinConfigLpc4337_t;

/* ------ End Pin Config Structs NXP LPC4337 ------ */


/* ------- Begin EDU-CIAA-NXP Peripheral Map ------ */

/* Defined for sapi_gpio.h */
typedef enum{

// EFM32HG
   PA0,  // Wakeup from EM4
   PA1, PA2, 
   PA8, PA9, PA10,
   
   PB7, PB8, 
   PB11, 
   PB13, PB14,
   
   PC0, PC1, PC2, PC3, PC4, 
   PC8, PC9, // Wakeup from EM4 - BOARD_PUSH_BUTTON_PB0
   PC10, // BOARD_PUSH_BUTTON_PB1
   // PC11, // Only exist in 222, USB_VREGI in 322
   // PC13, // Only exist in 222, USB_VREGO in 322   
   PC14, PC15,

   PD4,  // ADC CH4
   PD5,  // ADC CH5
   PD6,  // ADC CH6
   PD7,  // ADC CH7

   PE10, PE11,
   PE12, // ADC CH0
   PE13, // ADC CH1, Wakeup from EM4

   PF0,  // DEBUG_SWCLK
   PF1,  // Wakeup from EM4 - SWDIO DEBUG
   PF2,  // Wakeup from EM4
   PF3, PF4, PF5,
   
   SW  = PB8, 
   LED = PA8
} gpioMap_t;

/* Defined for sapi_adc.h */
typedef enum{
   CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7, ADC_TEMP=8
} adcMap_t;

/* Defined for sapi_idac.h */
typedef enum{
   IDAC
} idacMap_t;

/* Defined for sapi_uart.h */
typedef enum{
   UART0, UART1, UART_LE
} uartMap_t;

/*Defined for sapi_timer.h*/
//NOTE: if servo is enable (servoConfig used) the only available timer to use is TIMER0
/*
typedef enum{
   TIMER0, TIMER1, TIMER2, TIMER3
} timerMap_t;*/
typedef enum{
   TIMERCOMPAREMATCH0, TIMERCOMPAREMATCH1, TIMERCOMPAREMATCH2, TIMERCOMPAREMATCH3
} timerCompareMatch_t;

/*Defined for sapi_sct.h*/
// NOTE: CTOUT11 has no SCT mode associated, so it can't be used!
// NOTE: if pwm is enable (pwmConfig used) there will be no sct channels available
typedef enum{
   CTOUT0, CTOUT1, CTOUT2, CTOUT3, CTOUT4, CTOUT5, CTOUT6, CTOUT7, CTOUT8,
   CTOUT9, CTOUT10, CTOUT11, CTOUT12, CTOUT13
} sctMap_t;

/*Defined for sapi_pwm.h*/
typedef enum{
   PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9, PWM10
} pwmMap_t;

/*Defined for sapi_servo.h*/
typedef enum{
   SERVO0, SERVO1, SERVO2, SERVO3, SERVO4, SERVO5, SERVO6, SERVO7, SERVO8
} servoMap_t;

/*Defined for sapi_i2c.h*/
/* Comment because already defined in "i2c_18xx_43xx.h"*/

/*
typedef enum{
   I2C0
} i2cMap_t;
*/

typedef uint8_t i2cMap_t;

/* ------- End EDU-CIAA-NXP Peripheral Map -------- */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_PERIPHERALMAP_H_ */
