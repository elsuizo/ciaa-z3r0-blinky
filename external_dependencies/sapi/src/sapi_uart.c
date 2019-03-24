/* Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

/* Date: 2016-02-26 */

/*==================[inclusions]=============================================*/

// emlib includes
#include "../inc/InitDevice.h"
#include "em_system.h"
//#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_leuart.h"
#include "em_usart.h"
#include "em_bus.h"

// Eric
#include "../inc/sapi_uart.h"
/* #include "../inc/string.h" */

/*==================[macros]=================================================*/


//#define UART_485_LPC LPC_USART0  /* UART0 (RS485/Profibus) */
//#define UART_USB_LPC LPC_USART2  /* UART2 (USB-UART) */
//#define UART_232_LPC LPC_USART3  /* UART3 (RS232) */

/*==================[typedef]================================================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal functions definition]==========================*/

/*==================[external data declaration]==============================*/

bool_t UART0_power   = OFF;
bool_t UART1_power   = OFF;
bool_t UART_LE_power = OFF;

/*==================[external functions declaration]=========================*/

waitForReceiveStringOrTimeoutState_t waitForReceiveStringOrTimeout(
   uartMap_t uart, waitForReceiveStringOrTimeout_t* instance ){

   uint8_t receiveByte;
   char receiveBuffer[100];

   switch( instance->state ){

      case UART_RECEIVE_STRING_CONFIG:

         delayConfig( &(instance->delay), instance->timeout );

         instance->stringIndex = 0;

         instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

      case UART_RECEIVE_STRING_RECEIVING:

         if( uartReadByte( uart, &receiveByte ) ){

            //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
            if( (instance->stringIndex) <= 100 ){
               receiveBuffer[instance->stringIndex] = receiveByte;
            }

            if( (instance->string)[(instance->stringIndex)] == receiveByte ){

               (instance->stringIndex)++;

               if( (instance->stringIndex) == (instance->stringSize - 1) ){
                  instance->state = UART_RECEIVE_STRING_RECEIVED_OK;

                  receiveBuffer[instance->stringIndex] = '\0';

                  uartWriteString( UART_DEBUG, receiveBuffer ); // TODO: DEBUG
                  uartWriteString( UART_DEBUG, "\r\n" );        // TODO: DEBUG
               }

            }

         }

         if( delayRead( &(instance->delay) ) ){
            instance->state = UART_RECEIVE_STRING_TIMEOUT;
            uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
         }

      break;

      case UART_RECEIVE_STRING_RECEIVED_OK:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

      case UART_RECEIVE_STRING_TIMEOUT:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

      default:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;
   }

   return instance->state;
}


bool_t waitForReceiveStringOrTimeoutBlocking(
   uartMap_t uart, char* string, uint16_t stringSize, tick_t timeout ){

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringOrTimeout_t waitText;
   waitForReceiveStringOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_CONFIG;

   waitText.state = UART_RECEIVE_STRING_CONFIG;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ){
      waitTextState = waitForReceiveStringOrTimeout( uart, &waitText );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ){
      retVal = FALSE;
   }

   return retVal;
}




waitForReceiveStringOrTimeoutState_t receiveBytesUntilReceiveStringOrTimeout(
   uartMap_t uart, waitForReceiveStringOrTimeout_t* instance,
   char* receiveBuffer, uint32_t* receiveBufferSize ){

   uint8_t receiveByte;
   static uint32_t i = 0;
   //uint32_t j = 0;

   switch( instance->state ){

      case UART_RECEIVE_STRING_CONFIG:

         delayConfig( &(instance->delay), instance->timeout );

         instance->stringIndex = 0;
         i = 0;

         instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

      case UART_RECEIVE_STRING_RECEIVING:

         if( uartReadByte( uart, &receiveByte ) ){

            //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
            if( i < *receiveBufferSize ){
               receiveBuffer[i] = receiveByte;
               i++;
            } else{
               instance->state = UART_RECEIVE_STRING_FULL_BUFFER;
               *receiveBufferSize = i;
               i = 0;
               return instance->state;
            }

            if( (instance->string)[(instance->stringIndex)] == receiveByte ){

               (instance->stringIndex)++;

               if( (instance->stringIndex) == (instance->stringSize - 1) ){
                  instance->state = UART_RECEIVE_STRING_RECEIVED_OK;
                  *receiveBufferSize = i;
                  /*
                  // TODO: For debug purposes
                  for( j=0; j<i; j++ ){
                     uartWriteByte( UART_DEBUG, receiveBuffer[j] );
                  }
                  uartWriteString( UART_DEBUG, "\r\n" );
                  */
                  i = 0;
               }

            }

         }

         if( delayRead( &(instance->delay) ) ){
            instance->state = UART_RECEIVE_STRING_TIMEOUT;
            uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
            *receiveBufferSize = i;
            i = 0;
         }

      break;

      case UART_RECEIVE_STRING_RECEIVED_OK:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

      case UART_RECEIVE_STRING_TIMEOUT:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

      case UART_RECEIVE_STRING_FULL_BUFFER:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

      default:
         instance->state = UART_RECEIVE_STRING_CONFIG;
      break;
   }

   return instance->state;
}


bool_t receiveBytesUntilReceiveStringOrTimeoutBlocking(
   uartMap_t uart, char* string, uint16_t stringSize,
   char* receiveBuffer, uint32_t* receiveBufferSize,
   tick_t timeout ){

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringOrTimeout_t waitText;
   waitForReceiveStringOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_CONFIG;

   waitText.state = UART_RECEIVE_STRING_CONFIG;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ){
      waitTextState = receiveBytesUntilReceiveStringOrTimeout(
                         uart, &waitText,
                         receiveBuffer, receiveBufferSize );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ){
      retVal = FALSE;
   }

   return retVal;
}



void uartConfig( uartMap_t uart, uint32_t baudRate ){
   
   switch(uart){
      case UART0:

         // Enable clock for USART0
         CMU_ClockEnable(cmuClock_USART0, true);

         // USART_InitAsync
         USART_InitAsync_TypeDef initasync0 = USART_INITASYNC_DEFAULT;
         initasync0.baudrate = baudRate;
         initasync0.databits = usartDatabits8;
         initasync0.parity = usartNoParity;
         initasync0.stopbits = usartStopbits1;
         initasync0.oversampling = usartOVS16;
         #if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
            initasync0.mvdis = 0;
            initasync0.prsRxEnable = 0;
            initasync0.prsRxCh = 0;
         #endif
         USART_InitAsync(USART0, &initasync0);
         // USART_InitPrsTrigger
         USART_PrsTriggerInit_TypeDef initprs0 = USART_INITPRSTRIGGER_DEFAULT;
         initprs0.rxTriggerEnable = 0;
         initprs0.txTriggerEnable = 0;
         initprs0.prsTriggerChannel = usartPrsTriggerCh0;
         USART_InitPrsTrigger(USART0, &initprs0);

         // Port E Configuration for USART0
         // Pin PE10 is configured to Push-pull (USART0_TX)
         GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
               | GPIO_P_MODEH_MODE10_PUSHPULL;
         // Pin PE11 is configured to Input enabled (USART0_RX)
         GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
               | GPIO_P_MODEH_MODE11_INPUT;

         // Route Configuration for USART0
         // Enable signals RX, TX
         USART0->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

         // Set Power ON
         UART0_power = ON;
      break;

      case UART1:

         // Enable clock for USART1
         CMU_ClockEnable( cmuClock_USART1, true );      
               
         // USART1 InitAsync    
         USART_InitAsync_TypeDef initasync1 = USART_INITASYNC_DEFAULT;
         initasync1.baudrate = baudRate;
         initasync1.databits = usartDatabits8;
         initasync1.parity = usartNoParity;
         initasync1.stopbits = usartStopbits1;
         initasync1.oversampling = usartOVS16;
         #if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
         initasync1.mvdis = 0;
         initasync1.prsRxEnable = 0;
         initasync1.prsRxCh = 0;
         #endif      
         USART_InitAsync(USART1, &initasync1);

         // USART1 InitPrsTrigger
         USART_PrsTriggerInit_TypeDef initprs1 = USART_INITPRSTRIGGER_DEFAULT;
         initprs1.rxTriggerEnable = 0;
         initprs1.txTriggerEnable = 0;
         initprs1.prsTriggerChannel = usartPrsTriggerCh0;
         USART_InitPrsTrigger(USART1, &initprs1);
      
         // Port C Configuration for USART1
         // Pin PC0 is configured to Push-pull
         GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | 
                            GPIO_P_MODEL_MODE0_PUSHPULL;
         // Pin PC1 is configured to Input enabled
         GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | 
                            GPIO_P_MODEL_MODE1_INPUT;
      
         // Route Configuration for USART1
      	// Enable signals RX, TX
         USART1->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;
         
         // Set Power ON
         UART1_power = ON;
      break;

      case UART_LE: // baudRate hasta 32000

         // Enable clock for LEUART0
         CMU_ClockEnable( cmuClock_LEUART0, true );

         // LEUART0 initialization
         LEUART_Init_TypeDef initleuart = LEUART_INIT_DEFAULT;
      
         initleuart.enable = leuartEnable;
         initleuart.baudrate = baudRate; // baudRate hasta 32000
         initleuart.databits = leuartDatabits8;
         initleuart.parity = leuartNoParity;
         initleuart.stopbits = leuartStopbits1;
      
         LEUART_Init(LEUART0, &initleuart);
      
         // Configuring non-standard properties
         LEUART_TxDmaInEM2Enable(LEUART0, 0);
         LEUART_RxDmaInEM2Enable(LEUART0, 0);


         // Port C Configuration for LEUART0 - location 5
         // Pin PC14 is configured to Push-pull (LEUART0_TX)
         //GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
         //      | GPIO_P_MODEH_MODE14_PUSHPULL;
         // Pin PC15 is configured to Input enabled (LEUART0_RX)
         //GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
         //      | GPIO_P_MODEH_MODE15_INPUT;
         // Route Configuration for LEUART0
         // Module LEUART0 is configured to location 5
         //LEUART0->ROUTE = (LEUART0->ROUTE & ~_LEUART_ROUTE_LOCATION_MASK)
         //      | LEUART_ROUTE_LOCATION_LOC5;
               
         
         // Port B Configuration for LEUART0 - location 1
         // Pin PB13 is configured to Push-pull
         GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | 
                            GPIO_P_MODEH_MODE13_PUSHPULL;
         // Pin PB14 is configured to Input enabled 
         GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE14_MASK) | 
                           GPIO_P_MODEH_MODE14_INPUT;    
         // Route Configuration for LEUART0                       
         // Module LEUART0 is configured to location 1
         LEUART0->ROUTE = (LEUART0->ROUTE & ~_LEUART_ROUTE_LOCATION_MASK) | 
                          LEUART_ROUTE_LOCATION_LOC1;                          
                          
               
         // Enable signals RX, TX
         LEUART0->ROUTE |= LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN;

         // Set Power ON
         UART_LE_power = ON;
      break;
   }
}


bool_t uartReadByte( uartMap_t uart, uint8_t* receivedByte ){

   bool_t retVal = TRUE;

   switch(uart){
   case UART0:
      if ( (USART0->STATUS & USART_STATUS_RXDATAV) ) {
         *receivedByte = (uint8_t)USART0->RXDATA;
      } else{
         retVal = FALSE;
      }
   break;
   case UART1:
      if ( (USART1->STATUS & USART_STATUS_RXDATAV) ) {
         *receivedByte = (uint8_t)USART1->RXDATA;
      } else{
         retVal = FALSE;
      }
   break;
   case UART_LE:
      if ( (LEUART0->STATUS & LEUART_STATUS_RXDATAV) ) {
         *receivedByte = (uint8_t)LEUART0->RXDATA;
      } else{
         retVal = FALSE;
      }
   break;
   }

   return retVal;
}


void uartWriteByte( uartMap_t uart, uint8_t byte ){

   switch( uart ){
   case UART0:
      if(UART0_power){
         USART_Tx( USART0, byte );
      }
   break;
   case UART1:
      if(UART1_power){
         USART_Tx( USART1, byte );
      }
   break;
   case UART_LE:
      if(UART_LE_power){
         LEUART_Tx( LEUART0, byte );
      }
   break;
   }
}


void uartWriteString( uartMap_t uart, char* str ){
   while(*str != 0){
     uartWriteByte( uart, (uint8_t)*str );
     str++;
   }
}



// Enable or disable the peripheral energy and clock
bool_t uartPowerSet( uartMap_t uart, bool_t power ){

   switch(uart){

      case UART0:
         if( power ){

            // Port E Configuration for USART0
            // Pin PE10 is configured to Push-pull (USART0_TX)
            GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                  | GPIO_P_MODEH_MODE10_PUSHPULL;
            // Pin PE11 is configured to Input enabled (USART0_RX)
            GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                  | GPIO_P_MODEH_MODE11_INPUT;

            // Enable clock for USART0
            CMU_ClockEnable( cmuClock_USART0, true );

            // Enable USART0
            USART_Enable( USART0, usartEnable );

            UART0_power = ON;

         } else{

            // Disable USART0
            USART_Enable( USART0, usartDisable );

            // Disable clock for USART0
            CMU_ClockEnable( cmuClock_USART0, false );

            // Port E Configuration for USART0
            // Pin PE10 is configured to Push-pull (USART0_TX) // @ Important!! Si no deshabilito este pin consume mucho!!
            GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK)
                  | GPIO_P_MODEH_MODE10_DISABLED;
            // Pin PE11 is configured to Input enabled (USART0_RX)
            GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK)
                  | GPIO_P_MODEH_MODE11_DISABLED;

            UART0_power = OFF;
         }
      break;

      case UART1:
         if( power ){
            // Port C Configuration for USART1
            // Pin PC0 is configured to Push-pull
            GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | 
                               GPIO_P_MODEL_MODE0_PUSHPULL;
            // Pin PC1 is configured to Input enabled
            GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | 
                               GPIO_P_MODEL_MODE1_INPUT;

            // Enable clock for USART1
            CMU_ClockEnable( cmuClock_USART1, true );

            // Enable USART1
            USART_Enable( USART1, usartEnable );

            UART1_power = ON;

         } else{
            // Disable USART1
            USART_Enable( USART1, usartDisable );

            // Disable clock for USART1
            CMU_ClockEnable( cmuClock_USART1, false );
            
            // Port C Configuration for USART1
            // Pin PC0 is configured to Disabled
            GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | 
                               GPIO_P_MODEL_MODE0_DISABLED;
            // Pin PC1 is configured to Disabled
            GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | 
                               GPIO_P_MODEL_MODE1_DISABLED;

            UART1_power = OFF;
         }
      break;

      case UART_LE:
         if( power ){

            // Port C Configuration for LEUART0
            // Pin PC14 is configured to Push-pull (LEUART0_TX)
            GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                  | GPIO_P_MODEH_MODE14_PUSHPULL;
            // Pin PC15 is configured to Input enabled (LEUART0_RX)
            GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                  | GPIO_P_MODEH_MODE15_INPUT;

            // Enable clock for LEUART0
            CMU_ClockEnable( cmuClock_LEUART0, true );

            // Enable USART0
            LEUART_Enable( LEUART0, leuartEnable );

            UART_LE_power = ON;

         } else{

            // Disable LEUART0
            LEUART_Enable( LEUART0, leuartDisable );

            // Disable clock for LEUART0
            CMU_ClockEnable( cmuClock_LEUART0, false );

            // Port C Configuration for LEUART0 // @ Important!! Si no deshabilito este pin consume mucho!!
            // Pin PC14 is configured to Disabled (LEUART0_TX)
            GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK)
                  | GPIO_P_MODEH_MODE14_DISABLED;
            // Pin PC15 is configured to Disabled (LEUART0_RX)
            GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK)
                  | GPIO_P_MODEH_MODE15_DISABLED;

            UART_LE_power = OFF;
         }
      break;

      default:
      break;
   }

   return TRUE;
}

/*==================[ISR external functions definition]======================*/

/*
__attribute__ ((section(".after_vectors")))

// 0x28 0x000000A0 - Handler for ISR UART0 (IRQ 24)
void UART0_IRQHandler(void){
}

// 0x2a 0x000000A8 - Handler for ISR UART2 (IRQ 26)
void UART2_IRQHandler(void){
}

// 0x2b 0x000000AC - Handler for ISR UART3 (IRQ 27)
void UART3_IRQHandler(void){
   //if (Chip_UART_ReadLineStatus(UART_232) & UART_LSR_RDR) {
//      receivedByte = Chip_UART_ReadByte(UART_232);
   //}
}
*/


/*==================[example 1]==============================================*/

/*
   uint32_t i = 0;
   uint32_t size = 100;
   char receiveBuffer[100];
   memset(receiveBuffer, 0, 100); // Lleno el vector con ceros


   char *str1 = "Borland International", *str2 = "pepe", *ptr;
   ptr = strstr(str1, str2);
   if( ptr != 0 ){
      uartWriteString( UART_DEBUG, "cadena:" );
      uartWriteString( UART_DEBUG, ptr );
      uartWriteString( UART_DEBUG, "." );
   } else{
      uartWriteString( UART_DEBUG, "no encontro." );
   }

   consolePrintEnter();

   char *str3 = "Borland International", *str4 = "nation";
   ptr = strstr(str3, str4);
   if( ptr != 0 ){
      uartWriteString( UART_DEBUG, "cadena:" );
      uartWriteString( UART_DEBUG, ptr );
      uartWriteString( UART_DEBUG, "." );
   } else{
      uartWriteString( UART_DEBUG, "no encontro." );
   }


   consolePrintEnter();
   consolePrintEnter();
   consolePrintEnter();

   uint8_t value = 0;

   if( asciiHexToUint8( "15", &value ) ){
      uartWriteString( UART_DEBUG, "valor: 0x" );
      uartWriteString( UART_DEBUG, uintToAsciiHex( value, 8 ) );
      uartWriteString( UART_DEBUG, "." );
   } else{
      uartWriteString( UART_DEBUG, "invalido." );
   }

   while( TRUE ){

      if( receiveBytesUntilReceiveStringOrTimeoutBlocking(
             UART_DEBUG, "\r\n", 2,
             receiveBuffer, &size,
             2500 )
      ){
         consolePrintlnString( "llego" );
         for( i=0; i<size; i++ ){
            uartWriteByte( UART_DEBUG, receiveBuffer[i] );
            receiveBuffer[i] = 0;
         }
         uartWriteString( UART_DEBUG, "\r\n" );
         size = 100;
      }
   }

*/


/*==================[end of file]============================================*/
