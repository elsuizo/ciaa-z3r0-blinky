/* Copyright 2016, Eric Pernia.
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

/* Date: 2017-04-17 */

/*==================[inclusions]=============================================*/

#include "../inc/sapi_print.h"   // <= own header

#include "../inc/sapi_uart.h"    // <= UART header

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

// Print uart configuration

void printSetUart( print_t* printer, uartMap_t uart ){
   *printer = uart;
}

void printConfigUart( print_t* printer, uartMap_t uart, uint32_t baudRate ){
   *printer = uart;
   uartConfig( uart, baudRate );
}


// Print String

void printChar( print_t printer, char character ){
   uartWriteByte( printer, (uint8_t)character );
}

void printString( print_t printer, char* string ){
   uartWriteString( printer, string );
}

void printEnter( print_t printer ){
   uartWriteString( printer, PRINT_ENTER_STRING );
}


// Print Integer

void printIntFormat( print_t printer, int64_t number, numberFormat_t format ){

   char strNumber[65];

   if( int64ToString( number, strNumber, format ) ){
      uartWriteString( printer, strNumber );
   }
}

void printUIntFormat( print_t printer, uint64_t number, numberFormat_t format ){

   char strNumber[65];

   if( uint64ToString( number, strNumber, format ) ){
      uartWriteString( printer, strNumber );
   }
}

void printHexBitSize( print_t printer, uint64_t number, uint8_t bitSize ){
   printString( printer, uintToAsciiHex( number , bitSize ) );
}

void printHex( print_t printer, uint64_t number ){

   uint8_t i;
   bool_t zeroLeft = FALSE;
   uint64_t hex = 0;

   if( number ){
      for( i=0; i<8; i++ ){
         hex = (number>>(7-i)*8) & 0x00000000000000FF;
         if( (hex != 0) || zeroLeft ){
            printString( printer, uintToAsciiHex( hex, 8 ) );
            zeroLeft = TRUE;
         }
      }
   } else{
      printString( printer, uintToAsciiHex( 0, 8 ) );
   }
}


// C++ version 0.4 char* style "itoa":
// Written by Lukas Chmela
// Released under GPLv3.
// Modified by Eric Pernia.
bool_t int64ToString( int64_t value, char* result, uint8_t base ){
   // check that the base if valid
   if( base < 2 || base > 36 ){
      *result = '\0';
      return FALSE;
   }

   char* ptr = result, *ptr1 = result, tmp_char;
   int64_t tmp_value;

   do {
      tmp_value = value;
      value /= (int64_t)base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * (int64_t)base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return TRUE;
}

// C++ version 0.4 char* style "itoa":
// Written by Luk�s Chmela
// Released under GPLv3.
// Modified by Eric Pernia.
bool_t uint64ToString( uint64_t value, char* result, uint8_t base ){
   // check that the base if valid
   if( base < 2 || base > 36 ){
      *result = '\0';
      return FALSE;
   }

   char* ptr = result, *ptr1 = result, tmp_char;
   uint64_t tmp_value;

   do {
      tmp_value = value;
      value /= (uint64_t)base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * (uint64_t)base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return TRUE;
}



/*
// ITOA Seba
char *itoa(long int num, char *s){     // Funcion ITOA (Entero to ASCII)
     unsigned long int temp=1;         // Declaracion de valor temporal
     unsigned int8 i, cnt=0;           // Declaracion de indices y contadores
     char c;                           // Declaracion de variable de caracter de salida
     while(temp>0){                    // Rutina de Conversion (Queda invertida)
        temp=(num/10);                 // Conversion de caracter a caracter
        s[cnt]=(num%10)+'0';           // utilizando divisiones y resto
        if(s[cnt]>0x39)                // sumando el offset de la tabla ASCII
           s[cnt]+=0x7;
        cnt++;
        num=temp;
     }
     for(i=0;i<(int8)(cnt/2);i++){     // Rutina para invertir el numero convertido
        c=s[i];                        // Intercambio de variables
        s[i]=s[cnt-i-1];
        s[cnt-i-1]=c;
     }
     s[cnt]='\0';                      // Caracter nulo, fin de la conversion ITOA
     return s;                         // Retorno del valor ASCII
}
*/


uint8_t asciiUint8ToUint8( char* str, uint8_t* digits ){

   uint8_t i = 0;
   uint8_t res = 0;
   uint8_t resVec[3];

   *digits = 0;

   while( isDecimalDigit(str[i]) && (i<3) ){
      resVec[i] = str[i] - '0';
      i++;
   }

   switch( i ){
      case 1:
         res = resVec[0];
      break;

      case 2:
         res = resVec[0]*10 + resVec[1];
      break;

      case 3:
         res = resVec[0]*100 + resVec[1]*10 + resVec[2];
      break;

      case 0:
      default:
         *digits = 0;
         return 0;
      break;
   }

   *digits = i;

   return res;
}

int32_t asciiInt32ToInt32( char* str, int8_t* digits ){

   if( *str == 0 ){
      return 0;
   }

   int32_t res = 0;  // Initialize result
   int32_t sign = 1; // Initialize sign as positive
   uint8_t i = 0;    // Initialize index of first digit
   uint8_t digitMaxSize = 10;

   // If number is negative, then update sign
   if( str[i] == '-' ){
      sign = -1;
      digitMaxSize = 11;
      i++;
   }

   // Iterate through all digits of input string and update result
   while( isDecimalDigit(str[i]) && (i<=digitMaxSize) ){ // 10 digits + sign
      res = res*10 + str[i] - '0';
      i++;
   }

   if( (sign == -1) && (i<2) ){
      *digits = -1;
   } else if( (sign == 1) && (i<1) ){
      *digits = -1;
   } else{
      *digits = i;
   }

   // Return result with sign
   return sign*res;
}


int64_t asciiInt64ToInt64( char* str, int8_t* digits ){

   int64_t res = 0;  // Initialize result
   int64_t sign = 1; // Initialize sign as positive
   uint8_t i = 0;    // Initialize index of first digit

   // If number is negative, then update sign
   if( str[i] == '-' ){
      sign = -1;
      i++;
   }

   // Iterate through all digits of input string and update result
   while( isDecimalDigit(str[i]) && (i<=20) ){ // 20 digits, or 19 + sign
      res = res*10 + str[i] - '0';
      i++;
   }

   if( (sign == -1) && (i<2) ){
      *digits = -1;
   } else if( (sign == 1) && (i<1) ){
      *digits = -1;
   } else{
      *digits = i;
   }

   // Return result with sign
   return sign*res;
}


uint64_t asciiUint64ToUint64( char* str, int8_t* digits ){

   uint64_t res = 0;  // Initialize result
   uint8_t i = 0;    // Initialize index of first digit

   // Iterate through all digits of input string and update result
   while( isDecimalDigit(str[i]) && (i<=20) ){ // 10 digits + sign
      res = res*10 + str[i] - '0';
      i++;
   }

   if( (i<1) ){
      *digits = -1;
   } else{
      *digits = i;
   }

   // Return result
   return res;
}


char* uintToAsciiHex( uint64_t value, uint8_t bitSize ){
   
   static char result[17];
   uint8_t i = 0;
   uint8_t vectorNumHex[] = "0123456789ABCDEF";
   
   result[bitSize/4] = 0;
   
   for( i=0; i<bitSize/4; i++ ){
      result[(bitSize/4)-i-1] = vectorNumHex[ (uint8_t)(( value & (((uint64_t)0x0F)<<(4*i)) ) >> (4*i)) ];
   }

   return result;
}

bool_t asciiHexDigitToUint8( char c, uint8_t* value ){

   uint8_t res = 0;
   uint8_t i;

   for( i=0; i<2; i++ ){

      if( (c >= '0') && (c <= '9') ){
         res = c - '0';
      } else{
         if( (c >= 'A') && (c <= 'F') ){
            res = c - 'A' + 10;
         } else{
            if( (c >= 'a') && (c <= 'f') ){
               res = c - 'a' + 10;
            } else{
               return FALSE;
            }
         }
      }

   }

   *value = res;

   return TRUE;
}

bool_t asciiHexToUint8( char* str, uint8_t* value ){

   uint8_t res = 0;
   uint8_t val = 0;
   uint8_t i = 0;     // Initialize index of first digit

   // Iterate through all digits of input string and update result
   while( isHexDigit(str[i]) && (i<=2) ){
     asciiHexDigitToUint8( str[i], &val );
     res = res*16 + val;
     i++;
   }

   *value = res;

   return TRUE;
}


uint64_t asciiHex64ToUint64( char* str, int8_t* digits ){

   uint8_t value = 0;
   uint64_t res = 0;  // Initialize result
   uint8_t i = 0;     // Initialize index of first digit

   // Iterate through all digits of input string and update result
   while( isHexDigit(str[i]) && (i<=16) ){
     asciiHexDigitToUint8( str[i], &value );
     res = res*16 + value;
     i++;
   }

   if( (i<1) ){
      *digits = -1;
   } else{
      *digits = i;
   }

   // Return result
   return res;
}


bool_t isDecimalDigit( char c ){
   return ((c >= '0') && (c <= '9'));
}

bool_t isHexDigit( char c ){
   return ( ((c >= '0') && (c <= '9')) ||
            ((c >= 'a') && (c <= 'f')) ||
            ((c >= 'A') && (c <= 'F'))
          );
}



// -1 not found
// 0 or more, index of decimal digit in string
int32_t indexOfDecimalCharInSring( char* str, int32_t strLen ){

   uint8_t i = 0;

   for( i=0; i<strLen; i++ ){
      if( isDecimalDigit(str[i]) ){
         return i;
      }
   }
   return -1;
}

// -1 not found
// 0 or more, index of hex digit in string
int32_t indexOfHexCharInSring( char* str, int32_t strLen ){

   uint8_t i = 0;

   for( i=0; i<strLen; i++ ){
      if( isHexDigit(str[i]) ){
         return i;
      }
   }
   return -1;
}

// -1 not found
// 0 or more, index of char c in string
int32_t indexOfCharInSring( char c, char* str, int32_t strLen ){

   uint8_t i = 0;

   for( i=0; i<strLen; i++ ){
      if( str[i] == c ){
         return i;
      }
   }
   return -1;
}


bool_t saveNumberInByteArray( uint64_t number,
                              uint8_t* byteArray,
                              uint8_t  byteArraySize ){
   uint8_t i = 0;
   uint8_t* ptr = (uint8_t*)&number;

   if( byteArraySize > 8 ){
      return FALSE;
   }

   for( i=0; i<byteArraySize; i++ ){
      byteArray[i] = *ptr;
      ptr++;
   }

   return TRUE;
}


uint64_t loadNumberFromByteArray( uint8_t*  byteArray,
                                  uint8_t   byteArraySize ){
   uint8_t i = 0;
   uint64_t number = 0;

   for( i=0; i<byteArraySize; i++ ){
      number |= ((uint64_t)(byteArray[i]) << 8*i);
   }

   return number;
}


/*==================[examples]===============================================*/

/*
   uint8_t value = 0;
   uint8_t valueSize = 0;

   value = asciiUint8ToUint8( "2074 ", &valueSize );

   consolePrintlnInt( value );
   consolePrintlnInt( valueSize );
*/

/*
   uint8_t i = 0;
   uint8_t byteArray[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

   uint8_t  a = 0x08;
   uint16_t b = 0x00CA;
   uint32_t c = 0x00EE00FE;
   uint64_t d = 0x000055443322CAFE;

   numberToByteArray( a, byteArray, 3 );

   debugPrintlnString( "byteArray[i] = " );
   for( i=0; i<8; i++ ){
      debugPrintlnHex( byteArray[i] );
   }

   uint64_t numb = 0;
   byteArrayToNumber( &numb, byteArray, 8 );
   debugPrintString( "number = 0x" );
   debugPrintlnHex( numb );
*/

/*==================[end of file]============================================*/
