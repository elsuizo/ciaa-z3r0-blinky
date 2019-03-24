/* Copyright 2017, Eric Pernia.
 * All rights reserved.
 *
 * Author: Eng. Eric Pernia.
 * Date: 2017-04-17
 */

#ifndef _CONSOLE_PRINT_H_
#define _CONSOLE_PRINT_H_

/*==================[inclusions]=============================================*/

#include "sapi_print.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

#define CONSOLE_PRINT   ON


#if( CONSOLE_PRINT == ON )

   // Initialize
   #define CONSOLE_PRINT_ENABLE                       static print_t consolePrint;
   #define consolePrintSetUart(uart);                 printSetUart(&(consolePrint),(uart));
   #define consolePrintConfigUart(uart,baudRate);     printConfigUart(&(consolePrint),(uart),(baudRate));

   // Print String
   #define consolePrintChar(string);                  printChar((debugPrint),(string));
   #define consolePrintString(string);                printString((consolePrint),(string));
   #define consolePrintEnter();                       printEnter(consolePrint);
   #define consolePrintlnString(string);              printlnString((consolePrint),(string));

   // Print Integer
   #define consolePrintIntFormat(number,format);      printIntFormat((consolePrint),(number),(format));
   #define consolePrintUIntFormat(number,format);     printUIntFormat((consolePrint),(number),(format));
   #define consolePrintlnIntFormat(number,format);    printlnIntFormat((consolePrint),(number),(format));
   #define consolePrintlnUIntFormat(number,format);   printlnUIntFormat((consolePrint),(number),(format));
   #define consolePrintInt(number);                   printInt((consolePrint),(number));
   #define consolePrintUInt(number);                  printUInt((consolePrint),(number));
   #define consolePrintlnInt(number);                 printlnInt((consolePrint),(number));
   #define consolePrintlnUInt(number);                printlnUInt((consolePrint),(number));

   #define consolePrintHexBitSize(number,bitSize);    printHexBitSize((consolePrint),(number),(bitSize));
   #define consolePrintlnHexBitSize(number,bitSize);  printlnHexBitSize((consolePrint),(number),(bitSize));

   #define consolePrintHex(number);                   printHex((consolePrint),(number));
   #define consolePrintlnHex(number);                 printlnHex((consolePrint),(number));

#else

   // Initialize
   #define CONSOLE_PRINT_ENABLE                       ;
   #define consolePrintConfigUart(uart,baudRate);     ;
   #define consolePrintSetUart(uart);                 ;

   // Print String
   #define consolePrintString(string);                ;
   #define consolePrintEnter();                       ;
   #define consolePrintlnString(string);              ;

   // Print Integer
   #define consolePrintIntFormat(number,format);      ;
   #define consolePrintUIntFormat(number,format);     ;
   #define consolePrintlnIntFormat(number,format);    ;
   #define consolePrintlnUIntFormat(number,format);   ;
   #define consolePrintInt(number);                   ;
   #define consolePrintUInt(number);                  ;
   #define consolePrintlnInt(number);                 ;
   #define consolePrintlnUInt(number);                ;

   #define consolePrintHexBitSize(number,bitSize);    ;
   #define consolePrintlnHexBitSize(number,bitSize);  ;

   #define consolePrintHex(number);                   ;
   #define consolePrintlnHex(number);                 ;

#endif

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _CONSOLE_PRINT_H_ */
