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
 */

/* Date: 2017-04-17 */

#ifndef _SAPI_PRINT_H_
#define _SAPI_PRINT_H_

/*

API
---

// Initialize
void printSetUart( print_t* printer, uartMap_t uart );
void printConfig( print_t printer, uartMap_t uart, uint32_t baudRate );


// Print String
void printString( print_t printer, char* string );
void printlnString( print_t printer, char* string );
void printEnter( print_t printer );

// Print Integer
void printIntFormat( print_t printer, int64_t number, numberFormat_t format );
void printUIntFormat( print_t printer, uint64_t number, numberFormat_t format );
void printlnIntFormat( print_t printer, int64_t number, numberFormat_t format );
void printlnUIntFormat( print_t printer, uint64_t number, numberFormat_t format );
void printInt( print_t printer, int64_t number );
void printUInt( print_t printer, uint64_t number );
void printlnInt( print_t printer, int64_t number );
void printlnUInt( print_t printer, uint64_t number );
void printHex( uint64_t number, uint8_t bitSize );

// C++ version 0.4 char* style "itoa":
bool_t int64ToString( int64_t value, char* result, uint8_t base );
bool_t uint64ToString( uint64_t value, char* result, uint8_t base );

char* uintToAsciiHex( uint64_t value, uint8_t bitSize );
*/

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[macros]=================================================*/

#define PRINT_ENTER_STRING   "\r\n"

// Any printer

#define printlnString(printer,string);   printString((printer),(string));\
                                         printEnter((printer));


#define printInt(printer,number);        printIntFormat((printer),(number),(DEC_FORMAT));

#define printUInt(printer,number);       printUIntFormat((printer),(number),(DEC_FORMAT));


#define printlnInt(printer,number);      printInt((printer),number);\
                                         printEnter((printer));

#define printlnUInt(printer,number);     printUInt((printer),number);\
                                         printEnter((printer));


#define printlnIntFormat(printer,number,format);    printIntFormat((printer),(number),(format));\
                                                    printEnter((printer));

#define printlnUIntFormat(printer,number,format);   printUIntFormat((printer),(number),(format));\
                                                    printEnter((printer));
                                            
#define printlnHexBitSize(printer,number,bitSize);  printHexBitSize((printer),(number),(bitSize));\
                                                    printEnter((printer));

#define printlnHex(printer,number);     printHex((printer),(number));\
                                        printEnter((printer));

/*==================[typedef]================================================*/

typedef uartMap_t print_t;

typedef enum{
   BIN_FORMAT = 2,
   DEC_FORMAT = 10,
   HEX_FORMAT = 16
} numberFormat_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Initialize
void printSetUart( print_t* printer, uartMap_t uart );
void printConfigUart( print_t* printer, uartMap_t uart, uint32_t baudRate );

// Print String
void printChar( print_t printer, char character );
void printString( print_t printer, char* string );
void printEnter( print_t printer );

// Print Integer
void printIntFormat( print_t printer, int64_t number, numberFormat_t format );
void printUIntFormat( print_t printer, uint64_t number, numberFormat_t format );
void printHexBitSize( print_t printer, uint64_t number, uint8_t bitSize );
void printHex( print_t printer, uint64_t number );

// C++ version 0.4 char* style "itoa":
bool_t int64ToString( int64_t value, char* result, uint8_t base );
bool_t uint64ToString( uint64_t value, char* result, uint8_t base );


uint8_t asciiUint8ToUint8( char* str, uint8_t* digits );
int32_t asciiInt32ToInt32( char* str, int8_t* digits );

int64_t asciiInt64ToInt64( char* str, int8_t* digits );
uint64_t asciiUint64ToUint64( char* str, int8_t* digits );

char* uintToAsciiHex( uint64_t value, uint8_t bitSize );
bool_t asciiHexDigitToUint8( char c, uint8_t* value );
bool_t asciiHexToUint8( char* str, uint8_t* value );
uint64_t asciiHex64ToUint64( char* str, int8_t* digits );

bool_t isDecimalDigit( char c );
bool_t isHexDigit( char c );

// -1 not found
// 0 or more, index of decimal digit in string
int32_t indexOfDecimalCharInSring( char* str, int32_t strLen );

// -1 not found
// 0 or more, index of hex digit in string
int32_t indexOfHexCharInSring( char* str, int32_t strLen );

// -1 not found
// 0 or more, index of char c in string
int32_t indexOfCharInSring( char c, char* str, int32_t strLen );


bool_t saveNumberInByteArray( uint64_t number,
                              uint8_t* byteArray,
                              uint8_t  byteArraySize );

uint64_t loadNumberFromByteArray( uint8_t*  byteArray,
                                  uint8_t   byteArraySize );

/*==================[examples]===============================================*/

/*
printlnString( printer, "Uart Debug configurada." );
printlnString( printer, "Uart LoRa RN2903 configurada." );

printString( printer, "Numero de algo: " );
printInt( printer, 45454578 );
printlnString( printer, "" );

printlnInt( printer, -457 );
printlnUInt( printer, -457 );

printString( printer, "Numero de algo: " );
printIntFormat( printer, 45454578, DEC_FORMAT );
printlnString( printer, "" );

printString( printer, "Numero de algo: 0b" );
printIntFormat( printer, 45454578, BIN_FORMAT );
printlnString( printer, "" );

printString( printer, "Numero de algo: 0x" );
printIntFormat( printer, 45454578, HEX_FORMAT );
printlnString( printer, "" );
*/

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_PRINT_H_ */
