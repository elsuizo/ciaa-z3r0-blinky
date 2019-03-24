
/*============================================================================
 * Licencia:
 * Autor:
 * Fecha:
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

//#include "program.h"   // <= su propio archivo de cabecera (opcional)
#include "external_dependencies/sapi/inc/sapi.h"        // <= Biblioteca sAPI

//#include "c_i18n_es.h" // <= para traducir el codigo C al espa�ol (opcional)
//#include "c_i18n_es.h" // <= para traducir la sAPI al espa�ol (opcional)

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

CONSOLE_PRINT_ENABLE

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------

   // Inicializar y configurar la plataforma
   boardConfig();
   
   // Inicializar pines del Switch y Led de la CIAA-Z3R0
   gpioConfig( SW, GPIO_INPUT );
   gpioConfig( LED, GPIO_OUTPUT );

   // Inicializar UART_USB como salida de consola
   consolePrintConfigUart( UART_DEBUG, 9600 );

   // Crear varias variables del tipo booleano
   bool_t swValue = OFF;
   bool_t ledValue = OFF;

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {
      /* Si se presiona SW (PB8), deja de parpadear el LED (PA8) */

      // Leer pin conectado a la tecla.
      // Lee un 0 (OFF) con tecla presionada y 1 (ON) al liberarla.
      swValue = gpioRead( SW );        
      // Si swValue = 1 (boton liberado), intercambiar el valor del LED
      if(  swValue ){
         // Intercambiar el valor del pin conectado a LED
         gpioToggle( LED );
         // Retardo bloqueante durante 250ms
         delay( 250 );
      }
      

      /* Mostrar por UART_USB (8N1 115200) el estado del LEDB */

      // Leer el estado del pin conectado al led
      ledValue = gpioRead( LED );
      // Chequear si el valor leido es encedido
      if( ledValue == ON ){
         // Si esta encendido mostrar por UART_DEBUG "LEDB encendido."
         consolePrintlnString( "LED encendido." );
      } else{
         // Si esta apagado mostrar por UART_DEBUG "LEDB apagado."
         consolePrintlnString( "LED apagado." );
      }
      
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

/*==================[fin del archivo]========================================*/
