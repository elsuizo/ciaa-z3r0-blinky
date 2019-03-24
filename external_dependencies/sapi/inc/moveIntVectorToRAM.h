#ifndef _MOVE_INT_VECTOR_TO_RAM_H_
#define _MOVE_INT_VECTOR_TO_RAM_H_

#define VECTOR_SIZE (16 + EXT_IRQ_COUNT)

#define VTOR_FLASH_ADDR (0)

#define VTABLE_EM2_ISR_OFFSET   GPIO_EVEN_IRQn + 16 // This should equal the highest IRQ that
                                   // will be used to wake from EM2.  In this
                                   // example, 17 is the GPIO EVEN IRQ.

void moveInterruptVectorToRam(void);

#endif
