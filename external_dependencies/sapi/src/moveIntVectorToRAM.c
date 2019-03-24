
#include "em_device.h"
#include "em_ramfunc.h"
#include "string.h"
#include "../inc/moveIntVectorToRAM.h"


uint32_t vectorTableNew[VECTOR_SIZE] __attribute__ ((section (".noinit"), aligned (256) ));

volatile uint32_t Flash_Address;
const uint32_t* ptrFlashVectorTable = VTOR_FLASH_ADDR;

SL_RAMFUNC_DEFINITION_BEGIN
static void CheckFlash_IRQHandler(void)
{
  void (*fptr_irqhandler)(void) = NULL;

  Flash_Address = ptrFlashVectorTable[VTABLE_EM2_ISR_OFFSET];     // Do a dummy read of the flash.  It is important to make sure
                                                                  // the compiled version of the code retains this call and is not
                                                                  // optimized out.
  Flash_Address = ptrFlashVectorTable[__get_IPSR()];              // Read the actual address for the active interrupt request.
  fptr_irqhandler = (void(*)(void))Flash_Address;                 // Use the original vectorTable located in flash, with IRQ
                                                                  // offset (IPSR)

  (*fptr_irqhandler)();
}
SL_RAMFUNC_DEFINITION_END

void moveInterruptVectorToRam(void)
{
  // If we know the wake source from EM2, we can limit the size of the RAM vector table to be the size of the maximum vector location index
  memcpy(vectorTableNew, (uint32_t*)VTOR_FLASH_ADDR, sizeof(uint32_t) * (VECTOR_SIZE));  // Copy the flash vector table to RAM
  vectorTableNew[VTABLE_EM2_ISR_OFFSET] = (uint32_t)CheckFlash_IRQHandler;   // The only location in the RAM based vector table required is at the index 
                                                                             // of the IRQ handler that services the IRQ that exits EM2.  If there are more
																			 // IRQs that can exit EM2 they need to be added as well (they are not shown here).
  SCB->VTOR = (uint32_t)vectorTableNew; 
}

