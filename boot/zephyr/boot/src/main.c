#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <hal/nrf_spu.h>
#include "dmtypes.h"


// main 'settings' for the BOOTLOADER in BOOTLOADER.H
// static BYTE mabChunk[256] __attribute__((aligned)); // buffer for reading from flash and writing to CPU flash - must be word aligned

// static INT32 InitHw(const struct device *dev);
#define SPM_ADDRESS 0x20030000
#define USER_CODE_ADDRESS SPM_ADDRESS

static void ExecuteUserCode(void);

// // Mux pins ASAP
// SYS_INIT(InitHw, PRE_KERNEL_1, 1);

// static INT32 InitHw(const struct device *dev)
// {
//     // SWGND
//     if (retentionBitfield_Read(RETENTION_BITFIELD_PERSIST_SWGND))
//         nrf_gpio_pin_set(21);
//     else
//         nrf_gpio_pin_clear(21);

//     nrf_gpio_cfg_output(21);

//     // GPS is active high
//     nrf_gpio_pin_clear(17);
//     nrf_gpio_cfg_output(17);
    
//     // Battery charger enable pin is active low
//     nrf_gpio_cfg_input(24, NRF_GPIO_PIN_PULLUP);  
//     return 0;    
// }

void main(void)
{
    ExecuteUserCode();
}


static void ExecuteUserCode(void)
{
	// Allow any pending interrupts to be recognized
	__ISB();
	__disable_irq();
    
	// Disable NVIC interrupts 
	for (BYTE i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) 
		NVIC->ICER[i] = 0xFFFFFFFF;

	// Clear pending NVIC interrupts
	for (BYTE i = 0; i < ARRAY_SIZE(NVIC->ICPR); i++) 
		NVIC->ICPR[i] = 0xFFFFFFFF;

    // Reset sys tick
	SysTick->CTRL = 0;

	// Disable fault handlers used by the bootloader
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);

	// Activate the MSP if the core is currently running with the PSP 
	if (CONTROL_SPSEL_Msk & __get_CONTROL()) 
		__set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);

	__DSB(); // Force Memory Write before continuing 
	__ISB(); // Flush and refill pipeline with updated permissions 


	SCB->VTOR = USER_CODE_ADDRESS;
	UINT32 *pVectorTable = (UINT32 *)USER_CODE_ADDRESS;

	// Set MSP to the new address and clear any information from PSP

	UINT32 iSp = pVectorTable[0];
	UINT32 iPc = pVectorTable[1];
	__set_MSP(pVectorTable[0]);
	__set_PSP(0);
	


	// Call reset handler.
	((void (*)(void))pVectorTable[1])();
	CODE_UNREACHABLE;    
}
