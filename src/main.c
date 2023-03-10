/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <modem/nrf_modem_lib.h>



void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
    __NOP();
    // LOG_INF("Modem fault! Reason: 0x%08X    PC: 0x%08X", pFaultInfo->reason, pFaultInfo->program_counter);
    k_busy_wait(10000000);
}



void main(void)
{
	int32_t iResponse = nrf_modem_lib_init(NORMAL_MODE);

	printk("Response code is: %d", iResponse);

	for(;;)
	{
		__NOP();
	}
}
