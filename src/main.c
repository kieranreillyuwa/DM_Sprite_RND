/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <zephyr/sys/reboot.h>
#include <string.h>


#ifndef __NOP
#define __NOP(x) arch_nop()
#endif 

void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
    __NOP();
    // LOG_INF("Modem fault! Reason: 0x%08X    PC: 0x%08X", pFaultInfo->reason, pFaultInfo->program_counter);
	printk("modem fault!!\r\n");
    k_busy_wait(10000000);
}


	
void main(void)
{
	printk("entering!\r\n");

	int32_t iResponse = INT32_MIN;
	iResponse = nrf_modem_lib_init(NORMAL_MODE);

	printk("iResp = %d\r\n", iResponse);

	uint8_t abResp[100];
    memset(abResp, 0, 100 * sizeof(char));
	
	abResp[0] = 0x0A;



	const char *atCmd = "AT+CGMI";

	iResponse = nrf_modem_at_cmd(abResp, 100 * sizeof(uint8_t), atCmd);

	printk("iRespAtCmd = %d\r\n", iResponse);



	for(;;)
	{
		__NOP();
	}
}
