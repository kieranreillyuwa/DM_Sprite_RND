.. _hello_world:

Sprite RND
###########

Overview
********

This is a test project for the nRF9160 DK for exploring why the modem library does not want to initialise in RAM.
There are three components:
    1.  Bootloader: this is just a small image to allow debugging in VSCode. This bootloader jumps to the SPM (once the SPM is loaded).
    2.  SPM: This is loaded in at 0x20030000 in SRAM. This just modifies partitions as secure or non-secure then jumps to application.
    3.  Application: All this tries to do is initialise the nrf modem lib. It fails. This image is non-secure and loads in at 0x20000000.

Building and Running
********************
To build, just do a pristine build.

To run in debug: 
    1. Put a breakpoint in the bootloader main.c before the line of code {SCB->VTOR = USER_CODE_ADDRESS;}
    2. Start execution (flashing the board will not load the SPM and application images).
    3. Load the SPM and application images in the debug terminal using the following commands:
        -exec load build/spm/zephyr/zephyr.elf
        -exec load build/zephyr/zephyr.elf
        Using the .hex equivalents makes no difference. 
    4. Step through and/or execute as desired. 

