#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/arm/aarch32/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_spu.h>
#include "spmInternal.h"
#include "dmtypes.h"


#if !defined(CONFIG_ARM_SECURE_FIRMWARE)
#error "Module requires compiling for Secure ARM Firmware"
#endif

/* Include required APIs for TrustZone-M */
#include <arm_cmse.h>
#include <aarch32/cortex_m/tz.h>

#include <nrfx.h>

#if USE_PARTITION_MANAGER
#include <pm_config.h>
#define NON_SECURE_APP_ADDRESS PM_SRAM_NONSECURE_ADDRESS
// #define NON_SECURE_APP_ADDRESS PM_APP_ADDRESS
#ifdef PM_SRAM_SECURE_SIZE
#define NON_SECURE_RAM_OFFSET PM_SRAM_SECURE_SIZE
#else
#define NON_SECURE_RAM_OFFSET 0
#define SECURE_RAM_OFFSET 0x00030000
#endif
#else
#include <storage/flash_map.h>
#define NON_SECURE_APP_ADDRESS FLASH_AREA_ID(image_0_nonsecure)
#define NON_SECURE_RAM_OFFSET 0x8000
#endif /* USE_PARTITION_MANAGER */

#define NON_SECURE_FLASH_REGION_INDEX \
    ((NON_SECURE_APP_ADDRESS) / (FLASH_SECURE_ATTRIBUTION_REGION_SIZE))


#define SECURE_RAM_REGION_INDEX \
    ((SECURE_RAM_OFFSET) / (RAM_SECURE_ATTRIBUTION_REGION_SIZE))
// #define NON_SECURE_RAM_REGION_INDEX \
//     ((NON_SECURE_RAM_OFFSET) / (RAM_SECURE_ATTRIBUTION_REGION_SIZE))
 
typedef enum PeriphType_t
{
    SPU_PERIPH                  =   3,              /*!< 3  SPU                                                                    */
    REGULATORS_PERIPH           =   4,              /*!< 4  Regulators                                                             */
    CLOCK_POWER_PERIPH          =   5,              /*!< 5  CLOCK_POWER                                                            */
    CTRLAPPERI_PERIPH           =   6,              /*!< 6  CTRLAPPERI_PERIPH                                                      */
    UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_PERIPH=   8,     /*!< 8  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0                                         */
    UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_PERIPH=   9,     /*!< 9  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1                                         */
    UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_PERIPH=  10,     /*!< 10 UARTE2_SPIM2_SPIS2_TWIM2_TWIS2                                         */
    UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_PERIPH=  11,     /*!< 11 UARTE3_SPIM3_SPIS3_TWIM3_TWIS3                                         */
    GPIOTE0_PERIPH              =  13,              /*!< 13 GPIOTE0                                                                */
    SAADC_PERIPH                =  14,              /*!< 14 SAADC                                                                  */
    TIMER0_PERIPH               =  15,              /*!< 15 TIMER0                                                                 */
    TIMER1_PERIPH               =  16,              /*!< 16 TIMER1                                                                 */
    TIMER2_PERIPH               =  17,              /*!< 17 TIMER2                                                                 */
    RTC0_PERIPH                 =  20,              /*!< 20 RTC0                                                                   */
    RTC1_PERIPH                 =  21,              /*!< 21 RTC1                                                                   */
    DPPIC_PERIPH                =  23,              /*!< 23 DPPIC                                                                  */
    WDT_PERIPH                  =  24,              /*!< 24 WDT                                                                    */
    EGU0_PERIPH                 =  27,              /*!< 27 EGU0                                                                   */
    EGU1_PERIPH                 =  28,              /*!< 28 EGU1                                                                   */
    EGU2_PERIPH                 =  29,              /*!< 29 EGU2                                                                   */
    EGU3_PERIPH                 =  30,              /*!< 30 EGU3                                                                   */
    EGU4_PERIPH                 =  31,              /*!< 31 EGU4                                                                   */
    EGU5_PERIPH                 =  32,              /*!< 32 EGU5                                                                   */
    PWM0_PERIPH                 =  33,              /*!< 33 PWM0                                                                   */
    PWM1_PERIPH                 =  34,              /*!< 34 PWM1                                                                   */
    PWM2_PERIPH                 =  35,              /*!< 35 PWM2                                                                   */
    PWM3_PERIPH                 =  36,              /*!< 36 PWM3                                                                   */
    PDM_PERIPH                  =  38,              /*!< 38 PDM                                                                    */
    I2S_PERIPH                  =  40,              /*!< 40 I2S                                                                    */
    IPC_PERIPH                  =  42,              /*!< 42 IPC                                                                    */
    FPU_PERIPH                  =  44,              /*!< 44 FPU                                                                    */
    GPIOTE1_PERIPH              =  49,              /*!< 49 GPIOTE1                                                                */
    KMU_NVMC_PERIPH             =  57,              /*!< 57 KMU_NVMC                                                               */
    VMC_PERIPH                  =  58,              /*!< 58 VMC                                                                    */
    CRYPTOCELL_PERIPH           =  64,              /*!< 64 CRYPTOCELL                                                             */
    GPIO_PERIPH                 =  66               /*!< 66 GPIO                                                                   */
} PeriphType_t;

typedef struct NoneSecurePeripheral_t
{
   PeriphType_t iId;
   IRQn_Type iIRQ;
} NoneSecurePeripheral_t;

static const NoneSecurePeripheral_t maNonSecurePeripherals[] =
{
    {REGULATORS_PERIPH, 0},
    {CLOCK_POWER_PERIPH, CLOCK_POWER_IRQn},
    {UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_PERIPH, SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn},
    {UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_PERIPH, SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn},
    {UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_PERIPH, SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn},
    {UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_PERIPH, SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn},
    {SAADC_PERIPH, SAADC_IRQn},
    {TIMER0_PERIPH, TIMER0_IRQn},
    {TIMER1_PERIPH, TIMER1_IRQn},
    {TIMER2_PERIPH, TIMER2_IRQn},
    {RTC0_PERIPH, RTC0_IRQn},
    {RTC1_PERIPH, RTC1_IRQn},
    {DPPIC_PERIPH, 0},
    {WDT_PERIPH, WDT_IRQn},
    {EGU0_PERIPH, EGU0_IRQn},
    {EGU1_PERIPH, EGU1_IRQn},
    {EGU2_PERIPH, EGU2_IRQn},
    {EGU3_PERIPH, EGU3_IRQn},
    {EGU4_PERIPH, EGU4_IRQn},
    {EGU5_PERIPH, EGU5_IRQn},
    {PWM0_PERIPH, PWM0_IRQn},
    {PWM1_PERIPH, PWM1_IRQn},
    {PWM2_PERIPH, PWM2_IRQn},
    {PWM3_PERIPH, PWM3_IRQn},
    {PDM_PERIPH, PDM_IRQn},
    {I2S_PERIPH, I2S_IRQn},
    {IPC_PERIPH, IPC_IRQn},
    {FPU_PERIPH, FPU_IRQn},
    {GPIOTE1_PERIPH, GPIOTE1_IRQn},
    {KMU_NVMC_PERIPH, KMU_IRQn},
    {VMC_PERIPH, 0},
    {GPIO_PERIPH, 0}
};

extern irq_target_state_t irq_target_state_set(unsigned int irq, irq_target_state_t irq_target_state);
extern int irq_target_state_is_secure(unsigned int irq);
static void ConfigNonSecureRegion(void);
static void JumpNonSecureRegion(void);

/*
 *  * The security configuration for depends on where the non secure app
 *  * is placed. All flash regions before the region which contains the
 *  * non secure app is configured as Secure.
 *
 *                FLASH
 *  1 MB  |---------------------|
 *        |                     |
 *        |                     |
 *        |                     |
 *        |                     |
 *        |                     |
 *        |     Non-Secure      |
 *        |       Flash         |
 *        |                     |
 *  X kB  |---------------------|
 *        |                     |
 *        |     Secure          |
 *        |      Flash          |
 *  0 kB  |---------------------|
 *
 *  * The security configuration for SRAM is applied:
 *
 *                SRAM
 * 256 kB |---------------------|
 *        |                     |
 *        |                     |
 *        |                     |
 *        |     Non-Secure      |
 *        |    SRAM (image)     |
 *        |                     |
 * 128 kB |.................... |
 *        |     Non-Secure      |
 *        |  SRAM (BSD Library) |
 *  64 kB |---------------------|
 *        |      Secure         |
 *        |       SRAM          |
 *  0 kB  |---------------------|
 */

void main(void)
{
    // watchdog_Init();
    ConfigNonSecureRegion();
    JumpNonSecureRegion();
}

static void ConfigNonSecureRegion(void)
{
    for (UINT32 iFlashIndex = 0; iFlashIndex < NUM_FLASH_SECURE_ATTRIBUTION_REGIONS; iFlashIndex++)
    {
        if (iFlashIndex < NON_SECURE_FLASH_REGION_INDEX)
            NRF_SPU->FLASHREGION[iFlashIndex].PERM = FLASH_READ | FLASH_WRITE | FLASH_EXEC | FLASH_LOCK | FLASH_SECURE;
        else
            NRF_SPU->FLASHREGION[iFlashIndex].PERM = FLASH_READ | FLASH_WRITE | FLASH_EXEC | FLASH_LOCK | FLASH_NONSEC;
    }
    // watchdog_Feed();

    // UINT32 iNscLen = FLASH_NSC_SIZE_FROM_ADDR(__sg_start);
    // nrf_spu_flashnsc_set(NRF_SPU, 0, FLASH_NSC_SIZE_REG(iNscLen), FLASH_NSC_REGION_FROM_ADDR(__sg_start), false);
    // secureServices_Init();

    for (UINT32 iRamIndex = 0; iRamIndex < NUM_RAM_SECURE_ATTRIBUTION_REGIONS; iRamIndex++)
    {
        if (iRamIndex >= SECURE_RAM_REGION_INDEX)
            NRF_SPU->RAMREGION[iRamIndex].PERM = SRAM_READ | SRAM_WRITE | SRAM_EXEC | SRAM_LOCK | SRAM_SECURE;
        else
            NRF_SPU->RAMREGION[iRamIndex].PERM = SRAM_READ | SRAM_WRITE | SRAM_EXEC | SRAM_LOCK | SRAM_NONSEC;
    }
    // for (UINT32 iRamIndex = 0; iRamIndex < NUM_RAM_SECURE_ATTRIBUTION_REGIONS; iRamIndex++)
    // {
    //     if (iRamIndex < NON_SECURE_RAM_REGION_INDEX)
    //         NRF_SPU->RAMREGION[iRamIndex].PERM = SRAM_READ | SRAM_WRITE | SRAM_EXEC | SRAM_LOCK | SRAM_SECURE;
    //     else
    //         NRF_SPU->RAMREGION[iRamIndex].PERM = SRAM_READ | SRAM_WRITE | SRAM_EXEC | SRAM_LOCK | SRAM_NONSEC;
    // }
    // watchdog_Feed();

    for (UINT32 i = 0; i < ARRAY_SIZE(maNonSecurePeripherals); i++)
    {
        if (maNonSecurePeripherals[i].iIRQ)
        {
            NVIC_DisableIRQ(maNonSecurePeripherals[i].iIRQ);
            __DSB(); __ISB(); // ensure irq is off
        }

        NRF_SPU->PERIPHID[maNonSecurePeripherals[i].iId].PERM = PERIPH_PRESENT | PERIPH_NONSEC | PERIPH_LOCK;

        if (maNonSecurePeripherals[i].iIRQ)
        {
            __DSB(); __ISB();
            irq_target_state_set(maNonSecurePeripherals[i].iIRQ, IRQ_TARGET_STATE_NON_SECURE);
        }
    }
    // watchdog_Feed();

    NRF_SPU->DPPI[0].PERM = 0;
    NRF_SPU->GPIOPORT[0].PERM = 0;
}

static void JumpNonSecureRegion(void)
{
    /* Extract initial MSP of the Non-Secure firmware image.
     * The assumption is that the MSP is located at VTOR_NS[0].
     */
    UINT32 *vtor_ns = (UINT32 *)NON_SECURE_APP_ADDRESS;

    /* Configure Non-Secure stack */
    tz_nonsecure_setup_conf_t spm_ns_conf =
    {
        .vtor_ns = (UINT32)vtor_ns,
        .msp_ns = vtor_ns[0],
        .psp_ns = 0,
        .control_ns.npriv = 0, /* Privileged mode*/
        .control_ns.spsel = 0 /* Use MSP in Thread mode */
    };
    
    /* Configure core register block for Non-Secure state. */
    tz_nonsecure_state_setup(&spm_ns_conf);
    /* Prioritize Secure exceptions over Non-Secure */
    tz_nonsecure_exception_prio_config(1);
    /* Set non-banked exceptions to target Non-Secure */
    tz_nbanked_exception_target_state_set(0);
    /* Configure if Non-Secure firmware should be allowed to issue System
     * reset. If not it could be enabled through a secure service.
     */
    tz_nonsecure_system_reset_req_block(0);
    /* Allow SPU to have precedence over (non-existing) ARMv8-M SAU. */
    tz_sau_configure(0, 1);

    tz_nonsecure_fpu_access_enable();

    /* Generate function pointer for Non-Secure function call. */
    TZ_NONSECURE_FUNC_PTR_DECLARE(reset_ns);
    reset_ns = TZ_NONSECURE_FUNC_PTR_CREATE(vtor_ns[1]);
    // watchdog_Feed();

    __DSB();
    __ISB();

    /* Jump to Non-Secure firmware */
    reset_ns();

    CODE_UNREACHABLE;
}
