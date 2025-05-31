
/**
 * @file platform/gpio.c
 * @brief Platform-support routines, GPIO component + initialization entrypoints
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @date   28 Oct 2024
 */

/*
 * PIC32CM5164LS00048 initial configuration:
 * -- Architecture: ARMv8 Cortex-M23
 * -- GCLK_GEN0: OSC16M @ 4 MHz, no additional prescaler
 * -- Main Clock: No additional prescaling (always uses GCLK_GEN0 as input)
 * -- Mode: Secure, NONSEC disabled
 * 
 * New clock configuration:
 * -- GCLK_GEN0: 24 MHz (DFLL48M [48 MHz], with /2 prescaler)
 * -- GCLK_GEN2: 4 MHz  (OSC16M @ 4 MHz, no additional prescaler)
 * 
 * HW configuration for the corresponding Curiosity Nano+ Touch Evaluation
 * Board:
 * -- PA15: Active-HI LED
 * -- PA23: Active-LO PB w/ external pull-up
 */

// Common include for the XC32 compiler
#include <xc.h>
#include <stdbool.h>
#include <string.h>

#include "../platform.h"

// Initializers defined in other platform/*.c files
extern void platform_systick_init(void);
extern void platform_usart_esp_init(void);
extern void platform_usart_co2_init(void);
extern void platform_usart_pms_init(void);
extern void platform_usart_gps_init(void);
extern void platform_usart_tick_handler(const platform_timespec_t *tick);

/////////////////////////////////////////////////////////////////////////////

// Enable higher frequencies for higher performance
static void raise_perf_level(void)
{
	uint32_t tmp_reg = 0;
	
	/*
	 * The chip starts in PL0, which emphasizes energy efficiency over
	 * performance. However, we need the latter for the clock frequency
	 * we will be using (~24 MHz); hence, switch to PL2 before continuing.
	 */
	PM_REGS->PM_INTFLAG = 0x01;
	PM_REGS->PM_PLCFG = 0x02;
	while ((PM_REGS->PM_INTFLAG & 0x01) == 0)
		asm("nop");
	PM_REGS->PM_INTFLAG = 0x01;
	
	/*
	 * Power up the 48MHz DFPLL.
	 * 
	 * On the Curiosity Nano Board, VDDPLL has a 1.1uF capacitance
	 * connected in parallel. Assuming a ~20% error, we have
	 * STARTUP >= (1.32uF)/(1uF) = 1.32; as this is not an integer, choose
	 * the next HIGHER value.
	 */
	NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2 << 1) ;
	SUPC_REGS->SUPC_VREGPLL = 0x00000302;
	while ((SUPC_REGS->SUPC_STATUS & (1 << 18)) == 0)
		asm("nop");
	
	/*
	 * Configure the 48MHz DFPLL.
	 * 
	 * Start with disabling ONDEMAND...
	 */
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0000;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	/*
	 * ... then writing the calibration values (which MUST be done as a
	 * single write, hence the use of a temporary variable)...
	 */
	tmp_reg  = *((uint32_t*)0x00806020);
	tmp_reg &= ((uint32_t)(0b111111) << 25);
	tmp_reg >>= 15;
	tmp_reg |= ((512 << 0) & 0x000003ff);
	OSCCTRL_REGS->OSCCTRL_DFLLVAL = tmp_reg;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	// ... then enabling ...
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0002;
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	// ... then restoring ONDEMAND.
//	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0080;
//	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
//		asm("nop");
	
	/*
	 * Configure GCLK_GEN2 as described; this one will become the main
	 * clock for slow/medium-speed peripherals, as GCLK_GEN0 will be
	 * stepped up for 24 MHz operation.
	 */
	GCLK_REGS->GCLK_GENCTRL[2] = 0x00000105;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 4)) != 0)
		asm("nop");
    
    GCLK_REGS->GCLK_GENCTRL[3] = 0x00000105;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 5)) != 0)
		asm("nop");
	
	// Switch over GCLK_GEN0 to DFLL48M, with DIV=2 to get 24 MHz.
	GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 2)) != 0)
		asm("nop");
	
	// Done. We're now at 24 MHz.
	return;
}

/*
 * Configure the EIC peripheral
 * 
 * NOTE: EIC initialization is split into "early" and "late" halves. This is
 *       because most settings within the peripheral cannot be modified while
 *       EIC is enabled.
 */
static void EIC_init_early(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 10);
	
	/*
	 * In order for debouncing to work, GCLK_EIC needs to be configured.
	 * We can pluck this off GCLK_GEN2, configured for 4 MHz; then, for
	 * mechanical inputs we slow it down to around 15.625 kHz. This
	 * prescaling is OK for such inputs since debouncing is only employed
	 * on inputs connected to mechanical switches, not on those coming from
	 * other (electronic) circuits.
	 * 
	 * GCLK_EIC is at index 4; and Generator 2 is used.
	 */
	GCLK_REGS->GCLK_PCHCTRL[4] = 0x00000042;
	while ((GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000042) == 0)
		asm("nop");
	
	// Reset, and wait for said operation to complete.
	EIC_SEC_REGS->EIC_CTRLA = 0x01;
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x01) != 0)
		asm("nop");
	
	/*
	 * Just set the debounce prescaler for now, and leave the EIC disabled.
	 * This is because most settings are not editable while the peripheral
	 * is enabled.
	 */
	EIC_SEC_REGS->EIC_DPRESCALER = (0b0 << 16) | (0b0000 << 4) |
		                       (0b1111 << 0);
	return;
}
static void EIC_init_late(void)
{
	/*
	 * Enable the peripheral.
	 * 
	 * Once the peripheral is enabled, further configuration is almost
	 * impossible.
	 */
	EIC_SEC_REGS->EIC_CTRLA |= 0x02;
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x02) != 0)
		asm("nop");
	return;
}

// Configure the EVSYS peripheral
static void EVSYS_init(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 0);
	
	/*
	 * EVSYS is always enabled, but may be in an inconsistent state. As
	 * such, trigger a reset.
	 */
	EVSYS_SEC_REGS->EVSYS_CTRLA = 0x01;
	asm("nop");
	asm("nop");
	asm("nop");
	return;
}

//////////////////////////////////////////////////////////////////////////////

void TCC1_Init(void){
    GCLK_REGS->GCLK_PCHCTRL[25] = 0x00000040; // Enable TCC0 Bus Clock (TCC0 usually corresponds to PCHCTRL[27])
    while ((GCLK_REGS->GCLK_PCHCTRL[25] & 0x00000040) == 0); // Wait for clock to stabilize

    /* Reset TCC3 */
    TCC1_REGS->TCC_CTRLA = 0x01; // Set SWRST bit to 1 for a software reset
    while (TCC1_REGS->TCC_SYNCBUSY & (1 << 0)); // Wait for reset to complete
    
    /* Clock Prescaler */
    TCC1_REGS->TCC_CTRLA = (1 << 12) | (7 << 8); // Configure presynchronization and set prescaler to 1024

    /* Waveform Configuration */
    TCC1_REGS->TCC_WEXCTRL = TCC_WEXCTRL_OTMX(0); // Default configuration for waveform extension control
    TCC1_REGS->TCC_WAVE = (2 << 0) | (0 << 4); // Configure PWM mode (single-slope)

    /* Set Period and Duty Cycle */
    TCC1_REGS->TCC_PER = 234370; // Set period value for desired PWM frequency
    
    /* Set Initial Duty Cycle for a starting color (Color 0: purple) */
    TCC1_REGS->TCC_CC[0] = 2000;   // PA03 for Red channel

    /* Enable TCC3 */
    TCC1_REGS->TCC_CTRLA |= (1 << 1); // Enable TCC3
    while (TCC1_REGS->TCC_SYNCBUSY & (1 << 1)); // Wait for enable synchronization
}

//////////////////////////////////////////////////////////////////////////////

/*
 * Configure the NVIC
 * 
 * This must be called last, because interrupts are enabled as soon as
 * execution returns from this function.
 */
static void NVIC_init(void)
{
	/*
	 * Unlike AHB/APB peripherals, the NVIC is part of the Arm v8-M
	 * architecture core proper. Hence, it is always enabled.
	 */
	__DMB();
	__enable_irq();
	NVIC_SetPriority(EIC_EXTINT_2_IRQn, 3);
	NVIC_SetPriority(SysTick_IRQn, 3);
	NVIC_EnableIRQ(EIC_EXTINT_2_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
	return;
}

/////////////////////////////////////////////////////////////////////////////

// Initialize the platform
void platform_init(void)
{
	// Raise the power level
	raise_perf_level();
	
	// Early initialization
	EVSYS_init();
	EIC_init_early();
	
	// Regular initialization
	platform_usart_esp_init();
    platform_usart_co2_init();
    platform_usart_pms_init();
    platform_usart_gps_init();
	TCC1_Init();
            
	// Late initialization
	EIC_init_late();
	platform_systick_init();
	NVIC_init();
	return;
}

// Do a single event loop
void platform_do_loop_one(void)
{
	platform_timespec_t tick;
	
	/*
	 * Some routines must be serviced as quickly as is practicable. Do so
	 * now.
	 */
	platform_tick_hrcount(&tick);
	platform_usart_tick_handler(&tick);
}
