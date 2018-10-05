
//#include "em_gpio.h"
//#include "em_adc.h"
//#include "em_dac.h"
//#include "em_acmp.h"
//#include "em_usart.h"
#include "em_cmu.h"
//#include "em_rtc.h"
#include "em_rmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "em_burtc.h"
#include "wait_burtc.h"

// Adapted from file main_gg_stk.c etc in AN0041, and other files from Silicon Labs / Micro Energy.
// Initialization of BURTC that we use to make sure we won't stay forever in EM3.


static volatile bool burtc_initialized = false;
static volatile bool burtc_delay_complete;
static volatile uint32_t burtc_counter;


/******************************************************************************
 * @brief   Configure backup RTC
 *****************************************************************************/
void burtc_config(void)
{
     burtc_initialized = true;

	 RMU_ResetCauseClear(); // Maybe we don't need this, we don't presently use BURTC for reset.
	 RMU_ResetControl(rmuResetBU, false);

	 /* Enable clock to low energy modules */
	 // Do we need this when using ULFRCO? Are we wasting energy?
	 // Tested without, seems to work fine. If anything an expected 10 second wait seemd slighly shorter (a few tenths of a second).
     //CMU_ClockEnable(cmuClock_CORELE, true);

	 /* Start ULFRC0 and wait until it is stable */
     CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);

     /* Create burtcInit struct and fill with default values */
	 BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

	 /* Set burtcInit to proper values for this application */
	 /* To make this example easier to read, all fields are listed,
	 even those which are equal to their default value */
	 burtcInit.enable = false;
	 burtcInit.mode = burtcModeEM3;
	 burtcInit.debugRun = false;
	 // IV 20140802: I am confused: an0041 says
	 //   "but for higher precision BURTC can also be clocked by either the LFRCO or LFXO down to EM4."
	 // while according to http://cdn.energymicro.com/dl/documentation/doxygen/EM_CMSIS_3.20.6_DOC/emlib_giant/html/index.html,
	 // "In addition, the low frequency clocks, ie LFXO and LFRCO are disabled by SW" (EMU_EnterEM3).
	 // Also, "LFXO" and LFRCO are really oscillators, not clocks, right? See http://downloads.energymicro.com/devices/pdf/d0001_efm32g_reference_manual.pdf
	 // Anyway, ULFRCO should be OK for us.
	 //  In fact, the code worked intermiitently until i shifted to ULFRCO, or
	 // was the reason something else?
	 burtcInit.clkSel = burtcClkSelULFRCO;

	 // With 128 it seems we get (ca.) 1KHz. With 1 it seems we get ca. 2 KHz.
	 // I believe I have read somewhere "1KHZ/2KHz" or similar, so could be those two speeds depending on clkDiv.
	 burtcInit.clkDiv = burtcClkDiv_128;

	 burtcInit.timeStamp = false;
	 burtcInit.compare0Top = false;
	 burtcInit.lowPowerMode = burtcLPDisable; // ?

	 /* Initialize BURTC with burtcInit struct */
	 BURTC_Init( &burtcInit );

//	 // Some times it hung in some sync due to "freeze" it seemed, and then
//	 // (at least afterwards) refused to update e g COMP0
//	 BURTC_FreezeEnable(false);
	 BURTC_IntClear(BURTC_IF_COMP0);

    /* Start BURTC */
    BURTC_Enable( true );
    NVIC_ClearPendingIRQ( BURTC_IRQn );
    NVIC_EnableIRQ( BURTC_IRQn );
}


void BURTC_IRQHandler(void)
{
    burtc_delay_complete = true;
    burtc_counter = BURTC_CounterGet();
    BURTC_Enable(false);
    BURTC_IntClear( BURTC_IF_COMP0 );
    BURTC_IntDisable( BURTC_IF_COMP0 );
}



// Wait for up to (ca.) <ms_max> milliseconds in EM mode <em>, em normally from 1 to 3 (I guess).
// 4 might work, but it is possible that we would have to do some more setup.
// "Up to" because the wait is stopped early if an interrupt comes and at least <ms_min> ms have expired.
// The number of milliseconds waited is returned (approximately).
// We are depending on the counter runnning at 1 KHz after running burtc_config(),
// which seems to be correct (could be better documented).
// First call could take a little longer if burtc_config() has not already been called.
unsigned int wait_from_to_burtc(const unsigned int ms_min, const unsigned int ms_max, const unsigned int em)
{
    BURTC_Enable(false);

    if (!burtc_initialized)
        burtc_config();

    burtc_delay_complete = false;

    // Frequency: See comment in burtc_config()
    const unsigned int burt_freq = 1e3;

    BURTC_CounterReset(); // Might already have been done by BURTC_Enable(false) above
    BURTC_IntClear(BURTC_IF_COMP0);
    BURTC_CompareSet(0, (unsigned int)(burt_freq * ms_max / 1000 + 0.5));
    // const uint32_t co = BURTC_CompareGet(0);
    // Do we need to enable both BURTC itself and interrupts every time, and disable in callback?
    // That is how rtcdrv.c does it for RTC, so maybe best.
    BURTC_Enable(true);
    BURTC_IntEnable(BURTC_IF_COMP0);

    while (!burtc_delay_complete) {
    	switch (em) {
			case 1:
			EMU_EnterEM1();
			break;

		case 2:
			EMU_EnterEM2(true);
			break;

		case 3:
			EMU_EnterEM3(true);
			break;

		case 4:
			EMU_EnterEM4();
		}
		if (ms_min < ms_max) { // If-check only for efficiency
			INT_Disable();
			const unsigned int counter = burtc_delay_complete ? burtc_counter : BURTC_CounterGet();
			INT_Enable();
			const bool do_break =  (unsigned int)((float)counter / burt_freq * 1000 + 0.5) >= ms_min;
			if (do_break)
			  break;
		}
	}

	INT_Disable();
	if (!burtc_delay_complete)
	    BURTC_IRQHandler(); // Set burtc_counter and disable interrupt.
	INT_Enable();

    return (unsigned int)((float)burtc_counter / burt_freq * 1000 + 0.5);

}


