/*
 * debug_printf.h
 *
 *  Created on: 21. aug. 2014
 *      Author: iv
 */

#ifndef DEBUG_PRINTF_H_
#define DEBUG_PRINTF_H_

/* Functionality for debug print and Energy Profiler
 *
 * With this header file included, and debug_printf.c and system files bsp_trace.c and retargetio.c in your project,
 * you may easily debug your program using
 *   PRINT(...) // Same parameters as printf(), similar effect
 *   PRINTF(...) // Same parameters as printf(), same as PRINT() except adds filename and line number
 * Make sure to add a '\n' when you want the line to be flushed.
 *
 * If you are planning to use the Energy Profiler, or just want to avoid a possible delay
 * the first time you use PRINT(F), you may use "DEBUG_INIT;" explicitly, early in the program.
 *
 * There might be some configuration needed on the system, but for me (IV) it worked "as delivered to me".
 *
 * To see the output, start SEGGER J-Link SWO Viewer. For "geophone test project", select 14000 KHz and Cortex M3.
 *
 * You may risk the internally used printf() to be re-entered if you PRINT(F) from an ISR,
 * to avoid this you may uncomment INT_Disable() / INT_Enable() in the macro below,
 * but this could easily lead you to miss interrupts!
 * This re-entering may not be safe, but we don't use the debug code when in release mode.
 * If you want to disable it even for debug mode, set ENABLE_DEBUG below to 0.
 *
 * At some point it seemed I had to restart the SWO Viewer every time i restarted the
 * program, but that has not been the case lately. Change in initialization, new SEGGER software version, or what?
 * It could be wise to have a PRINT(F) near the start of the program, to verify that output works.
 */

#define ENABLE_DEBUG 1 // If 0 => do not use debug code below even in debug mode

#if ENABLE_DEBUG && !defined(NDEBUG)
#include <stdio.h>
#include "em_int.h"
#include "bsp_trace.h"

	extern int gl_debug_printf_initialized;

	#define DEBUG_INIT \
		do { \
           if (!gl_debug_printf_initialized) { \
			  BSP_TraceSwoSetup2(); /*Energy profiler */ \
			  /*SWO_Setup(); */\
			  /*// Not needed for us it seems BSP_TraceEtmSetup(); */\
			  gl_debug_printf_initialized = 1; \
			} \
		} while (0)

	#define PRINT(...) \
		do { \
			/*INT_Disable();*/\
			DEBUG_INIT; \
			printf(__VA_ARGS__); \
			/*INT_Enable();*/\
		} while (0)

	#define PRINTF(...) do {PRINT("%s() #%d: ",__func__, __LINE__); PRINT(__VA_ARGS__); } while (0)
#else
	#define DEBUG_INIT
	#define PRINT(...)
	#define PRINTF(...)
#endif


void BSP_TraceSwoSetup2(void);


#endif /* DEBUG_PRINTF_H_ */
