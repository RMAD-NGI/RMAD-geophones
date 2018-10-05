/*
 * adjustable_params.c
 *
  * Encapsulates parameters used around the program that may be changed
 * through a received communication packet and is initialized, and on change written to, (NOR?) FLASH.
 * For the time being, initialize it before use (adjustable_params_init()),
 * access values by reading *gl_adjustable_params, and change params by calling adjustable_params_set().
 *  Created on: 24. aug. 2014
 *      Author: iv
 */


#include <assert.h>
#include "em_system.h" // Get FLASH_PAGE_SIZE and FLASH_SIZE
#include "em_int.h"
#include "em_msc.h"
#include "em_acmp.h"
#include "adjustable_params.h"

// The address where we keep the parameters (including Magic Number and filler), at the top of initial 1MB (NOR) FLASH.
// Little point in "using" less than a page, since we anyway need to erase a full page.
#define FLASH_ADDR (FLASH_SIZE - FLASH_PAGE_SIZE)

//#define NORFLASH_MAGIC_NUMBER 0.11446F // Must have "F" or "f" since adjustable_params_ext_t uses float
//#define NORFLASH_MAGIC_NUMBER 0.11646F // Must have "F" or "f" since adjustable_params_ext_t uses float - new hardware new number
#define NORFLASH_MAGIC_NUMBER 0.11846F // Must have "F" or "f" since adjustable_params_ext_t uses float - hw3/sw3 fikset feilskriving i flash, trenger ny mn

EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
	adjustable_params_t params;
	float magic_number; // For checking whether the FLASH is initialized
	// Make sure the number of bytes is divisible by 4:
	unsigned char filler[((sizeof(adjustable_params_t) + sizeof(float)) % 4 == 0)
			                 ? 0
			                 : (4 - (sizeof(adjustable_params_t) + sizeof(float)) % 4)];
} adjustable_params_ext_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems


// Master copy of values, including magic and filler.
// Initialized from FLASH if possible (adjustable_params_init()),
// otherwise the below values are used (until changed).
// OBS If you change any of the values, consider changing also NORFLASH_MAGIC_NUMBER
static adjustable_params_ext_t gl_adjustable_params_ext = {
		3000,
		2000,
		36000,
		{100, 100, 100, 100},
		{120, 120, 120, 120},
		{70, 4},
		{acmpChannel0, acmpChannel2},
		5,
		15,
	NORFLASH_MAGIC_NUMBER, // Don't change
	// filler doesn't have to be initialized.
};


// Users read the actual params through this *pointer*:
const adjustable_params_t *const gl_adjustable_params = &gl_adjustable_params_ext.params;


// Pointer to the FLASH copy of the parameters, with magic number and filler up to nearest 4 bytes,
// unless a FLASH error has been detected, in which case it is 0.
adjustable_params_ext_t *gl_flash_adjustable_params_ext = (adjustable_params_ext_t *)FLASH_ADDR;

// Pointer to the FLASH copy of the parameters (without magic number and filler up to nearest 4 bytes),
// unless a FLASH error has been detected, in which case it is 0.
adjustable_params_t *gl_flash_adjustable_params = (adjustable_params_t *)FLASH_ADDR;


// Initialize by writing to FLASH if it is uninitialized, otherwise read from FLASH into gl_adjustable_params.
// If FLASH write error, we continue to work in RAM.
int adjustable_params_init()
{
	assert(sizeof(adjustable_params_ext_t) % 4 == 0); // Needed by FLASH

	if (gl_flash_adjustable_params_ext->magic_number != NORFLASH_MAGIC_NUMBER) {
		// Uninitialized NAND FLASH.
       return adjustable_params_set(&gl_adjustable_params_ext.params);
	}
	else {
		gl_adjustable_params_ext.params = *gl_flash_adjustable_params;
		return 0;
	}
}


// Set master copy to <*pars> and write to FLASH.
int adjustable_params_set(const adjustable_params_t *const pars)
{
    INT_Disable();
	gl_adjustable_params_ext.params = *pars;


	// EFM32_ALIGN in case we are using anything but gcc, __attribute__ ... for gcc.
	EFM32_ALIGN(4) adjustable_params_ext_t pars_aligned __attribute__ ((aligned(4))) = {
		*pars,
		NORFLASH_MAGIC_NUMBER
	};

	bool ok = false;

	if (gl_flash_adjustable_params_ext != 0) {
		// FLASH is OK as far as we know.
        MSC_Init();
		ok = MSC_ErasePage((uint32_t *)FLASH_ADDR) == mscReturnOk;
		if (ok) {
			ok = MSC_WriteWord((uint32_t *)FLASH_ADDR, &pars_aligned, sizeof(adjustable_params_ext_t)) == mscReturnOk; // mscReturnOk, not flashReturnOk! Yet another error in the documentation.
		}
		MSC_Deinit();
		if (!ok) {
			// FLASH error. From now on we work only in RAM
			gl_flash_adjustable_params_ext = 0;
			gl_flash_adjustable_params = 0;
		}
	}

    INT_Enable();
	return ok ? 0 : -1;

}

