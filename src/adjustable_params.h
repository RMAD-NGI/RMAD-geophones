/*
 * adjustable_params.h
 *  Created on: 24. aug. 2014
 *      Author: iv
 */

#ifndef ADJUSTABLE_PARAMS_H_
#define ADJUSTABLE_PARAMS_H_
#include <stdint.h>
#include "em_common.h"
#include "config.h" // Get CONFIG_AD_NCHANS

EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
	uint32_t min_nscans_after_uart;
	uint32_t min_nscans_after_cont_lim;
	uint32_t max_nscans; // Overrides the other settings above.

	//float std_lim[CONFIG_AD_NCHANS]; // Give warning if std. dev. is >= <std_lim>
	//float cont_lim[CONFIG_AD_NCHANS]; // FLASH-log at least <min_nscans_after_cont_lim> scans after std. dev. is >= <cont_lim>.

	float std_lim[4]; // Give warning if std. dev. is >= <std_lim>
	float cont_lim[4]; // FLASH-log at least <min_nscans_after_cont_lim> scans after std. dev. is >= <cont_lim>.

	uint32_t comp_trig_levels[2]; // Comparator trig levels.
	ACMP_Channel_TypeDef comp_pos_sels[2]; // Input selection for ACMP0 and ACMP1
	uint16_t preamp_cmp_trigg; //bool array of preamps turned on for comparator trigging
	uint16_t preamp_logging; //bool array of preamps turned on during logging
} adjustable_params_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems


extern const adjustable_params_t * const gl_adjustable_params;
int adjustable_params_init();
int adjustable_params_set(const adjustable_params_t *const pars);


#endif /* ADJUSTABLE_PARAMS_H_ */
