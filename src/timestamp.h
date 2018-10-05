/*
 * timestamp.h
 *
 * Functionality for 96-bit timestamp modelled on "SmartMesh IP Mote Serial API
*  Guide" (UTC_TIME_L) consisting of
 * a) 64-bit int #secs since base (1970 if that matters)
 * b) 32-bit int microsecs.
 *
 * If you use double-representation of timestamps, and your double is 64 bits IEEE 754 double,
 * the resolution for timestamps around year 2015 is less than a microsecond according
 * to my calculations:
 * ca. 9.15 decimal digits for seconds => ca. 6.8 decimal digits for fractional seconds. See http://en.wikipedia.org/wiki/IEEE_floating_point
 * Does not quite fit with what I remember, but I guess that was for MATLAB with base at year 0 instead of 1970.
 * Calculations may give worse accuracy!
 *
 *  Created on: 13. aug. 2014
 *      Author: iv
 */

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_
#include <stdbool.h>
#include "em_common.h"


EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
	int64_t secs;
	int32_t microsecs;
} timestamp_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems

// Add delta (positive, negative or 0) seconds to t
#define TIMESTAMP_ADD_SECS(t, delta) timestampd(timestamp_secs(t) + (delta))

timestamp_t timestampd(const double secs);

double timestamp_secs(const timestamp_t t);

timestamp_t timestamp(const int64_t secs, const int32_t microsecs);

bool timestamp_bigger(const timestamp_t t0, const timestamp_t t1);



#endif /* TIMESTAMP_H_ */
