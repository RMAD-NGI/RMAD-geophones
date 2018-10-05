/*
 * timestamp.c
 *
 *  Created on: 13. aug. 2014
 *      Author: iv
 */
#include <math.h>
#include "timestamp.h"


// Return generated timestamp
timestamp_t timestampd(const double secs)
{
	timestamp_t t;
	t.secs = floor(secs);
	t.microsecs = round((secs - t.secs) * 1e6);

	return t;
}


// Return (possibly non-integer) seconds since base
double timestamp_secs(const timestamp_t t)
{
	return t.secs + t.microsecs / 1.0e6;
}


// Return generated timestamp
timestamp_t timestamp(const int64_t secs, const int32_t microsecs)
{
	timestamp_t t;
	t.secs = secs;
	t.microsecs = microsecs;

	return t;
}


// True iff t0 > t1
bool timestamp_bigger(const timestamp_t t0, const timestamp_t t1)
{
	return t0.secs > t1.secs || t0.secs == t1.secs && t0.microsecs > t1.microsecs;
}



