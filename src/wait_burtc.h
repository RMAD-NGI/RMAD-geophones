/*
 * wait_burtc.h
 *
 *  Created on: 3. aug. 2014
 *      Author: iv
 */

#ifndef WAIT_BURTC_H_
#define WAIT_BURTC_H_


void burtc_config(void);


unsigned int wait_from_to_burtc(const unsigned int ms_min, const unsigned int ms_max, const unsigned int em);




#endif /* WAIT_BURTC_H_ */
