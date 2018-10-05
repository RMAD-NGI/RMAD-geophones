/*
 * process.h
 *
 *  Created on: 5. aug. 2014
 *      Author: iv
 */

#ifndef PROCESS_H_
#define PROCESS_H_
#include <stdbool.h> // Needed by nandflash.h, should have been included there
#include <stdint.h> // Needed by nandflash.h, should have been included there
#include <nandflash.h>
#include "packet.h"
#include "timestamp.h"
#include "config.h" // Get CONFIG_AD_NCHANS


typedef uint16_t sample_t;
typedef sample_t scan_t[CONFIG_AD_NCHANS];


extern timestamp_t gl_timestamp;

EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
// Used in data requests and data sending.

    uint16_t    id; // "temporary ID", will be unique for many passes through FLASH.
    uint8_t     channels; // Bit-mask of channel flags, 1 <=> first channel, ...
    uint32_t    iscan_start, iscan_stop; /* Index of first and last scan, starting at 0.
                                          * DO NOT DEFINE ANY FIELD AFTER iscan_stop,
                                          * without searching for "offsetof(data_desc_t"
                                          * and amending the code!
                                          */
} data_desc_t; // Description of data requested sent
EFM32_PACK_END(); // Actually a no-op for GNU it seems

typedef struct {
	uint16_t latest_flash_id;
	timestamp_t last_timestamp;
	uint16_t sampling_rate;
	uint16_t num_samples;
} event_t;


unsigned int process_log_until_add(unsigned int add);

unsigned char send_data(const unsigned int iscan_start, const unsigned int iscan_end, data_desc_t * const data_desc, const unsigned int nchans, sample_t *data);

unsigned int process_log_until();

int process_ringbuf_entries();

void process_ringbuf_lock_traceset();

void process_ringbuf_add_scan(scan_t scan);

void process_ringbuf_add();

void process_ringbuf_apply_timestamp(const seq_num_t packet_ind, timestamp_t timestamp);

uint16_t process_latest_flash_id();

timestamp_t process_latest_flash_timestamp();

event_t process_latest_flash_event();

void process_list_data();

void process_send_data();

void process_config();

#endif /* PROCESS_H_ */
