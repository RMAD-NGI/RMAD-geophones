/*
 * flash.h
 *
 *  Created on: 5. aug. 2014
 *      Author: iv
 */

#ifndef FLASH_H_
#define FLASH_H_
#include <stdbool.h> // Needed by nandflash.h, should have been included there
#include <stdint.h> // Needed by nandflash.h, should have been included there
#include <nandflash.h>
#include "em_common.h"
#include "timestamp.h"

// Not all of these defines are needed outside of flash.c, but let us keep them together.

// Magic number showing we have a buffer entry
#define FLASH_MAGIC_NUMBER 0.733535F // Must have "F" or "f" since flash_block_header_t uses float

// Algorithm-defined and helper defines and typedefs.
#define FLASH_HEADER_PAGES 1 // Timestamp etc

// Hardware-defined defines. Do not change unless the hardware changes.
// We could write the code with dynamic
// allocation everywhere we need it, but impractical. Instead we check
// that the defines matches the real values in assert's in the code (only tested in debug mode).
#define FLASH_SIZ (1 << 25) // 32 MB. Ah, seems "FLASH_SIZE" is already defined by system, cutting last letter
#define FLASH_PAGE_SIZ 512 // 512 B. Ah, seems "FLASH_PAGE_SIZE" is already defined by system (for NOR FLASH?), cutting last letter
#define FLASH_BLOCK_SIZE 16384 // 16 KB

#define FLASH_PAGE_ADDR(ib, ip) (flash_info()->baseAddress+ ib*FLASH_BLOCK_SIZE + ip*FLASH_PAGE_SIZ)
#define FLASH_BLOCK_NPAGES (FLASH_BLOCK_SIZE/FLASH_PAGE_SIZ)
#define FLASH_NBLOCKS (FLASH_SIZ/FLASH_BLOCK_SIZE)


// Maybe no point in packing this struct??
EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
	float magic_number;

	// 16 bit values: See comments for each. In FLASH we have plenty of space
	// to use 32 bits or more, but we want to cache them in RAM.
	uint16_t id; // ID of traceset. 16 bits is plenty, since we only need the ID to be unique
	             // at any given time and for a little time ahead (enough that the controller PC can use the ID to ask for data),
	             // not throughout the lifetime of the program.
	uint16_t seq_num; // 0 for first block in traceset, then 1, 2, ... 16 bits is plenty since we have just 2048 blocks or so,
	                  // and also because we probably never want to have tracesets longer than 2^16 blocks.

    uint32_t num_scans;
    timestamp_t timestamp;
    timestamp_t timestamp_initial; // Start of trace, corresponding to seq_num == 0
    unsigned char timestamp_is_correct;
} flash_block_header_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems


// I 'guess' the only reason to pack this struct is to save RAM.
EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
	enum { BLOCK_USED, BLOCK_UNUSED, BLOCK_BAD, BLOCK_ACTIVE, FLASH_BLOCK_NSTATES};

	// One of enums above. BLOCK_USED means the block and cached values below are ready for use.
	// BLOCK_ACTIVE is set when functions in this file writes data, but the header page in a block is still not written.
	// The header page in a block is still not written <=> the block is not ready for use.
	unsigned char status ;

	// Cache (when status == BLOCK_USED) of some of the fields of flash_block_header_t.
	// See (hopefully coming) explanation in process.c.
	uint16_t id;
	uint16_t seq_num;
	timestamp_t timestamp_initial;
	unsigned char timestamp_is_correct;
} block_info_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems


// FLASH block status and cache
extern block_info_t gl_block_info[FLASH_NBLOCKS];


int flash_block_to_use();
const NANDFLASH_Info_TypeDef *flash_info();// OK X.
int flash_write_page(const unsigned int iblock, const unsigned int ipage_in_block, const unsigned char * const page);
int flash_safe_write_page(unsigned int ipage_in_block, const unsigned char * const page, bool start_new_block);
int flash_find_next_good_block();
int flash_init(); // Run before anything else in this "module".

#endif /* FLASH_H_ */
