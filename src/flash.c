/*
 * flash.c
 *
 *  Created on: 5. aug. 2014
 *      Author: iv
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <bsp.h>
#include <em_int.h>
#include "timestamp.h"
#include "flash.h"

// Functionality for *NAND* flash.

/****** Principles for writing to external NAND FLASH
 *
 * Seems like the previousy (now deleted) principles were for the internal (NOR?) flash.
 * See Visual SourceSafe if you are interested in the principles.
 *
 * For using NAND flash, see references below.
 *I used [2] for some code, as well as to test the following: Does the FLASH
 * module handle erasing blocks before writing pages? Maybe that is not possible,
 * since erasure is of a block, while writing is of a page, but the example
 * allows you to perform pagewrites without erasing first.
 * So I wrote the pattern 0:N to a page, using a modified [2].
 * No errors from write. Then i wrote 1:N+1 to the same page. Still no write
 * error from the API. Then I tried to read back the same page. "Uncorrectable data error in page" (-5)!
 * Data was X = [0 0 2 0 4 4 6 0 8 8 ...]
 * Postulate: X is 0:N AND 1:N+1 (bitwise applied to each vector element), based on [1].
 * Result: bitand(0:9, 1:10) in MATLAB gives exactly the initial part of the read page, reproduced above.
 *
 * So it appears that if we don't erase the block of a page before writing it, we can only
 * change contents by making more bits zero.
 * In practice, for us, I guess we will erase a block before writing to any page within the block.
 *
 * Another observation: Before ever writing to some page (after factory or last block erase I guess), we will get an error return
 * from NANDFLAS_ReadPage() trying to read that page, presumably due to the ECC being just 0xff's.
 *
 * Plan:
 * Current hardware: Mem. = 32GB, block = 16KB, page = 512B.
 * We need to timestamp data, and maybe have some magic number showing that the data
 * are from this application. Maybe also other info. Let us call this "header".
 * Minimum unit of write is one page, one header for less
 * than a page seems way overkill. A header for each page also seems overkill,
 * wastes space and lot to read when searching through data. A header for each block
 * seems more relevant. Good and bad memory areas are handled on a per-block-basis,
 * which is OK. If a block goes bad, we lose one header and the contents following.
 * With current setup, one block will have approximately 2.4 seconds worth of data (4 channels, 16 bits).
 * We typically loose some space at the end of the last block in a recording run, though
 * not too much (equivalent to on average ca. 1.2 seconds?).
 *
 * References:
 * 1 http://en.wikipedia.org/wiki/Flash_memory#NAND_memories
 * 2 STK3700_nandflash software example.
 * 3 Chapter 14 in P:\2014\00\20140081\Design\Software\Inge\d0053_efm32gg_reference_manual.pdf
*/


NANDFLASH_Info_TypeDef *gl_flash_info;// OK X.

static int gl_next_block_to_use; // "next" is maybe better understood as "current"


// RAM info about NAND-FLASH blocks.
block_info_t gl_block_info[FLASH_NBLOCKS];


int flash_block_to_use()
{
	return gl_next_block_to_use;
}


const NANDFLASH_Info_TypeDef *flash_info()
{
	return gl_flash_info;
}


// When one of the flash write functions succeed, this functions sets
// corresponding fields in gl_block_info. If the page is page 0,
// the block is marked as ready to use by setting status == BLOCK_USED,
// and other values are taken directly from the page. Otherwise,
// status <- BLOCK_ACTIVE and other fields are set to "default values".
void set_fields_on_success(const unsigned int iblock, const unsigned int ipage_in_block, const unsigned char *const page)
{
	// data in *<page> could be misaligned:
    // In fact I checked the code, and as it is called as of 20140923 it cannot be
    // misaligned when ipage_in_block == 0, so this change is unneccessary,
    // but OK for future use.
    flash_block_header_t block_header;
	memcpy(&block_header, page, sizeof(block_header));
	flash_block_header_t * const fh = &block_header; // Just to avoid changing the code below.

	const bool is_header_page = ipage_in_block == 0;
	gl_block_info[iblock].id = is_header_page ? fh->id : -1;
	gl_block_info[iblock].seq_num = is_header_page ? fh->seq_num : -1;
	gl_block_info[iblock].timestamp_initial = is_header_page ? fh->timestamp_initial : timestamp(0, 0);
	gl_block_info[iblock].timestamp_is_correct = is_header_page ? fh->timestamp_is_correct : false;
	gl_block_info[iblock].status = is_header_page ? BLOCK_USED : BLOCK_ACTIVE;
}


// Tries to write page to already erased (if necessary) block.
// Special "low-level" function, does not move the block
// in case of write error, does not access gl_next_block_to_use,
// maybe etc.
// Consider using flash_safe_write_page() instead.
//
// See also description of set_fields_on_success().
int flash_write_page(const unsigned int iblock, const unsigned int ipage_in_block, const unsigned char * const page)
{
	const uint32_t block_addr = FLASH_PAGE_ADDR(gl_next_block_to_use, ipage_in_block);
	if (NANDFLASH_EraseBlock(block_addr) == NANDFLASH_WRITE_ERROR) {
		NANDFLASH_MarkBadBlock(block_addr);
		gl_block_info[iblock].status = BLOCK_BAD;

		return -1;
	}

	set_fields_on_success(iblock, ipage_in_block, page);
	return 0;
}


// Writes data pointed to by <page> into NAND FLASH, block <gl_next_block_to_use>,
// page (within block) <ipage_in_block>.
// Iff <start_new_block>, the block will be erased first.
// The difference between calling this and calling NANDFLASH_EraseBlock/WritePage directly,
// is that this function buffers a whole block, with write-through, and if
// the FLASH fails, the whole contents are moved to the next good block.
// Iff no good block can be found, -1 (or another negative value?) is returned, otherwise the index of (last) block written to.
//
// See also description of set_fields_on_success().
int flash_safe_write_page(unsigned int ipage_in_block, const unsigned char * const page, bool start_new_block)
{
	// We buffer a whole block:
	static unsigned char block[FLASH_BLOCK_SIZE];
	static unsigned char is_written_to[FLASH_BLOCK_NPAGES]; // bool is 4 bytes in current implementation

	static bool first_call = true;

    if (first_call || start_new_block) {
		memset(block, 0xff, sizeof(block));
		memset(is_written_to, 0, sizeof(is_written_to));
		first_call = false;
    }

    int npages = 1;
    memcpy(block+ipage_in_block*FLASH_PAGE_SIZ, page, FLASH_PAGE_SIZ);
	is_written_to[ipage_in_block] = true;

	// Now <block> contains what we want the 'current or just finished flash block' to contain when we finish.

	// This check could be put on the very start of the function.
	if (gl_next_block_to_use < 0)
		return -1; // No good blocks.

	bool flash_error;
	do {
		const uint32_t block_addr = FLASH_PAGE_ADDR(gl_next_block_to_use, 0);
		flash_error = start_new_block && NANDFLASH_EraseBlock(block_addr) == NANDFLASH_WRITE_ERROR;
		if (!flash_error) {
			for (unsigned int page_cnt = 0; page_cnt < npages; ++page_cnt) {
				const unsigned int ipage = ipage_in_block + page_cnt;
				if (is_written_to[ipage]) {
					uint32_t addr = FLASH_PAGE_ADDR(gl_next_block_to_use, ipage);
					flash_error = NANDFLASH_WritePage(addr, block+ipage*FLASH_PAGE_SIZ) == NANDFLASH_WRITE_ERROR;
					if (flash_error)
						break;
				}
			}
		}
		if (flash_error) {
			NANDFLASH_MarkBadBlock(block_addr);
			gl_block_info[gl_next_block_to_use].status = BLOCK_BAD;
			ipage_in_block = 0;
			npages = FLASH_BLOCK_NPAGES;
			start_new_block = true;
			flash_find_next_good_block();
		}
		else {
			// Success:
			set_fields_on_success(gl_next_block_to_use, ipage_in_block, page);
	    }
	} while (gl_next_block_to_use >= 0 && flash_error);

	return gl_next_block_to_use;

}


// Propagates gl_next_block_to_use to index of next good block (used or unused).
// -1 (or any other more negative value) as input is taken to mean "all blocks are already detected to be bad, do nothing".
int flash_find_next_good_block()
{
	if (gl_next_block_to_use <= -1)
		return gl_next_block_to_use; // All blocks are bad! We will not write to flash

	const int check_block = gl_next_block_to_use = (gl_next_block_to_use + 1) % FLASH_NBLOCKS;
	while (gl_block_info[gl_next_block_to_use].status == BLOCK_BAD) {
		gl_next_block_to_use = (gl_next_block_to_use + 1) % FLASH_NBLOCKS;
		if (gl_next_block_to_use == check_block) {
			gl_next_block_to_use = -1; // All blocks are bad! We will not write to flash
			break;
		}
	}
	return gl_next_block_to_use;
}


// Run before writing to flash.
// Will initialize FLASH, gl_block_info and gl_next_block_to_use,
// and perform sanity checks (in debug mode).
// Returns the highest used traceset ID.
int flash_init()
{

	/* Setup EBI for NAND Flash. */
	BSP_EbiInit();
	NANDFLASH_Init(5); // DMA channel 5 used in example, so we use it.
	gl_flash_info = NANDFLASH_DeviceInfo();// OK X.

	// Sanity checks.
	assert(FLASH_SIZ == gl_flash_info->deviceSize);
	assert(FLASH_BLOCK_SIZE == gl_flash_info->blockSize);
	assert(FLASH_PAGE_SIZ == gl_flash_info->pageSize);
	assert(FLASH_SIZ % FLASH_BLOCK_SIZE == 0);
	assert(FLASH_BLOCK_SIZE % FLASH_PAGE_SIZ == 0);

	unsigned char buffer[EFM32_MAX(gl_flash_info->spareSize, gl_flash_info->pageSize)];
	unsigned int addr = gl_flash_info->baseAddress;
	int latest_used_block = -1,
		latest_used_id = -1,
		latest_used_seq_num = -1;
	for (unsigned int iblock = 0; iblock < FLASH_NBLOCKS; ++iblock) {
		NANDFLASH_ReadSpare(addr, buffer); // Peeked in the implementation, seems safe to ignore return value
        /* Manufacturer puts bad-block info in byte 6 of the spare area */
        /* of the first page in each block.                             */
        const bool is_bad = buffer[NAND_SPARE_BADBLOCK_POS] != 0xFF;
        if (is_bad) {
        	gl_block_info[iblock].status = BLOCK_BAD;
        }
        else if (NANDFLASH_ReadPage(addr, buffer) != NANDFLASH_STATUS_OK) {
			gl_block_info[iblock].status = BLOCK_UNUSED;
		}
		else {
			flash_block_header_t header;
			memcpy(&header, buffer, sizeof(flash_block_header_t));
		    gl_block_info[iblock].status = (header.magic_number == FLASH_MAGIC_NUMBER) ? BLOCK_USED : BLOCK_UNUSED;
			if (gl_block_info[iblock].status == BLOCK_USED) {

				gl_block_info[iblock].id = header.id;
				gl_block_info[iblock].seq_num = header.seq_num;
				gl_block_info[iblock].timestamp_initial = header.timestamp_initial;

				if (latest_used_block < 0
					|| header.id > latest_used_id
					|| header.id == latest_used_id && header.seq_num > latest_used_seq_num) {

					latest_used_id = header.id;
					latest_used_seq_num = header.seq_num;
					latest_used_block = iblock;
				}
			}
	    }
        addr += gl_flash_info->blockSize;
	}

    gl_next_block_to_use = (latest_used_block < 0) ? (FLASH_NBLOCKS - 1) : latest_used_block;
    flash_find_next_good_block();

    return latest_used_id;
}



