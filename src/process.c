/*
 * process.c
 *
 *  Created on: 5. aug. 2014
 *      Author: iv
 *
 *      endret av YK 27/9 -2016
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bsp.h"
#include "em_common.h"
#include "em_int.h"
#include "em_wdog.h"
#include "debug_printf.h"
#include "dust_com.h"
#include "timestamp.h"
#include "geophone.h" // Get logging_is_running()
#include "process.h"

// Do not uncomment the following line unless you know what you do.
// This define exists so we do not have to delete code that might
// be useful in future.
// You may read more info in comment above process_ringbuf_add_scan().
//#define ALLOW_CALL_PROCESS_RINGBUF_ENTRIES

/* Regarding CMSIS DSP:
 * Some time has been spent trying to use the library, since this could be useful for
 * future calculations.
 * We have a big-endian system, as finally "clinched" by the memory view, the
 * "Expressions" view seems to not handle expressions like ((char *)&variable) correctly.
 * [Update 20140823: Actually I have just clinched that we have a little-endian system!
 * BUT in the Memory Monitor You must specify big-endian to get the correct memory layout (separate for each "Memory Monitor"). Crazy!]
 * CMSIS DSP is distributed with prebuilt versions, see
 *   P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\Documentation\DSP\html\index.html
 *   C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2\CMSIS\Lib\*
 * HOWEVER, as can be seen under the C:\.. link above, for GCC, which is the toolchain we use,
 * only little-endian libraries have been pre-built!
 * Also, the first link above says
 *   "The library has been developed and tested with MDK-ARM version 4.60. The library is being tested in GCC and IAR toolchains and updates on this activity will be made available shortly.".
 * So not properly tested for GCC? And no pre-built libraries that we can use.
 *
 * I tried to include source code directly in the application:
 * Arg, trying that I managed to delete MANY files under C:\SiliconLabs\SimplicityStudio\v2\developer\sdks\efm32\v2\CMSIS\DSP_Lib\Source\StatisticsFunctions,
 * may need to reinstall later if I need any of them.
 * I then linked almost everything under P:\2014\00\20140081\Design\Software\Inge\CMSIS-SP-00300-r4p1-00rel0\CMSIS\DSP_Lib\Source,
 * which is a newer version of the library. I managed to compile it, but the standard deviation
 * it gave was 0, same as I got when I linked the small-endian prebuilt library.
 * I tried to debug, and found that it at least used integer division internally.
 * Not what I want I guess. Tried to debug more by disabling interrupts, then
 * I lost contact with first one, then the second mote!
 * I have struggled to get them back on line, but now they are OK, and I have cleaned
 * up most of the CMSIS-DSP stuff from the project settings (left some define that is OK to keep).
 *
 * We define ARM_MATH_CM3 to indicate Cortex M3 (in code, not project, since project setting(s) ("StudioSDKPath") is/are sometimes forgotten).
*/
//#define ARM_MATH_MATRIX_CHECK
//#define ARM_MATH_ROUNDING
//#define UNALIGNED_SUPPORT_DISABLE
//#define ARM_MATH_CM3
//#include "arm_math.h"

#include "debug_printf.h"
#include "timestamp.h"
#include "config.h"
#include "packet.h"
#include "flash.h"
#include "adjustable_params.h"
#include "process.h"


/****** Principles for handling logged data.
 * In the following we assume that ALLOW_CALL_PROCESS_RINGBUF_ENTRIES is NOT defined,
 * it should not be defined unless you really know what you do:
 *
 * gl_rrb is a ring buffer of "runs", where each "run" is a ringbuffer of scans
 * corresponding to the same comparator trig (or "trig" from controller PC
 * asking the MCU to start logging).
 * ADC0_IRQHandler() stores a new scan into the "current" run-ringbuffer
 * (calling process_ringbuf_add_scan()), and when
 * the run is finished "locks" it (setting a flag), after which no new data
 * can be added (the run is also locked when a new run is added to the ring buffer
 *  of runs, locking twice is OK).
 * The data in the runs is consumed by process_data() which is called
 * from non-interrupt-service code (main()).
 * If process_data() does not manage to keep up with incoming data so that either
 * the current ringbuffer of scans is filled, or the ringbuffer of runs is filled,
 * nothing is added! So if process_data() is not called often enough or takes too long
 * to run, you may lose scans or even whole runs.
 * process_data() handles both calculations on data blocks (standard deviance/shift between channels),
 * sending warnings based on standard deviance "too high", extending logging period based
 * on high standard deviance and storing data to NANDFLASH.
 * Timestamping of FLASH blocks in a run is delayed until a time-message is received
 * from mote or the run is over, see code.
 */

/****** Principles for writing to external NAND FLASH
 *      (old description, but maybe somewhat complimentary to the above)
 *
 * For storing data, we will use a ring buffer of ring buffers.
 * Each ring buffer will correspond to one "recording_start", so that we
 * can handle new comparator-trigs quickly after each other. This outer ring buffer may
 * have few elements, just 2 should be enough.
 * Each inner ring buffer is associated with its start time. They are filled out
 * by AD ISR, and read and processed in (functions called from?) main().
 * Why not process and write to FLASH in the ISR? Because it perhaps could take
 * enough time (at least FLASH) to make us miss AD samples. We might later consider
 * putting the processing (for warning) in the ISR, if necessary/possible. If so, remember
 * that currently ISR's are not allowed to send any USART package except ack's, though
 * I think that restriction is easily removed. In fact, I guess the code (not the description)
 * already may allow it, after introducing INT_Disable/Enable in tx_to_mote()?
*/

///////////////////// PARAMETERS YOU MAY CHANGE "FREELY"
#define PROCESS_NPAGES 3 // We must process an integer number of pages at a time with current algorithm
#define PROCESS_MIN_NSCANS_REQUIRED_FOR_CALCULATIONS (PROCESS_NSCANS/4) // The divisor of 4 can be changed quite freely.
///////////////////// END PARAMETERS YOU MAY CHANGE "FREELY"



#define FLASH_PAGE_NSCANS (FLASH_PAGE_SIZ/sizeof(scan_t))
#define FLASH_BLOCK_EFFECTIVE_NSCANS ((FLASH_BLOCK_SIZE/FLASH_PAGE_SIZ - FLASH_HEADER_PAGES)* FLASH_PAGE_NSCANS)
#define PROCESS_NSCANS (FLASH_PAGE_NSCANS * PROCESS_NPAGES)

// Buffer sizes (#elements).
// Outer must be >= 2 if we want to allow a new trace-set to start (on a new comparator trig)
// while we haven't yet flashed/processed previous data.
// Inner should be >> what fits in a FLASH page and also >> the processing length in number of scans.
#define FLASH_OUTER_BUFSIZE 3 // Should be at least 2 (which probably is enoguh).
#define FLASH_INNER_BUFSIZE (10*PROCESS_NSCANS) // #Scans. Does not have to be n*PROCESS_NSCANS, but most efficient.


EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
typedef struct __attribute__ ((__packed__)) {
    timestamp_t timestamp; // Timestamp of first sample in interval
    unsigned char reached_limit;
    float std_devs[CONFIG_AD_NCHANS];

   // float shifts[CONFIG_AD_NCHANS*(CONFIG_AD_NCHANS-1)/2];

    sample_t max[CONFIG_AD_NCHANS];
    sample_t min[CONFIG_AD_NCHANS];
    uint16_t max_samples[CONFIG_AD_NCHANS];
    uint16_t min_samples[CONFIG_AD_NCHANS];
    //float mean[CONFIG_AD_NCHANS];

} avalanche_warning_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems


// Inner ringbuffer type.
// Will be populated by the AD ISR and read by other code.
typedef struct {
	uint64_t start_packet_ind; // Incoming packet index when starting. Needed to define start_time
	timestamp_t start_time; //
	bool timestamp_is_correct;
	bool locked; // locked => trace set is teminated, do not add any more scans
	unsigned int istart, // Start logical index of scans, does not wrap on FLASH_INNER_BUFSIZE, so counts the number of processed scans in this set of traces.
	             num; // Elements in buf
	scan_t buf[FLASH_INNER_BUFSIZE];
} scan_ringbuf_t;


static unsigned int gl_next_page_in_block_to_use = 1; // Skip 0 for now, that will be the header page in the block.

// Outer ring buffer (ringringbuf)
typedef struct {
	unsigned int istart, // Start logical index, does not wrap on FLASH_OUTER_BUFSIZE. Set in process_config() for use as "id".
	             num, // Elements in buf
	             log_until; // If logging is active, log until istart == log_until. May change during logging
	scan_ringbuf_t buf[FLASH_OUTER_BUFSIZE];
} traceset_ringbuf_t;


// The ring buffer
traceset_ringbuf_t gl_rrb = {0, 0, 0};




// Latest run ID (attempted) written to flash.
static uint16_t gl_latest_flash_id;

timestamp_t gl_timestamp;
static timestamp_t gl_last_timestamp;

static unsigned int max;
static unsigned int min;
static unsigned int maxsample;
static unsigned int minsample;
static float sample_mean;

// Traceset ID used/to use. IDs must be unique at any given point in time,
// but otherwise no problem to eg. let it wrap around.
// Pretty much the same as gl_rrb.istart % 65536, except that the latter starts
// at 0 every time the program is restarted. Also some details about when
// the values are increased, maybe.
// We might get rid of it if we initialized gl_rrb.istart the same way as we now initialize this?
// UPDATE: Trying gl_rrb.istart!
// static uint16_t gl_traceset_id_to_use;


unsigned int process_log_until()
{
	INT_Disable();
	unsigned int ret = gl_rrb.log_until;
	INT_Enable();
	return ret;
}


void log_until_add(unsigned int add)
{
	const scan_ringbuf_t *rb = &gl_rrb.buf[gl_rrb.istart% FLASH_OUTER_BUFSIZE]; // Traceset we are processing
	if (rb->locked)
		return; // The traceset is locked, we cannot mark for future logging based on this. The logging may or may not be working on a new traceset already.
	INT_Disable();
	gl_rrb.log_until = EFM32_MIN(gl_adjustable_params->max_nscans, EFM32_MAX(rb->istart + add, gl_rrb.log_until));
	PRINTF("gl_rrb.log_until set to %u\n", gl_rrb.log_until);
	INT_Enable();
}


// We use the same formula as in CMSIS-DSP. I verified it, more or less, using Wikipedia.
float std_deviance(const uint16_t trace[], const unsigned int num_samples, float *const mean)
{
	if (num_samples <= 1)
		return 0.0;

	double sum = 0.0, sum_sqr = 0.0, sum_offset = 0.0;

	max = 0; //0x7FFF;
	min = 0xFFFF;
	maxsample = 0;
	minsample = 0;

	for (unsigned int isample = 0; isample < num_samples; ++isample) {
		sum += trace[isample] - 0x7FFF;
		sum_sqr += (trace[isample] - 0x7FFF) * (trace[isample] - 0x7FFF);
		sum_offset += trace[isample];

		if (trace[isample]>max){

			max = trace[isample];
			maxsample = isample;
		}
		if (trace[isample]<min){

			min = trace[isample];
			minsample = isample;
		}

	}

	if (mean != 0)
	    *mean = sum / num_samples;
		sample_mean = sum_offset / num_samples;
	return sqrt((sum_sqr - sum*sum/num_samples) / (num_samples - 1));
}



// This function evaluates the scans, and if necessary extends the period of logging.
// If the number of scans are deemed high enough, output values include a bitmap of channels above std_lim and
// std_devs and phases (phases only if implemented, see code), and 0 is returned.
// Otherwise -1 is returned.
int process_data(const scan_t scans[], const uint32_t num_scans,
                 // Output values, only guaranteed to be defined if return value == 0:
                 unsigned char *reached_limit, float std_devs[], sample_t maxs[], sample_t mins[], uint16_t maxs_samples[],uint16_t mins_samples[], float sample_means[])
{

    if (num_scans < PROCESS_MIN_NSCANS_REQUIRED_FOR_CALCULATIONS)
		return -1; // If much less data than the processing period, we don't use it.

	float means[CONFIG_AD_NCHANS];
	uint16_t trace[num_scans];
	bool reached_continue_limit = false;

    *reached_limit = 0; // bit map of channel flags
    for (int ichan = 0; ichan < CONFIG_AD_NCHANS; ++ichan) {
		for (int iscan = 0; iscan < num_scans; ++iscan)
			trace[iscan] = scans[iscan][ichan];
		std_devs[ichan] = std_deviance(trace, num_scans, means + ichan);
//	    arm_std_q15(trace, num_scans, &std_dev);
		if (std_devs[ichan] >= gl_adjustable_params->cont_lim[ichan])
			reached_continue_limit = true;
		if (std_devs[ichan] >= gl_adjustable_params->std_lim[ichan])
			*reached_limit |= 1 << ichan;

		maxs[ichan] = max;
		mins[ichan] = min;
		maxs_samples[ichan] = maxsample;
		mins_samples[ichan] = minsample;
		sample_means[ichan] = sample_mean;

    }


	if (reached_continue_limit)
		log_until_add(gl_adjustable_params->min_nscans_after_cont_lim + num_scans); // From end of current processing interval (" +num_scans"), maybe use midpoint instead? Not so important I guess


	return 0; // success
}


// Checks if (enough) new AD scans are available in the ringringbuf, and handles them accordingly
// by processing (possibly sending messages about alarms), and flashing.
// We copy data from the ring buffer in chunks of FLASH_PROCESS_NSCANS scans.
// If the inner ring buffers are long enough,it should be fairly safe to
// use the values directly in the ring buffers, but to be safe we make a copy.
// Making a copy is itself a bit dangerous, since it may take some time, and
// we disable interrupts while doing it to make sure we have a consistent
// picture. If we keep the interrupts disabled for too long, we might miss AD samples.
// We might copy data out in smaller chunks instead, enabling interrupts inbetween,
// but then we could in principle get some inconsistency in data (different parts of the traces mixed)
// if the AD interrupts are coming fast, then we might as well (?) use the values inline in the ring buffer.
// If we make the inner ring buffers to hold an integer number of FLASH_PROCESS_NSCANS,
// we should never have to call more than one memcpy(), i.e. data should be contiguous,
// which should make things even faster.
// The current code is based on the belief that copying the memory is faster than
// processing and flashing it.
//
// In addition to disabling and enabling of IRQs in certain portions of the function,
// the function has a lock ("already_running") to prevent re-entrancy of the function (from an ISR),
// since this might create havoc in e.g. the FLASH storage.
// Update 20140910:
// This lock is only there if ALLOW_CALL_PROCESS_RINGBUF_ENTRIES is defined,
// otherwise it would be OK but unneccessary.


int process_ringbuf_entries()
{
#ifdef ALLOW_CALL_PROCESS_RINGBUF_ENTRIES
	static bool already_running = false;

    INT_Disable();
    if (already_running) {
    	INT_Enable();
    	return -1;
    }
    else {
    	already_running = true;
    	INT_Enable();
    }
#endif

	scan_t buf[PROCESS_NSCANS];
	unsigned int num_scans, rb_istart, rb_num, rrb_istart;
	bool rb_locked;
	bool check_for_more_data;
	scan_ringbuf_t *rb;
	do {
		// Loop iterations are independent, no problem re-entering (which would be from an ISR) between loop iterations.
		// Doesn't help us much.
		INT_Disable();
		if (gl_rrb.num <= 0) {
			// No traceset available
			INT_Enable();
			break;
		}
		rb = &gl_rrb.buf[gl_rrb.istart % FLASH_OUTER_BUFSIZE];
		/* A bit messy code: I guess rb->istart and gl_rrb.istart are only
		 * changed by this code, so maybe not necessary to copy into
		 * temporary variables, and in the following code maybe we used
		 * a bit of each (temporary variables/pointers)?
		 * To avoid making changes I keep the code as it is.
		 */
		rb_istart = rb->istart;
		rb_num = rb->num;
		rrb_istart = gl_rrb.istart;
		rb_locked = rb->locked;
		if (rb_num >= PROCESS_NSCANS)
		    num_scans = PROCESS_NSCANS;
		else if (rb_locked)
			num_scans = rb_num; // New traces have already been started, finish current.
		else
			num_scans = 0; // Nothing or not enough available

		// Safe to enable interrupts again before memcpy(), since we made the flash_ringbuf_add*() functions
		// always back off if buffers are full (and couldn't be emptied).
		// If we enable before memcpy, we allow AD samples to be gathered even while memcpying,
		// decreasing the chance of loosing a sample if the code including memcpy takes longer than 1/AD sample frequency. On the other side,
		// outer or inner buffer might get full while memcpying,
		// so we might loose AD samples or even whole trace sets.
		// So not clear which is best.
		// With large ring buffers and slow memcpy, enabling before memcpy is maybe best,
		// with smaller buffers and fast memcpy it may be opposite.
        INT_Enable();

        //PRINTF("!!!!!!!!!!num_scans=%u, rb_locked=%u, rb_num=%u, gl_rrb.num=%u\n", num_scans, rb_locked, rb_num, gl_rrb.num);
        if (num_scans > 0) {
			const unsigned int i = rb->istart % FLASH_INNER_BUFSIZE;
			// Might need two memcpys.
			// If bufsize is an integer number of process lengths the second memcpy will never be run.
			const int num_scans_1 = EFM32_MIN(num_scans, FLASH_INNER_BUFSIZE-i);
			memcpy(buf, rb->buf + i, num_scans_1 * sizeof(rb->buf[0]));
			if (num_scans_1 < num_scans)
				memcpy(buf + num_scans_1, rb->buf + 0, (num_scans - num_scans_1) * sizeof(rb->buf[0]));

			//// Process and if necessary send messages

#if 0
            /* gcc gives warning for the following line, as well as in below call to process_data() (etc?).
             * Why? The code (process_data, shift calculation) seems to work all right,
             * even though the debugger does not know how to show the data properly.
             */
            volatile uint16_t (*test)[4] = buf;
            volatile int l = sizeof(*test);
#endif

			// Annoying to declare local variables and then having to copy to fields
			// inside <warning>, but pointers to fields in a *packed* structure
			// might be dangerous. In fact, CORTEX M30 (and 40 I guess) "allows misaligned access by default",
			// and has worked well using direct pointers to members, but slightly nasty to depend on that.
			unsigned char reached_limit; // bit flags
            float std_devs[CONFIG_AD_NCHANS];

            sample_t maxs[CONFIG_AD_NCHANS];
            sample_t mins[CONFIG_AD_NCHANS];
            uint16_t maxs_samples[CONFIG_AD_NCHANS];
            uint16_t mins_samples[CONFIG_AD_NCHANS];

            float sample_means[CONFIG_AD_NCHANS];

            //float shifts[CONFIG_AD_NCHANS*(CONFIG_AD_NCHANS-1)/2];

            avalanche_warning_t warning;
            const int ret = process_data(buf, num_scans, &reached_limit, std_devs, maxs, mins, maxs_samples, mins_samples, sample_means);

            if (ret == 0 && reached_limit != 0) {
                // Send message.
                warning.reached_limit = reached_limit;
                memcpy(warning.std_devs, std_devs, sizeof(std_devs));

                memcpy(warning.max, maxs, sizeof(maxs));
                memcpy(warning.min, mins, sizeof(mins));
                memcpy(warning.max_samples, maxs_samples, sizeof(maxs_samples));
                memcpy(warning.min_samples, mins_samples, sizeof(mins_samples));

                //memcpy(warning.mean, sample_means, sizeof(sample_means));

               // memcpy(warning.shifts, shifts, sizeof(shifts));

                const scan_ringbuf_t *rb = &gl_rrb.buf[gl_rrb.istart% FLASH_OUTER_BUFSIZE]; // Traceset we are processing
                warning.timestamp = TIMESTAMP_ADD_SECS(rb->start_time, rb->istart/gl_ad_scan_rate); // Timestamp of first sample in interval

                // Setting gl_dont_wait_for_acks is done in main() when start_logging,
                // and reset afterwards, but maybe a chance that we may get here before/after.
                const bool prev_gl_dont_wait_for_acks = gl_dont_wait_for_acks;
                gl_dont_wait_for_acks = true; // Cannot afford to wait for very long, so we don't wait at all.

                if (gl_socket_id >= 0)											//satt inn for og redusere trafikk mellom efm32 og dust when mote lost
                	dust_tx_msg_data(OMSG_AVALANCHE, &warning, sizeof(warning));


                gl_dont_wait_for_acks = prev_gl_dont_wait_for_acks;
            }

			// Write to NAND FLASH.
			if (flash_block_to_use() >= 0) {

				memset(buf + num_scans, 0xff, sizeof(buf) - num_scans * sizeof(scan_t)); // 0xff is default value, maybe best for flash.
				const int npages = ceil((float)num_scans / FLASH_PAGE_NSCANS);

				for (int ipage = 0; ipage < npages; ++ipage) {
					const unsigned int nscans_since_traceset_start = rb_istart + ipage * FLASH_PAGE_NSCANS; // at start of page
					const unsigned int traceset_id = rrb_istart % 65536;
					gl_latest_flash_id = traceset_id;
					const unsigned int block_seq_num_in_traceset = nscans_since_traceset_start/FLASH_BLOCK_EFFECTIVE_NSCANS; // Integer division intentional
					const bool start_of_new_block = nscans_since_traceset_start % FLASH_BLOCK_EFFECTIVE_NSCANS == 0;
					const bool end_of_traceset = rb_locked && rb_num == num_scans && ipage == npages-1;

					PRINTF("Writing page %d, block %d, ID %d, seq_num %d\n", gl_next_page_in_block_to_use, flash_block_to_use(), traceset_id, block_seq_num_in_traceset);
					if (flash_safe_write_page(gl_next_page_in_block_to_use++,
							                  buf + ipage * FLASH_PAGE_NSCANS, start_of_new_block)
						< 0)
						break;

					PRINTF("If %d == %d\n", nscans_since_traceset_start % FLASH_BLOCK_EFFECTIVE_NSCANS, FLASH_BLOCK_EFFECTIVE_NSCANS - FLASH_PAGE_NSCANS);
					const bool last_page_in_block_just_written = nscans_since_traceset_start % FLASH_BLOCK_EFFECTIVE_NSCANS == FLASH_BLOCK_EFFECTIVE_NSCANS - FLASH_PAGE_NSCANS;
					if (end_of_traceset
						|| rb->timestamp_is_correct
						   && last_page_in_block_just_written) {
						// At end of traceset, or finished with current block and with correct timestamp,
						// so we have to write page 0 (header) of the current block and any
						// previous blocks we delayed writing the header
						// for because correct time was not yet available.
						// We only need to possibly write earlier headers once
						// but easiest to write code so that it checks every time instead of having yet another flag to keep track of.
						// Not much extra CPU.

						bool break_ipage_loop = false;
						const int iblock_current = flash_block_to_use(); // Signed to avoid underflow problems
						int seq_num = block_seq_num_in_traceset; // Will be decreased for every non-bad block.
						for (int nsteps_back = 0; nsteps_back < FLASH_NBLOCKS; ++nsteps_back) { // Each step is one block
							int iblock = iblock_current - nsteps_back;
							if (iblock < 0)
								iblock += FLASH_NBLOCKS;

							if (gl_block_info[iblock].status == BLOCK_BAD)
								continue; // Skip over. Will never happen for nsteps_back == 0

							if (gl_block_info[iblock].status != BLOCK_ACTIVE // I guess we only need this line, the additional checks below is if I have overlooked something
								// Can no longer use gl_block_info[...].id when .status != BLOCK_USED: || gl_block_info[iblock].id != traceset_id
								|| seq_num < 0)
								break;

							flash_block_header_t flash_block_header =
								{ FLASH_MAGIC_NUMBER,
								  traceset_id,
								  seq_num,
								  (nsteps_back == 0)
								      ? (nscans_since_traceset_start + EFM32_MIN(num_scans - ipage*FLASH_PAGE_NSCANS, FLASH_PAGE_NSCANS) - block_seq_num_in_traceset * FLASH_BLOCK_EFFECTIVE_NSCANS)
								      : FLASH_BLOCK_EFFECTIVE_NSCANS,
                                  TIMESTAMP_ADD_SECS(rb->start_time, ((double)seq_num*FLASH_BLOCK_EFFECTIVE_NSCANS)/gl_ad_scan_rate),
                                  rb->start_time,
                                  rb->timestamp_is_correct
								};
							// Messy, and code worked well without it, but we want to make sure the flash code
							// do not access memory locations outside of known variables (undefined memory contents (in padding) is OK).
							// On some architectures this really could be important.
							struct {
							    flash_block_header_t header;
							    unsigned char padding[FLASH_PAGE_SIZ - sizeof(flash_block_header_t)];
							} flash_block_header_buf = { flash_block_header };

							if (nsteps_back == 0) {
								// For the latest block we can use flash_safe_write_page() which provides
								// protection against FLASH errors when used correctly.
								PRINTF("Writing header page %d, block %d, ID %d, seq_num %d\n", 0, flash_block_to_use(), traceset_id, seq_num);
								if (flash_safe_write_page(0, &flash_block_header_buf, false) < 0) { // Write block header in page 0 of block.
									break_ipage_loop = true;  // The FLASH is all bad, no need to continue writing
									break;
								}
							}
							else {
								// For earlier block headers we cannot use flash_safe_write_page() (unless we make new code that moves
								// blocks at higher addresses).
								// So we try to write, and if the write fails the block is marked as bad,
								// and we lose the data.
								// The only situation this happens is when all *data* pages in a block have already been successfully written,
								// but the *header* page is attempted written delayed because of missing timestamp, and that write fails.
								// We only lose that block of data.
								PRINTF("Writing unsafely header page %d, block %d, ID %d, seq_num %d\n", 0, iblock, traceset_id, seq_num);
								flash_write_page(iblock, 0, &flash_block_header_buf);
							}

							--seq_num;
						} // for nsteps_back

						if (break_ipage_loop)
							break;


					} // if write header(s)

					if (end_of_traceset || last_page_in_block_just_written) {
						gl_next_page_in_block_to_use = 1; // Skip header page of new block for now
						flash_find_next_good_block();
					}

				} // for ipage
			}
	    }

        // Update ring buffers.
        // All access to gl_rrb./rb->num needs IRQ disabling.
		rb->istart += num_scans;
		INT_Disable();
		rb->num -= num_scans;
		check_for_more_data =
		    rb->num >= PROCESS_NSCANS // Plenty of data in current traceset
			|| rb->num > 0 && rb->locked // Some data left in current traceset, and it is locked so no point in waiting for more
			|| rb->num == 0 && gl_rrb.num > 1; // Finished with current traceset (both reading from and writing to)
		                                       // but at least one more traceset is available that might have data.
		                                       // We don't check whether it actual has data here, rather in the next loop iteration, but we could have.
		                                       // The next lines will advance to next traceset since gl_rrb.num > 1 => rb->locked
		if (rb->locked && rb->num == 0) {
			++gl_rrb.istart;
			--gl_rrb.num;
		}
		INT_Enable();

	} while (check_for_more_data);

#ifdef ALLOW_CALL_PROCESS_RINGBUF_ENTRIES
	INT_Disable();
   	already_running = false;
  	INT_Enable();
#endif
  	return 0;
}


// Locks current traceset (inner ring buffer), if any.
// Intended to be called from ISR (no disabling of interrupt), though should
// probably be safe anyway since we just set a bool.
void process_ringbuf_lock_traceset()
{
	if (gl_rrb.num > 0)
		gl_rrb.buf[(gl_rrb.istart + gl_rrb.num - 1) % FLASH_OUTER_BUFSIZE].locked = true;
}


// Intended to be called from an ISR (unprotected read/writes to ring buffer).
// Add another element (scan) in the most recent inner ringbuf, overwriting the oldest if any.
// Before overwriting the oldest element, flash_handle_ringbuf_entries() is called
// to handle all unprocessed data.This should be a very rare situation, since
// this function is intended to be called from an (ADC0) ISR, and spending
// time might cause lost packets.
// Anyway, this situation should in practice hopefully never happen.
// UPDATE 20140910: Skipping call to process_ringbuf_entries().
// If we allow the call, code that reads the FLASH or gl_flash_block_info will have to disable interrupts
// while reading, which means thay cannot e. g. communicate with a standard mote (receive ack).
// This is not acceptable for e g process_send_data().
void process_ringbuf_add_scan(scan_t scan)
{
	scan_ringbuf_t *rb = &gl_rrb.buf[(gl_rrb.istart + gl_rrb.num - 1) % FLASH_OUTER_BUFSIZE];
	if (rb->locked)
		return;
	if (rb->num >= FLASH_INNER_BUFSIZE) // Should never be >
#ifdef ALLOW_CALL_PROCESS_RINGBUF_ENTRIES
	    if (process_ringbuf_entries() < 0)
#endif
			return; // The buffer is full [#ifdef: AND flash_handle_ringbuf_entries is already running] (chances close to 0 I guess). Best we can do is to throw away the scan.

	const unsigned int istop = (rb->istart + rb->num++) % FLASH_INNER_BUFSIZE;
	memcpy(rb->buf[istop], scan, sizeof(scan_t));
}


// Intended to be called from an ISR (unprotected read/writes to ring buffer).
// Add another element in the outer ringbuf, overwriting the oldest if any.
// Before overwriting the oldest element, process_ringbuf_entries() is called
// to handle all unprocessed data.This should be a very rare situation, since
// this function is intended to be called from an (ADC0) ISR, and spending
// time might cause lost packets.
// Please make sure that the outer ring buffer is big enough that it hardly ever gets
// full!
// UPDATE 20140910: Skipping call to process_ringbuf_entries().
// See explanation in comment above process_ringbuf_add_scan().
void process_ringbuf_add()
{
	process_ringbuf_lock_traceset(); // No more samples allowed in "old" traces (if any).

	if (gl_rrb.num >= FLASH_OUTER_BUFSIZE) // Should never be >
#ifdef ALLOW_CALL_PROCESS_RINGBUF_ENTRIES
	    if (process_ringbuf_entries() < 0)
#endif
			// The buffer is full [#ifdef: AND flash_handle_ringbuf_entries is already running] (chances close to 0 I guess).
			// The best we can do is throw away the new set of traces (which will happen because we locked
			// the old set of traces above).
			return;

	scan_ringbuf_t *rb = &gl_rrb.buf[(gl_rrb.istart + gl_rrb.num++) % FLASH_OUTER_BUFSIZE];
	rb->num = rb->istart = 0; // No contents
	rb->locked = false;
	// Fill rb->start_time with default value, in case we don't receive a time notification.
	// At least we hope to get it in the correct order.

	if (rb->num > 1) {
		scan_ringbuf_t *rb_prev = &gl_rrb.buf[(gl_rrb.istart + gl_rrb.num - 2) % FLASH_OUTER_BUFSIZE];
		rb->start_time = rb_prev->start_time;
		++rb->start_time.secs; // Add one second
    }
	else {
		rb->start_time.secs = 0;
		rb->start_time.microsecs = 0;
	}
	rb->timestamp_is_correct = false;

	// We later want to fill rb->start_time with the time stamp of
	// the first time notification packet with incoming packet sequence number
	// >= rb->start_packet_ind. We will then set time_is_correct = true
	rb->start_packet_ind = packet_seq_num_end();

	gl_rrb.log_until = gl_adjustable_params->min_nscans_after_uart;
}


// Apply timestamp from mote to most likely traceset, if any.
// Small job, probably OK to disable interrupts for more or less the whole function.
// The first attempt found the oldest traceset ("COMP-trigg") that had not had its
// time properly set and a start_packet_ind <= packet_ind, but imagine if we lost
// a time-packet (e.g. overwritten in our packet ring-buffer). In that case we might
// find ourselves in a situation where all future tracesets were timestamped with
// the time for the next later traceset? Actually probably no, since the older tracesets
// would be processed, written to FLASH and removed from our ring-buffer, but if we
// got a contiguous set of tracesets, the situation could persist for a while.
// So instead we now find the traceset, if any, which has start_packet_ind <= packet_ind,
// has not had the timestamp set properly before, and has the *lowest* (positive) difference
// between packet_ind and start_packet_ind.
void process_ringbuf_apply_timestamp(const seq_num_t packet_ind, timestamp_t timestamp)
{
	INT_Disable();

	unsigned int min_diff_ipacket, min_cnt;
	scan_ringbuf_t *min_rb = 0;
	for (unsigned int cnt = 0; cnt < gl_rrb.num; ++cnt) {
		const unsigned int ind = (gl_rrb.istart + cnt) % FLASH_OUTER_BUFSIZE;
		scan_ringbuf_t *const rb = &gl_rrb.buf[ind];
		const int diff_ipacket = packet_ind - rb->start_packet_ind;
	    if (!rb->timestamp_is_correct && diff_ipacket >= 0 && (diff_ipacket < min_diff_ipacket || cnt == 0)) {
	    	min_rb = rb;
	    	min_diff_ipacket = diff_ipacket;
	    	min_cnt = cnt;
	    }
	}
	//min_rb = 0; // OBS TEST!
	if (min_rb != 0) {
	    min_rb->start_time = timestamp;
	    min_rb->timestamp_is_correct = true;
	    // Iterate trough the following tracesets until the next confirmed timestamp or end,
	    // and update non-confirmed timestamps just so they are in order.
	    // Not sure if we need it. In theory timestamps might still be out of order,
	    // if earlier tracesets have been triggered with less than one second distance to previous start
	    // (but this we probably don't allow, a traceset will probably always be > 1 second long),
	    // or if time notifications arrive in wrong order (would be a bug in mote if that happened, I guess).
		for (unsigned int cnt = min_cnt+1; cnt < gl_rrb.num; ++cnt) {
			const unsigned int ind = (gl_rrb.istart + cnt) % FLASH_OUTER_BUFSIZE;
			scan_ringbuf_t *const rb = &gl_rrb.buf[ind];
			if (rb->timestamp_is_correct) {
				break;
			}
			else {
				timestamp = TIMESTAMP_ADD_SECS(timestamp, 1); // Add one second
				rb->start_time = timestamp;
			}
		}
	}

	gl_last_timestamp = timestamp;
	INT_Enable();
}


uint16_t process_latest_flash_id()
{
    return gl_latest_flash_id;
}

timestamp_t process_latest_flash_timestamp()
{
    return gl_last_timestamp;
}

event_t process_latest_flash_event()
{

	event_t lastevent;

	const uint32_t *const trig_levels_temp = gl_adjustable_params->comp_trig_levels;

	float kiloseconds = 1000 * (gl_timestamp.secs - gl_last_timestamp.secs) + (gl_timestamp.microsecs - gl_last_timestamp.microsecs)/1000;
	float samplingrate = (1000 * (gl_num_samples + trig_levels_temp[1])) / kiloseconds;
	float samplingrate100 = 100 * samplingrate;

	lastevent.latest_flash_id = gl_latest_flash_id;
	lastevent.last_timestamp = gl_last_timestamp;
	lastevent.sampling_rate = samplingrate100;
	lastevent.num_samples = gl_num_samples;

	return lastevent;
}


/*
 * Send list of available "runs" (contiguous scans/traces, except may have missing data due to FLASH errors or full FLASH, basically one per COMP trig),
 * to controller PC.
 * We traverse gl_block_info twice, first just to count number of distinct runs, the second time we send them.
 * One packet is sent per run.
 * Does not access FLASH, only gl_block_info cached values.
 * Uses functions from dust_com, does it belong in this file? As good as any other existing file I guess.
*/
void process_list_data()
{
EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
// More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
    typedef struct __attribute__ ((__packed__)) {
        uint16_t    id; // "Temporary ID", will be unique for many passes through FLASH.
        uint16_t    run_ind; // 0...run_cnt-1
        uint16_t    run_cnt; // Number of runs available
        uint16_t    run_nblocks; // Number of blocks available in run, not including holes/missing start/end
        timestamp_t run_start_time; // Start time of run, i. e. the timestamp the block that has seq_num 0 has (or would have if it is missing).
        unsigned char timestamp_is_correct; // Non-zero iff run_start_time was set using a mote time packet.
    } data_list_item_t;
EFM32_PACK_END(); // Actually a no-op for GNU it seems

    const int iblock_current = flash_block_to_use();
    int id_cnt;
    if (iblock_current < 0)
        return; // No good blocks in FLASH

    //dust_enable_backup_radio_if_necessary();
    for (int final = 0; final <= 1; ++final) {
        int iblock_next, iblock_last;
        int id_ind = 0;
        for (unsigned int iblock = iblock_current; ;) {

            const block_info_t *const bl = gl_block_info + iblock;
            if (bl->status == BLOCK_USED) {
                // At this points bl/iblock "points" to the oldest block in a "run" (same id).
                // Normally this will have seq_num 0, but may not have if the initial blocks
                // have been overwritten (full FLASH).
                // The following do-while sets iblock_last to "point" to the last
                // block in the run, iblock_next to a suitable place to start searching for a new run
                // (>= iblock_last in modulo fashion) or alternatively at the starting point (iblock_current),
                // and nscans to the sum of scans in the run (which have not been overwritten).
                // Note that within [iblock, iblock_last] there may be blocks with STATUS BLOCK_BAD
                // which do not have usable content, but which does not necessarily terminate the run.
                // Any other status will terminate the run (BLOCK_ACTIVE suggests that more data will soon be available, though).
                iblock_next = iblock_last = iblock;
                unsigned int nblocks = 1;
                do {
                    iblock_next = (iblock_next + 1) % FLASH_NBLOCKS;
                    block_info_t *bl_next = gl_block_info + iblock_next;
                    if (bl_next->status != BLOCK_BAD && (bl_next->status != BLOCK_USED || bl_next->id != bl->id) || iblock_next == iblock_current)
                        break;
                    if (bl_next->status == BLOCK_USED) {
                        iblock_last = iblock_next;
                        ++nblocks;
                    }
                } while (true);

                if (final) {
                    data_list_item_t run_info = {
                        bl->id,
                        id_ind,
                        id_cnt,
                        nblocks,
                        bl->timestamp_initial,
                        bl->timestamp_is_correct
                    };


                    if (logging_is_running()) {
                        //dust_disable_backup_radio_if_necessary();
                        return; // Return early if we got a comparator trig and started logging.
                    }

                    if (gl_socket_id < 0) {
                        //dust_disable_backup_radio_if_necessary();
                        return; // Return early if mote lost.
                    }

                    dust_tx_msg_data_txDone(OMSG_DATALIST, &run_info, sizeof(run_info), 0);

                    WDOG_Feed(); // Messy, but could be necessary to put somewhere here, this seems like an OK place.

                }
                ++id_ind;
                iblock = iblock_next;
            }
            else {
                iblock = (iblock + 1) % FLASH_NBLOCKS;
            }
            if (iblock == iblock_current)
                break; // Transversed whole "ring buffer"
        }

        id_cnt = id_ind;
    }
    //dust_disable_backup_radio_if_necessary();
}


// Helper function for process_send_data().
unsigned char send_data(const unsigned int iscan_start, const unsigned int iscan_end,
               data_desc_t * const data_desc,
               const unsigned int nchans, sample_t *data)
{
    data_desc->iscan_start = iscan_start;
    data_desc->iscan_stop = iscan_end - 1;
    const unsigned int size_data_desc = offsetof(data_desc_t, iscan_stop) + sizeof(data_desc->iscan_stop); // Make sure we don't include trailing padding, if any
    const unsigned int nbytes_data = (data != 0) ? (iscan_end - iscan_start)*nchans*sizeof(sample_t) : 0;
    unsigned char payload[size_data_desc + nbytes_data];
    memcpy(payload, data_desc, size_data_desc);
    if (data != 0)
        memcpy(payload+size_data_desc, data, nbytes_data);
    //dust_tx_msg_data(OMSG_DATA, &payload, size_data_desc + nbytes_data);
    unsigned char status = dust_tx_msg_data_txDone(OMSG_DATA, &payload, size_data_desc + nbytes_data, 0);
    return status;
}


/*
 * Send data from requested "run" (contiguous scans/traces, except may have missing data due to FLASH errors or full FLASH, basically one per COMP trig),
 * to controller PC.
 * Unlike process_list_data(), we traverse gl_block_info only once, so no total number of packets is given in each packet.
 * We access real FLASH and a fair amount of data, so seems a bit excessive
 * to traverse twice, and anyway it would be (theoretically) possible for FLASH to go bad
 * between traverses.
 * Data is sent chronologically in packet(s), each packet with first and last
 * scan index of the packet. If the packet scan indices indicate there should be data,
 * but there is none, this indicates there are missing data.
 * If e. g. all data have been requested but the FLASH misses reliable data from scan 0-999,
 * the first packet might contain start/stop ind [0 999] while containing no data
 * (there might also be two packets [0 N] [N+1 999], and so forth).
 * The whole range of requested scan-indices are reflected in the sent packets.
 * If e.g. request is [0 100], and data is only available in [10 40] U [50 90]
 * a possible stream of packets would be
 *  [0 10]{no data} [10 40]{31 scans} [41 49]{no data} [50 90]{41 scans} [91 100]{no data}.
 *
 * Every packet either contains no data or the full amount given by the indices in the packet header.
 *
 * Uses functions from dust_com, and also accesses FLASH, so I guess it belongs reasonably in this file?
*/
void process_send_data(const packet_with_meta_t *const packet_with_meta)
{

    const unsigned int page_nscans = FLASH_PAGE_SIZ / sizeof(scan_t); // Intentional integer division

    unsigned char status = 0;

    // memcpy instead of pointer cast, since alignment might matter
    data_desc_t data_request;
    memcpy(&data_request, packet_with_meta->packet + DUST_SENDTO_PREFIX_LEN + DUST_COMMAND_ID_LEN, sizeof(data_request));

    data_desc_t data_desc = data_request;

    const int iblock_current = flash_block_to_use();
    if (data_request.iscan_stop <= data_request.iscan_start || iblock_current < 0) {
        dust_tx_msg_data(OMSG_DATA, &data_desc, sizeof(data_desc)); // No data
        return; // User asked for no data, or no good blocks in FLASH
    }

    // Count number of channels requested:
    unsigned int nchans = 0;
    for (int k = 0; k < CONFIG_AD_NCHANS; ++k)
        if ((data_request.channels & 1 << k) != 0)
            ++nchans;

    const unsigned int packet_data_nbytes = DUST_API_MAX_PAYLOAD_EXPERIENCED - DUST_SENDTO_PREFIX_LEN - DUST_COMMAND_ID_LEN - sizeof(data_desc_t);
    const unsigned int packet_nscans = packet_data_nbytes / (sizeof(sample_t) * nchans); // Integer division intentional
    sample_t data[packet_nscans][nchans]; // Array big enough to hold all scans for one full packet

    flash_block_header_t block_header;
    unsigned char page[FLASH_PAGE_SIZ], block_header_page[FLASH_PAGE_SIZ];
    int iscan_page_start = data_request.iscan_start,
        iscan_page_end = data_request.iscan_start; // No data in page. Start value of iscan_page_start might not matter.
    int iscan_handled_end = data_request.iscan_start; // Handled scans, either copied to <data> or sent, including "holes". No scans handled so far, i. e. [iscan_start, iscan_start)
    int data_end = 0; // Number of scans in <data>. Nothing, i. e. data[[0, 0), :] filled
    int iblock; // The block we are currently using. Not to be confused with iblock_current, which is where the program will write next.

    // Try to find start of requested run data
    bool first_round = true;
    for (iblock = iblock_current;
         first_round || iblock != iblock_current;
         iblock = (iblock + 1) % FLASH_NBLOCKS) {
        const block_info_t *const bl = gl_block_info + iblock;
        if (bl->status == BLOCK_USED && bl->id == data_request.id)
            // Found first and only run in FLASH with requested ID.
            break;
        first_round = false;

    }
    if (iblock == iblock_current && !first_round) {
        // Run not found
        dust_tx_msg_data(OMSG_DATA, &data_desc, sizeof(data_desc)); // No data
        return;
    }

    // At this point, iblock is the index of the first available block of the run.
    // Set things up for proper initialization:
    --iblock; // Will be increased again "immediately"
    bool is_initialization = true;
    unsigned int block_ndatapages_used = 0;
    unsigned int ipage_in_block = block_ndatapages_used + 1; // Read block header before reading anything other

    //dust_enable_backup_radio_if_necessary();
    while (true) {

        ///////////////////////////////////////////////
        // Get data from FLASH if needed
        bool confirmed_nothing_available = false;
        if (iscan_handled_end >= iscan_page_end) { // > should not happen
            // No more data available in read page (if any), read data from FLASH
            assert(iscan_handled_end == iscan_page_end);

            bool got_data = false;
            while (true) {
                if (ipage_in_block <= block_ndatapages_used) { // If we used 5 pages, they are page 1-5 since page 0 is the block header
                    if (NANDFLASH_ReadPage(FLASH_PAGE_ADDR(iblock, ipage_in_block), page) == NANDFLASH_STATUS_OK) {
                        const int iscan_block_start = block_header.seq_num * FLASH_BLOCK_EFFECTIVE_NSCANS;
                        iscan_page_start = iscan_block_start + (ipage_in_block-1)*page_nscans;
                        iscan_page_end = EFM32_MIN(EFM32_MIN(iscan_block_start + block_header.num_scans, iscan_page_start + page_nscans),
                                                   data_request.iscan_stop+1);
                        ++ipage_in_block;
                        got_data = true;
                        break;
                    }
                    ++ipage_in_block;
                }
                if (ipage_in_block == block_ndatapages_used + 1) {  // hvis iscan_start > 64 så begynner denne algoritmen og returnere feil verdier. Regner med at dette henger sammen med at de første dataenen da ikke er fra iblock = 1
                    // Find and read next acceptable block header, if any.
                    bool read_ok;
                    while (true)  {
                        iblock = (iblock + 1) % FLASH_NBLOCKS;
                        const block_info_t *bl;
                        bl = gl_block_info + iblock;
                        read_ok = false;
                        if (!is_initialization && iblock == iblock_current
                            || (bl->status != BLOCK_BAD
                                && (bl->status != BLOCK_USED
                                    || bl->id != data_request.id
                                    || (read_ok = NANDFLASH_ReadPage(FLASH_PAGE_ADDR(iblock, 0), block_header_page) == NANDFLASH_STATUS_OK))))
                            break;
                        is_initialization = false;
                    };
                    is_initialization = false;
                    confirmed_nothing_available = !read_ok;
                    if (confirmed_nothing_available)
                        break;
                    memcpy(&block_header, block_header_page, sizeof(block_header));
                    block_ndatapages_used = ceil((float)block_header.num_scans/page_nscans);
                    ipage_in_block = 1;
                }
            }
            assert(got_data ^ confirmed_nothing_available); // got_data xor nothing available
        }

        ///////////////////////////////////////////////
        // Handle a few things we want to do:
        WDOG_Feed(); // Messy, but could be necessary to put somewhere here, this seems like an OK place.

        if (logging_is_running())
            break; // Break early if we got a comparator trig and started logging.

        if (gl_socket_id < 0) //Tester automatisk terminereing av overføring ved mote lost
        	break;

        //if (status != 0) //Tester automatisk terminereing av overføring ved not txDone
        //	break;

        ///////////////////////////////////////////////
        // Copy data from page to data, send messages as needed.

        // Handle missing data:
        const unsigned int iscan_after_any_missing =
                confirmed_nothing_available
                ? data_request.iscan_stop+1
                : EFM32_MIN(data_request.iscan_stop+1, iscan_page_start);
        const bool hole_or_similar = iscan_after_any_missing > iscan_handled_end;
        const bool packet_full = data_end == packet_nscans;
        if (hole_or_similar || packet_full) {
            // Missing data (hole or before start/until end) or packet full, we have to send the existing data, if any,
            // then if necessary send a packet documenting any missing data.
            if (data_end > 0) {
            	status = send_data(iscan_handled_end - data_end, iscan_handled_end, &data_desc, nchans, &data[0][0]);

                data_end = 0;
            }
            if (hole_or_similar) {
            	status = send_data(iscan_handled_end, iscan_after_any_missing, &data_desc, nchans, 0);
                iscan_handled_end = iscan_after_any_missing;
            }
        }

        if (!confirmed_nothing_available) {
            // Following code to copy data uses memcpy() on each sample_t! This allows for varying size of sample_t.
            // The code also supports selection of subset of channels.
            // The code should be efficient enough. memcpy() might anyway be inlined.
            const int nscans_to_copy = EFM32_MIN(packet_nscans - data_end, iscan_page_end - iscan_handled_end);
            for (int isc = 0; isc < nscans_to_copy; ++isc) {
                int ich = 0;
                for (int ic = 0; ic < CONFIG_AD_NCHANS; ++ic)
                    if ((data_request.channels & 1 << ic) != 0)
                        memcpy(&data[data_end+isc][ich++],
                               page + (iscan_handled_end-iscan_page_start+isc)*sizeof(scan_t)+ic*sizeof(sample_t),
                               sizeof(sample_t));
            }
            //memcpy(data + data_end, page + iscan_handled_end - iscan_page_start, nscans_to_copy);
            data_end += nscans_to_copy;
            iscan_handled_end += nscans_to_copy;
        }

        if (iscan_handled_end == data_request.iscan_stop + 1) {
            if (data_end > 0)
            	status = send_data(iscan_handled_end - data_end, iscan_handled_end, &data_desc, nchans, &data[0][0]);
            break; // Finished!
        }
    };
    //dust_disable_backup_radio_if_necessary();

}


// Run before using other functionality.
void process_config()
{
	// Sanity check(s).
	assert(FLASH_PAGE_SIZ % sizeof(scan_t) == 0);
	assert(PROCESS_NSCANS % FLASH_PAGE_NSCANS == 0);

    //gl_traceset_id_to_use =
	gl_rrb.istart = flash_init()+1; // So that gl_rrb.istart can be used as an ID
}



