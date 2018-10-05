#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "em_common.h"
#include "ngi_rtcdrv.h"
#include "debug_printf.h"
#include "packet.h"




// gl_packet_buf will (at some point, before they are overwritten)
// contain all incoming packets that are not detected as retransmits of an OK packet, and have OK CRC, start/end HDLC byte (etc?).
// In addition it may contain retransmits of packets with the sync flag set.
volatile static struct packet_buf_t {
  volatile seq_num_t seq_num_total; // Number of packets inserted
  packet_with_meta_t packets[PACKET_BUF_SIZE];
} gl_packet_buf = { 0 };

// Updated by packet_non_ack_sent() (e.g. called by dust_send_request() when sending a non-ack command).
// Used for default values to calls to packet_search_for(_f) and packet_wait_for(_f).
volatile static seq_num_t gl_packet_seq_num_at_last_sent_command = 0;
volatile static packet_t gl_packet_last_sent_command;


seq_num_t packet_get_seq_num_at_last_sent_command()
{
	return gl_packet_seq_num_at_last_sent_command;
}


const unsigned char *packet_get_last_sent_command()
{
	return &gl_packet_last_sent_command[0];
}


void packet_non_ack_sent(packet_t last_sent_command, unsigned int packet_length)
{
	gl_packet_seq_num_at_last_sent_command = gl_packet_buf.seq_num_total;
	memcpy(gl_packet_last_sent_command, last_sent_command, packet_length);
}


unsigned int packet_ind(const seq_num_t seq_num)
{
	return seq_num  % ARRAY_LEN(gl_packet_buf.packets);
}


volatile struct packet_with_meta_t *packet_ptr(const seq_num_t seq_num)
{
	return &gl_packet_buf.packets[packet_ind(seq_num)];
}


// This function should only be called from code locations so that the caller can guarantee
// that it will not be interrupted in a way that causes the interrupt routine to re-enter the
// function or try to use the contents of the ring buffer.
// The typical setup is that this function is called from a single interrupt handler (e. g. GPIO_ODD_IRQHandler()),
// and packets only read from non-interrupt routines or from interrupt routines of lower priority.
// UPDATE: If needed, we could disable interrupts in the function body, then it could be called
// from anywhere I guess. I didn't know about INT_Enable/Disable when I wrote this.
// Would that incur a delay?
void packet_register(const unsigned char packet[], const int length)
{
  volatile struct packet_with_meta_t * const packp = packet_ptr(gl_packet_buf.seq_num_total);

  // Something similar to my idea from previous job.
  // Not necessary here to be safe for multiple processes/threads,
  // only for the possiblity of this function being called in an interrupt
  packp->seq_num = gl_packet_buf.seq_num_total++;
  (void)memcpy(packp->packet, packet, length);
  packp->length = length;
  PRINTF("Packet registered as #%d: [%d,%d,%d]\n", (int)(gl_packet_buf.seq_num_total-1), packet[0], packet[1], packet[2]);
}


// Returns the next seq_num to be used in the ring buffer == number of packets so far added to the ring buffer.
// Intended to be safe for use by non-interrupt -handles and interrupt-handlers alike.
seq_num_t packet_seq_num_end()
{
	// gl_packet_buf and thus all its members are volatile (C99 6.5.2.3.7), though maybe not atomic
	// (in fact on the current platform unlikely to be atomic).
	// Thus in the two following assignments the compiler should not buffer the value, but interrupt(s) might occur in any or both.

	// How safe is this?
	// If no interrupt occurs => OK, both are correct, loop stops
	// Interrupt with update only between assignments => OK, loop continues
	// Interrupt inside one assignment only and possibly between assigments=> OK, at least one assignment is OK so will loop or possibly leave loop with correct values
	// Interrupts in both assignments and possibly between assignments => Almost certainly the least-significant atomic bit will be different => OK, loop continues
	//
	// In the last case, assume that 32-bit ints are atomic (which a little snooping suggests on the current platform):
	// If the most significant part is read first in each assignment, and the first
	// assignment has a least significant int of 0, and you have exactly 2^32-1 interrupts
	// after the first assignmnent until the most significant int in the second assignment,
	// then another one just before the least significant int in the second assignment
	// => Hmm, actually OK if we want to return the first assignment.
	// Well, maybe there are some strange situations like this which are not safe, not sure.
	// UPDATE: If needed, we could disable interrupts in the function body. I didn't know about INT_Enable/Disable when I wrote this.
	// Would that incur a delay?
	seq_num_t seq1, seq2;
	do {
		seq1 = gl_packet_buf.seq_num_total;
		seq2 = gl_packet_buf.seq_num_total;
	} while (seq1 != seq2);
	return seq1;
}


int packet_buf_start_ind()
{
	const int N = ARRAY_LEN(gl_packet_buf.packets);
	seq_num_t seq_num = packet_seq_num_end();
	return (seq_num > N)
			 ? packet_ind(seq_num+1)
			 : 0;
}


// Returns the seq_num of the oldest packet in the ring buffer (0 is returned also if the buffer is empty).
seq_num_t packet_seq_num_start()
{
	const seq_num_t seq_num = packet_seq_num_end();
    if (seq_num < ARRAY_LEN(gl_packet_buf.packets))
    	return 0;
    return seq_num - ARRAY_LEN(gl_packet_buf.packets);
}




int packet_get(const seq_num_t seq_num, packet_with_meta_t *packetp)
{
    // Checking seq_num is enough when no multithreading.
	const volatile struct packet_with_meta_t * const packp = packet_ptr(seq_num);
    if (packp->seq_num != seq_num)
    	return -1;
    *packetp = *packp;
    return (packp->seq_num != seq_num) ? -1 : 0;
}


int (packet_search_f)(packet_with_meta_t *packet_out,
		            const packet_search_opts_t opts)
{
    seq_num_t seq_num_end = packet_seq_num_end();
    if (seq_num_end == 0)
    	return -1; // Empty ring buffer

    seq_num_t seq_num_start = packet_seq_num_start(); // First in ring buffer

    if (opts.return_on_missing_packet && opts.find_first && opts.min_seq_num < seq_num_start)
    	return -2;

    if (seq_num_start < opts.min_seq_num)
    	seq_num_start = opts.min_seq_num;

    int add = 1;

    if (!opts.find_first) {
    	// User wants to search from most recent and backwards
    	// Will not work if seq_num_start at this point == 0.
    	// Should be simple enough to fix if we exchange the "end"-concept with the "last"-concept,
    	// but we haven't used backward-searching as of 20140821, and I guess we never will.
        const seq_num_t tmp = seq_num_start;
        seq_num_start = seq_num_end - 1;
        seq_num_end = tmp - 1;
        add = -1;
    }

    for (seq_num_t seq_num = seq_num_start;
         opts.find_first ? seq_num < seq_num_end : seq_num > seq_num_end;
         seq_num = seq_num + add) {
    	packet_with_meta_t packet;
    	if (packet_get(seq_num, &packet) < 0 && opts.return_on_missing_packet)
            return -3; // The packet has been overwritten in the ring buffer.

    	if (   opts.command_id >= 0 && (packet.length < 1 || opts.command_id != packet.packet[0])
    		|| opts.packet_len >= 0 && (packet.length < 2 || opts.packet_len != packet.packet[1])
    		|| opts.response >= 0 && (packet.length < 3 || opts.response != (packet.packet[2] & 1))
    		|| opts.packet_id >= 0 && (packet.length < 3 || opts.packet_id != (packet.packet[2] & 2) >> 1)
    		)
    		continue; // Failed at least one requirement

    	bool not_found = false;
    	for (int k = 0; k < opts.template_len; ++k)
    		if (opts.templat[k] >= 0 && opts.templat[k] != packet.packet[3+k]) {
    			not_found = true;
    			break;
    		}
    	if (not_found)
    		continue; // template did not match

        // Found packet
    	*packet_out = packet;
        return 0;
    }

    return -4;

}



int packet_wait_for_f(packet_with_meta_t *packet_out,
		              const int max_wait_msec, // Approximate.
		              const packet_search_opts_t opts)
{
	//RTC_CounterReset();
	//RTC_Enable(true);
	//const uint32_t rtcFreq = CMU_ClockFreqGet(cmuClock_RTC); // OK X.
    static const int min_wait_ms = 1; // Wait at least e. g. 1 ms between iterations, except if remaining time is less.
    static const int max_wait_ms = 10; // Wait at most e. g. 10 ms between iterations. If a packet is received in full before EM is entered, we should not wait too long.
	bool found;
	int wait_left_msec = max_wait_msec; // Must be signed
	do {
//	  // It makes little sense to search backwards for a packet we are waiting for,
//	  // so we override any setting of opts.find_first:
//	  opts.find_first = true;
	  PRINTF("Searching for packet ([%d, %d])\n", (int)opts.min_seq_num, (int)(gl_packet_buf.seq_num_total-1));
	  found = packet_search_f(packet_out, opts) == 0;
	  PRINTF("Packet %sfound\n", found ? "" : "not ");
	  // With what used to be the following wait_up_to_burtc-call, response seemed to become
	  // a bit erratic. I am not sure why this happened.
	  // Maybe some unluck with the debugger()/breakpoints/mote_state?
	  // I was thinking it might be because th function
	  // was re-entered, but it shouldn't be.
	  // Could have been some bug in the burtc-code, I collected it from
	  // here and there. Rewritten a bit now.
	  // Changed to wait_up_to().
	  // OBS: Maybe it is not worth it, energywise/responsewise to go into sleep here?
	  // Can do a little profiling/testing.
      if (!found && wait_left_msec > 0) {
    	  const int wait_min =  EFM32_MIN(min_wait_ms, wait_left_msec);
    	  const int wait_max =  EFM32_MIN(max_wait_ms, wait_left_msec);
    	  PRINTF("Wait for [%d %d] ms\n", wait_min, wait_max);
    	  const int actually_waited = wait_from_to(wait_min, wait_max);
    	  PRINTF("Waited for %d ms\n", actually_waited);
    	  wait_left_msec = wait_left_msec - actually_waited;
     }
	} while	(!found && wait_left_msec > 0); // (max_wait_msec < 0 || RTC_CounterGet() < (uint32_t)(rtcFreq * max_wait_msec/1000)));// OK X.

	if (!found) {
		PRINTF("Final: Searching for packet ([%d, %d])\n", (int)opts.min_seq_num, (int)(gl_packet_buf.seq_num_total-1));
		found = packet_search_f(packet_out, opts) == 0;
		PRINTF("Final: Packet %sfound\n", found ? "" : "not ");
	}
	//RTC_Enable(false);

	return found ? 0 : -1;
}


// Wait for ack
// If we always followed the Serial API Guide in that we always waited for ack
// before sending a new command, we woudn't have to check for packetId I guess.
// In practice we cannot wait forever on an ack (imagine the mote booting).
// Still, maybe requiring a matching packet Id is a bit over the top?
// For the time being we don't check the packet length. Usually that would be 0,
// but not for e. g. "socketId-response".
int packet_wait_for_ack(const int max_wait_msec) // Approximate.
{
  packet_with_meta_t packet;
  const int ret =
	  packet_wait_for(&packet, max_wait_msec,
		              .response = 1, // Default, but be sure if default changes
		              .packet_id = -1, //(gl_packet_last_sent_command[2]&1) >> 1, // OBS Testing if -1, or actually it seems the mote may respond with different packetID, so -1 is perhaps best.
		              // OBS 20140903: Regarding comment for packet_id above: I am using packet_id in geophone.c without noticing any problem. Hopefully it is safe.
		              // OBS 20140910: I now see that in geophone.c I can get an ack with a different packetID! Removing it there.
		              .command_id = gl_packet_last_sent_command[0]); // OBS Testing if -1

  return (ret == 0) ? packet.packet[3] : -1;
}


