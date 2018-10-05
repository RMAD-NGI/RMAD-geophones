
#ifndef PACKET_H_
#define PACKET_H_

// packet.h/c contains functionality for maintaining a buffer of incoming communication packets from a mote.
// Only a single buffer instance is kept, this is not a C++ class implementation after all.
// I haven't added all the function declarations to this file, feel free to add more.

#define PACKET_BUF_SIZE 20 // The bigger the 'safer', except that we may run out of memory.

// In the ARM inplementation we use, 'sig_atomic_t' is 'int' as far as I can see.
#include <signal.h> // Get sig_atomic_t

// Max/fixed lengths.
// Based on info in SmartMesh_IP_mote_SerialAPI_Guide.pdf
enum {
    DUST_SENDTO_PREFIX_LEN = 22,
	DUST_COMMAND_ID_LEN = 1,// currently 8 bits command ID.
	DUST_API_PAYLOAD_HEADER = 3, // Unescaped header

	DUST_API_MAX_PAYLOAD = 125, // Unescaped payload, according to DOC. Use for e. g. incoming packet buffers.
	DUST_API_MAX_PAYLOAD_EXPERIENCED = 113, // With bigger the mote (at least sometimes) responds with "Invalid length" it seems. Use e. g. for outgoing packets.

	DUST_HDLC_MAX_PAYLOAD = DUST_API_PAYLOAD_HEADER + DUST_API_MAX_PAYLOAD, // Unescaped payload length
	DUST_HDLC_MAX = 2 + 2*(DUST_HDLC_MAX_PAYLOAD + 2) // Escaped. Two framing bytes, then up to 2*(hdlc payload + 16-bit crc)
};


#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[1])) // OK X

typedef unsigned char packet_t[DUST_HDLC_MAX_PAYLOAD];

typedef unsigned long long int seq_num_t;

typedef struct packet_with_meta_t {
	  packet_t packet;
	  int length; // includes API header
	  unsigned long long int seq_num;
} packet_with_meta_t;


typedef struct {
	seq_num_t min_seq_num; // Minimum sequence number
	bool find_first; // Search for oldest available packet (false => from most recent)
	bool return_on_missing_packet; // Set true if you want to get an error if some packet that could be the correct, is missing (overwritten)
	int command_id; // command id in API header
	int packet_len; // packet length in API header (remember that the response code in responses is not counted)
	int response; // Search for a response (false => command or notification)
	int packet_id; // packet ID in API header
	int template_len; // Length of API payload template
	int *templat; // API payload template
} packet_search_opts_t;


seq_num_t packet_get_seq_num_at_last_sent_command();


const unsigned char *packet_get_last_sent_command();


// Searches for packet in the ring buffer of incoming packets.
//The packet must have a seq_num >= min_seq_num.
// For each of response, command_id and packet_len:
//   Negative value means allow any. Non-negative value means exact match required.
// For packet_len, the value in the header is used (e.g. 0 for a one-byte response).
// If template_len > 0, templat is a template applied to the start of "API Payload",
// where a negative value means "don't care", and non-negative means exact match required.
// For responses, the API payload is considered to include the (initial) response code.
//
// Return values are 0 for found packet, negative for failure.
//
// If the ring buffer is updated through an interrupt while the function is running,
// it is unspecified whether the new packet(s) is found if it adheres to all requirements (see code).
//
// See also comment for packet_register().
//
// Technique to simulate default parameter values is inspired by http://stackoverflow.com/questions/1472138/c-default-arguments,
// search e. g. for "This is a continuation of bk.'s answer".
// 'Crazy' that C99 doesn't have default values already, but this technique simulates them splendidly.
// We keep a different function name from the macro so that we may send an option struct directly to the function if we want.
//
// 20140821: A bug if searching backwards and want to search until the start of
// buffer, I guess. Currently we do not use backward search. See comment in packet_search_f().
#define packet_search(packet_out, ...) \
 	packet_search_f(packet_out, \
			          (packet_search_opts_t){ \
	                       .min_seq_num = packet_get_seq_num_at_last_sent_command(), \
	                       .find_first = true, \
                           .return_on_missing_packet = false, \
	                       .command_id = -1, \
	                       .packet_len = -1, \
	                       .response = 1, \
	                       .packet_id = -1, \
	                       .template_len = 0, \
	                       .templat = 0, \
	                       __VA_ARGS__\
                      })
int (packet_search_f)(packet_with_meta_t *packet_out,
		            const packet_search_opts_t opts);


// Can be called from any function, interrupt handlers and not.
// See comment for packet_search(_f) for more information.
#define packet_wait_for(packet_out, max_wait_msec, ...) \
	packet_wait_for_f(packet_out, \
			          max_wait_msec, \
			          (packet_search_opts_t){ \
	                       .min_seq_num = packet_get_seq_num_at_last_sent_command(), \
	                       .find_first = true, \
					       .return_on_missing_packet = false, \
	                       .command_id = -1, \
	                       .packet_len = -1, \
	                       .response = 1, \
	                       .packet_id = -1,\
	                       .template_len = 0, \
	                       .templat = 0, \
	                       __VA_ARGS__\
                      })
int packet_wait_for_f(packet_with_meta_t *packet_out,
		              const int max_wait_msec, // Approximate. Negative value => forever
		              const packet_search_opts_t opts);


void packet_non_ack_sent(packet_t last_sent_command, unsigned int packet_length);


void packet_register(const unsigned char packet[], const int length);


seq_num_t packet_seq_num_end();


int packet_wait_for_ack(const int max_wait_msec);

#endif /* GEOPHONE_H_ */
