
#ifndef DUST_COM_H_
#define DUST_COM_H_
#include "packet.h"

// "Commands/notifications" (messages) sent from or received in this program when communicating with controller (PC).
// This will be the first byte in "user data".
enum { // Out, from MCU to PC
    OMSG_AVALANCHE      = 0, // Std dev >= limit. Payload: avalanche_warning_t
    OMSG_SYSTEMSTATUS   = 1, // System status. PAYLOAD IS NOT FIXED. Payload: system_status_t
    OMSG_LOGGINGSTARTED = 2, // Notification that logging has started. No payload.
    OMSG_LOGGINGSTOPPED = 3, // Notification that logging has stopped. Payload: uint16_t, which is
                             //   the "temporary ID" of the most recent run attempted written to FLASH.
    OMSG_PARS           = 4, // Copy of adjustable params in use. Payload: adjustable_params_t
    OMSG_DATALIST       = 5, // List of data "runs" (COMP trigs), one per packet. Payload: data_list_item_t
    OMSG_DATA           = 6, // Data belonging to a single "run" (COMP trig), split into packets as needed.
                             //   Payload: data_desc_t + either nothing or the data described by data_desc_t
    OMSG_AUX           	= 7, // AUX sensor data, to replace OMSG_SYSTEMSTATUS, PAYLOAD IS NOT FIXED. Payload: system_status_t
    OMSG_AVALANCHE_JBV  = 8, // Std dev >= limit. Payload: avalanche_warning_t - samme som 1, men med forskjellig innhold. to sensorer og means
};
enum { // In, from PC to MCU
    IMSG_REBOOT         = 0, // Command MCU to reboot. No payload.
    IMSG_CHECKSTATUS    = 1, // Command MCU to check its status, and respond with OMSG_STATUS. No payload.
    IMSG_CHECKMOTE      = 2, // Command MCU to check mote status, and if necessary command it to reset. No payload.
    IMSG_RESERVED       = 3, // Currently not used, so that we can get a logical correspondence between order of
                             //   OMSGs and IMSGs without breaking compatibility.
    IMSG_STARTLOGGING   = 4, // Command MCU to start logging. No payload.
    IMSG_GETPARS        = 5, // Command MCU to send adjustable params in use. No Payload.
    IMSG_SETPARS        = 6, // Command MCU to set new adjustable params. Payload: adjustable_params_t
    IMSG_LISTDATA       = 7, // Command MCU to send list of data in FLASH. No payload.
    IMSG_SENDDATA       = 8, // Command MCU to send data belonging to a single "run" (COMP trig). Payload: data_desc_t
    IMSG_CHECKBACKUPRADIO = 9, // Same as IMSG_CHECKSTATUS except forces transmit through backup radio
    IMSG_SETJOINDUTYCYCLE = 10, // Command MCU to set the mote's joinDutyCycle, followed by a reboot of the mote
                                // (needed for parameter to take effekt). In the mote is really down (not just the network),
                                // this will in effect be a delay, and the backup radio will also in this case be "enabled".
};


// do { } while (0) is an old trick to get exact(?) same syntax for use as a "command" (regarding ";" and maybe more).
// Anyway we need a scope.
#define DUST_SEND(...) do { unsigned char buf[] = __VA_ARGS__; dust_send_request(buf, ARRAY_LEN(buf)); } while (0)
#define DUST_SEND_RET(ret, ...) int ret; do { unsigned char buf[] = __VA_ARGS__; ret = dust_send_request(buf, ARRAY_LEN(buf)); } while (0)


extern int gl_socket_id;

extern bool gl_dont_wait_for_acks;

// Most low-level: Sends data to mote, adds 0x7E (HLDC) before and after..
void tx_to_mote(unsigned char tx_data[], long length);

// Packs data into packet with CRC and escapes, sends to mote, waits for ack, as necessary tries to retransmit up to 3 times.
// Does not set the length field (set by caller)!
// Wrong value put in LSB CRC(?)! AND MSB should be before LSB?
int dust_send_request(unsigned char tx_data[], const unsigned char length);

// Packs data into ACK-package with CRC and escapes, and sends to mote.
// Independent escaping/CRC of dust_send_request()! Should be in one place.
// No retransmit.
void dust_send_ack(unsigned char rx_data[]);

// IV: As far as I can see: Checks packet for integrity (header/trailer/"checksum") and unescapes data.
int dust_get_packet (unsigned char rx_data[], long length);

void dust_check_and_possibly_reset_mote();

/*handle radio (dust network) data with this function*/
void dust_handle_notification(const packet_with_meta_t *const packet_with_meta);

// See comment in dust_com.c
//void dust_enable_backup_radio_if_necessary();
//void dust_disable_backup_radio_if_necessary();

// Send data to controller
unsigned char dust_tx_msg_data_txDone(const unsigned char msg_id, const unsigned char pack[], const unsigned char length,
                                      uint16_t *seq_num); // Described more in .c.
void dust_tx_msg_data(const unsigned char msg_id, const unsigned char pack[], const unsigned char length);
void dust_tx_data (const unsigned char pack[], const unsigned char length);



#endif /* DUST_COM_H_ */
