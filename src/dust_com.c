#include <string.h>
//#include "em_rtc.h"
#include "em_usart.h"
#include "segmentlcd.h" /*only for debugging in this function, shall be removed*/
#include "em_int.h"
#include "em_gpio.h"
#include "ngi_rtcdrv.h"
#include "dust_com.h"
#include "geophone.h"
#include "fcs.h"
#include "process.h"
#include "flash.h"
#include "packet.h"
#include "timestamp.h"
#include "config.h"
#include "adjustable_params.h"
#include "debug_printf.h"
#include "dust_com.h"

// Socket for input and output:
// Iff negative, a socket has not been successfully opened, backup modem may be attempted.
int gl_socket_id = -1;

// Iff true, do not wait for acks and thus do not retransmit, in dust_send_request().
//
// Policy:
//   Any non-main() code that sets the value must set the value back to the original value (at call time) before returning.
//
// Maybe it would have been cleaner to have this as a function parameter, but that would involve changes
// in multiple functions.
bool gl_dont_wait_for_acks = false;

static bool gl_sync = true;
static bool gl_packet_ID = true;




// Most low-level: Sends data to mote, adds 0x7E (HLDC) before and after..
void tx_to_mote(unsigned char tx_data[], long length)
{

//#if 0
	/*output ro uart 1 for debugging*/
//	USART_Tx(UART1,0xbb);
//	USART_Tx(UART1,0x7e);
//	for (int n=0;n<length;n++)
//		{
//			USART_Tx(UART1,tx_data[n]);
//		}
//	USART_Tx(UART1,0x7e);
//	USART_Tx(UART1,0xff);
//	USART_Tx(UART1,0x00);
//	USART_Tx(UART1,0xff);
	/*output ro uart 1 for debugging*/
//#endif


	/* output to mote*/

	// gl_radio_output_is_busy is replaced by INT_Disable().
//	// "Lock" output. This way is not safe for multiple threads but safe for interrupt handlers.
//	// Not important that it is safe, either, I guess.
//	gl_radio_output_is_busy++;

	// Make sure no interrupts are called so there is no interrupt sending an ack
	// while  this code sends a command or ack
	//INT_Disable();

	//GPIO_IntDisable(256);

	USART_Tx(UART0,0x7e);
	for (int n=0;n<length;n++)
	{
		USART_Tx(UART0,tx_data[n]);
	}
	USART_Tx(UART0,0x7e);

	//GPIO_IntEnable(256);

	//INT_Enable();
//    gl_radio_output_is_busy--; // "Unlock" output.
}



// Adds CRC to data_in (MODYFYING IT), then byte-stuffs it into data_out.
// Does not include HLDC framing bytes.
// Returns the length of the final packet.
int make_hldc_packet(unsigned char data_in[], const int length, unsigned char data_out[])
{
	unsigned short fsc = fcs_calcFcs(data_in, length); /*calculate checksum*/

	data_in[length] = fsc & 0xff;
	data_in[length+1] = fsc >> 8;

	int i=0;
	int n=0;
	while (i<length+2)  /* byte stuffing of 7e and 7d*/
	{
		if (data_in[i]==0x7e)
		{
			data_out[n] = 0x7d;
			data_out[n+1] = 0x5e;
			n=n+2;
		}
		else if (data_in[i]==0x7d)
		{
			data_out[n] = 0x7d;
			data_out[n+1] = 0x5d;
			n=n+2;
		}
		else
		{
			data_out[n] = data_in[i];
			n=n+1;
		}
		i=i+1;

	}
	return n;
}


// Packs data into packet with CRC and escapes, sends to mote, waits for ack, as necessary tries to retransmit up to 3 times.
// Does not set the length field (set by caller)!
// Wrong value put in LSB CRC(?)! AND MSB should be before LSB?
// OBS: Earlier: "Modifies tx_data! There must be two free bytes at the end for CRC."
// Now we copy data into a big enough buffer.
int dust_send_request(unsigned char tx_data[], unsigned char length)
{
	unsigned char packet_with_crc[DUST_HDLC_MAX_PAYLOAD + 2]; // API HEADER + API_PAYLOAD + CRC
	unsigned char request_pack[DUST_HDLC_MAX-2]; // We don,'t need space for framing bytes
	(void)memcpy(packet_with_crc, tx_data, length);

	packet_with_crc[1] = length-DUST_API_PAYLOAD_HEADER;
	packet_with_crc[2] = 8*gl_sync + 2*gl_packet_ID; /*set sync and packet ID flaggs */

	const int request_pack_length = make_hldc_packet(packet_with_crc, length, request_pack); // Not including framing bytes

	// Make sure no interrupts are called so we register a well-defined input packet id
	// when we call packet_non_ack_sent().
	// We don't need this for retransmits.
	//INT_Disable();
	packet_non_ack_sent(tx_data, length); // tx_data, not packet_with_crc
	PRINTF("Sending tx_data [%d,%d,%d]\n", tx_data[0], tx_data[1], tx_data[2]);
	tx_to_mote(request_pack, request_pack_length); /* send data to mote and debugg */
	//INT_Enable(); // // OBS TEST COMMENT OUT THIS LINE AND 3 OTHERS

	int response_code;

	if (gl_dont_wait_for_acks) {
	    response_code = 0;
	}
	else {

        double wait_time_ms = 1000;
        PRINTF("Waiting for ack for [%d,%d,%d]\n", tx_data[0], tx_data[1], tx_data[2]);
        response_code = packet_wait_for_ack(wait_time_ms); /*wait for acknowledment from mote*/
        PRINTF("Got ack %d\n", response_code);

        int i=0;

        while (response_code != 0 && i<3)  /* tries to resend package 3 times before giving up */
        {
            PRINTF("Resending tx_data [%d,%d,%d]\n", tx_data[0], tx_data[1], tx_data[2]);
            tx_to_mote(request_pack, request_pack_length);
            wait_time_ms = 1.5*wait_time_ms; // OBS TEST (1), could be permanent
            PRINTF("Waiting for ack for [%d,%d,%d]\n", tx_data[0], tx_data[1], tx_data[2]);
            response_code = packet_wait_for_ack(wait_time_ms);
            PRINTF("Got ack %d\n", response_code);
            i=i+1;
        }
	}


	gl_sync = false;
	gl_packet_ID = !gl_packet_ID;
    return response_code;
}


// Packs data into ACK-package with CRC and escapes, and sends to mote.
// Independent escaping/CRC of dust_send_request()! Should be in one place.
// No retransmit.
void dust_send_ack(unsigned char rx_data[])
{

    // Used to have length 4, leaving no room for CRC
	unsigned char tx_ack[6] = {rx_data[0], 0x00, rx_data[2]|1, 0x00, };
	unsigned char ack_pack[10]; // 10 = 2[excape factor] * (4[packet length] + 2[CRC length]) - 2[Hard coded zeros, will not be escaped]

	int length = 4;

	length = make_hldc_packet(tx_ack, length, ack_pack);

	tx_to_mote(ack_pack, length);
	//fprintf("Ack [%d %d %d %d]\n", ack_pack[0], ack_pack[1], ack_pack[2], ack_pack[3]); // OK X.

	/*wait(50); can bb omitted when imlementing 3 lines tx ?? */

}


// IV: As far as I can see: Checks packet for integrity (header/trailer/"checksum") and unescapes data.
// 20140719: FIxed potentially serious bug: CRC was previously taken from packet *before* unescaping.
//           I also changed the layout/code cosmetics somewhat to my taste (but not all, so a bit incosnistent).
int dust_get_packet (unsigned char rx_data[], long length)
{

	// Shortest imaginable valid raw HLDC packet, not sure if 0 bytes API Payload/response code is even possible:
	// 2 guard bytes (126), 2 bytes frame check sequence (CRC?), 3 bytes API header, 0 bytes API Payload.
	if (length < 7)
		return 0;

	if (rx_data[0] == 0x7e && rx_data[length-1] == 0x7e) {
        int n, m;
		for (n = 1, m = 0; n < length-1; ++m, ++n) {

			if (rx_data[n] == 0x7d) {
				if (rx_data[n+1] == 0x5d)
					rx_data[m] = 0x7d;
				else if(rx_data[n+1] == 0x5e)
					rx_data[m] = 0x7e;
				else
					break; // error, erroneous escape
				++n;
			}
			else {
				rx_data[m] = rx_data[n];
			}

		}

		if (n < length - 1)
			return 0; // Erroneous escape => break

		length = m-2; // Of HDLC payload = API header and API payload

		unsigned short fsc_in = rx_data[length+1];
		fsc_in = (fsc_in << 8) + rx_data[length];


		unsigned short fsc_test = fcs_calcFcs(rx_data,length);

		if (fsc_in == fsc_test) {
			return length;
		}
		else {
			return 0;
		}

	}
	else {
		return 0;
	}

}


int dust_get_socket_ID()
{
    packet_with_meta_t packet;

    const unsigned char *last_sent_command = packet_get_last_sent_command();

    // If using 0 delay <=> no wait, we could call packet_search instead,
    // but would have to consider the default parameters (at least the minimum sequence number)
    const bool ack_with_socket_id_found =
        packet_wait_for(&packet, 0,
                        .packet_len = 1,
                        .response = 1, // Default, but be sure if default changes
                        .packet_id = -1, // (last_sent_command[2]&1) >> 1,
                        .command_id = last_sent_command[0])
        == 0;
    return ack_with_socket_id_found ? packet.packet[4] : -1;
}


void dust_close_any_sockets_and_optionally_allow_status_on_backup(const bool allow_status_message_on_backup)
{
    const bool prev_gl_dont_wait_for_acks = gl_dont_wait_for_acks;

    gl_dont_wait_for_acks = true;

    // Minimize the chance of having two or more sockets open for output.
    if (gl_socket_id >= 0) {
        DUST_SEND({0x16, 0x01, 0x02, gl_socket_id}); /*close socket*/

        gl_socket_id = -1;
        //wait(1000);

        // When main radio is "disabled", the wish is that the controller should get a status message on the backup.
        //if (allow_status_message_on_backup)
        //    check_status_and_report_on_backup_radio();
    }

    gl_dont_wait_for_acks = prev_gl_dont_wait_for_acks;
}


void dust_close_any_sockets()
{
    dust_close_any_sockets_and_optionally_allow_status_on_backup(false);
}


int dust_open_sockets_and_optionally_allow_status_on_backup(const bool allow_status_message_on_backup)
{
    // It is very important that this succeeds, so we loop a few times.
    const int max_loops = 5;

    dust_close_any_sockets_and_optionally_allow_status_on_backup(false);

    int temp_socket_id = -1;

    int delay = 1000;
    for (int k = 0; k < max_loops; ++k) {
        DUST_SEND({0x15, 0x01, 0x00, 0x00}); /*open socket*/
        if ((temp_socket_id = dust_get_socket_ID()) >= 0) {
            //wait(delay); // We do not want to busy-loop
            DUST_SEND_RET(ret, {0x17, 0x03, 0x02, temp_socket_id, 0xf0, 0xb9});  /*bind socket*/
            if (ret == 0){
            	gl_socket_id = temp_socket_id;
            	break;
            }
            DUST_SEND({0x16, 0x01, 0x02, temp_socket_id}); /*close socket*/
            gl_socket_id = -1;
        }
        if (k < max_loops - 1)
            wait(delay); // We do not want to busy-loop
        delay *= 2;
    }

    // When main radio is "disabled", the wish is that the controller should get a status message on the backup.
    //if (allow_status_message_on_backup && gl_socket_id < 0)
    //    check_status_and_report_on_backup_radio();

    return (gl_socket_id >= 0) ? 0 : -1;
}


int dust_open_sockets()
{
    return dust_open_sockets_and_optionally_allow_status_on_backup(false);
}


void dust_check_and_possibly_reset_mote()
{
    enum {GETPARAMETER=0x02, MAX_TRIES = 3, START_MILLISECS = 1000};
    const float MILLISECS_MULTIPLIER = 1.5;
    packet_with_meta_t packet;
    bool ack_with_mote_status_found = false;
    const seq_num_t min_seq = packet_seq_num_end(); // In effect same as we do in dust_tx_msg_data_txDone(), but simpler.
    unsigned int max_wait_msec = START_MILLISECS;
    for (int itrie = 0; itrie < MAX_TRIES && !ack_with_mote_status_found; ++itrie) {

        // The following line will look for ack and if needed retry a few times,
        // but maybe not enough times without the itrie loop.
        DUST_SEND({GETPARAMETER, 0, 0, 0x0E}); // getParameter moteStatus

        ack_with_mote_status_found =
            packet_wait_for(&packet, max_wait_msec,
                            .packet_len = 11,
                            .response = 1, // Default, but be sure if default changes
                            .packet_id = -1, // Default I guess. Seems we may get back acks with different packet id. (last_sent_command[2]&1) >> 1,
                            .command_id = GETPARAMETER,
                            .min_seq_num = min_seq)
            == 0;

        max_wait_msec *= MILLISECS_MULTIPLIER;
    }

    int status = ack_with_mote_status_found ? packet.packet[5] : -1;

    //if (status >= 2 && status <= 4 && gl_socket_id >= 0) {
        // On its way up (so do not reset), but not operational and socket >= 0!
    //    dust_close_any_sockets(); // As of 20141009: Close sockets, set gl_socket_id = -1 and send OMSG_STATUS on backup
    //    return;
    //}

   // const int old_gl_socket_id = gl_socket_id;

    bool ok = status >= 2 && status <= 5;

    //if (ok && status == 5 // operational
    //    && gl_socket_id < 0 && dust_open_sockets_and_optionally_allow_status_on_backup(false) < 0) // but socket not open and cannot open
    //    ok = false;

    if (!ok) {
        gl_socket_id = -1;

        // When main radio is "disabled", the wish is that the controller should get a status message on the backup.
        //if (old_gl_socket_id >= 0) // Check that is used to be "enabled" so we don't send message every time when mote is "down"
        //    check_status_and_report_on_backup_radio();

		GPIO_PinModeSet(gpioPortD,8,gpioModeWiredAnd,0); /*external hard reset*/
		wait(1000);
		GPIO_PinModeSet(gpioPortD,8,gpioModeInput,0); /*clear external reset*/

        //DUST_SEND({0x08, 0x00, 0x00}); // reset

        // We may consider emptying the buffer of incoming packets also, but not sure
        // what would happen considering registered packet sequence numbers here and there.
        // 20140922: Why should we empty? I guess it is OK as it is.
    } else {

    	dust_close_any_sockets();

    	dust_open_sockets();

    }
}


/*handle radio (dust network) data with this function*/
void dust_handle_notification(const packet_with_meta_t *const packet_with_meta)
{
	const unsigned char *rx_data = packet_with_meta->packet;

	int notification_ID = rx_data[0];
	/*long event_type = 256*rx_data[5] + rx_data[6];*/

	switch (notification_ID) //kan feilen ligge i at en notification kan inneholde flere events????
	{
		case 0x0f:/*event*/

			/*int mote_state = rx_data[7];*/
			/*int mote_alarms = (256*rx_data[10] + rx_data[11])*/

			switch (256*rx_data[5] + rx_data[6])
			{
				case 0x0001: /*boot event*/
				    dust_close_any_sockets();
					gl_sync = true;
					gl_packet_ID = true;
					//DUST_SEND({0x06, 0x00, 0x0a, }); /* join request */
					break;

				case 0x0002: /*alarms opened or closed*/
					dust_close_any_sockets();
					DUST_SEND({0x08, 0x00, 0x00}); // reset
					break;

				case 0x0004: /*utc time mapping changed*/
					break;

				case 0x0008: /*join operation failed*/
				    // I don't think we need to set gl_socket_id to -1 here since it probably already is -1,
				   	dust_close_any_sockets();
				   	//DUST_SEND({0x08, 0x00, 0x00}); // reset
					break;

				case 0x0010: /*mote disconnected from network*/
				    dust_close_any_sockets();
				    DUST_SEND({0x08, 0x00, 0x00}); // reset
					break;

				case 0x0020: /*mote operational and ready to send data*/
					if (dust_open_sockets() < 0){
						DUST_SEND({0x08, 0x00, 0x00}); // reset
					} else {
						get_aux_and_report();
					}
					break;

				case 0x0080: /*service allocation change*/
					//if (dust_open_sockets() < 0){
					//	DUST_SEND({0x08, 0x00, 0x00}); // reset
					//} else {
					//	get_aux_and_report();
					//}
					break;

				case 0x0100: /*mote started joining the network*/
					break;

			}

			//debuggeing mote event notifications on backupradio
			//dust_enable_backup_radio_if_necessary();
			//USART_Tx(UART1,0x7e);
			///USART_Tx(UART1,rx_data[5]);
			//USART_Tx(UART1,rx_data[6]);
			//USART_Tx(UART1,0x7e);
			//dust_disable_backup_radio_if_necessary();
			//debuggeing mote event notifications on backupradio

			break;

		case 0x0D: /*time information*/
		{
			// Packet has timestamp in Big-endian, while our system is little-endian
			timestamp_t timestamp;
			unsigned char *const ptr = (unsigned char *)&timestamp;
			for (unsigned int i = 0; i < sizeof(timestamp.secs); ++i)
				ptr[sizeof(timestamp.secs)-i-1] = rx_data[7+i];
			for (unsigned int i = 0; i < sizeof(timestamp.microsecs); ++i)
				ptr[sizeof(timestamp.secs)+sizeof(timestamp.microsecs)-i-1] = rx_data[7+sizeof(timestamp.secs)+i];

			if (gl_wait_for_timestamp)
			{
				gl_recording_stop = true;
				gl_wait_for_timestamp = false;
			}else{
				process_ringbuf_apply_timestamp(packet_with_meta->seq_num, timestamp);
			}

			gl_timestamp = timestamp;
			break;
		}
		case 0x19: {/*packet recived*/

			/* handle data generated by other NGI developed software/firmware at this point*/
			const int cmd_index = DUST_SENDTO_PREFIX_LEN;
			const int payload_length = packet_with_meta->length-1-cmd_index;
			if (packet_with_meta->length >= cmd_index+1) {
				const int command = rx_data[cmd_index];
				switch (command) {

					case IMSG_REBOOT:
	                    if (payload_length == 0)
        					//This function has been changed to perform an external reboot with hw4 and hw5

                    		GPIO_PinModeSet(gpioPortD,8,gpioModeWiredAnd,0); /*external hard reset*/
                    		wait(1000);
                    		GPIO_PinModeSet(gpioPortD,8,gpioModeInput,0); /*clear external reset*/


	                    	//DUST_SEND({0x08, 0x00, 0x00}); // reset
						break;

                    case IMSG_CHECKMOTE:
                        if (payload_length == 0)
                            //dust_check_and_possibly_reset_mote();
                        break;

                    case IMSG_CHECKSTATUS:
                        if (payload_length == 0)
                            //check_status_and_report();
                        	get_aux_and_report();
                        break;

					case IMSG_GETPARS:
					    if (payload_length == 0)
					        dust_tx_msg_data(OMSG_PARS, gl_adjustable_params, sizeof(adjustable_params_t));
						break;

					case IMSG_SETPARS:
						if (payload_length == sizeof(adjustable_params_t)) {
							adjustable_params_t pars;
							memcpy(&pars, rx_data+cmd_index+1, sizeof(adjustable_params_t)); // memcpy in case there is an issue with alignments
							adjustable_params_set(&pars);
							comp_config(gl_adjustable_params->comp_trig_levels, gl_adjustable_params->comp_pos_sels);
						}
						break;

					case IMSG_STARTLOGGING:
					    if (!logging_is_running() // Actually checked inside ACMP0_IRQHandler() also I guess
					        && payload_length == 0)
					        ACMP0_IRQHandler();
						break;

					case IMSG_LISTDATA:
                        if (!logging_is_running() // Don't run if logging is running, so we don't loose any AD interrupts due to serial IO.
                            && payload_length == 0)
					        process_list_data();
						break;

					case IMSG_SENDDATA:
                        if (!logging_is_running() // Don't run if logging is running, so we don't loose any AD interrupts due to serial IO.
					        && payload_length == sizeof(data_desc_t))
                            process_send_data(packet_with_meta);
					    break;

                   // case IMSG_CHECKBACKUPRADIO:
                   //     if (payload_length == 0)
                            //check_status_and_report_on_backup_radio();
                   //     break;

                   // case IMSG_SETJOINDUTYCYCLE:
                   //     if (payload_length == 1) {
                            // Should we only do this when gl_socket_id >= 0?
                            // No (?), it could be useful to set it even if the mote recently "went down"
                            // (after we received IMSG_SETJOINDUTYCYCLE).
                            // On the other hand, we might have to wait for timeouts in DUST_SEND() if the mote is really down.
                  //          DUST_SEND({0x01, 0x00, 0x00, 0x06, rx_data[cmd_index+1]}); // Set joinDutyCycle
                            // When main radio is "disabled", the wish is that the controller should get a status message on the backup.
                  //          if (gl_socket_id >= 0) {
                  //              gl_socket_id = -1;
                                //check_status_and_report_on_backup_radio();
                  //          }
                   //         wait(1000); // Probably not necessary (?)
                  //          DUST_SEND({0x08, 0x00, 0x00}); // reset
                  //      }
                  //      break;

				}
			}
			break;
		}

		case 0x25: /*data transmission completed*/
			break;

		case 0x26: /*recived adverticment*/
			break;

	}
}


// Used to be able to wait for txDone notification.
// For the time being, only dust_tx_msg_data_txDone() is allowed to change this.
static uint16_t gl_packet_to_master_seq_num = 0xffff; // 0xffff => we will not get txDone

// dust_tx_msg_data() + logic for retransmit if txDone with FAIL is received.
// Consider this function if you have a series of packets to send that should not overflow
// the mote.
// For the time being the rule is: Do not call this function from an ISR.
// At least as long as we stick to this rule, I do not see any reason to bother
// calling this function with non-0-pointer seq_num. In fact I recommend it to be 0.
// I keep the parameter to avoid code-changes and in case I would reconsider.
unsigned char dust_tx_msg_data_txDone(const unsigned char msg_id, const unsigned char pack[], const unsigned char length,
                                      uint16_t *seq_num)
{
    enum {OK = 0x00, FAIL = 0x01, TIMEOUT = 0x02};
    enum { MAX_ITERS = 10,
           MILLISECS_MAX_WAIT_FOR_CONFIRMATION = 1000,
           MILLISECS_AFTER_BACKUP_SEND = 500
    };
    static uint16_t static_seq_num = 0;

    unsigned char status = FAIL;

    if (seq_num == 0)
        seq_num = &static_seq_num;

    if (*seq_num == 0xffff) // output sequence number
        *seq_num = 0;

    gl_packet_to_master_seq_num = *seq_num;

    seq_num_t min_seq_num; // input sequence number

    // We only repeat if we get FAIL.
    // On timeout, the reason might be that the mote is down and we haven't caught on to it.
    // We could choose to repeat also on timeout, the decision is not clear.
    for (unsigned int iter = 0; status == FAIL && iter < MAX_ITERS; ++iter) {

        dust_tx_msg_data(msg_id, pack, length);
        if (iter == 0)
            min_seq_num = packet_get_seq_num_at_last_sent_command();

        if (gl_socket_id >= 0) {
            int templat[] = {*seq_num % 256, *seq_num >> 8};
            packet_with_meta_t packet;
            const bool ack_with_mote_status_found =
                packet_wait_for(&packet, MILLISECS_MAX_WAIT_FOR_CONFIRMATION,
                                .packet_len = 3,
                                .response = 0,
                                .min_seq_num = min_seq_num,
                                .packet_id = -1,
                                .command_id = 0x25, // txDone
                                .templat = templat,
                                .template_len = ARRAY_LEN(templat))
                == 0;
            status = ack_with_mote_status_found ? (packet.packet[5] == OK ? OK : FAIL) : 2;

            //if(status == FAIL)			//KAN VÆRE VÆRDT Å PRØVE DETTE FOR OG UNGÅ TAPTE PAKKER I OVERFØRING - Viker derimot litt usannsynlig siden det ligger inne en wait for tx_done i packet_wait_for funksjonen, tror jeg
            //	wait(1000);

        //}
        //else {
            // Packet sent to backup radio, so no chance we get txDone. Instead wait before we return.
        //    wait(MILLISECS_AFTER_BACKUP_SEND);
        //    status = 0x00;
        }
    }
    gl_packet_to_master_seq_num = 0xffff;
    ++*seq_num;

    return status;
}


void dust_tx_msg_data(const unsigned char msg_id, const unsigned char pack[], const unsigned char length)
{
	unsigned char buf[length+DUST_COMMAND_ID_LEN];
	for (int k =  0; k < DUST_COMMAND_ID_LEN-1; ++k)
	    buf[k] = 0;
	buf[DUST_COMMAND_ID_LEN-1] = msg_id;
	memcpy(buf+DUST_COMMAND_ID_LEN, pack, length);
	dust_tx_data(buf, ARRAY_LEN(buf));
}


/*
 * The following variables and (two) functions lets code pre-open the backup radio if the primary radio is down,
 * typically in expectation of a large number of packets out.
 * The functions should be called in pairs, first enable then disable.
 * Works a bit like a semaphore.
 * Intended for calls by non-ISR functions.
 */
//static bool gl_backup_radio_enabled = false;
//static int gl_backup_radio_count = 0;
//void dust_enable_backup_radio_if_necessary()
//{
//    if (gl_socket_id < 0 && !gl_backup_radio_enabled) {
        //GPIO_PinOutClear(gpioPortC,7); /*868 wake*/
    	//GPIO_PinOutSet(gpioPortC,7); /*868 on*/
//        wait(100);
 //       gl_backup_radio_enabled = true;
 //   }
 //   ++gl_backup_radio_count;
//}

//void dust_disable_backup_radio_if_necessary()
//{
 //   --gl_backup_radio_count;
//    if ((gl_backup_radio_count == 0 || gl_socket_id >= 0) && gl_backup_radio_enabled) {
//        wait(1000);
        //GPIO_PinOutSet(gpioPortC,7); /*868 sleep*/
        //GPIO_PinOutClear(gpioPortC,7); /*868 off*/
//        gl_backup_radio_enabled = false;
//    }
//}


void dust_tx_data (const unsigned char pack[], const unsigned char length)
{
	//SegmentLCD_Symbol(1,1);

	//wait(200); /*a short delay between sending an ack and the next request. Probably not needed anymore */


	if (gl_socket_id >= 0) {
	    // OBS Currently uses low priority!
	    static const unsigned char sendto_template[] =
	        {0x18, 0x00, 0x00, 0x00, 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	         0xf0, 0xb8, 0x00, 0x00};
        unsigned char buf[ARRAY_LEN(sendto_template)+2+length];
        memcpy(buf, sendto_template, ARRAY_LEN(sendto_template));
        buf[ARRAY_LEN(sendto_template)+0] = gl_packet_to_master_seq_num % 256;
        buf[ARRAY_LEN(sendto_template)+1] = gl_packet_to_master_seq_num >> 8;
        memcpy(buf+ARRAY_LEN(sendto_template)+2, pack, length);
        // Fill in some of the "template" part.
        // The packet length is handled by dust_send_request.
        buf[3] = gl_socket_id;
        dust_send_request(buf, ARRAY_LEN(buf)); /*send data*/

        //SegmentLCD_Symbol(1,0);
    }
	//else {
	    // Use backup radio:
	    //dust_enable_backup_radio_if_necessary();

	    //GPIO_IntDisable(256);
        //INT_Disable(); // Only really needed if dust_tx_data() is called from an ISR, which it isn't as of 20140912.
        //for (int n=0;n<length;n++)
        //     USART_Tx(UART1,pack[n]);
        //INT_Enable();
        //GPIO_IntEnable(256);

        //dust_disable_backup_radio_if_necessary();
	//}

}

