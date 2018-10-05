
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "em_device.h"
#include "em_chip.h"

#include "em_emu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_dac.h"
#include "em_acmp.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_wdog.h"
#include "em_int.h"
#include "em_common.h" // for EFM32_MIN(), needed at least for debugging
#include "segmentlcd.h"
#include "ngi_rtcdrv.h"
#include "wait_burtc.h"

#include "dust_com.h"
#include "config.h"
#include "flash.h"
#include "process.h"
#include "packet.h"
#include "adjustable_params.h"
#include "debug_printf.h"
#include "geophone.h"

// Some interesting parameters you may play around with:
#define IF_IS_SAFE // See coment in main(). Pretty sure it is OK to define.
#define RESET_MOTE_AT_START // OK defined and undefined, but probably best defined. Otherwise the socket bind likely fails and we get a reset anyway, after some time.
#define MOTE_REPORT_MIN_INTERVAL 900 // seconds. Too often => Perhaps lots of battery use (on mote). Also used for system check and reporting.
#define MOTE_CHECK_MIN_INTERVAL 86400 // seconds. Too often => Perhaps lots of battery use (on mote). Also used for system check and reporting.
#define MIN_WAIT_MS_EM  1  // 1 ms, > 0 => be sure not to (almost ?) busy-loop. I am guessing 0 is also OK.
// Seemed like we did, maybe because of some strange unknown interrupts,
// or maybe UART interrupts (but didn't break on breakpoint?), or maybe some bug.
// UPDATE: The bug seems to have been in process_ringbuf_entries(), the while loop, now fixed,
// and setting this to e.g. 10 ms is too much for the AD interrupts, or shouldn't be, but ran slowly.
// UPDATE 2: Runs slowly because it takes time to send commands or maybe rather wait for ack.
// It goes quickly e.g. when starting AD, many interrupts, slowly when AD is turned off.
#define MAX_WAIT_MS_EM1 1000 // 100 ms in EM1, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
#define MAX_WAIT_MS_EM2 1000 // 60 seconds, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
#define MAX_WAIT_MS_EM3 1000 // 60 seconds, MUST BE MUCH LOWER THAN 256 SECONDS! Watchdog!
// #define DEBUG_AD_DATA // Strictly for debugging. Anyway disabled outside DEBUG mode
// End parameters

#define MIN(x0, x1) ((x0) < (x1) ? (x0) : (x1))

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[1])) // OK X

/*
 ****** Principles for sleep (EM) modes and how to avoid infinite sleeps/bad responses.
 *
 * The strategy is to use a "basic algorithm" that is almost 100% safe, then close
 * the remainder by never sleeping forever even when no interrupt comes, even in EM3, by using BURTC.
 *
 * Initial attempt at basic algorithm:
 *   USART/GPIO wanted sleep level are define by the GPIO ISR, which is called on both start and end transmit.
 *   COMP/ADC wanted sleep level is defined by the COMP0 ISR and by the ADC ISR when finished reading samples.
 *   The minimum sleep level of the two could then be used, however:
 *   In both cases there is a vulnerability if the final ISR call (GPIO/ADC) (setting sleep level 3, though that is not the point)
 *   after the handling of recording_start/end and messages (notifications) is finished and before the sleep is entered.
 *   Depending on whether the interrupt happens before or after the calculation of the minimum sleep level,
 *   EM1 or EM3 is entered, and the final interrupt was already performed before that, so
 *   nothing wakes up the sleep.
 *
 *   It seems exceedingly difficult to fix this in the basic algorithm, but using BURTC, in addition to the basic algorithm,
 *   we can set an interrupt to wake up after a short period. The period can be very short, e.g. 0.1 seconds or less,
 *   for EM1, thus giving good performance in the EM1 case.
 *   However, probably more likely the final interrupt happened before the calculation
 *   of the minimum sleep mode, so maybe giving a minimum of 3. Having a very short BURTC wakeup
 *   for EM3 could be quite a drain on the power supply, we need a long timeout here (last resort),
 *   and make sure our basic algorithm almost never needs the last resort.
 *   Solution: Put a loop around the heavy processing, continuing depending on whether
 *  recording_start/end have been turned on or a message is available in the packet buffer or arrived while processing was ongoing. Something like:
 *     do {
 *       < handle recording >
 *       message_received = false;
 *       message_found = <handle next message> // not necessarily the most recent
 *       // Calculate sleep_mode inside loop, both to minimize "critical region"
 *       // and to get sleep_mode == 1 even if "worst case" end-IRQ(s) (which are no longer so bad) appear(s) in the critical region.
 *       sleep_mode = min(gl_sleep_mode_recording, gl_sleep_mode_uart);
 *     } while (recording_stop || message_found || message_received); // message_received might be true if new message just came in
 *     < Critical region
 *       Actually extending from somewhere in the calc of while-termination-expr above
 *       until sleep is entered).
 *       If we (e.g.? Any other possible scenario?) sleep_mode == 3,
 *       and we here get a GPIO start interrupt, all incoming bytes, and GPIO end interrupt,
 *       all before entering sleep mode, we are in the
 *       worst case scenario where we may have to wait up to e.g. 10 seconds in EM3
 *       before servicing the message. It may be less than (e.g.) 10 seconds if a new interrupt comes during sleep.
 *       If we put anything time-consuming here, the risk of this increases, and even a COMP interrupt
 *       might cause problems if all AD interrupts afterwards are received before sleep is entered.
 *     >
 *     switch (sleep_mode) {
 *       1: EM1 max e.g. 0.1 seconds
 *       3: EM3 max e.g. 10 seconds
 *     }
 *
 * The worst case scenario above seems almost impossible to get into, but if it does,
 * BURTC or any other type of interrupt capable of exiting EM3, will eventually get us out of EM3 anyway,
 * after max. e.g. ca. 10 seconds.
 */

/****** Principles for communication with the radio
 * Input from the mote is read exclusively by UART0_RX_IRQHandler (and GPIO_ODD_IRQHandler).
 * GPIO_ODD_IRQHandler interprets the input as packages and puts them in a ring-buffer
 * of packets (including acknowledgements). When the buffer is full, the oldest package is overwritten.
 *
 * Ack'ing of incoming packets is done by the machinery in GPIO_ODD_IRQHandler,
 * while waiting for acks from the mote as needed (it is up to you) is your responsibility when writing new code.
 *
 * Any non-IRQ-called routine is allowed to write
 * to the mote, and also to block
 * waiting for ack or another packet to arrive, by calling wait_for_packet().
 * It is, however, not recommended to block for longer than necessary, since this may delay
 * servicing of incoming requests, etc.
 *
 * Used to be: The only thing IRQ routines are allowed to write, is ack's.
 * Upgraded to: ISRs (IRQ routines) may send any packets/requests. They must, however,
 * take extreme care if they wait for the ACK from the mote, the GPIO/UART ISR's must have higher (>=? Not enough I guess) prio
 * so that ack may be read.
 * Also, they must take extreme care not to destroy non-ISR code which assume e.g. that
 * gl_packet_last_sent_command is not changed from an ISR.
 * In practice: Be extremely careful if writing anything except ACKs from an ISR! Analyze the code to be sure! Or don't do it!
 *              Non-ISR code is allowed to assume that ISRs only send ACKs!
 * As of 20140912 ISRs only send ACKs as far as I know. I pledge to update this line if that changes.
 *
 * Why use an input queue instead of just a single input packet buffer like before?
 * We want to be able to wait for e.g. an ack after sending a packet. That was already possible,
 * due to the relevant IRQ routine checking whether a packet was an ack or another type of packet.
 * In effect almost a two-packet buffer as long as one packet was an ack and one was not (though
 * the ack was not kept cleanly as a packet, so for accessing return info in ack one had
 * to read a lower-level buffer. Relevant at least for socketID).
 * However, receiving more than one notification while waiting for ack would lead to all but the last to be
 * overwritten and thus discarded. This becomes relevant when we want to make pretty sure that
 * e.g. "timeIndication" and "receive" notifications are not discarded. Discarding a "receive" might cause
 * the microcontroller to miss a command to start logging!
 * Secondly (less important) the ack-handling, including using information from an ack (socketID), hopefully will be cleaner.
 */

// Prefix "gl_" => global
// FYI In the ARM inplementation we use, 'sig_atomic_t' is 'int' as far as I can see.
volatile static sig_atomic_t gl_sleep_mode_recording = 3;
volatile static sig_atomic_t gl_sleep_mode_uart = 3;
volatile static int gl_prev_received_packet_id = -1;
volatile static int gl_current_sample = 0;
volatile static int gl_current_scan = 0;
volatile int gl_num_samples = 0;
volatile static int gl_num_aux = 0;
volatile static int charging = 1;

volatile static int gl_num_sample_skip = 0; //for uttesting av forkasting av de første samples under tigging

volatile static unsigned char gl_RX_in[DUST_HDLC_MAX];
volatile static int gl_RX_index;

volatile static bool gl_event_recived = false;

volatile static bool gl_recording_start = false;
volatile bool gl_recording_stop = false;
volatile static bool gl_recording_running = false;

volatile bool gl_wait_for_timestamp = false;

volatile static bool gl_get_aux_data = false;
volatile static bool gl_aux_data_recived = false;
sample_t aux_sample[6];

volatile static int sleep_mode;

bool logging_is_running() {
    return gl_recording_running;
}

void ADC0_IRQHandler(void) {
if (gl_get_aux_data == true) {

	ADC_IntDisable(ADC0, ADC_IF_SINGLE);
	aux_sample[gl_num_aux] = ADC_DataSingleGet(ADC0);

    ADC_Reset(ADC0);

    gl_aux_data_recived = true;
    gl_num_aux = gl_num_aux + 1;

} else {


    static const sample_t default_value = 0;

    static scan_t scan;

    if (gl_current_sample == 0) {
        process_ringbuf_add(); // BEFORE GPIO_PinOutSet()!
        GPIO_PinOutSet(gpioPortA, 14); //clear nTime

        memset(scan, default_value, sizeof(scan));
    }

#if defined(DEBUG_AD_DATA) && !defined(NDEBUG)
    const int shift_per_chan = 2; // 2 samples
    if (gl_current_scan % 100 == 0)
        PRINTF("DEBUGGING USING FAKED DATA!!!!!!!!!!!\n");
    static unsigned int ch = -1;
    ch = (ch + 1) % CONFIG_AD_NCHANS;
    const sample_t sample = sin(gl_current_scan * 1.0 / shift_per_chan + ch)
            * (ch + 1) * 10;
#else
    //const unsigned int ch = (ADC0 ->STATUS & 117440512) >> 24; // 3 bits, but for differential only lower 2 should be possibly non-zero
    const unsigned int ch = (ADC0 ->STATUS & 50331648) >> 24;// 2 bits, don't want to keep checking for overflow...
    assert(ch < CONFIG_AD_NCHANS);
    const sample_t sample = (sample_t)ADC_DataScanGet(ADC0);
#endif

    if (gl_num_sample_skip>0)
    {

    	--gl_num_sample_skip; //bruker comp_trigg_levels(2) til å definere antall samples som skal droppes i starten av ett trigg/dødbånd

    }else{
    	/*store ADC data to memory from this point*/
    	scan[ch] = sample;
    	// Samples should arrive in channel order. It is possible to lose samples.
    	// We assume that a scan is finished when we get a sample from the last channel.
    	if (ch == CONFIG_AD_NCHANS - 1) {
    		process_ringbuf_add_scan(scan);
    		++gl_current_scan;
    		memset(scan, default_value, sizeof(scan));
    	}
    }

    if (gl_current_scan <= process_log_until()) {
        ++gl_current_sample;
        gl_sleep_mode_recording = 1; // Superfluous I guess
    } else {

    	GPIO_PinOutClear(gpioPortA, 14); //get nTime end of dataset

    	preamp_set_status(gl_adjustable_params->preamp_cmp_trigg);

        ADC_Reset(ADC0);

        comp_config(gl_adjustable_params->comp_trig_levels, gl_adjustable_params->comp_pos_sels); //added for testing of preamp

        //ACMP_IntClear(ACMP0, ACMP_IF_EDGE);
        //ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
        //ACMP_IntClear(ACMP1, ACMP_IF_EDGE);
        //ACMP_IntEnable(ACMP1, ACMP_IF_EDGE);

        process_ringbuf_lock_traceset();

        gl_num_samples = gl_current_scan - 1;   //calculates the number of samples in a dataset
        //gl_recording_stop = true;
        gl_recording_running = false;
        gl_current_sample = 0;
        gl_current_scan = 0;
        gl_sleep_mode_recording = 3;
        gl_wait_for_timestamp = true;

        wait(10);
        GPIO_PinOutSet(gpioPortA, 14); //clear nTime end of dataset

    }

    ADC_IntClear(ADC0, ADC_IF_SCAN);
}
}

void ACMP0_IRQHandler(void) {
    // Why introduce gl_recording_running?
    // At least because this function may be called from dust_handle_notification(),
    // but we may also use it e.g. to check whether it is safe to reset the mote.
    // Kind of makes the disabling of ACMP0 at the bottom of this function unneccessary I guess (with slight rewrite of AD ISR), but the
    // code is there, and might be good for efficiency.
    if (gl_recording_running)
        return;

   // dac_config(); //added for testing of preamp

    DAC0 -> CH1DATA = 2048;

    preamp_set_status(gl_adjustable_params->preamp_logging);

    GPIO_PinOutClear(gpioPortA, 14); //get nTime

    ADC_Reset(ADC0);
    gl_get_aux_data = false;
    gl_aux_data_recived = false;
    gl_num_aux = 0;

    const uint32_t *const trig_levels_temp = gl_adjustable_params->comp_trig_levels;
    gl_num_sample_skip = trig_levels_temp[1]*CONFIG_AD_NCHANS;

    // According to our experience, this is needed here, not just at start of program.
    // Maybe because calls to ADC_Init and ADC_InitScan are needed.
    gl_ad_sampling_rate = AD_config();

    //wait(5);


    ADC_Start(ADC0, adcStartScan);

    gl_recording_start = true;
    gl_recording_running = true;
    gl_sleep_mode_recording = 1;

    ACMP_IntDisable(ACMP0, ACMP_IF_EDGE);
    ACMP_IntDisable(ACMP1, ACMP_IF_EDGE);

}


/* Not needed/used it seems, I guess ACMP0_IRQHandler() is called both for ACMP0 and ACMP1.
 * See following link for more information, and also Yme tested.
 * http://community.silabs.com/t5/32-Bit-Discussion/missing-ACMP1-IRQn/td-p/119452
 */
//void ACMP1_IRQHandler(void) {
//    ACMP0_IRQHandler();
//}

void get_uart0_bytes() {
#if 1 // Most recent code
    // If we would have overflowed RX_in, we just throw away the bytes and bet
    // that the hardware flow control, crc-checking etc will synchronize back.
    while (UART0 ->STATUS & USART_STATUS_RXDATAV) {
        unsigned char in = USART_RxDataGet(UART0 );
        if (gl_RX_index < sizeof(gl_RX_in)) {
            gl_RX_in[gl_RX_index] = in;
            ++gl_RX_index;
        }
    }
#else
    // If using this, comment out call from GPIO_ODD_IRQHANDLER
    unsigned char in = USART_Rx(UART0);
    if (gl_RX_index < sizeof(RX_in)) {
        gl_RX_in[gl_RX_index] = in;
        ++gl_RX_index;
    }
#endif
}

void UART0_RX_IRQHandler(void) {
    // Is this ISR safe?
    // 1. It does not clear the interrupt flag, could it be called again and again...?
    // 2. It always reads one character only. What if there are several incoming characters
    //    before this ISR is called? Will the ISR be re-issued?

    // Now trying fix of these issues/questions.
    get_uart0_bytes();

    USART_IntClear(UART0, USART_IF_RXDATAV);

}

// An odd GPIO port changes.
void GPIO_EVEN_IRQHandler(void) {
#define USE_ORIGINAL
    if (GPIO_PinInGet(gpioPortC, 8) == false) {

        gl_sleep_mode_uart = 1;
#ifdef USE_ORIGINAL

        gl_RX_index = 0;
#endif

        GPIO_PinOutClear(gpioPortC, 9);

    } else {
        // Will hold both full escaped HLDC packet and then unescaped API header and payload.
        unsigned char RX_packet[DUST_HDLC_MAX];

        GPIO_PinOutSet(gpioPortC, 9);
        get_uart0_bytes(); // Get any remaining bytes in the fifo.
        // We need to disable interrupts since GPIO interrupt has a lower priority,
        // than UART0_RX_IrqHandler():
        INT_Disable();
#ifdef USE_ORIGINAL // Original code, we trust hardware flow
        const int RX_packet_length = gl_RX_index;
        (void) memcpy(RX_packet, gl_RX_in, RX_packet_length);
#else
        int RX_packet_length;
        int istart, barrier = 0, k = 0;
        for (k = 0; k < gl_RX_index; ++k) {
            if (gl_RX_in[k] == 126) {
                if (++barrier == 1)
                istart = k;
                else if (barrier == 2) {
                    RX_packet_length = k - istart + 1;
                    (void)memcpy(RX_packet, gl_RX_in+istart, RX_packet_length);
                    gl_RX_index = gl_RX_index-k-1;
                    (void)memcpy(gl_RX_in, gl_RX_in+k+1, gl_RX_index);
                    break;
                }
            }
        }

        if (barrier == 0) {
            RX_packet_length = gl_RX_index = 0;
        }
        else if (barrier == 1) {
            gl_RX_index -= istart;
            (void)memcpy(gl_RX_in, gl_RX_in+istart, gl_RX_index);
            RX_packet_length = 0;
        }


#endif
        INT_Enable();

//		//IV: Safe to just add to UART1 and never read the other end?
//		/*output to uart1 for debugging*/
//		USART_Tx(UART1, 0xaa);
//		for (int n = 0; n < RX_length; n++) {
//			USART_Tx(UART1, RX_packet[n]);
//		}
//
//		USART_Tx(UART1, 0xff);
//		USART_Tx(UART1, 0x00);
//		USART_Tx(UART1, 0xff);
//		/*END output to uart1 for debugging*/

#ifndef NDEBUG
        const int max_bytes = 9;
        PRINTF("Received packet [");
        for (int k = 0; k < EFM32_MIN(max_bytes, RX_packet_length); ++k)
            PRINT("%d,", RX_packet[k]);
        PRINT("%s]\n", (RX_packet_length > max_bytes ? "..." : ""));
        if (RX_packet[1] == 37)
            PRINTF("txDone\n");
#endif

        const int RX_length = dust_get_packet(RX_packet, RX_packet_length);

        if (RX_length >= 3) {
            const bool is_ack = (RX_packet[2] & 0x01) != 0;
            const int received_packet_id = (RX_packet[2] & 0x02) >> 1;
            const bool is_sync = (RX_packet[2] & 0x08) != 0;
            if (is_ack) {
                PRINTF("Received ack [%d,%d,%d]\n", RX_packet[0], RX_packet[1],
                        RX_packet[2]);
                //ack_recived = true;
                packet_register(RX_packet, RX_length);
            } else {
                PRINTF("Received notification [%d,%d,%d]\n", RX_packet[0],
                        RX_packet[1], RX_packet[2]);
                // gl_radio_output_is_busy is replaced by INT_Disable().
//				if (!gl_radio_output_is_busy)
                dust_send_ack(RX_packet);

                // For retransmits we only send and ack (above), we don't re-register it as incoming,
                // except when the sync flag is set, since we then cannot see the difference between a newly booted
                // mote and a retransmit. A pity, since the main problem seems to be retransmit of packets
                // with sync set (or is the mote booting "thousands of times"?).
                // If we start this program with an already-running mote,
                // we should accept any initial packet_id even if sync is not on,
                // so we gove the initial gl_prev... the value -1
                if (is_sync
                        || received_packet_id != gl_prev_received_packet_id) {
                    gl_prev_received_packet_id = received_packet_id;
//
//					for (int n = 0; n < RX_length; n++) {
//						event_pack[n] = RX_packet[n];
//					}
//					event_pack_length = RX_length;
                    gl_event_recived = true;
                    packet_register(RX_packet, RX_length);
                }

            }
        } else {
            PRINTF("Packet is invalid\n");
        }

        gl_sleep_mode_uart = 3;
    }
    GPIO_IntClear(256);//8192
}

void check_status_and_report() {
    EFM32_PACK_START(1); // Actually a no-op for GNU it seems, for GNU we use __attribute__ ((__packed__))
    // More info: See https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html#Type-Attributes
    typedef struct
        __attribute__ ((__packed__)) {
            // NOT USED: uint8_t     life_phase; // Indicates where we are in the lifetime of the system: startup/status check requested/etc
    		uint8_t system;
            uint8_t revition;                   // Non-zero iff an out socket is open. Does not access the mote, so may not catch if the mote died "recently".
            uint8_t state;                 // True iff battery is charging

            uint16_t vdd3;
            int16_t temperature;

            uint8_t signed_number;

            //uint16_t ai4;
            //uint16_t ai5;
            //uint16_t ai6;
            uint16_t ai7;

            //uint16_t nandflash_status[FLASH_BLOCK_NSTATES];
                                                            /* Number of nandflash blocks in each of
                                                             * BLOCK_USED, BLOCK_UNUSED, BLOCK_BAD, BLOCK_ACTIVE
                                                             * (or whatever is defined in flash.h) states.
                                                             * Sums to total number of blocks (2000 at time of writing).*/

        } system_status_t;
        EFM32_PACK_END(); // Actually a no-op for GNU it seems

        //int sw_vertion = 1;
        //int hw_revision = 1;

        system_status_t status;

        status.system =  HW_CONFIGURATION * 16 + SW_VERTION;
        status.revition = HW_REVITION * 16 + HW_RADIO;

        //status.system = sw_vertion + hw_revision * 16;

        status.vdd3 = aux_sample[0];
        status.temperature = convertToCelsius(aux_sample[1]);

        //int charging = battery_charge_status(status.vdd3, status.temperature); //statusbit endret til charge enable - PC7 - i hw4 og hw5
        //int charging = 0; //!GPIO_PinInGet(gpioPortD, 8);
        int connected = (gl_socket_id >= 0) ? 1 : 0;
        int trigg1_status = (ACMP0 -> STATUS) & 2;
        int trigg2_status = (ACMP0 -> STATUS) & 2;

        status.state = charging +  2*connected + 2*trigg2_status + 4*trigg1_status;

        status.signed_number = 0;

        //status.ai4 = aux_sample[2];
        //status.ai5 = aux_sample[3];
        //status.ai6 = aux_sample[4];
        status.ai7 = aux_sample[5];

        //for (int k = 0; k < ARRAY_LEN(status.nandflash_status); ++k)
        //    status.nandflash_status[k] = 0;
        //for (int iblock = 0; iblock < FLASH_NBLOCKS; ++iblock)
        //    ++status.nandflash_status[gl_block_info[iblock].status];

        if (gl_socket_id > 0)	//satt inn for og redusere trafikk mellom efm32 og dust when mote lost
        {
        	dust_tx_msg_data(OMSG_AUX, &status, sizeof(status));
        }


        charging = battery_charge_status(status.vdd3, status.temperature); //statusbit endret til charge enable - PC7 - i hw4 og hw5 - flyttet hit for at sendt bit skal representere status ved avlesning av batterispenning

    }

	void get_aux_and_report(void){

        aux_sensor_config(adcSingleInpVDDDiv3);

        gl_get_aux_data = true;
        gl_sleep_mode_recording = 1;
        //sleep_mode = 1; //vil sannsynligvis stoppe feilmålinger ved kall utenfor main, på beskjed over radio eller ved reset/mote reset - men vurderer og beholde for å få oversikt over resetter.

        ADC_Start(ADC0, adcStartSingle);

	}



   // void check_status_and_report_on_backup_radio()
   // {
    //    const int gl_socket_id_backup = gl_socket_id;
    //    gl_socket_id = -1; // Ugly code to force packet to be sent through backup radio.
    //    check_status_and_report(); // bør dette endres til get aux and report??????????????????????????????????????????????????????????????
    //    gl_socket_id = gl_socket_id_backup;
    //}


    int convertToCelsius(sample_t adcSample)
    {
      int16_t temp;

      /* Factory calibration temperature from device information page. */
      float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                                 >> _DEVINFO_CAL_TEMP_SHIFT);

      float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                                   & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                                  >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

      /* Temperature gradient (from datasheet) */
      float t_grad = -6.3;

      temp = 100*(cal_temp_0 - ((cal_value_0 - adcSample/8) / t_grad));

      return temp;
    }


    /**************************************************************************//**
     * @brief  Main function
     *****************************************************************************/

    int main(void) {
        /*
         * Regarding padding in structs:
         * Some places in the code we assume that structs have no padding.
         * This is done where we use structs as input or output parameters for IO from/to radio.
         * For this purpose, we use the gcc-specific "__attribute__((__packed__))" (or equivalently "attribute((packed))" I guess),
         * and for "completeness" also EFM32_PACK_START/EFM32_PACK_END.
         * We use this also for flash headers (perhaps simpler to interpret FLASH in future if padding changes in a new compiler),
         * and for cached flash header info (save RAM).
         * For radio IO we assume the packed attribute not only packs fields together,
         * but also that there is no trailing padding. The latter is important when checking
         * that packets are of the right size by comparing with sizeof(<struct>),
         * when sending packets with data after a struct (e.g. OMSG_DATA), and maybe
         * even when sending packets with nothing after the struct (depending
         * on the receiver).
         * It has, however, been difficult to interpret the gcc documentation in a way that
         * guarantees no trailing padding. However,
         *   a) See Ambroz Bizjak's answer in http://stackoverflow.com/questions/7957363/effects-of-attribute-packed-on-nested-array-of-structures
         *      Seems to know what he talks about, and also in comments to his answer mtalexan has tested it, it seems.
         *      Note that i found at least one other person claiming the opposite about trailing padding in gcc.
         *      Also note (on the positive side) that I don't think we use any pointers to packed members
         *      (as of 20140921), and anyway "Cortex-M3 and M4 allow unaligned access by default"
         *      (http://stackoverflow.com/questions/18269181/unaligned-access-causes-error-on-arm-cortex-m4)
         *   b) Just below we add a test of our own.
         *
         *   One place in the code we have already coded around the question about *trailing* padding
         *   by using offsetof(<struct>, <last member>) + sizeof(<last member>) (function send_data() using data_desc_t).
         *   I won't change that code back for the time being.
         *
         *   If we change to another compiler than gcc I guess we will get
         *   compilation errors due to "__attribute__((__packed__))".
         *
         *   Here is the packing test:
         */
        { // Scope so we don't "use up" variable e, in case we wish to use it later.
            struct __attribute__((__packed__)) {
                unsigned char a;
                uint16_t b;
                uint32_t c;
                double d;
            } e;
            _Static_assert(sizeof(e) == 1+2+4+sizeof(double), "Packing not working as assumed!");
        }


        /* Chip errata */
        CHIP_Init();

        DEBUG_INIT;
        PRINTF("geophone.c starting\n");

        /* Enable peripheral clocks */
        CMU_ClockEnable(cmuClock_HFPER, true);
        CMU_ClockEnable(cmuClock_ADC0, true);
        CMU_ClockEnable(cmuClock_ACMP0, true);
        CMU_ClockEnable(cmuClock_ACMP1, true);
        CMU_ClockEnable(cmuClock_DAC0, true);
        CMU_ClockEnable(cmuClock_GPIO, true);
        CMU_ClockEnable(cmuClock_UART0, true);
        //CMU_ClockEnable(cmuClock_UART1, true);
        CMU_ClockEnable(cmuClock_RTC, true);

        CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);

        // Initialize adjustable parameters stored in FLASH between runs. Run this before using any parameter.
        adjustable_params_init();

        preamp_config(gl_adjustable_params->preamp_cmp_trigg);


        GPIO_PinModeSet(gpioPortC,7,gpioModeWiredAnd,1); /*charge enable - mcp73811   1=on 0=off*/



        uart0_config();
        //uart1_config();
        comp_config(gl_adjustable_params->comp_trig_levels,
                     gl_adjustable_params->comp_pos_sels);



        //AD_config();
        rtc_config(); // Simply calls RTCDRV_Setup().
        ADC_Reset(ADC0); // Make sure the AD is not running (only an issue during debugging, right?). Configuration is done in UART ISR.
        process_config(); // Also handles the NAND FLASH

        // BURTC, see main_gg_stk.c and others in AN0041
        // Everything put in burtc_config.
        // Grouping all calls together makes it simpler?
        burtc_config();

        wdog_config(); // May depend on setup of clocks done in burtc_config() (not tested).
        //WDOG_lock(); // Comment back in when debugged?

        //SegmentLCD_Init(false);


       // GPIO_PinModeSet(gpioPortB,11,gpioModeWiredAnd,0);// entered to test preamp ref



        ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
        NVIC_EnableIRQ(ACMP0_IRQn);
        NVIC_SetPriority(ACMP0_IRQn, 2);
        ACMP_IntEnable(ACMP1, ACMP_IF_EDGE);

        // http://community.silabs.com/t5/32-Bit-Discussion/missing-ACMP1-IRQn/td-p/119452   :
        // NVIC_EnableIRQ(ACMP1_IRQn);
        // NVIC_SetPriority(ACMP1_IRQn, 2);

        // Do we need this? Probably not.
        // Or maybe yes, see http://embeddedgurus.com/state-space/2014/02/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities/:
        // "default priority of zero".
        // Should have lower prio (ie higher number) than UART and GPIO, since
        // in rare circumstances it may send packets to mote (could be changed if
        // we skip waiting for ack's).
        NVIC_SetPriority(ADC0_IRQn, 4);

        USART_IntClear(UART0, USART_IF_RXDATAV);
        USART_IntEnable(UART0, USART_IF_RXDATAV);
        NVIC_EnableIRQ(UART0_RX_IRQn);
        NVIC_SetPriority(UART0_RX_IRQn, 1);

        GPIO_IntClear(256); //8192
        GPIO_IntEnable(256); //8192
        NVIC_EnableIRQ(GPIO_EVEN_IRQn);
        NVIC_SetPriority(GPIO_EVEN_IRQn, 3);

        // Read any bytes in the input buffer.
        // The test code in get_uart0_bytes() should not be enabled.
        get_uart0_bytes();
        gl_RX_index = 0;

        // At startup it is desired to get a status packet through the backup radio.
        // Status is later sent through main radio when(/if) operational.

       //check_status_and_report_on_backup_radio();

//#ifdef RESET_MOTE_AT_START
        // Reset mote.
        // This has been necessary in a case (or two) where not even pressing the reset button has helped.
        // If we get no ack, might one reason be that the mote expects a 0 (false) packet_ID?
        // Shouldn't be, since the sync flag is set in our packet, but to be extra cautious
        // we try sending a new packet.

        //antagerlig er eneste grunne til at dette trengs at sw ikke kobler opp mot en port for data pakker autmoatisk hvis moten er oppe når efm32 rebooter.

 //       unsigned char reset[] = { 0x08, 0x00, 0x00, }; /*reset*/
 //       if (dust_send_request(reset, ARRAY_LEN(reset)) != 0)
 //           dust_send_request(reset, ARRAY_LEN(reset));
//#else
//        dust_check_and_possibly_reset_mote(); //kommentert ut for å teste stabilitet - YK
//#endif

        DUST_SEND({0x01, 0x02, 0x00, 0x24, 0x01}); // set autojoin

        wait(500);

        DUST_SEND({0x01, 0x02, 0x02, 0x06, 0x0C}); // Set joinDutyCycle to 5%

        wait(500);

        DUST_SEND({0x08, 0x00, 0x00}); // reset

        float secs_since_mote_report = 1000; // Not exact. Used to avoid checking mote too often.
        float secs_since_mote_check = 0; // Not exact. Used to avoid checking mote too often.
        float secs_since_last_sleep = 0; // Not exact. Used to avoid checking mote too often.

        seq_num_t search_seq_num_start = 0;
        int sleep_mode;
        bool notification_found = true; // Set to true initially, at least #ifdef IF_IS_SAFE
        packet_with_meta_t packet;
        while (true) {

            /*realtime signal prosessing of ADC data (from memory) from this point
             * We loop until we have treated all incoming events, while still giving good
             * priority to recording_start/recodring_stop.
             */
            do {

                if (gl_recording_start) {
                    //			SegmentLCD_Symbol(0, 1);
                    //dust_enable_backup_radio_if_necessary();
                    gl_dont_wait_for_acks = true;
                    dust_tx_msg_data(OMSG_LOGGINGSTARTED, "", 0);
                    gl_recording_start = false;
                }

                process_ringbuf_entries();

                if (gl_recording_stop) {
                    gl_dont_wait_for_acks = false;
                    //const uint16_t flash_id = process_latest_flash_id();
                    //const timestamp_t flash_timestamp = process_latest_flash_timestamp();


                    const event_t lastevent = process_latest_flash_event();

                    dust_tx_msg_data(OMSG_LOGGINGSTOPPED, &lastevent, sizeof(lastevent));
                    //dust_tx_msg_data_txDone(OMSG_LOGGINGSTOPPED, &lastevent, sizeof(lastevent),0); //tester "sikker" sending av loggingstopped

                    //			SegmentLCD_Symbol(0, 0);
                    gl_recording_stop = false;


                    ACMP_IntClear(ACMP0, ACMP_IF_EDGE);
                    ACMP_IntEnable(ACMP0, ACMP_IF_EDGE);
                    ACMP_IntClear(ACMP1, ACMP_IF_EDGE);
                    ACMP_IntEnable(ACMP1, ACMP_IF_EDGE);
                }

                // If this thinking is surely correct, you may define IF_IS_SAFE, otherwise keep it commented-out:
                //
                // The following if-check is not necessary, it is only to improve efficiency.
                // In fact I had to think a bit on whether it is safe, but should be.
                // First time in the program execution it is obviously safe.
                // The first time in all later do-while loops are also safe, because
                // at exit of the previous do-while loop, every finished incoming notification has been processed,
                // so gl_event received <=> some unprocessed notification.
                // Loop-iteration after the first in each do-while loop:
                // If we do not enter the if-statement, we did not get a new event while searching for/processing the
                // previous notification and in fact the packet buffer had no more
                // available notifications already the previous round, so there can be no available notifications.
                // Thus it must be safe to skip.
                //
#ifdef IF_IS_SAFE
                if (gl_event_recived || notification_found) {
#endif
                    gl_event_recived = false;
                    notification_found = packet_search(&packet,
                            .min_seq_num = search_seq_num_start,
                            .response = false) == 0;
                    if (notification_found) {
                        search_seq_num_start = packet.seq_num + 1;

                        /*handle radio (dust network) data
                         * Any lower-level function actually implementing any possibly long-lasting handling should consider feeding the watchdog.
                         */
                        dust_handle_notification(&packet);
                    }
#ifdef IF_IS_SAFE
                } else {
                    notification_found = false;
                }
#endif

                // This code is a way to allow us to run more or less time-consuming tasks
                // only if the system is likely headed towards EM3, i. e. not too busy,
                // without jeopardizing the almost 100% safe "basic algorithm" (made 100% by BURTC).
                // Maybe not important to recalculate sleep_mode.
                sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart);
                const bool repeat_outer_loop_likely =
                           gl_recording_stop
                           || notification_found || gl_event_recived;


                if (secs_since_mote_check >= MOTE_CHECK_MIN_INTERVAL & mote_sleep==false){

                	dust_check_and_possibly_reset_mote();
                	secs_since_mote_check = 0;
                }

                if (secs_since_mote_report >= MOTE_REPORT_MIN_INTERVAL
                    && sleep_mode == 3 && !repeat_outer_loop_likely) {
                    // Perform code which should not be performed when the system is busy.

                	secs_since_mote_report = 0;

                    get_aux_and_report();

                    sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart); // Maybe not necessary, but OK.

                }

                if (gl_aux_data_recived == true){

                	if (gl_num_aux == 1){

                	    aux_sensor_config(adcSingleInpTemp);
                	    ADC_Start(ADC0, adcStartSingle);

                	}else if (gl_num_aux == 2){

                	    aux_sensor_config(adcSingleInpCh4);
                	    ADC_Start(ADC0, adcStartSingle);

                	}else if (gl_num_aux == 3){

                	    aux_sensor_config(adcSingleInpCh5);
                	    ADC_Start(ADC0, adcStartSingle);

                	}else if (gl_num_aux == 4){

                	    aux_sensor_config(adcSingleInpCh6);
                	    ADC_Start(ADC0, adcStartSingle);

                	}else if (gl_num_aux == 5){

                	    aux_sensor_config(adcSingleInpCh7);
                	    ADC_Start(ADC0, adcStartSingle);

                	} else {

                	     check_status_and_report();

                	     gl_get_aux_data = false;
                	     gl_aux_data_recived = false;

                	     gl_num_aux = 0;
                	     gl_sleep_mode_recording = 3;

                	}

                	sleep_mode = MIN(gl_sleep_mode_recording, gl_sleep_mode_uart); // Maybe not necessary, but OK.

                }

            } while (gl_recording_stop || notification_found || gl_event_recived);

            switch (sleep_mode) {
            case 0:

                break;
            case 1:

            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM1, 1) / 1000.0;
                break;

            case 2:
            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM2, 2) / 1000.0;
                break;

            case 3:

            	secs_since_last_sleep = wait_from_to_burtc(MIN_WAIT_MS_EM, MAX_WAIT_MS_EM3, 3) / 1000.0;
                break;
            }

            secs_since_mote_report += secs_since_last_sleep;
            secs_since_mote_check += secs_since_last_sleep;
            secs_since_last_sleep = 0;

            WDOG_Feed();

        }
    }
