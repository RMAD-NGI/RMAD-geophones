
#ifndef GEOPHONE_H_
#define GEOPHONE_H_

#define HW_CONFIGURATION 1
#define SW_VERTION 6

#define HW_REVITION 5
#define HW_RADIO 0

extern volatile int gl_num_samples;
extern volatile bool gl_wait_for_timestamp;
extern volatile bool gl_recording_stop;

#include <signal.h> // Get sig_atomic_t

bool logging_is_running();

void check_status_and_report();

//void check_status_and_report_on_backup_radio();

void check_and_possibly_reset_mote();

void get_aux_and_report(void);

int convertToCelsius(sample_t);

#endif /* GEOPHONE_H_ */
