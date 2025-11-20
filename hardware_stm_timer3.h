#ifndef HARDWARE_STM_TIMER3_H
#define HARDWARE_STM_TIMER3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void timer3_ch3_init_oc_toggle_1Hz(void);
void timer3_ch1_init_input_capture_rising_1MHz(void);


#ifdef __cplusplus
}
#endif

#endif

