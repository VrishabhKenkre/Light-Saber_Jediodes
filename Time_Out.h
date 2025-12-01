#ifndef TIME_OUT_H_
#define TIME_OUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OK    = 0,
    Error = 1

} TypdedefStatus;

/* Initialize TIM10-based timebase for 1 ms tick */
void     Ticks_Init(uint32_t freq_hz);

/* Return millisecond tick count */
uint32_t get_Ticks(void);

/* Busy-wait delay in milliseconds */
void     delay(uint32_t delay_ms);

/* Called by TIM10 IRQ once per millisecond */
void     TimeOut_OnTick(void);

#ifdef __cplusplus
}
#endif

#endif /* TIME_OUT_H_ */

