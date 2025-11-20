// Time_Out.h
#ifndef TIME_OUT_H_
#define TIME_OUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    OK   = 0,
    Error = 1

} TypdedefStatus;

void     Ticks_Init(uint32_t freq);
uint32_t get_Ticks(void);
void     delay(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* TIME_OUT_H_ */
