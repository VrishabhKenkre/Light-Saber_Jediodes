/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED1_H_
#define __LED1_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Macros for Everyone--------------------------------------------------------*/




/*Function definitions---------------------------------------------------------*/

void init_LED1(void);
void toggle_LED1( void);
void init_PC6_input(void);
uint32_t read_PC6(void);

/********* LED1 (PB0) helpers *********/
void set_LED1(void);    // drive LED1 ON (PB0 = 1)
void clear_LED1(void);  // drive LED1 OFF (PB0 = 0)

#ifdef __cplusplus
}
#endif

#endif /*__LED1_H */
