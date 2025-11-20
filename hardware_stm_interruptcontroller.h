#ifndef HARDWARE_STM_INTERRUPTCONTROLLER_H
#define HARDWARE_STM_INTERRUPTCONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Enable the NVIC line for TIM3 global interrupt */
void enableNVIC_Timer3(void);

/* Configure TIM3 to generate update + CC3 interrupts (Q6) */
void initTimer3ToInterrupt(void);

/* TIM3 IRQ handler (called from vector table) */
void TIM3_IRQHandler(void);
void enableEXTI6OnPortC(void);
void EXTI9_5_IRQHandler(void);


void debugToggleFromInterruptFile(void);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_STM_INTERRUPTCONTROLLER_H */
