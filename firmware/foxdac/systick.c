#include <stdio.h>
#include "systick.h"

void start_systick()
{
    SysTick_RVR = SysTick_START;
    SysTick_CVR = 0;
    SysTick_CSR |=  (SysTick_Enable | SysTick_ClockSource);
}

uint32_t stop_systick()
{
    SysTick_CSR &= ~SysTick_Enable;

    uint32_t cycles = (SysTick_START - SysTick_CVR);

    if (SysTick_CSR & 0x10000)
        printf("WARNING: counter has overflowed, more than 16,777,215 cycles");

    return(cycles);
}
