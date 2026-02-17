#include <stdint.h>

void start_systick(void);
uint32_t stop_systick(void);

/* SysTick variables */
#define SysTick_BASE          (0xE000E000UL +  0x0010UL)
#define SysTick_START         0xFFFFFF

#define SysTick_CSR           (*((volatile uint32_t*)(SysTick_BASE + 0x0UL)))
#define SysTick_RVR           (*((volatile uint32_t*)(SysTick_BASE + 0x4UL)))
#define SysTick_CVR           (*((volatile uint32_t*)(SysTick_BASE + 0x8UL)))

#define SysTick_Enable        0x1
#define SysTick_ClockSource   0x4
