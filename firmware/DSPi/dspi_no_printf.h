/*
 * dspi_no_printf.h: force-included when DSPI_DEBUG_PRINTF is OFF (the default).
 * No stdio driver is linked, so printf()/puts() become empty inline functions.
 * Arguments are still evaluated and formats still checked by -Wformat.
 */
#ifndef DSPI_NO_PRINTF_H
#define DSPI_NO_PRINTF_H

#include <stdio.h>   // declare the real ones first; the macros only affect later uses

static inline __attribute__((format(printf, 1, 2)))
int dspi_printf_nop(const char *fmt, ...) { (void)fmt; return 0; }

static inline int dspi_puts_nop(const char *s) { (void)s; return 0; }

#define printf dspi_printf_nop
#define puts   dspi_puts_nop

#endif // DSPI_NO_PRINTF_H
