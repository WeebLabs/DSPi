/*
 * loop_prof.h; bench-only main-loop section profiler (DSPI_LOOP_PROF)
 *
 * LOOP_PROF(n) closes the running section and opens section n; the worst
 * duration of each section is read (and cleared) through REQ_GET_STATUS
 * wValue LOOP_PROF_STATUS_BASE + n. Compiles to nothing in normal builds.
 */
#ifndef LOOP_PROF_H
#define LOOP_PROF_H

#include <stdint.h>

#define LOOP_PROF_SECTIONS     14
#define LOOP_PROF_STATUS_BASE  64

#ifdef DSPI_LOOP_PROF
void loop_prof_mark(uint8_t section);
uint32_t loop_prof_take(uint8_t section);
#define LOOP_PROF(n) loop_prof_mark(n)
#else
#define LOOP_PROF(n) ((void)0)
#endif

#endif // LOOP_PROF_H
