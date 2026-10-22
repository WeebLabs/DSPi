/*
 * ac3_bench.h; on-device AC-3 decode bench (DSPI_AC3_BENCH builds only).
 *
 * Decodes an embedded frame set in a loop on the main loop, times it with
 * time_us_32(), and checks the output checksum against the host harness.
 * Phase 1 gate of Documentation/Features/ac3_dts_decoding_plan.md.
 */

#ifndef AC3_BENCH_H
#define AC3_BENCH_H

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t  state;         // 0 idle, 1 requested, 2 done
    uint8_t  ram_hot;       // 0 XIP only, 1 kernels in RAM, 2 kernels + block parser
    uint8_t  frame_count;   // frames in the embedded set
    uint8_t  checksum_ok;   // 1 when the decode matched the host checksum
    uint32_t passes;        // passes over the set in the last run
    uint32_t total_us;      // wall time of the whole run
    uint32_t min_frame_us;  // fastest single frame (header + 6 blocks)
    uint32_t max_frame_us;  // slowest single frame
    uint32_t header_us;     // summed a52_frame() time
    uint32_t block_us;      // summed a52_block() time
    uint32_t sys_clk_hz;
    uint32_t checksum;      // FNV-1a of the decoded samples, last pass
    uint32_t expected;      // host checksum embedded with the frames
    uint32_t errors;        // frame or block decode failures
} Ac3BenchResult;           // 44 bytes

// From the vendor handler: schedule a run of `passes` passes (0 = 10).
void ac3_bench_request(uint32_t passes);
// Main loop: runs a requested bench to completion (blocks the loop).
void ac3_bench_service(void);
void ac3_bench_get(Ac3BenchResult *out);

#endif // AC3_BENCH_H
