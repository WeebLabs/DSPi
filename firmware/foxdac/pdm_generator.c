#include <string.h>
#include <math.h>
#include "pdm_generator.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

// ----------------------------------------------------------------------------
// DATA STRUCTURES
// ----------------------------------------------------------------------------

typedef struct {
    int32_t sample;
    bool reset;
} pdm_msg_t;

#define RING_SIZE 256
static volatile pdm_msg_t pdm_ring[RING_SIZE];
static volatile uint8_t pdm_head = 0;
static volatile uint8_t pdm_tail = 0;

// DMA Circular Buffer
static uint32_t __attribute__((aligned(PDM_DMA_BUFFER_SIZE * 4))) pdm_dma_buffer[PDM_DMA_BUFFER_SIZE];
static int pdm_dma_chan = -1;

// PIO program offset and current pin (needed for pdm_change_pin)
static uint pdm_pio_offset;
static uint8_t pdm_current_pin = PICO_PDM_PIN;

// Enable/disable flag — set by Core 0 via pdm_set_enabled(), read by Core 1
volatile bool pdm_enabled = false;

// Core 1 mode and EQ worker handshake
volatile Core1Mode core1_mode = CORE1_MODE_IDLE;
Core1EqWork core1_eq_work = {0};

// Idle-time CPU load metering (Core 1)
static uint32_t c1eq_last_work_end = 0;
static uint32_t c1eq_load_q8 = 0;
static bool c1eq_load_primed = false;
static uint32_t pdm_load_q8 = 0;

// ----------------------------------------------------------------------------
// RAW PIO PROGRAM
// ----------------------------------------------------------------------------
static const uint16_t pio_pdm_instr[] = { 0x6001 }; // 0: out pins, 1
static const struct pio_program pio_pdm_program = { .instructions = pio_pdm_instr, .length = 1, .origin = -1 };

// ----------------------------------------------------------------------------
// UTILS
// ----------------------------------------------------------------------------
static uint32_t rng_state = 123456789;
static inline uint32_t fast_rand() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

// ----------------------------------------------------------------------------
// NOISE-SHAPED DITHER (IIR Error Feedback)
// ----------------------------------------------------------------------------
// 2nd-order IIR high-pass filter for noise shaping
// Pushes dither energy above ~8kHz with 12dB/octave slope
// Coefficients for Butterworth HP, fc=8kHz at fs=384kHz (8 chunks * 48kHz)
// Fixed-point Q14 format (divide by 16384)
#define NS_B0  15778   // 0.9630
#define NS_B1 -31556   // -1.9260
#define NS_B2  15778   // 0.9630
#define NS_A1  31531   // 1.9245 (negated for subtraction)
#define NS_A2  15580   // 0.9511

typedef struct {
    int32_t x1, x2;    // Input history
    int32_t y1, y2;    // Output history
    int32_t err_acc;   // Error accumulator for feedback
} noise_shaper_t;

static inline int32_t noise_shaped_dither(noise_shaper_t *ns, int32_t raw_dither, int32_t quant_error) {
    // Accumulate quantization error with decay (LP filter on error)
    // Decay factor 0.97 in Q8 = 248
    ns->err_acc = ((ns->err_acc * 248) >> 8) + (quant_error >> 6);

    // Combine raw dither with error feedback
    int32_t input = raw_dither - ns->err_acc;

    // Apply 2nd-order Butterworth HP filter
    int32_t output = (NS_B0 * input + NS_B1 * ns->x1 + NS_B2 * ns->x2
                    + NS_A1 * ns->y1 - NS_A2 * ns->y2) >> 14;

    // Update filter state
    ns->x2 = ns->x1;
    ns->x1 = input;
    ns->y2 = ns->y1;
    ns->y1 = output;

    return output;
}

// ----------------------------------------------------------------------------
// FUNCTIONS
// ----------------------------------------------------------------------------

void pdm_set_enabled(bool enabled) {
    pdm_enabled = enabled;
    __sev();  // Wake Core 1 if sleeping
}

void pdm_update_clock(uint32_t freq) {
    float div = (float)clock_get_hz(clk_sys) / (float)(freq * PDM_OVERSAMPLE);
    pio_sm_set_clkdiv(PDM_PIO, PDM_SM, div);
}

void pdm_setup_hw(uint8_t pin) {
    pdm_current_pin = pin;

    // Pre-fill buffer with 50% duty cycle silence before DMA starts
    for (int i = 0; i < PDM_DMA_BUFFER_SIZE; i++) {
        pdm_dma_buffer[i] = 0xAAAAAAAA;
    }

    pdm_pio_offset = pio_add_program(PDM_PIO, &pio_pdm_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pdm_pio_offset, pdm_pio_offset + (pio_pdm_program.length - 1));
    sm_config_set_out_pins(&c, pdm_current_pin, 1);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_gpio_init(PDM_PIO, pdm_current_pin);
    pio_sm_set_consecutive_pindirs(PDM_PIO, PDM_SM, pdm_current_pin, 1, true);
    pio_sm_init(PDM_PIO, PDM_SM, pdm_pio_offset, &c);

    pdm_update_clock(48000);
    pio_sm_set_enabled(PDM_PIO, PDM_SM, true);

    pdm_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dmac = dma_channel_get_default_config(pdm_dma_chan);
    channel_config_set_transfer_data_size(&dmac, DMA_SIZE_32);
    channel_config_set_read_increment(&dmac, true);
    channel_config_set_write_increment(&dmac, false);
    channel_config_set_dreq(&dmac, pio_get_dreq(PDM_PIO, PDM_SM, true));
    channel_config_set_ring(&dmac, false, PDM_DMA_RING_BITS);
    dma_channel_configure(pdm_dma_chan, &dmac, &PDM_PIO->txf[PDM_SM], pdm_dma_buffer, 0xFFFFFFFF, true);
}

void pdm_change_pin(uint8_t new_pin) {
    assert(!pdm_enabled);
    assert(core1_mode != CORE1_MODE_PDM);

    // Safety: stop SM and abort DMA (should already be stopped)
    pio_sm_set_enabled(PDM_PIO, PDM_SM, false);
    dma_channel_abort(pdm_dma_chan);

    // Release old pin → high-Z
    gpio_set_function(pdm_current_pin, GPIO_FUNC_NULL);
    gpio_set_dir(pdm_current_pin, GPIO_IN);

    // Init new pin for PIO
    pio_gpio_init(PDM_PIO, new_pin);
    pio_sm_set_consecutive_pindirs(PDM_PIO, PDM_SM, new_pin, 1, true);

    // Rebuild SM config with new pin
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, pdm_pio_offset, pdm_pio_offset + (pio_pdm_program.length - 1));
    sm_config_set_out_pins(&c, new_pin, 1);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(PDM_PIO, PDM_SM, pdm_pio_offset, &c);

    // Restore clock divider (pio_sm_init resets it)
    pdm_update_clock(48000);

    pdm_current_pin = new_pin;
}

void pdm_push_sample(int32_t sample, bool reset) {
    uint8_t next_head = pdm_head + 1;
    if (next_head != pdm_tail) {
        pdm_msg_t msg;
        msg.sample = sample;
        msg.reset = reset;
        pdm_ring[pdm_head] = msg;
        pdm_head = next_head;
        __sev();
    } else {
        pdm_ring_overruns++;
    }
}

// ----------------------------------------------------------------------------
// PDM PROCESSING LOOP (extracted from former pdm_core1_entry)
// Runs sigma-delta modulation when core1_mode == CORE1_MODE_PDM
// ----------------------------------------------------------------------------
static void pdm_processing_loop() {
    int32_t local_pdm_err = 0;
    int32_t local_pdm_err2 = 0;
    uint32_t active_us_accumulator = 0;
    uint32_t sample_counter = 0;
    noise_shaper_t ns = {0};

    const int32_t TARGET_LEAD = 256;

    uint32_t local_pdm_write = 0;
    bool hw_running = false;  // Always go through restart path on fresh entry

    // Fade state
    uint32_t fade_in_pos = 0;
    uint32_t fade_out_pos = 0;
    int32_t fade_base_pcm = 0;  // Last pcm_val before fade-out

    while (core1_mode == CORE1_MODE_PDM || fade_out_pos > 0) {
        // ---- Enable/disable state machine ----
        if (!pdm_enabled && fade_out_pos == 0) {
            if (hw_running) {
                // Start fade-out instead of immediate stop
                fade_out_pos = PDM_FADE_IN_SAMPLES;
                continue;
            }
            __wfe();
            continue;
        }

        // Cancel fade-out if re-enabled
        if (pdm_enabled && fade_out_pos > 0) {
            fade_in_pos = PDM_FADE_IN_SAMPLES - fade_out_pos;
            fade_out_pos = 0;
        }

        if (!hw_running) {
            // Re-enable: reset state, refill silence, restart hardware
            for (int i = 0; i < PDM_DMA_BUFFER_SIZE; i++)
                pdm_dma_buffer[i] = 0xAAAAAAAA;
            pdm_tail = pdm_head;  // drain ring buffer
            local_pdm_err = 0;
            local_pdm_err2 = 0;
            ns = (noise_shaper_t){0};
            active_us_accumulator = 0;
            sample_counter = 0;
            pdm_load_q8 = 0;
            fade_in_pos = 0;
            fade_out_pos = 0;
            fade_base_pcm = 0;

            pio_sm_set_enabled(PDM_PIO, PDM_SM, true);

            dma_channel_config dmac = dma_channel_get_default_config(pdm_dma_chan);
            channel_config_set_transfer_data_size(&dmac, DMA_SIZE_32);
            channel_config_set_read_increment(&dmac, true);
            channel_config_set_write_increment(&dmac, false);
            channel_config_set_dreq(&dmac, pio_get_dreq(PDM_PIO, PDM_SM, true));
            channel_config_set_ring(&dmac, false, PDM_DMA_RING_BITS);
            dma_channel_configure(pdm_dma_chan, &dmac, &PDM_PIO->txf[PDM_SM],
                                  pdm_dma_buffer, 0xFFFFFFFF, true);

            uint32_t ra = dma_hw->ch[pdm_dma_chan].read_addr;
            uint32_t ri = (ra - (uint32_t)pdm_dma_buffer) / 4;
            local_pdm_write = (ri + TARGET_LEAD) & (PDM_DMA_BUFFER_SIZE - 1);

            hw_running = true;
        }

        int32_t sample_value;

        // Check buffer position relative to DMA read pointer
        uint32_t read_addr = dma_hw->ch[pdm_dma_chan].read_addr;
        uint32_t current_read_idx = (read_addr - (uint32_t)pdm_dma_buffer) / 4;
        int32_t delta = (local_pdm_write - current_read_idx) & (PDM_DMA_BUFFER_SIZE - 1);

        // Underrun recovery - write pointer fell behind read pointer
        if (delta > (PDM_DMA_BUFFER_SIZE / 2)) {
            pdm_dma_underruns++;
            local_pdm_err = 0;
            local_pdm_err2 = 0;
            local_pdm_write = (current_read_idx + TARGET_LEAD) & (PDM_DMA_BUFFER_SIZE - 1);
            delta = TARGET_LEAD;
        }

        bool have_sample = (pdm_head != pdm_tail);

        if (have_sample) {
            pdm_msg_t msg = pdm_ring[pdm_tail];
            pdm_tail++;
            sample_value = msg.sample;
        } else if (delta < TARGET_LEAD) {
            pdm_ring_underruns++;
            sample_value = 0;
        } else {
            while (pdm_head == pdm_tail) {
                read_addr = dma_hw->ch[pdm_dma_chan].read_addr;
                current_read_idx = (read_addr - (uint32_t)pdm_dma_buffer) / 4;
                delta = (local_pdm_write - current_read_idx) & (PDM_DMA_BUFFER_SIZE - 1);

                if (delta < TARGET_LEAD || delta > (PDM_DMA_BUFFER_SIZE / 2)) break;

                if (delta > TARGET_LEAD + 128) {
                    __wfe();
                }
            }

            if (pdm_head != pdm_tail) {
                pdm_msg_t msg = pdm_ring[pdm_tail];
                pdm_tail++;
                sample_value = msg.sample;
            } else {
                sample_value = 0;
            }
        }

        uint32_t start_time = time_us_32();

        int32_t target;

        if (fade_out_pos > 0) {
            // Audio fade-out: ramp held audio level to silence
            pdm_tail = pdm_head;  // drain ring buffer (Core 0 stopped pushing)
            fade_out_pos--;
            target = ((fade_base_pcm * (int32_t)fade_out_pos) >> PDM_FADE_IN_SHIFT) + 32768;

            if (fade_out_pos == 0) {
                // Fade-out complete — stop hardware
                pio_sm_set_enabled(PDM_PIO, PDM_SM, false);
                dma_channel_abort(pdm_dma_chan);
                hw_running = false;
                global_status.cpu1_load = 0;
                continue;
            }

            // DMA throttle — no ring buffer pacing during fade
            while (1) {
                uint32_t ra = dma_hw->ch[pdm_dma_chan].read_addr;
                uint32_t ri = (ra - (uint32_t)pdm_dma_buffer) / 4;
                int32_t d = (local_pdm_write - ri) & (PDM_DMA_BUFFER_SIZE - 1);
                if (d > (PDM_DMA_BUFFER_SIZE / 2)) {
                    local_pdm_write = (ri + TARGET_LEAD) & (PDM_DMA_BUFFER_SIZE - 1);
                    break;
                }
                if (d < TARGET_LEAD + 16) break;
            }
        } else {
            // Normal operation (including audio fade-in)
            // Input Hard Limiter
            int32_t pcm_val = (sample_value >> 14);
            if (pcm_val > PDM_CLIP_THRESH) pcm_val = PDM_CLIP_THRESH;
            if (pcm_val < -PDM_CLIP_THRESH) pcm_val = -PDM_CLIP_THRESH;

            // Audio fade-in
            if (fade_in_pos < PDM_FADE_IN_SAMPLES) {
                pcm_val = (pcm_val * (int32_t)fade_in_pos) >> PDM_FADE_IN_SHIFT;
                fade_in_pos++;
            }

            fade_base_pcm = pcm_val;
            target = pcm_val + 32768;
        }

        // 256x Oversampling with 2nd-order sigma-delta modulator
        for (int chunk = 0; chunk < 8; chunk++) {
            int32_t raw_rand = (int32_t)(fast_rand() & PDM_DITHER_MASK) - (PDM_DITHER_MASK >> 1);
            int32_t dither = noise_shaped_dither(&ns, raw_rand, local_pdm_err2 >> 8);

            uint32_t pdm_word = 0;
            for (int k = 0; k < 32; k++) {
                int32_t fb_val = ((local_pdm_err2 + dither) >= 0) ? 65535 : 0;
                if ((local_pdm_err2 + dither) >= 0) pdm_word |= (1u << (31 - k));

                local_pdm_err += (target - fb_val);
                local_pdm_err2 += (local_pdm_err - fb_val);
            }

            pdm_dma_buffer[local_pdm_write] = pdm_word;
            local_pdm_write = (local_pdm_write + 1) & (PDM_DMA_BUFFER_SIZE - 1);
        }

        // Check for overrun after writing
        {
            uint32_t new_read_addr = dma_hw->ch[pdm_dma_chan].read_addr;
            uint32_t new_read_idx = (new_read_addr - (uint32_t)pdm_dma_buffer) / 4;
            int32_t new_delta = (local_pdm_write - new_read_idx) & (PDM_DMA_BUFFER_SIZE - 1);
            if (new_delta < 32) {
                pdm_dma_overruns++;
            }
        }

        // Leaky integrators
        local_pdm_err  -= (local_pdm_err >> PDM_LEAKAGE_SHIFT);
        local_pdm_err2 -= (local_pdm_err2 >> PDM_LEAKAGE_SHIFT);

        uint32_t end_time = time_us_32();
        active_us_accumulator += (end_time - start_time);
        sample_counter++;

        if (sample_counter >= 48) {
            uint32_t inst_q8 = (active_us_accumulator * 26);  // ~25.6, +1.5% conservative
            if (inst_q8 > 25600) inst_q8 = 25600;
            pdm_load_q8 = pdm_load_q8 - (pdm_load_q8 >> 3) + (inst_q8 >> 3);
            global_status.cpu1_load = (uint8_t)((pdm_load_q8 + 128) >> 8);
            active_us_accumulator = 0;
            sample_counter = 0;
        }
    }

    // Exiting PDM mode — clean up hardware
    if (hw_running) {
        pio_sm_set_enabled(PDM_PIO, PDM_SM, false);
        dma_channel_abort(pdm_dma_chan);
        hw_running = false;
    }
    global_status.cpu1_load = 0;
}

// ----------------------------------------------------------------------------
// EQ WORKER LOOP (RP2350 only — requires DCP and block-based EQ)
// Processes EQ + gain for outputs CORE1_EQ_FIRST_OUTPUT..CORE1_EQ_LAST_OUTPUT
// in parallel with Core 0 processing outputs 0-1
// ----------------------------------------------------------------------------
#if PICO_RP2350
static void __not_in_flash_func(eq_worker_loop)() {
    c1eq_load_primed = false;
    c1eq_load_q8 = 0;

    while (core1_mode == CORE1_MODE_EQ_WORKER) {
        // Wait for work from Core 0
        while (!core1_eq_work.work_ready) {
            if (core1_mode != CORE1_MODE_EQ_WORKER) return;
            __wfe();
        }
        __dmb();

        uint32_t work_start = time_us_32();

        // Read work descriptor
        float (*buf_out)[192] = core1_eq_work.buf_out;
        uint32_t sample_count = core1_eq_work.sample_count;
        float vol_mul = core1_eq_work.vol_mul;

        // Process EQ + gain for outputs assigned to Core 1
        extern MatrixMixer matrix_mixer;
        for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;

            // Output EQ
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }

            // Combined gain + volume
            float gain = matrix_mixer.outputs[out].mute ? 0.0f
                         : matrix_mixer.outputs[out].gain_linear * vol_mul;
            if (gain == 0.0f) {
                memset(buf_out[out], 0, sample_count * sizeof(float));
            } else if (gain != 1.0f) {
                float *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] *= gain;
            }
        }

        // Delay for Core 1 outputs
        if (any_delay_active) {
            for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                float *dst = buf_out[out];
                float *dline = delay_lines[out];
                uint32_t widx = core1_eq_work.delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // S/PDIF conversion for pairs 1-3
        for (int p = 0; p < 3; p++) {
            int32_t *out_ptr = core1_eq_work.spdif_out[p];
            if (!out_ptr) continue;
            int left_out = CORE1_EQ_FIRST_OUTPUT + p * 2;
            int right_out = left_out + 1;
            if (!matrix_mixer.outputs[left_out].enabled &&
                !matrix_mixer.outputs[right_out].enabled) {
                memset(out_ptr, 0, sample_count * 8);
                continue;
            }
            for (uint32_t i = 0; i < sample_count; i++) {
                float dl = fmaxf(-1.0f, fminf(1.0f, buf_out[left_out][i]));
                float dr = fmaxf(-1.0f, fminf(1.0f, buf_out[right_out][i]));
                out_ptr[i*2]   = (int32_t)(dl * 8388607.0f);
                out_ptr[i*2+1] = (int32_t)(dr * 8388607.0f);
            }
        }

        uint32_t work_end = time_us_32();

        if (c1eq_load_primed) {
            uint32_t busy_us = work_end - work_start;
            uint32_t idle_us = work_start - c1eq_last_work_end;
            if (idle_us > 2000) idle_us = 0;

            uint32_t total_us = busy_us + idle_us;
            if (total_us > 0) {
                uint32_t inst_q8 = (busy_us * 25600) / total_us;
                c1eq_load_q8 = c1eq_load_q8 - (c1eq_load_q8 >> 3) + (inst_q8 >> 3);
            }
            global_status.cpu1_load = (uint8_t)((c1eq_load_q8 + 128) >> 8);
        } else {
            c1eq_load_primed = true;
        }
        c1eq_last_work_end = work_end;

        // Signal completion to Core 0
        core1_eq_work.work_ready = false;
        __dmb();
        core1_eq_work.work_done = true;
        __sev();
    }

    global_status.cpu1_load = 0;
}
#else
// ----------------------------------------------------------------------------
// EQ WORKER LOOP (RP2040 — block-based Q28 fixed-point)
// Processes EQ + gain + delay + SPDIF for outputs CORE1_EQ_FIRST_OUTPUT..CORE1_EQ_LAST_OUTPUT
// in parallel with Core 0 processing outputs 0-1
// ----------------------------------------------------------------------------
static void __not_in_flash_func(eq_worker_loop)() {
    c1eq_load_primed = false;
    c1eq_load_q8 = 0;

    while (core1_mode == CORE1_MODE_EQ_WORKER) {
        // Wait for work from Core 0
        while (!core1_eq_work.work_ready) {
            if (core1_mode != CORE1_MODE_EQ_WORKER) return;
            __wfe();
        }
        __dmb();

        uint32_t work_start = time_us_32();

        // Read work descriptor
        int32_t (*buf_out)[192] = core1_eq_work.buf_out;
        uint32_t sample_count = core1_eq_work.sample_count;
        int32_t vol_mul = core1_eq_work.vol_mul;
        bool is_bypassed = bypass_master_eq;

        // Process EQ + gain for outputs assigned to Core 1
        extern MatrixMixer matrix_mixer;
        for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
            if (!matrix_mixer.outputs[out].enabled) continue;

            // Output EQ (block-based)
            if (!matrix_mixer.outputs[out].mute) {
                uint8_t eq_ch = CH_OUT_1 + out;
                if (!is_bypassed && !channel_bypassed[eq_ch]) {
                    dsp_process_channel_block(filters[eq_ch], buf_out[out], sample_count, eq_ch);
                }
            }

            // Combined gain + volume (Q15)
            int32_t gain = matrix_mixer.outputs[out].mute ? 0
                           : (int32_t)(matrix_mixer.outputs[out].gain_linear * (float)vol_mul);
            if (gain == 0) {
                memset(buf_out[out], 0, sample_count * sizeof(int32_t));
            } else {
                int32_t *dst = buf_out[out];
                for (uint32_t i = 0; i < sample_count; i++)
                    dst[i] = fast_mul_q15(dst[i], gain);
            }
        }

        // Delay for Core 1 outputs
        if (any_delay_active) {
            for (int out = CORE1_EQ_FIRST_OUTPUT; out <= CORE1_EQ_LAST_OUTPUT; out++) {
                int32_t dly = channel_delay_samples[out];
                if (dly <= 0) continue;
                int32_t *dst = buf_out[out];
                int32_t *dline = delay_lines[out];
                uint32_t widx = core1_eq_work.delay_write_idx;
                for (uint32_t i = 0; i < sample_count; i++) {
                    dline[widx] = dst[i];
                    dst[i] = dline[(widx - dly) & MAX_DELAY_MASK];
                    widx = (widx + 1) & MAX_DELAY_MASK;
                }
            }
        }

        // S/PDIF conversion for Core 1's pair (outputs 2-3 → int32 24-bit)
        {
            int32_t *out_ptr = core1_eq_work.spdif_out[0];
            if (out_ptr) {
                int left_out = CORE1_EQ_FIRST_OUTPUT;
                int right_out = CORE1_EQ_FIRST_OUTPUT + 1;
                if (!matrix_mixer.outputs[left_out].enabled &&
                    !matrix_mixer.outputs[right_out].enabled) {
                    memset(out_ptr, 0, sample_count * 8);
                } else {
                    for (uint32_t i = 0; i < sample_count; i++) {
                        out_ptr[i*2]   = clip_s24((buf_out[left_out][i] + (1 << 5)) >> 6);
                        out_ptr[i*2+1] = clip_s24((buf_out[right_out][i] + (1 << 5)) >> 6);
                    }
                }
            }
        }

        uint32_t work_end = time_us_32();

        if (c1eq_load_primed) {
            uint32_t busy_us = work_end - work_start;
            uint32_t idle_us = work_start - c1eq_last_work_end;
            if (idle_us > 2000) idle_us = 0;

            uint32_t total_us = busy_us + idle_us;
            if (total_us > 0) {
                uint32_t inst_q8 = (busy_us * 25600) / total_us;
                c1eq_load_q8 = c1eq_load_q8 - (c1eq_load_q8 >> 3) + (inst_q8 >> 3);
            }
            global_status.cpu1_load = (uint8_t)((c1eq_load_q8 + 128) >> 8);
        } else {
            c1eq_load_primed = true;
        }
        c1eq_last_work_end = work_end;

        // Signal completion to Core 0
        core1_eq_work.work_ready = false;
        __dmb();
        core1_eq_work.work_done = true;
        __sev();
    }

    global_status.cpu1_load = 0;
}
#endif

// ----------------------------------------------------------------------------
// CORE 1 ENTRY — mode dispatcher
// ----------------------------------------------------------------------------
void pdm_core1_entry() {
#if PICO_RP2350
    // Enable FTZ+DN in FPDSCR for Core 1 IRQ handlers (same reason as Core 0)
    {
        volatile uint32_t *fpdscr = (volatile uint32_t *)0xE000EF3C;
        *fpdscr |= (1u << 24) | (1u << 25);  // FZ (bit 24) + DN (bit 25)
    }
#endif

    while (1) {
        switch (core1_mode) {
            case CORE1_MODE_PDM:
                pdm_processing_loop();
                break;
            case CORE1_MODE_EQ_WORKER:
                eq_worker_loop();
                break;
            default:
                global_status.cpu1_load = 0;
                __wfe();
                break;
        }
    }
}
