#include "pdm_generator.h"
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
// FUNCTIONS
// ----------------------------------------------------------------------------

void pdm_update_clock(uint32_t freq) {
    float div = (float)clock_get_hz(clk_sys) / (float)(freq * PDM_OVERSAMPLE);
    pio_sm_set_clkdiv(PDM_PIO, PDM_SM, div);
}

void pdm_setup_hw(void) {
    uint offset = pio_add_program(PDM_PIO, &pio_pdm_program);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + (pio_pdm_program.length - 1));
    sm_config_set_out_pins(&c, PICO_AUDIO_SPDIF_SUB_PIN, 1);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    pio_gpio_init(PDM_PIO, PICO_AUDIO_SPDIF_SUB_PIN);
    pio_sm_set_consecutive_pindirs(PDM_PIO, PDM_SM, PICO_AUDIO_SPDIF_SUB_PIN, 1, true);
    pio_sm_init(PDM_PIO, PDM_SM, offset, &c);
    
    pdm_update_clock(48000);
    pio_sm_set_enabled(PDM_PIO, PDM_SM, true);
    
    pdm_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dmac = dma_channel_get_default_config(pdm_dma_chan);
    channel_config_set_transfer_data_size(&dmac, DMA_SIZE_32);
    channel_config_set_read_increment(&dmac, true);
    channel_config_set_write_increment(&dmac, false);
    channel_config_set_dreq(&dmac, pio_get_dreq(PDM_PIO, PDM_SM, true));
    channel_config_set_ring(&dmac, false, 12);
    dma_channel_configure(pdm_dma_chan, &dmac, &PDM_PIO->txf[PDM_SM], pdm_dma_buffer, 0xFFFFFFFF, true);
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
        overruns++; // Uses the extern from config.h/main.c
    }
}

void pdm_core1_entry() {
    int32_t local_pdm_err = 0;
    int32_t local_pdm_err2 = 0;
    uint32_t local_pdm_write = 0;
    uint32_t active_us_accumulator = 0;
    uint32_t sample_counter = 0;

    while (1) {
        while (pdm_head == pdm_tail) {
            __wfe();
        }
        
        pdm_msg_t msg = pdm_ring[pdm_tail];
        pdm_tail++; // Auto-wrap

        uint32_t start_time = timer_hw->timerawl;

        uint32_t read_addr = dma_hw->ch[pdm_dma_chan].read_addr;
        uint32_t current_read_idx = (read_addr - (uint32_t)pdm_dma_buffer) / 4;
        int32_t delta = (local_pdm_write - current_read_idx) & (PDM_DMA_BUFFER_SIZE - 1);
        if (delta > (PDM_DMA_BUFFER_SIZE / 2)) {
            local_pdm_write = (current_read_idx + 64) & (PDM_DMA_BUFFER_SIZE - 1);
        }

        if (msg.reset) {
            local_pdm_err = 0;
            local_pdm_err2 = 0;
            msg.sample = 0;
        }

        // TPDF Dither
        int32_t r1 = (int32_t)(fast_rand() & 2047);
        int32_t r2 = (int32_t)(fast_rand() & 2047);
        int32_t dither = r1 - r2;
        int32_t dithered_sample = msg.sample;

        // Input Hard Limiter
        int32_t pcm_val = (dithered_sample >> 14);
        if (pcm_val > PDM_CLIP_THRESH) pcm_val = PDM_CLIP_THRESH;
        if (pcm_val < -PDM_CLIP_THRESH) pcm_val = -PDM_CLIP_THRESH;

        int32_t target = pcm_val + 32768;
        
        // 256x Oversampling
        for (int chunk = 0; chunk < 8; chunk++) {
            uint32_t pdm_word = 0;
            for (int k = 0; k < 32; k++) {
                int32_t fb_val = (local_pdm_err2 >= 0) ? 65535 : 0;
                if (local_pdm_err2 >= 0) pdm_word |= (1u << (31 - k));
                
                local_pdm_err += (target - fb_val);
                local_pdm_err2 += (local_pdm_err - fb_val);
            }
            pdm_dma_buffer[local_pdm_write] = pdm_word;
            local_pdm_write = (local_pdm_write + 1) & (PDM_DMA_BUFFER_SIZE - 1);
        }

        uint32_t end_time = timer_hw->timerawl;
        active_us_accumulator += (end_time - start_time);
        sample_counter++;

        if (sample_counter >= 48) {
            global_status.cpu1_load = (uint8_t)(active_us_accumulator / 10);
            active_us_accumulator = 0;
            sample_counter = 0;
        }
    }
}
