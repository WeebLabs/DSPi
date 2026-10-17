/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include "pico/time.h"
#include "pico/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PICO_SPDIF_RX_DMA_IRQ
#define PICO_SPDIF_RX_DMA_IRQ 0
#endif

#ifndef PICO_SPDIF_RX_PIO
#define PICO_SPDIF_RX_PIO 0
#endif

// PICO_CONFIG: SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK, Spinlock number for the audio prepared list, min=0, max=31, default=8, group=audio
#ifndef SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK
#define SPINLOCK_ID_SPDIF_RX_PREPARED_LISTS_LOCK 12
#endif

#if !(PICO_SPDIF_RX_DMA_IRQ == 0 || PICO_SPDIF_RX_DMA_IRQ == 1)
#error PICO_SPDIF_RX_DMA_IRQ must be 0 or 1
#endif

#if PICO_RP2350
  #if !(PICO_SPDIF_RX_PIO == 0 || PICO_SPDIF_RX_PIO == 1 || PICO_SPDIF_RX_PIO == 2)
  #error PICO_SPDIF_RX_PIO must be 0, 1, or 2
  #endif
#else
  #if !(PICO_SPDIF_RX_PIO == 0 || PICO_SPDIF_RX_PIO == 1)
  #error PICO_SPDIF_RX_PIO must be 0 or 1
  #endif
#endif

// DSPi: sys_clk 307.2 MHz, PIO clock 122.88 MHz (div = 2.5). Cycles per symbol
// at that clock, one decoder program each: 44.1k 21.77 (cy 22), 48k 20 (cy 20),
// 88.2k 10.88 (cy 11), 96k 10 (cy 10), 192k 5 (cy 5, also used for 176.4k).
#ifndef SPDIF_RX_SYS_CLK_FREQ
#define SPDIF_RX_SYS_CLK_FREQ (307200000)
#endif
#ifndef SPDIF_RX_PIO_CLK_FREQ
#define SPDIF_RX_PIO_CLK_FREQ (122880000)
#endif

#define SPDIF_RX_FLAGS_NONE         (0)
#define SPDIF_RX_FLAG_CHECK_PARITY  (1<<0)
#define SPDIF_RX_FLAG_C_BITS        (1<<1)
#define SPDIF_RX_FLAG_CALLBACKS     (1<<2)
#define SPDIF_RX_FLAGS_ALL          (SPDIF_RX_FLAG_CHECK_PARITY | SPDIF_RX_FLAG_C_BITS | SPDIF_RX_FLAG_CALLBACKS)

/** spdif_rx_config_t */
typedef struct _spdif_rx_config_t {
    uint8_t data_pin;           /**< data pin for SPDIF Rx */
    uint pio_sm;                /**< PIO state mechine resource id */
    uint dma_channel0;          /**< dma 0 resource id */
    uint dma_channel1;          /**< dma 1 resource id */
    alarm_pool_t* alarm_pool;   /**< alarm pool pointer */
    uint8_t flags;              /**< flags for functions: set SPDIF_RX_FLAGS_ALL or SPDIF_RX_FLAGS_NONE, otherwise set combination of SPDIF_RX_FLAG_xxx */
} spdif_rx_config_t;

/** spdif_rx_samp_freq_t  */
typedef enum _spdif_rx_samp_freq_t {
    SAMP_FREQ_NONE = 0,        /**< Not valid */
    SAMP_FREQ_44100 = 44100,   /**< 44.1 KHz */
    SAMP_FREQ_48000 = 48000,   /**< 48.0 KHz */
    SAMP_FREQ_88200 = 88200,   /**< 88.2 KHz */
    SAMP_FREQ_96000 = 96000,   /**< 96.0 KHz */
    SAMP_FREQ_176400 = 176400, /**< 176.4 KHz */
    SAMP_FREQ_192000 = 192000  /**< 192.0 KHz */
} spdif_rx_samp_freq_t;

/** spdif_rx_state_t  */
typedef enum _spdif_rx_state_t  {
    SPDIF_RX_STATE_NO_SIGNAL = 0,  /**< No Signal */
    SPDIF_RX_STATE_WAITING_STABLE, /**< Frequency detected. Waiting for stable sync */
    SPDIF_RX_STATE_STABLE          /**< Got stable sync */
} spdif_rx_state_t;

#define SPDIF_BLOCK_SIZE (384) // sub frames per block
#define NUM_BLOCKS (8) // must be >= 2
#define SPDIF_RX_FIFO_SIZE (NUM_BLOCKS * SPDIF_BLOCK_SIZE)

/**
* start S/PDIF receiver
*
* @param[in] config configulation by spdif_rx_config_t
*/
void spdif_rx_start(const spdif_rx_config_t* config);

/**
* end S/PDIF receiver
*/
void spdif_rx_end();

/**
* set on_stable callback function
*
* @param[in] func callback function pointer on stable
*/
void spdif_rx_set_callback_on_stable(void (*func)(spdif_rx_samp_freq_t samp_freq));

/**
* set on_lost_stable callback function
*
* @param[in] func callback function pointer on lost stable
*/
void spdif_rx_set_callback_on_lost_stable(void (*func)());

/**
* get status
*
* @return spdif_rx_state_t SPDIF_RX_STATE_NO_SIGNAL or SPDIF_RX_STATE_WAITING_STABLE or SPDIF_RX_STATE_STABLE
*/
spdif_rx_state_t spdif_rx_get_state();

/**
* get actual sampling frequency
*
* @return float actual sampling frequency
*/
float spdif_rx_get_samp_freq_actual();

/**
* DSPi patch: coherent {decoded block count, completing IRQ time} pair; each
* stable block is SPDIF_BLOCK_SIZE / 2 frames. Seqlock read, thread context.
*
* @param[out] block_cnt decoded-block count
* @param[out] time_us IRQ entry timestamp (us since boot) pairing block_cnt
*/
void spdif_rx_get_block_ref(uint32_t* block_cnt, uint64_t* time_us);

/**
* DSPi patch: fifo count including in-flight words; 0 unless STABLE.
*
* @return uint32_t fifo count in 32bit words (2 words per sample for stereo)
*/
uint32_t spdif_rx_get_fifo_count_live();

/**
* DSPi patch: spdif_rx_read_fifo that may consume into the in-flight block
* (sync/parity not yet verified). Returns 0 words unless STABLE.
*
* @param[out] buff pointer to get fifo address to read
* @param[in] req_count 32bit word count to request
* @return uint32_t 32bit word count actually read
*/
uint32_t spdif_rx_read_fifo_live(uint32_t** buff, uint32_t req_count);

/**
* get sampling frequency
*
* @return spdif_rx_samp_freq_t SAMP_FREQ_NONE, SAMP_FREQ_44100, SAMP_FREQ_48000, SAMP_FREQ_88200, SAMP_FREQ_96000, SAMP_FREQ_176400 or SAMP_FREQ_192000 
*/
spdif_rx_samp_freq_t spdif_rx_get_samp_freq();

/**
* get C bits
*
* @param[out] ptr pointer to get C bits
* @param[in] size byte size to get C bits (1 ~ 24)
* @param[in] offset C bits byte offset to get (0 ~ 23)
*/
void spdif_rx_get_c_bits(void* ptr, size_t size, uint32_t offset);

/**
* get number of parity errors
*
* @return uint32_t number of parity errors after getting stable
*/
uint32_t spdif_rx_get_parity_err_count();

/**
* get FIFO count
*
* @return uint32_t word (32bit) count which can read from FIFO
*/
uint32_t spdif_rx_get_fifo_count();

/**
* read FIFO by buffer
*
* @param[out] buff pointer to buffer reference
* @param[in] req_count word (32bit) count to request to read
* @return uint32_t actual read word (32bit) count
*/
uint32_t spdif_rx_read_fifo(uint32_t** buff, uint32_t req_count);

#ifdef __cplusplus
}
#endif