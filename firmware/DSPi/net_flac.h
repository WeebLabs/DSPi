/*
 * net_flac.h - FLAC decode stage of the Sendspin NET input (phase 2).
 *
 * Sits between sendspin_client.c (which owns the TCP/Noise record layer) and
 * the NET PCM ring (net_input.c).  The libfoxenflac decoder is push-style and
 * heap-free; everything it needs lives in ONE borrowed arena (see below).
 *
 * Data flow for one audio chunk (a Sendspin binary record, id 4):
 *
 *   TCP --streaming ChaCha20 decrypt--> nfl_chunk_buf()      (compressed bytes,
 *        NOT yet trusted)                                      never decoded yet)
 *   Poly1305 tag verifies --> nfl_chunk_submit(len)          (chunk is "pending")
 *   nfl_pump() (every sendspin_client_poll pass, work-bounded)
 *        --> fx_flac_process() --> interleave to S16/S24 --> net_input_stage_pcm
 *            + net_input_commit_pcm                           (only ring space that
 *                                                              is really free)
 *
 * so decoded audio is published only after the record authenticated, and a
 * full ring merely stalls the pump (the chunk stays pending; the record layer
 * stops consuming TCP data, which closes the window) - nothing is dropped or
 * reordered because of backpressure.
 *
 * ------------------------------------------------------------------------
 * MEMORY: borrowed ADAT/TDM output ring (NO new .bss, NO heap)
 * ------------------------------------------------------------------------
 * The decoder needs ~37 KB (fx_flac_size(4608, 2): 2 x 4608 x int32 block
 * buffers) plus a compressed-chunk buffer, far more than any of the small idle
 * buffers (SPDIF RX fifo 12 KB, ADAT RX ring 8 KB, flash write buffer 8 KB,
 * PDM 10 KB) and those are not contiguous.  The one contiguous idle region of
 * that size is `adat_ring` (adat_output.c, 56 KB), which is only touched while
 * the ADAT/TDM bulk output is enabled.  It is therefore LOANED:
 *
 *   - adat_output_ring_loan() succeeds only while the bulk output is neither
 *     enabled (config) nor running; a granted loan makes adat_output_resync()
 *     call nfl_evict_cb() BEFORE it would ever start the output, and the
 *     output stays off until the loan is returned;
 *   - the loan is taken at stream/start (codec flac), returned at stream/end,
 *     stream/start of another codec, connection loss and net_input_stop();
 *   - if the user enables ADAT/TDM output while a FLAC stream runs, the
 *     decoder is torn down synchronously (output has priority) and the client
 *     reconnects (nfl_take_evicted()); the next client/hello no longer lists
 *     flac (nfl_supported_now() is false while the bulk output is enabled),
 *     so the server falls back to PCM.
 *   Arena layout (16-byte aligned base):
 *     [ foxen decoder (fx_flac_size(NFL_MAX_BLOCK, 2)) | int32 out slice | compressed chunk buffer ]
 *   All of it lives only while a loan is held; nothing else in the firmware
 *   references adat_ring while the loan is out (adat_output.c: every access is
 *   gated on adat_running, which cannot become true during a loan).
 *
 * Accepted streams: FLAC Subset at 44.1/48 kHz, exactly 2 channels, 16 bit
 * (24 bit decodes too, but is not advertised, see sendspin_client.c), with
 * STREAMINFO max_blocksize <= NFL_MAX_BLOCK (4608 = the Subset limit for
 * <= 48 kHz) or unknown (0).  The largest compressed chunk is
 * nfl_chunk_cap() (~17 KB); larger chunks are authenticated and dropped.
 */
#ifndef NET_FLAC_H
#define NET_FLAC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NFL_MAX_BLOCK   4608u   /* max samples per channel per FLAC frame we can hold */

/* nfl_begin() results */
enum {
    NFL_OK = 0,
    NFL_ERR_NO_ARENA,     /* ADAT/TDM output enabled/running, or arena too small */
    NFL_ERR_HEADER,       /* codec_header missing/not 'fLaC' + STREAMINFO */
    NFL_ERR_FORMAT,       /* channels/rate/bit depth/block size unsupported or != stream/start */
    NFL_ERR_DECODER       /* foxen refused the STREAMINFO */
};

/* True when a flac stream could be started right now (loan available or held).
 * Used to decide whether client/hello lists flac. */
bool nfl_supported_now(void);

/* stream/start: borrow the arena, decode `b64` (standard Base64 codec_header:
 * 'fLaC' + STREAMINFO), initialise the decoder.  rate/bits come from
 * stream/start and must match STREAMINFO.  Returns NFL_OK or NFL_ERR_*; on error
 * the arena is not held. */
int nfl_begin(const char *b64, size_t b64_len, uint32_t rate, uint8_t bits);

/* stream/clear: drop the pending chunk and resynchronise the decoder. */
void nfl_reset(void);

/* stream/end, codec change, connection loss: drop everything, return the arena. */
void nfl_end(void);

bool nfl_active(void);

/* Compressed chunk buffer (valid while nfl_active() and !nfl_pending()). */
uint8_t *nfl_chunk_buf(void);
uint32_t nfl_chunk_cap(void);

/* The first `len` bytes of nfl_chunk_buf() are an AUTHENTICATED chunk of whole
 * FLAC frames; queue it for decoding.  Precondition: !nfl_pending(). */
void nfl_chunk_submit(uint32_t len);
bool nfl_pending(void);
uint32_t nfl_chunk_remaining(void);   /* undecoded bytes of the pending chunk */

/* Decode as much of the pending chunk as ring space and the per-call work
 * bound allow (<= NFL_MAX_SLICES slices / NFL_BUDGET_US, see net_flac.c).
 * Returns true while the chunk is still pending. */
bool nfl_pump(void);

/* Estimated duration of one chunk in microseconds (for the late-chunk check). */
uint32_t nfl_chunk_dur_us(void);

/* True once (then cleared) when the arena was reclaimed by the ADAT/TDM output
 * while a stream was active; the client must reconnect. */
bool nfl_take_evicted(void);

/* Diagnostics */
uint32_t nfl_stat_frames_dropped(void);   /* decoder resyncs (bad CRC / header) */

#ifdef __cplusplus
}
#endif
#endif
