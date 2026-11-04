/*
 *  libfoxenflac -- Tiny FLAC Decoder Library
 *  Copyright (C) 2018-2025  Andreas Stöckel
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file foxen-flac.h
 *
 * Provides a decoder for FLAC (Free Lossless Audio Codec).
 *
 * @author Andreas Stöckel
 */

#ifndef FOXEN_FLAC_H
#define FOXEN_FLAC_H

#include <stdint.h>

#ifndef FX_EXPORT
#if __EMSCRIPTEN__
#import <emscripten.h>
#define FX_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define FX_EXPORT
#endif /* __EMSCRIPTEN__ */
#endif /* FX_EXPORT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Value returned by the fx_flac_get_streaminfo() method if the given streaminfo
 * key is invalid.
 */
#define FLAC_INVALID_METADATA_KEY 0x7FFFFFFFFFFFFFFFULL

/**
 * Maximum number of channels that can be encoded in a FLAC stream.
 */
#define FLAC_MAX_CHANNEL_COUNT 8U

/**
 * Maximum block size that can be used if the stream is encoded in the FLAC
 * Subset format and the sample rate is smaller than 48000 kHz.
 */
#define FLAC_SUBSET_MAX_BLOCK_SIZE_48KHZ 4608U

/**
 * Maximum block size than can always be safely used if the stream is encoded
 * in the FLAC Subset format.
 */
#define FLAC_SUBSET_MAX_BLOCK_SIZE 16384U

/**
 * Maximum block size in samples that can be used in a FLAC stream.
 */
#define FLAC_MAX_BLOCK_SIZE 65535U

/**
 * Opaque struct representing a FLAC decoder.
 */
struct fx_flac;

/**
 * Typedef for the fx_flac struct.
 */
typedef struct fx_flac fx_flac_t;

/**
 * Enum representing the state of a FLAC decoder instance.
 */
typedef enum {
	/**
	 * The decoder is in an error state; the decoder cannot recover from this
	 * error. This error may for example occur if the data in the stream is
	 * invalid, or the stream has a format that is outside the maximum specs
	 * that are supported by the decoder. Call fx_flac_reset() and start anew!
	 */
	FLAC_ERR = -1,

	/**
	 * The decoder is currently in its initial state, fx_flac_process() has not
	 * been called.
	 */
	FLAC_INIT = 0,

	/**
	 * The decoder found the beginning of the metadata packet!
	 */
	FLAC_IN_METADATA = 1,

	/**
	 * The decoder is done reading the current metadata block, this may be
	 * followed by more metadata blocks, in which case the state is reset to
	 * FLAC_IN_METADATA.
	 */
	FLAC_END_OF_METADATA = 2,

	/**
	 * The decoder is currently searching for an audio frame.
	 */
	FLAC_SEARCH_FRAME = 3,

	/**
	 * The decoder is currently inside the stream of audio frames.
	 */
	FLAC_IN_FRAME = 4,

	/**
	 * The decoder successfully decoded an entire frame. Write the data to the
	 * client.
	 */
	FLAC_DECODED_FRAME = 5,

	/**
	 * The decoder reached the end of a block.
	 */
	FLAC_END_OF_FRAME = 6
} fx_flac_state_t;

/**
 * Enum used in fx_flac_get_streaminfo() to query metadata about the stream.
 */
typedef enum {
	FLAC_KEY_MIN_BLOCK_SIZE = 0,
	FLAC_KEY_MAX_BLOCK_SIZE = 1,
	FLAC_KEY_MIN_FRAME_SIZE = 2,
	FLAC_KEY_MAX_FRAME_SIZE = 3,
	FLAC_KEY_SAMPLE_RATE = 4,
	FLAC_KEY_N_CHANNELS = 5,
	FLAC_KEY_SAMPLE_SIZE = 6,
	FLAC_KEY_N_SAMPLES = 7,
	FLAC_KEY_MD5_SUM_0 = 128,
	FLAC_KEY_MD5_SUM_1 = 129,
	FLAC_KEY_MD5_SUM_2 = 130,
	FLAC_KEY_MD5_SUM_3 = 131,
	FLAC_KEY_MD5_SUM_4 = 132,
	FLAC_KEY_MD5_SUM_5 = 133,
	FLAC_KEY_MD5_SUM_6 = 134,
	FLAC_KEY_MD5_SUM_7 = 135,
	FLAC_KEY_MD5_SUM_8 = 136,
	FLAC_KEY_MD5_SUM_9 = 137,
	FLAC_KEY_MD5_SUM_A = 138,
	FLAC_KEY_MD5_SUM_B = 139,
	FLAC_KEY_MD5_SUM_C = 140,
	FLAC_KEY_MD5_SUM_D = 141,
	FLAC_KEY_MD5_SUM_E = 142,
	FLAC_KEY_MD5_SUM_F = 143,
} fx_flac_streaminfo_key_t;

/**
 * Returns the size of the FLAC decoder instance in bytes. This assumes that the
 * FLAC audio that is being decoded uses the maximum settings, i.e. the largest
 * bit depth and block size. See fx_flac_init() regarding parameters.
 *
 * @return zero if the given parameters are out of range, the number of bytes
 * required to hold the FLAC decoder structure otherwise.
 */
FX_EXPORT uint32_t fx_flac_size(uint32_t max_block_size, uint8_t max_channels);

/**
 * Initializes the FLAC decoder at the given memory location. Each decoder can
 * decode exactly one stream at a time.
 *
 * @param mem is a pointer at the memory region at which the FLAC decoder should
 * store its private data. The memory region must be at last as large as
 * indicated by fx_flac_size(). May be NULL, in which case NULL is returned.
 * @param max_block_size is the maximum block size for which the FLAC instance
 * will provide a buffer. For streams in the Subset format (which is used per
 * default in most FLAC encoders), max_block_size should can be set to 4608 if
 * the sample rate is <= 48000kHz, otherwise, for larger sample rates,
 * max_block_size must be set to 16384.
 * @param max_channels is the maximum number of channels that will be decoded.
 * @return a pointer at the FLAC decoder instance; note that this pointer may be
 * different from what was passed to mem. However, you may still pass the
 * original `mem` as `inst` parameter to other functions. Returns NULL if the
 * input pointer is NULL or the given parameters are invalid.
 */
FX_EXPORT fx_flac_t *fx_flac_init(void *mem, uint16_t max_block_size,
                                  uint8_t max_channels);

/**
 * Macro which calls malloc to allocate memory for a new fx_flac instance. The
 * returned pointer must be freed using free. Returns NULL if the allocation
 * fails or the given parameters are invalid.
 *
 * Note that this code is implemented as a macro to prevent explicitly having
 * a dependency on malloc while still providing a convenient allocation routine.
 */
#define FX_FLAC_ALLOC(max_block_size, max_channels)                            \
	(fx_flac_size((max_block_size), (max_channels)) == 0U)                     \
	    ? NULL                                                                 \
	    : fx_flac_init(malloc(fx_flac_size((max_block_size), (max_channels))), \
	                   (max_block_size), (max_channels))

/**
 * Returns a new fx_flac instance that is sufficient to decode FLAC streams in
 * the FLAC Subset format with DAT parameters, i.e. up to 48 kHz, and two
 * channels. This will allocate about 40 kiB of memory.
 */
#define FX_FLAC_ALLOC_SUBSET_FORMAT_DAT() \
	FX_FLAC_ALLOC(FLAC_SUBSET_MAX_BLOCK_SIZE_48KHZ, 2U)

/**
 * Returns a new fx_flac instance that is sufficient to decode FLAC streams in
 * the FLAC Subset format. This will allocate about 1.5 MiB of memory.
 */
#define FX_FLAC_ALLOC_SUBSET_FORMAT_ANY() \
	FX_FLAC_ALLOC(FLAC_SUBSET_MAX_BLOCK_SIZE, FLAC_MAX_CHANNEL_COUNT)

/**
 * Returns a new fx_flac instance that is sufficient to decode any valid FLAC
 * stream. Note that this will allocate between 2-3 MiB of memory.
 */
#define FX_FLAC_ALLOC_DEFAULT() \
	FX_FLAC_ALLOC(FLAC_MAX_BLOCK_SIZE, FLAC_MAX_CHANNEL_COUNT)

/**
 * Resets the FLAC decoder.
 *
 * @param inst is the FLAC decoder that should be reset.
 */
FX_EXPORT void fx_flac_reset(fx_flac_t *inst);

/**
 * Returns the current decoder state.
 *
 * @param inst is the FLAC decoder instance for which the state should be
 * returned.
 * @return the current state of the decoder.
 */
FX_EXPORT fx_flac_state_t fx_flac_get_state(const fx_flac_t *inst);

/**
 * Returns metadata about the FLAC stream that is currently being parsed. This
 * function may only be called if the decoder is in the state
 * FLAC_END_OF_METADATA or greater, otherwise the result may be undefined
 * (it will likely return zero for most of the metadata keys).
 *
 * @param inst is a pointer at the FLAC decoder instance for which the metadata
 * should be retrieved.
 * @param key is the metadata that should be retrieved.
 * @return the requested metadata value or FLAC_INVALID_METADATA_KEY if the
 * given key is unknown.
 */
FX_EXPORT int64_t fx_flac_get_streaminfo(const fx_flac_t *inst,
                                         fx_flac_streaminfo_key_t key);

/**
 * Decodes the given raw FLAC data; the given data must be RAW FLAC data as
 * specified in the FLAC format specification https://xiph.org/flac/format.html
 * This function will always return right after the decoder transitions to a new
 * relevant state.
 *
 * @param inst is the decoder instance.
 * @param in is a pointer at the encoded bytestream.
 * @param in_len is a pointer at a integer containing the number of valid bytes
 * in "in". After the function returns, in will contain the number of bytes that
 * were actually read. This number may be zero if the decoder is in the FLAC_ERR
 * or FLAC_STREAM_DONE state, or the internal buffers are full and need to be
 * flushed to the provided output first.
 * @param out is a pointer at a memory region that will accept the decoded
 * interleaved audio data. Samples are decoded as 32-bit signed integer; the
 * minimum and maximum value will depend on the original bit depth of the audio
 * stored in the bitstream. If this is NULL, the decoder will silently discard
 * the output.
 * @param out_len is a pointer at an integer containing the number of available
 * signed 32-bit integers at the memory address pointed at by out. After the
 * function returns, this value will contain the number of samples that were
 * written. If this is NULL, the deocder will silently discard the output.
 * @return the current state of the decoder. If the state transitions to
 * FLAC_END_OF_METADATA, FLAC_END_OF_FRAME or FLAC_END_OF_STREAM this function
 * will return immediately; only the data up to the point causing the transition
 * has been read.
 */
FX_EXPORT fx_flac_state_t fx_flac_process(fx_flac_t *inst, const uint8_t *in,
                                          uint32_t *in_len, int32_t *out,
                                          uint32_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* FOXEN_FLAC_H */
