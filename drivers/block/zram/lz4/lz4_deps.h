#ifndef ZRAM_LZ4_DEPS_H
#define ZRAM_LZ4_DEPS_H

#include <linux/compiler.h>
#include <linux/string.h>
#include <linux/types.h>

#undef current

#define LZ4LIB_VISIBILITY static __maybe_unused
#define LZ4_PUBLISH_STATIC_FUNCTIONS
#define LZ4_STATIC_LINKING_ONLY
#define LZ4_DISABLE_DEPRECATE_WARNINGS 1
#define LZ4_USER_MEMORY_FUNCTIONS
#define LZ4_HEAPMODE 1
#define LZ4HC_HEAPMODE 1
#define LZ4_FORCE_NOINLINE static noinline

static __maybe_unused void *LZ4_malloc(size_t size)
{
	return NULL;
}

static __maybe_unused void *LZ4_calloc(size_t count, size_t size)
{
	return NULL;
}

static __maybe_unused void LZ4_free(void *pointer)
{
}

#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
#define LZ4_FORCE_MEMORY_ACCESS 0
#else
#define LZ4_FORCE_MEMORY_ACCESS 1
#endif

static __maybe_unused int LZ4_compress_destSize_extState(void *state,
		const char *source, char *dest, int *source_size,
		int target_dest_size, int acceleration);

#include "upstream/lz4.h"

static __maybe_unused int LZ4_compress_forceExtDict(LZ4_stream_t *stream,
		const char *source, char *dest, int source_size);
static __maybe_unused int LZ4_decompress_safe_forceExtDict(const char *source,
		char *dest, int compressed_size, int max_output_size,
		const void *dict_start, size_t dict_size);
static __maybe_unused int LZ4_decompress_safe_partial_forceExtDict(
		const char *source, char *dest, int compressed_size,
		int target_output_size, int dest_capacity,
		const void *dict_start, size_t dict_size);

#endif
