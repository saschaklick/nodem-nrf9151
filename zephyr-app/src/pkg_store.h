#ifndef PKG_STORE_H_
#define PKG_STORE_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Raw flash-backed staging area for an uploaded nodem-rs binary UI/Media
 * package (*not* firmware) - the "pkg" partition manager partition (see
 * pm_static.yml), written by nodem-ffi's control.rs (IControlLoader impl) as
 * nodem-rs's "pkg" upload protocol streams bytes in, one process_command
 * call at a time. Unlike config_store's NVS, this is a raw byte range: no
 * filesystem, no wear-levelling - callers erase once up front, then write
 * forward-only, as a package is streamed in a single pass and never
 * modified in place. Loading a fully-written package back into the running
 * DOM/Surface is nodem-rs's own concern, not this store's.
 */

/* Largest package pkg_store can ever hold - the size of the "pkg"
 * partition. */
size_t pkg_store_capacity(void);

/* Longest single write pkg_store_write() accepts in one call - must match
 * control.rs's PKG_CHUNK_LEN, which never buffers more than this many bytes
 * before flushing. */
#define PKG_STORE_MAX_CHUNK 512

/* Erases enough whole flash pages, starting at offset 0, to hold `len`
 * bytes. Must be called once before any pkg_store_write() for a new
 * upload - a later, in-place pkg_store_write() would otherwise be unable to
 * clear bits an earlier upload left set. Returns 0 on success, a negative
 * errno otherwise (including -ENOSPC if `len` exceeds pkg_store_capacity()).
 */
int pkg_store_erase(size_t len);

/* Writes `len` (at most PKG_STORE_MAX_CHUNK) bytes from `buf` at `offset`
 * bytes into the "pkg" partition; `offset`..`offset+len` must already have
 * been covered by pkg_store_erase(). If `len` isn't already a multiple of
 * the flash's write-block size, the write is padded up to it with 0xFF (a
 * no-op on already-erased flash) so callers never need to know that size
 * themselves - this is what lets a final, odd-sized tail chunk be written
 * as-is. Returns 0 on success, a negative errno otherwise. */
int pkg_store_write(size_t offset, const uint8_t *buf, size_t len);

/* Direct read-only pointer to the "pkg" partition's raw contents, with its
 * capacity written to `*len`. No separate read call is needed (unlike
 * pkg_store_write()'s erase-then-write dance) because the nRF9151's
 * internal flash is memory-mapped at its own physical address - this is
 * just that address. Bytes beyond whatever a prior pkg_store_write() series
 * actually wrote read back as 0xFF (erased); the caller (nodem-ffi's
 * nodem_load_pkg(), see lib.rs) is responsible for telling real content
 * apart from that via the package format's own header. */
const uint8_t *pkg_store_data(size_t *len);

#endif /* PKG_STORE_H_ */
