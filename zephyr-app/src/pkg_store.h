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
 * capacity written to `*len`. Safe to read from directly (ordinary loads,
 * no driver call) because pm_static.yml also declares a "nonsecure_storage"
 * partition with the exact same address/size as "pkg" - TF-M's
 * NRF_NS_STORAGE feature (on by default) grants the SPU Non-Secure access
 * to whichever flash range Partition Manager resolves "nonsecure_storage"
 * to, and reads that resolution from pm_config.h same as everything else,
 * so it picks up "pkg"'s exact bounds without either partition needing to
 * know about the other.
 *
 * This wasn't always safe: an earlier version of pkg_store_read() went
 * through flash_area_read() instead, specifically because a raw pointer
 * here faulted with a SecureFault once MCUboot entered the picture -
 * MCUboot's own Non-Secure boundary configuration is scoped to its own
 * image slots (mcuboot_primary/mcuboot_secondary) and TF-M's default
 * Non-Secure grant only covers the main app image, neither of which has
 * any notion of "pkg", tacked on past mcuboot_secondary's end. The
 * "nonsecure_storage" declaration above is what closes that gap - without
 * it, go back to flash_area_read() (see pkg_store_write()'s use of it for
 * the pattern), which routes through TF-M's platform service instead of
 * depending on the SPU grant directly, at the cost of not being usable for
 * a direct, zero-copy read the way nodem_load_pkg() (lib.rs) wants.
 *
 * Bytes beyond whatever a prior pkg_store_write() series actually wrote
 * read back as 0xFF (erased); the caller is responsible for telling real
 * content apart from that via the package format's own header. */
const uint8_t *pkg_store_data(size_t *len);

#endif /* PKG_STORE_H_ */
