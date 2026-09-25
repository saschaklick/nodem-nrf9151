#ifndef NODEM_FFI_H_
#define NODEM_FFI_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* C ABI exported by the `nodem-ffi` Rust staticlib (nodem-ffi/src/lib.rs). */

void *nodem_runtime_new(uint8_t *fb_ptr, size_t fb_len, uint16_t width, uint16_t height);

/* Points `handle`'s rendering at `fb_ptr`/`fb_len` (1bpp, `width` x
 * `height`, same rules as nodem_runtime_new()'s) from now on - the next
 * nodem_runtime_run() renders the current page at the new size. The
 * previous framebuffer is no longer touched once this returns. */
void nodem_runtime_resize(void *handle, uint8_t *fb_ptr, size_t fb_len, uint16_t width,
			  uint16_t height);

/*
 * Loads whatever's in the "pkg" flash partition (pkg_store.c/pm_static.yml)
 * into `handle`'s Media, if it's a valid nodem-rs package. Call once at
 * boot, right after the *first* nodem_runtime_run() - not right after
 * nodem_runtime_new(), before any render has happened: that first run() is
 * nodem's own initialization (it loads nodem-rs's built-in PKG_SYS as source
 * 0), and this needs to land after that, not race it. Call again any time
 * nodem_control_take_pkg_ready() (below) returns true, so a freshly-uploaded
 * package takes effect immediately rather than only on the next reboot. An
 * erased partition (nothing ever uploaded - see control.rs's IControlLoader
 * impl, which is what writes it) is the normal first-boot case, not an
 * error, and is silently skipped. Returns whether a package was actually
 * loaded.
 */
bool nodem_load_pkg(void *handle);

bool nodem_runtime_run(void *handle);

/*
 * Sets a short status message nodem draws in a centered popup box every
 * frame while set (nodem-rs's runtime.rs, RuntimePrivate::popup(), called
 * from DOM::run()) - a new call replaces whatever was set before, it doesn't
 * queue. `text_ptr`/`text_len` only need to stay valid for the call itself;
 * they're copied into a static buffer on the Rust side (64 bytes - see
 * lib.rs's STATUS_MESSAGE_BUF_SIZE), not borrowed. Returns false (message
 * left unset/unchanged) if `text_len` is too long or isn't valid UTF-8.
 */
bool nodem_status_message_set(void *handle, const uint8_t *text_ptr, size_t text_len);

/*
 * Clears the status message set by nodem_status_message_set() - nothing is
 * drawn for it from the next frame onward.
 */
void nodem_status_message_clear(void *handle);
/*
 * True while a "pkg"/"ota" upload is streaming through
 * nodem_process_command() - from the first length byte after the "pkg"/"ota"
 * line until the last payload byte. nodem_task.c uses it to stop hex-dumping
 * every received chunk to the console for the length of a (binary,
 * potentially 300KB) upload.
 */
bool nodem_loader_active(void *handle);

size_t nodem_process_command(void *handle, const uint8_t *input_ptr, size_t input_len,
			      uint8_t *output_ptr, size_t output_cap, size_t *output_len);

/*
 * `nodem_process_command`'s ZephyrControl (control.rs) sets a flag on a
 * "#reset" command rather than resetting synchronously - the reply has to
 * reach the caller (over UART) before the reset actually happens. Returns
 * true (once) if a reset was requested since the last call; the caller
 * (nodem_task.c, after flushing the response) is then responsible for
 * calling nodem_listener.h's nodem_hw_reset().
 */
bool nodem_control_take_restart(void);

/*
 * Returns true (once) if a "pkg" upload finished (successfully) since the
 * last call - control.rs's IControlLoader impl (process_loader_end) sets
 * this rather than loading the package itself, since that trait gives it no
 * way to reach the live Media/Surface. The caller (nodem_task.c) is then
 * responsible for calling nodem_load_pkg() again to actually apply it.
 */
bool nodem_control_take_pkg_ready(void);

/*
 * Called BY nodem-ffi (not exported from it) - nodem-rs logs internally via
 * the `log` crate; nodem-ffi installs a logger that forwards the level and
 * formatted message here rather than calling printk() directly (Rust can't
 * call a C variadic function). Neither string is null-terminated.
 * Implemented in nodem_task.c, which owns the "[nodem] LEVEL message"
 * output format.
 */
void nodem_log_write(const uint8_t *level_ptr, size_t level_len, const uint8_t *msg_ptr,
		      size_t msg_len);

/*
 * Also called BY nodem-ffi (see control.rs): config_store.h's key/value
 * setter and lister ("#reg"/"#nvs") and nodem_listener.h's nodem_hw_reset()
 * ("#reset", via nodem_control_take_restart() above). Their prototypes live
 * in those headers, not here, since C callers besides nodem-ffi may also use
 * them directly.
 */

#endif /* NODEM_FFI_H_ */
