#ifndef HUB75_TASK_H_
#define HUB75_TASK_H_

#include <stddef.h>
#include <stdint.h>

#include "nodem_config.h"

/*
 * HUB75 RGB LED matrix driver. There's no GPIO-port DMA on the nRF91, so
 * the column data is shifted out by two PWM instances in Individual
 * decoder mode, used as a 4-bit parallel port each: EasyDMA loads one
 * 16-bit value per channel per PWM period, and every value is either
 * "low all period" or "high all period" - except CLK, which rises
 * mid-period, while the data lines are stable. Both instances run off the
 * same 16MHz clock and are started back to back, so they stay in step to
 * within a tick or two. OE rides on PWM2's spare channel, so brightness
 * (how long each row is lit) is timed by the same sequence, in PWM ticks,
 * not by interrupt latency.
 *
 * Laid out for a straight two-row ribbon to the HUB75 connector, with GND
 * next to P0.13 for its pins 4/8/16 (8 only on panels without E):
 *
 *   HUB75 pin:  1   3   5   7   9  11  13  15     2   6   8  10  12  14
 *   signal:     R1  B1  R2  B2  A  C   CLK OE     G1  G2  E  B   D   LAT
 *   nRF pin:    .00 .01 .02 .03 .04 .05 .06 .07   .08 .09 .10 .11 .12 .13
 *
 *   PWM1: R1, G1, B1, CLK - PWM2: R2, G2, B2, OE (active low) - GPIO: the
 *   rest. D only with 16 or 32 row addresses, E only with 32.
 *
 * P0.00/.01/.04/.05 are the DK's LED1-LED4 (LED1: main.c's heartbeat, see
 * hub75_task.c), P0.08/.09 its BUTTON1/BUTTON2 - pressing either while the
 * panel runs shorts that output to GND. The DK's external flash shares
 * P0.11-P0.13 with B/D/LAT, so the board overlay holds its CS (P0.20) high,
 * and the board's pwm0 (P0.00) is disabled there. While disabled, every one
 * of these pins is released (back to its reset state, an unconnected input)
 * - except OE, held high so the panel stays blanked rather than showing
 * whatever it last latched, and P0.00, a plain output for the heartbeat.
 *
 * One row address is shifted per PWM sequence. With fewer row addresses
 * than rows per half (a 1/8-scan 64x32 panel: 8 addresses, 16 rows per
 * half), each address's chain runs through several panel rows, one panel
 * width each - see frame_set_pixel() for the order. When both instances
 * stop at its end (OE falls back to its GPIO level, high - blanked), the
 * PWM1 interrupt selects that row's address, latches it, and starts
 * shifting the next row, during which OE lights the latched one for a
 * brightness-dependent part of the shift time. Rows are rendered one ahead
 * by the interrupt itself, into the other of two ping-ponged row buffers.
 *
 * Self-starting (K_THREAD_DEFINE), configured through config_store's
 * "hub75" (see hub75_config_set()). Shows nodem's framebuffer through the
 * "nodem" config's "hub75" entry (placement and scaling, same as the
 * OLED's - see nodem_config.h), lit pixels in the configured colour, unlit
 * ones in the off colour; without an entry, the same blinking dotted edge
 * the OLED shows.
 */

/* A sanity limit on the display's size (64x256, 128x128, ...) - how much
 * of it is actually driven depends on the memory there is, see
 * hub75_task.c's hub75_alloc(). */
#define HUB75_MAX_PIXELS 16384

/* Brightness range: the share of each row's time it's lit for, out of this
 * (OE duty cycle, in PWM ticks - finer than 8 bits on every panel size). */
#define HUB75_BRIGHTNESS_MAX 0x7FFF

/* What a never-configured device uses (and persists): this DK's 64x32 1/8
 * scan panel, input connector on the right, plain shift-register drivers,
 * red, at a low brightness (~1.6%). hub75_task.c's hub75_default() is the
 * same, parsed. */
#define HUB75_DEFAULT "64:32:rt:1x1,8:n:500:shift:4,#ff0000,#000000,512"

/*
 * Validates and persists config_store's "hub75" and has hub75_task apply it
 * right away. The format is
 *
 *   <geometry>,<protocol>,<color>,<off_color>,<brightness>
 *
 * - <geometry>: "<width>:<height>:<layout>[:<grid>]" - the whole display.
 *   <layout> is up to two order-independent letters, "l"/"r" (the input
 *   connector's side, seen from the front) and "t"/"b" (row 0 at the top
 *   or the bottom) - "rb" is a panel mounted upside down. Each defaults to
 *   "l"/"t". <grid> is "<cols>x<rows>[i]" (default "1x1", at most 16
 *   panels; a lone number is that many side by side): the display is a
 *   grid of that many equal panels on one chain, which - seen with the
 *   layout's mirroring applied - enters at the top left, runs through the
 *   top row of panels, then the next one. With "i" (serpentine), every
 *   second row of panels is mounted upside down and the chain runs back
 *   through it, so each panel's output is right next to the next one's
 *   input. Two panels stacked, entering at the top right: "rt:1x2i".
 * - <protocol>: a preset - "4s", "8s", "8sf", "16s", "32s" (the number of
 *   row addresses; "8sf": chain "f") - or
 *   "<scan>:<chain>:<clock_khz>:<chip>:<depth>", each part optional:
 *   - <scan>: row addresses - 2 (A), 4 (A-B), 8 (A-C), 16 (A-D) or 32
 *     (A-E). On a panel with fewer row addresses than rows per half
 *     (labels like "8S" on a 32-row panel), each address's chain runs
 *     through several rows - see <chain>.
 *   - <chain>: "<n|f>[<segment>][z]" - how it does: one piece of
 *     <segment> columns (default: a whole panel width) per row in turn,
 *     the first row's nearest the input ("n") or furthest ("f"), every
 *     second piece backwards with "z". Most panels are "n" or "f"; 1/4
 *     and some 1/8 scan ones interleave in 8- or 16-column pieces ("n8",
 *     "f16z", ...). Ignored with one row per address.
 *   - <clock_khz>: the pixel clock, 100-1000 kHz.
 *   - <chip>: the column driver - "shift" (plain shift registers: ICN2037,
 *     ICN2038, MBI5124, DP5125, ...) or "fm6126a" (also "fm6124",
 *     "icn2038s"; set up with a register write before anything shows).
 *   - <depth>: bits per colour channel, 1-8 (2-256 levels each, 8 to 16.7
 *     million colours). Each bit is one more shift of every row (a bit plane, lit
 *     for its binary weight), so the refresh rate divides by it - 4 bits
 *     on two 64x32 1/8 scan panels at 500 kHz is ~60 Hz; the CPU time
 *     doesn't change (one render per shift either way).
 *   Presets set every part (500 kHz, "shift", 4 bits).
 * - <color>/<off_color>: for lit/unlit pixels, "#rrggbb" (shorter is
 *   fine - missing digits are 0) or an effect, "<effect>[:<speed>]":
 *   "plasma" (moving waves of colour, formerly "rainbow"), "stripes"
 *   (diagonal, scrolling - a barber pole), "wave" (horizontal bands waving
 *   like a flag), "scroll"
 *   (vertical bands sliding sideways), "cycle" (one colour for the whole
 *   panel, blending round the wheel), "fade" (each colour in turn, fading
 *   in from black and out again), "fire" (flickering red/yellow/white
 *   rising from the bottom), "sparkle" (every pixel twinkling - fading in and out in a
 *   random colour, at its own time), "ripple" (rings spreading from the centre) or "gradient"
 *   (a still diagonal rainbow). The full form is
 *   "<effect>[:<speed>[:<palette>[:<scale>]]]", an empty part its
 *   default: <speed> (1-32, default per effect - none for "gradient") is
 *   how far it moves per frame, at 25 frames a second; <palette> (up to 8
 *   of k r g y b m c w - black, red, green, yellow, blue, magenta, cyan,
 *   white) the colours it blends between, in order and in equal shares,
 *   as a loop - "fire": a ramp, coldest first - repeats allowed (default: the six colours of the wheel; for "fire" "kryw",
 *   coldest to hottest; for "sparkle" the wheel and white); <scale> (1-32,
 *   default 8) the pattern's size, larger stretching it. E.g.
 *   "ripple:4:rb", "plasma::cg:16", "fire:::4".
 *   "#rrggbb" and the effects' blends go through gamma 2 to <depth> bits
 *   per channel - at depth 1, every blend is back to bands of the palette's
 *   colours.
 * - <brightness>: 0-HUB75_BRIGHTNESS_MAX.
 *
 * Any field left empty (or missing at the end) takes HUB75_DEFAULT's. The
 * display must fit HUB75_MAX_PIXELS; as many of its panels as fit in the
 * driver's memory are driven (the first ones along the chain), the rest
 * stay blank - see the log. An empty value as a
 * whole disables the panel. Stored with every part spelled out, presets
 * expanded, so "#cfg" shows exactly what's in use. Returns 0 on success,
 * -EINVAL for a malformed value (nothing stored), or config_store's error
 * if persisting it failed.
 */
int hub75_config_set(const char *value);

/*
 * Hands hub75_task nodem-rs's framebuffer `buf` and the config it's
 * rendered with - same contract as display_task_submit(): read in place,
 * under display_fb_lock(), so it must stay valid until the next call. The
 * panel is redrawn from it after every call.
 */
void hub75_task_submit(const uint8_t *buf, const struct nodem_config *config);

/*
 * The "hub75" line of "#stat" (control.rs), shaped like nodem-esp32's
 * "oled" one: "hub75,0,," while disabled, else "hub75,1,<w>x<h>,<status>" -
 * status "ok", "low_mem <driven>/<configured> panels" (the memory was only
 * enough for some, the rest stay blank), "no_mem" (not even for one) or
 * "overload" (stopped after tripping too often - see hub75_task.c). No line
 * break. snprintf semantics: returns the untruncated length.
 */
size_t hub75_stat_format(char *buf, size_t buf_len);

/* Blanks the panel while `hold` is true and sets it up again from the config
 * once it is false. nodem_task.c holds it during a "pkg"/"ota" upload: the
 * PWM interrupt's row rendering overruns whenever the UART interrupt or a
 * flash write preempts it, and after a few overruns the panel switches itself
 * off until the config changes. */
void hub75_task_hold(bool hold);

#endif /* HUB75_TASK_H_ */
