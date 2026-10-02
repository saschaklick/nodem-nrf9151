#ifndef HUB75_TASK_H_
#define HUB75_TASK_H_

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

/* Panels up to this many pixels (64x64, 128x32, ...) - every buffer is
 * sized for it once, so a new config never needs to allocate. */
#define HUB75_MAX_PIXELS 4096

/* Brightness range: the share of each row's time it's lit for, out of this
 * (OE duty cycle, in PWM ticks - finer than 8 bits on every panel size). */
#define HUB75_BRIGHTNESS_MAX 0x7FFF

/* What a never-configured device uses (and persists): this DK's 64x32 1/8
 * scan panel, input connector on the right, plain shift-register drivers,
 * red, at a low brightness (~1.6%). hub75_task.c's hub75_default() is the
 * same, parsed. */
#define HUB75_DEFAULT "64:32:rt:1x1,8:n:500:shift,#ff0000,#000000,512"

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
 *   "<scan>:<chain>:<clock_khz>:<chip>", each part optional:
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
 *   Presets set every part (500 kHz, "shift").
 * - <color>/<off_color>: for lit/unlit pixels, "#rrggbb" (shorter is
 *   fine - missing digits are 0) or an effect, "<effect>[:<speed>]":
 *   "rainbow" (moving plasma), "stripes" (diagonal, scrolling - a barber
 *   pole), "wave" (horizontal bands waving like a flag), "scroll"
 *   (vertical bands sliding sideways), "cycle" (one colour stepping round
 *   the wheel), "fade" (the same, fading through black between colours -
 *   by dimming the whole panel, so a non-black other colour dims along),
 *   "fire" (flickering red/yellow/white rising from the
 *   bottom), "sparkle" (every pixel its own random colour, changing at
 *   random), "ripple" (rings spreading from the centre) or "gradient"
 *   (a still diagonal rainbow). <speed> (1-32, default per effect - not
 *   for "gradient") is how far it moves per frame, at 25 frames a second.
 *   Only on/off per channel for now - a channel is on wherever its value
 *   isn't 0, and effects move as bands of the fully saturated colours.
 * - <brightness>: 0-HUB75_BRIGHTNESS_MAX.
 *
 * Any field left empty (or missing at the end) takes HUB75_DEFAULT's. The
 * display must fit HUB75_MAX_PIXELS and each row address's chain 256
 * columns (width times rows of panels times the rows of a panel half
 * sharing an address). An empty value as a
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

#endif /* HUB75_TASK_H_ */
