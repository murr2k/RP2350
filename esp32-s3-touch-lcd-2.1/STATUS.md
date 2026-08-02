# Status and open items

Where the port stands on real hardware, and what is still outstanding.
See [ARCHITECTURE_LOG.md](ARCHITECTURE_LOG.md) for how it got here and
[ROLLBACK.md](ROLLBACK.md) for backing any of it out.

## Verified on hardware

| | |
|---|---|
| Panel | 11 MHz, 40 Hz refresh, 498 us per bounce buffer |
| Worst demo | `display_test` at 399 us, 89% of the trim threshold |
| Overrun | A frame too expensive to draw trims itself instead of freezing the panel |
| IMU axes | Settled by tilting: the sensor is mounted a quarter turn round |
| `intuitive_cube` | Quaternions, so it turns through any attitude without a clamp |
| `finger_cube` | Trackball, sensitive to how fast the finger is going |
| Picker | Caps, taller rows, calmer flinging; taps land, no self-relaunch |
| Clock | Fetched over WiFi at boot into the PCF85063, time and date on the picker |
| Rain | Sums in intensity so crossings flare, wake behind the front |
| Everything else | 14 demos launch, no panics, filters at 250 Hz on core 1 |

The picker constants, all at the top of `main/main.c`, since they are the ones
most likely to want turning again:

| | | |
|---|---|---|
| `CAROUSEL_PITCH` | 84 | pixels between items |
| `CAROUSEL_FRICTION` | 7.0 | higher stops a fling sooner |
| `CAROUSEL_SNAP` | 14.0 | higher pulls to centre harder |
| `CAROUSEL_TAP_SLOP` | 12 | pixels of movement still counted as a tap |
| `CAROUSEL_MAX_FLING` | 7.0 | items per second |

A fling carries `MAX_FLING / FRICTION` items past the finger, currently one.

## Open items

**1. `intuitive_cube` cannot see heading, and it shows.** Turning the board flat
on the table spins the cube about the screen normal. That is not a bug in the
demo: gravity is unchanged by that motion, so an accelerometer alone has no way
to know it happened, and the demo carries the unobservable degree of freedom
over rather than inventing one. Fixing it means bringing the gyroscope in, which
is what `madgwick_cube` and the Kalman demos already do. Deferred deliberately.

**2. The clock has no backup cell**, so the PCF85063 comes up after every power
cycle with its oscillator stop flag set and the time is fetched over the network
instead. Fitting one needs no code change: the flag stops coming up set and the
fetch becomes a correction rather than the only source. Out of WiFi range and
without the cell, the picker simply shows `--:--:--`.

**3. The instrumentation is still in the build**, by request. It prints a frame
breakdown every 100 frames from `demo_frame_end()` and runs a PSRAM benchmark in
the `diagnostic` demo. It has earned its keep repeatedly: it caught the rain
overrun, a 32 bit overflow that silently reported a 2 us composition budget, and
the 129% `display_test` overrun behind the aberrant pixels. Costs a per frame
`printf` and some timing calls if you want it out.

## Tuning the rain

| | | |
|---|---|---|
| `RAIN_TRAIL` | 2.6 | wake length; 1.0 is the original's symmetric ring |
| `RAIN_RING_SIGMA` | 26 | ring thickness |
| `RAIN_AREA_BUDGET` | 240000 | blended pixels before it trims itself |

## Settled, for the record

The IMU frame is done. The sensor sits a **quarter turn** round from the screen,
which no combination of signs can undo, so the map swaps its X and Y. Measured by
holding the board at 45 degrees each way: before the fix a right hand edge tilt
read `ay -0.75` and left `ax` alone, after it reads `ax +0.85` and leaves `ay`
alone. Flat has always read `az +0.98`.

The touch panel is **single contact** and cannot be made otherwise. The CST820
senses rows and columns separately, so two fingers and their two mirror
positions are indistinguishable; the register map has nowhere to put a second
point and the count register never reports one. `touch_test` shows the live
count. A one finger equivalent of a two finger twist would be an arcball, where
dragging around the rim turns the model about the screen normal.

**No LVGL, and no external dependencies at all**: there is not even a component
manager manifest. Drawing is `components/display/gfx.c`, about 700 lines of font,
integer rasterisers and display list. LVGL is retained mode and needs somewhere
to render into before flushing, which is exactly what this architecture does not
have, so adopting it would mean reinstating the frame buffer and the copy that
the whole design exists to avoid. Considered and declined on 2 August 2026;
LVGL is being explored in a separate project instead. The board bring up is what
would carry over: the ST7701S sequence, the 9 bit SPI that drives it, the touch
and expander drivers, and `sdkconfig.defaults`.

## Working notes

* The board is on **COM3**, a CH343 bridge on the socket marked **UART**. The
  socket marked USB only enumerates if the firmware brings it up.
* Opening the serial port resets the board. Wait a few seconds after opening
  before sending keys to the demo menu, or they land during boot and are lost.
* There is no local ESP-IDF. Build in Docker and flash with esptool; both
  commands are in [ROLLBACK.md](ROLLBACK.md).
* WiFi credentials are in `main/wifi_secrets.h`, gitignored because the GitHub
  repo is public. `wifi_secrets.h.example` shows the shape. The radio comes up
  only to fetch the time and is then shut down: while associated it tripled the
  render loop's per frame cost, since the WiFi stack executes from the same
  PSRAM the composer reads.
* Composition happens in the panel's interrupt against a hard deadline that
  `LCD_2IN1_ComposeBudgetUs()` reports. Anything visually dense added later
  should watch `LCD_2IN1_ComposeMaxUs()` and trim itself rather than trusting a
  static guess, as `rain` does.
