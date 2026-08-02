# Status and open items

Where the port stands on real hardware, and what is still outstanding.
See [ARCHITECTURE_LOG.md](ARCHITECTURE_LOG.md) for how it got here and
[ROLLBACK.md](ROLLBACK.md) for backing any of it out.

## Verified on hardware

| | |
|---|---|
| Picker | Taps land on the right item, no self-relaunch on exit |
| Aberrant pixels | Gone. Was a composition overrun at 58 fps |
| Ball smearing | Gone. Animation is time based rather than per frame |
| Panel | 11 MHz, 40 Hz refresh, 498 us per bounce buffer |
| Worst demo | `display_test` at 399 us, 89% of the trim threshold |
| Rain | Sums in intensity so crossings flare, wake behind the front |
| IMU axes | Settled by tilting: the sensor is mounted a quarter turn round |
| Overrun | A frame too expensive to draw trims itself instead of freezing the panel |
| Everything else | 14 demos launch, no panics, filters at 250 Hz on core 1 |

## Open items

**1. The picker needs tweaking.** Reported as "could use some tweaking" without
specifics. The question to settle before changing anything: is it the scroll
distance per drag, the snap being too eager, or the fling carrying too far? The
constants are at the top of `main/main.c`:

| | | |
|---|---|---|
| `CAROUSEL_PITCH` | 64 | pixels between items |
| `CAROUSEL_FRICTION` | 6.0 | higher stops a fling sooner |
| `CAROUSEL_SNAP` | 14.0 | higher pulls to centre harder |
| `CAROUSEL_TAP_SLOP` | 12 | pixels of movement still counted as a tap |
| `CAROUSEL_MAX_FLING` | 14.0 | items per second |

**2. `intuitive_cube` has a convention of its own.** Settling the board frame
raised a separate question one level up: that demo feeds `acc.y` to the
horizontal screen axis and `acc.x` to the vertical one, which is not the same
convention `board_config.h` now documents. The board frame is right either way,
so this is a demo level question about how a tilt should read on screen, not a
sensor one. Worth a look at demo `4` to see whether the cube leans the way the
board leans before changing anything.

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

## Working notes

* The board is on **COM3**, a CH343 bridge on the socket marked **UART**. The
  socket marked USB only enumerates if the firmware brings it up.
* There is no local ESP-IDF. Build in Docker and flash with esptool; both
  commands are in [ROLLBACK.md](ROLLBACK.md).
* Composition happens in the panel's interrupt against a hard deadline that
  `LCD_2IN1_ComposeBudgetUs()` reports. Anything visually dense added later
  should watch `LCD_2IN1_ComposeMaxUs()` and trim itself rather than trusting a
  static guess, as `rain` does.
