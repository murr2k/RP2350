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
| Worst demo | `display_test` at 387 us, 78% of budget |
| Rain | Sums in intensity so crossings flare, wake behind the front |
| Everything else | 13 demos launch, no panics, filters at 250 Hz on core 1 |

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

**2. The IMU axis spin, unresolved since bring-up.** The sensor frame was
corrected by a 180 degree rotation about X, which leaves one ambiguity: a
further 180 degrees about Z, deciding whether tilting the board right leans the
cube right or left. Both are right handed, so the filters are correct either
way and only the on screen direction differs.

To settle it, hold the board with its **right hand edge raised** for a few
seconds and read the accelerometer. The convention the demos document is "tilt
right is +X", so a positive `ax` confirms the current mapping and a negative one
means switching both `BOARD_IMU_*_SIGN` lines in
`components/board/include/board_config.h` to `{ -1.0f, 1.0f, -1.0f }`.

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

## Working notes

* The board is on **COM3**, a CH343 bridge on the socket marked **UART**. The
  socket marked USB only enumerates if the firmware brings it up.
* There is no local ESP-IDF. Build in Docker and flash with esptool; both
  commands are in [ROLLBACK.md](ROLLBACK.md).
* Composition happens in the panel's interrupt against a hard deadline that
  `LCD_2IN1_ComposeBudgetUs()` reports. Anything visually dense added later
  should watch `LCD_2IN1_ComposeMaxUs()` and trim itself rather than trusting a
  static guess, as `rain` does.
