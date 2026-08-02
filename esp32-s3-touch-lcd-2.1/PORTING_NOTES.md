# Porting notes: RP2350-LCD-1.28 to ESP32-S3-Touch-LCD-2.1

What changed, what stayed the same, and why. Read this before touching the
code if you know the RP2350 tree.

## Hardware side by side

| | Waveshare RP2350-LCD-1.28 | Waveshare ESP32-S3-Touch-LCD-2.1 |
|---|---|---|
| MCU | RP2350, dual Cortex-M33 @ 150 MHz | ESP32-S3R8, dual Xtensa LX7 @ 240 MHz |
| RAM | 520 KB SRAM | 512 KB SRAM + 8 MB octal PSRAM |
| Flash | 4 MB | 8 or 16 MB |
| Display | 1.28" round, 240x240, GC9A01A over SPI | 2.1" round, 480x480, ST7701S over 16-bit parallel RGB |
| Frame | 115 KB, fits in SRAM | 450 KB, has to live in PSRAM |
| Touch | none | CST820, I2C, single point plus gestures |
| IMU | QMI8658 on I2C1 (GPIO6/7) | QMI8658 on the shared I2C bus (GPIO15/7) |
| Extras | battery ADC | RTC PCF85063, microSD, battery ADC, TCA9554 IO expander |
| Console | USB CDC from the RP2350 USB block | USB Serial/JTAG on the native USB pins |
| Flashing | drag a .uf2 onto the RPI-RP2 drive | esptool over the serial port |

Pin numbers are all in `components/board/include/board_config.h`.

## Build system

Pico SDK plus CMake becomes ESP-IDF plus CMake. The shape is familiar:
`idf.py build` where you used `make`, components where the original had
`add_library` per directory.

The one structural difference: **ESP-IDF links one application per project.**
The RP2350 tree built about twenty separate `.uf2` images and you chose a demo
by flashing it. Here every demo is compiled into one image and chosen at
runtime, from the touch screen or by pressing a key on the serial console. The
demo names are unchanged, so `configurable_cube.uf2` becomes the
`configurable_cube` entry in the launcher.

| RP2350 image | Here |
|---|---|
| `minimal_lcd`, `waveshare_test`, `spi_test`, `waveshare_fixed` | `display_test` |
| `buffered_cube`, `rotating_cube`, `simple_cube`, `wireframe_cube` | `buffered_cube` |
| `intuitive_cube` | `intuitive_cube` |
| `gravity_cube`, `gravity_locked_cube` | `gravity_locked_cube` |
| `madgwick_cube` | `madgwick_cube` |
| `configurable_cube` | `configurable_cube` |
| `rotation_test` | `rotation_test` |
| `axis_test` | `axis_test` |
| `kalman_6dof`, `kalman_balance`, `kalman_fixed`, `quaternion_kalman` | `kalman_6dof` |
| `kalman_6dof_config` | `kalman_6dof_config` |
| `rp2350_diagnostic` | `diagnostic` |
| (no counterpart) | `touch_test` |

The near duplicate cube images in the original differ only in projection
constants and how much of the IMU they use, so they collapsed into the demo
that superseded them.

## Display

The GC9A01A took a stream of pixels over SPI. The ST7701S is fed by the ESP32-S3
LCD peripheral over a 16-bit parallel RGB bus and needs a one-time register
sequence over a 3-wire SPI link before that starts. Both halves live in
`components/display/lcd_2in1.c`:

* The init sequence is Waveshare's, verbatim, as a table. It is sent as 9-bit
  frames (one data/command bit, then the byte) which the ESP32 SPI master
  produces natively with `command_bits = 1, address_bits = 8`, so there is no
  bit banging.
* Chip select and reset are not GPIOs on this board. They hang off the TCA9554
  expander, so `components/board/tca9554.c` has to be alive before the panel.
* The SPI bus is released once the sequence is sent, because GPIO1 and GPIO2 are
  also the microSD CMD and CLK lines.
* Two frame buffers are allocated in PSRAM by the RGB driver.
  `LCD_2IN1_Display()` on one of them flips which is being scanned out, so
  drawing never lands on the visible frame.

### API change the demos had to absorb

Each RP2350 demo owned `static uint16_t frame_buffer[240*240]`. At 480x480 that
array is 450 KB and cannot be static in internal RAM, so the buffer belongs to
the driver now:

```c
uint16_t *fb = demo_frame_begin(GFX_BLACK);   /* grab, bind and clear */
/* ... draw ... */
demo_frame_end();                             /* present */
```

### Byte order

The 1.28" demos wrote `((color << 8) & 0xFF00) | (color >> 8)` into every pixel,
byte swapping RGB565 by hand because the driver pushed the buffer to SPI without
swapping. The RGB peripheral takes RGB565 in native order, so the swap is gone
and the colour constants finally mean what they say. If a demo looked
off-colour on the original hardware, that is why.

### Coordinates

Demo geometry still uses the 240x240 numbers from the original sources and gets
scaled on the way out, either with the `S()` macro or by `demo_project()`. The
pictures are the same, at twice the resolution.

### Text

The 1.28" demos had no font. `intuitive_cube` even carries a comment saying
"would need text rendering to display this" above a `sprintf` whose result is
thrown away. There is a 5x7 font here (`components/display/font5x7.c`), so the
demos show their numbers as well as their bar graphs.

## IMU

Same QMI8658 part, so the maths carried over untouched. Two things changed in
`components/sensors/qmi8658.c`:

1. **Range registers now match the scale factors.** The RP2350 demos wrote
   `CTRL2 = 0x04` and `CTRL3 = 0x54` and then divided by 16384 LSB/g and
   128 LSB/dps. `CTRL3 = 0x54` actually selects +/-512 dps, which is 64 LSB/dps,
   so every gyro reading on the original was half the true rate. This port
   writes `CTRL2 = 0x05` (+/-2 g, 250 Hz) and `CTRL3 = 0x45` (+/-256 dps,
   250 Hz), which is what the README, the demos and the host test scripts all
   assume. Angles now integrate at the right speed.
2. **One burst read instead of twelve.** `CTRL1` enables address auto-increment
   and all six samples come back in a single I2C transaction.

Axis orientation is **not** verified on this board. The sensor sits differently
from the 1.28" module, so run `rotation_test` and `configurable_cube` and record
what you find in `BOARD_IMU_*` in `board_config.h`, which every demo reads.

## Serial console

`stdio_init_all()` plus `getchar_timeout_us(0)` becomes the USB Serial/JTAG
peripheral. `printf` goes through the ESP-IDF console as usual; input is read
non blocking with `usb_serial_jtag_read_bytes(..., 0)` behind
`demo_read_char()`, which is a drop-in for `getchar_timeout_us(0)`.

The command grammars are unchanged, so the host tools in the repository root
work as they did. `tools/serial_bridge.py` is the same TCP bridge as
`serial_bridge_fixed.py` with the port as an argument instead of hardcoded.

ESC and `~` are reserved by the launcher for leaving a demo, and never reach a
demo's command handler.

### CSV field order

Two quirks are preserved on purpose, because the host scripts were written
against them:

* `kalman_6dof` and `kalman_6dof_config` disagree about which angle comes first.
  `kalman_6dof.c` put the `asin` angle in the "pitch" column;
  `kalman_6dof_configurable.c` put the `atan2` angle there. Each demo here emits
  what its ancestor emitted.
* `test_injection.py` reads the angle from `parts[7]` of a `CSV,` line, which is
  the gyro Z field, not pitch: the angle columns start at `parts[8]`. That is a
  pre-existing bug in the host script, left alone here since it is shared with
  the RP2350 build. Fix it in one place if you want the injection test to
  actually check the angle.

## Timing

| RP2350 | Here |
|---|---|
| `time_us_32()` | `demo_micros()` over `esp_timer_get_time()` |
| `sleep_ms(n)` | `demo_delay_ms(n)` over `vTaskDelay()` |

`CONFIG_FREERTOS_HZ=1000` keeps the 5 ms and 10 ms loop delays meaningful, and
yielding on every pass is what stops the task watchdog from firing.

## Deliberate behaviour changes

Everything else is a faithful port. These are the exceptions, all of them fixes:

* **Axis swap keys work.** `configurable_cube` advertised six swap keys in its
  menu that `process_command()` never implemented. They are implemented here:
  accelerometer on `q`/`w`/`s`, gyroscope on `a`/`z`/`x`.
* **Gyro scale**, as described above.
* **Colour byte order**, as described above.
* **`asinf()` domain guard** in `intuitive_cube`. A sharp shake pushes the
  filtered acceleration past 1 g, where `asinf()` returns NaN and the cube
  disappears until the filter recovers.
* **Gyro calibration lives in the driver** rather than being repeated, slightly
  differently, in five demos.

## Not ported

* `lib/config_manager.c`, `lib/data_logger.c`, `lib/event_queue.c`,
  `lib/state_machine.c`, `drivers/common/*.c`, `include/demo_app.h`,
  `include/event_handler.h`, `include/system_init.h`: nine-line placeholders in
  the original, with no callers.
* `tests/CMakeLists.txt`: refers to `test_display.c`, `mock_hardware.c` and
  `drivers/display/graphics.c`, none of which exist in the repository.
* `src/serial_test.c`: listed in the root `CMakeLists.txt` but absent from the
  tree, so the original build does not complete as checked in.
* `src/ImageData.c` and the 240x240 splash bitmap: geometry specific to the
  1.28" panel.

## Threading

The RP2350 ran everything in one loop. Here the work is split across the two
cores:

* **Core 0** draws, and does nothing else. The main task renders and calls
  `LCD_2IN1_Display()`, which blocks until the panel has released the outgoing
  buffer.
* **Core 1** reads the IMU at its 250 Hz output rate, applies the board axis
  map, publishes the sample, and runs whichever filter the current demo
  installed with `demo_set_filter()`. It also carries the LCD interrupt: the
  RGB panel's ISR is allocated on whichever core creates the panel, so
  `main.c` creates it from a task pinned here, and the bounce buffer copy stays
  off the drawing core.

### Drawing is recorded, then composed a row at a time

`gfx_*` calls do not paint. They append to a display list, and `gfx_flush()`
composes the frame one row at a time: build the row in internal SRAM from the
background up, painting each primitive that crosses it in order, then write the
finished row to PSRAM once. Every pixel is written exactly once with its final
value, and the same path draws everything, whatever the demo.

The API is unchanged, so no demo needed touching. Content that no primitive
describes, such as the gradient in `display_test`, registers a row painter with
`gfx_row_painter()` and is composed in list order like anything else.

Measured, per frame:

| | clear plus draw | record plus compose |
|---|---|---|
| `buffered_cube` | 10.9 + 3.7 = 14.6 ms | 0.1 + 13.8 = 13.9 ms |
| `madgwick_cube` | 10.9 + 2.8 = 13.7 ms | 0.3 + 13.8 = 14.1 ms |
| `axis_test` | 11.0 + 3.7 = 14.7 ms | 0.6 + 12.5 = 13.1 ms |

So it is close to a wash, between half a millisecond saved and a couple of
tenths lost. The reason is that the PSRAM write was never the part being
avoided: composing still writes every visible pixel, which is the same ~11 ms,
and what changed is that the scattered per-pixel drawing moved from PSRAM into
SRAM, roughly trading three milliseconds of one for two of the other.

What did change by an order of magnitude is the recording cost, now 0.05 to
0.6 ms. Putting more on screen is nearly free until it is composed.

### There is no frame buffer

The step that made the row based renderer worth having. The panel runs in
`no_fb` mode: nothing is stored at frame size at all. Its interrupt calls
`on_bounce_empty` whenever the DMA has drained a bounce buffer, and those ten
rows are composed straight into it from the display list.

The display lists are double buffered instead of the frames: two lists of about
15 KB in SRAM, swapped at a frame boundary, in place of two 450 KB frames in
PSRAM.

| | with frame buffers | composed on demand |
|---|---|---|
| render core, per frame | 13.9 ms | 0.03 to 0.5 ms |
| compose, per bounce buffer | n/a | 161 to 225 us of a 680 us budget |
| frame rate | 58.5 fps | 58.5 fps, panel limited |
| PSRAM free | 6903 KB | 7803 KB |
| PSRAM bandwidth left for the app | 30.5 MB/s | 48.6 MB/s |

The frame rate cannot improve, the panel sets it. What changed is that the
drawing core is now idle 97% of the time, and the display has stopped competing
for memory: the same benchmark that measured 30.5 MB/s while two frame buffers
were being streamed measures 48.6 MB/s now.

**No floating point in the rasterisers.** Xtensa forbids the FPU in an interrupt
handler: touching a float there raises a coprocessor exception and panics the
core. `CONFIG_FREERTOS_FPU_IN_ISR` exists but is ESP32 only, not S3. So
everything reachable from `gfx_compose_rows()` is integer, including an integer
square root for the circles and half row arithmetic for line interpolation.
Recording runs in a task and still uses floats freely, which is why `gfx_arrow()`
can call `atan2f()`.

Two consequences worth knowing. `LCD_2IN1_DisplayWindows()` and
`LCD_2IN1_DisplayPoint()` are gone, because both push pixels into a stored frame
and there is no longer one: this is the point where parity with the 1.28" driver
had to break. And `CONFIG_LCD_RGB_ISR_IRAM_SAFE` is now off, because the
rasteriser lives in PSRAM and calls into the standard library; nothing here
writes flash at runtime, which is the only thing that would pull the cache out
from under it.

### The pixel clock is a composition budget, not just a frame rate

Waveshare's sample runs this panel at 16 MHz, which is 58.5 Hz. That was kept
through the frame buffer era, where it only decided how often the DMA read
memory. Once the display was composed on demand it became something else: the
pixel clock sets how long the interrupt has to produce each row, and nothing
here needs 58 frames a second.

At 16 MHz a bounce buffer of ten rows had to be ready in 342 us. `display_test`
needed 440, so it overran on every frame, which showed on the glass as small
groups of wrong pixels in consistent places. Not random, because the overrun was
not random.

Dropping to 11 MHz gives 40 Hz and 498 us per buffer. With that, plus merging
runs of lit columns in the text rasteriser and tabulating the gradient's per
column arithmetic, the worst demo sits at 387 us, 78% of budget, and the rest
are between a third and a half:

| | at 16 MHz, 342 us | at 11 MHz, 498 us |
|---|---|---|
| `display_test` | 440 us, over budget | 387 us, 78% |
| `rain` | 376 us | 291 us, 58% |
| `buffered_cube` | 249 us | 178 us, 36% |
| the rest | 162 to 206 us | 170 to 190 us, about 37% |

Everything timing related now derives from `BOARD_LCD_PCLK_HZ`, and
`LCD_2IN1_ComposeBudgetUs()` reports the deadline, so moving the clock moves the
budget and anything self-tuning with it. Watch the intermediate arithmetic:
`H_TOTAL * 1000000000` overflows 32 bits and quietly reported a 2 us budget,
which starved the rain screensaver to its minimum quality before anyone noticed.

### What composing on demand costs, and what it does not suit

Dropping the frame buffer changed what a frame costs, in a way worth knowing
before adding anything visually dense.

With a frame buffer, a scene is drawn once per *update* and the DMA streams it
however often the panel refreshes. Without one, the scene is recomposed on every
*refresh*, 58.5 times a second, whether or not anything moved, and each bounce
buffer has a hard 342 us to be ready. Cost is set by the panel, not by how often
you choose to redraw.

That is excellent for sparse content. A wireframe cube composes in about 200 us
of the 342 available. It is poor for anything that covers the screen. The `rain`
screensaver draws soft glowing rings whose blended area is roughly `2 pi r` per
unit of ring width, and at the parameters scaled faithfully from the original it
needed 499 us. Overrunning does not degrade gracefully: the composer falls behind
the DMA, the bounce position stops wrapping, the frame boundary event never
arrives, and every frame waits out the 100 ms backstop. 58 fps becomes 10.

Three attempts to pick parameters that fit by hand all missed, in both
directions, because the per pixel cost turned out to be about twice what the
arithmetic suggested. What worked was letting the demo measure itself:

* **Feed forward.** Blended area is about `2 pi r * width`, so cap the width to
  hold that product roughly constant. A ripple thins as it spreads, which is
  both cheaper and closer to how water behaves.
* **Feedback.** Read `LCD_2IN1_ComposeMaxUs()` each frame and trim quality when
  it approaches the deadline, recovering slowly while there is headroom.

That combination holds 58.6 fps with roughly one brief overrun per 40 seconds,
where fixed parameters either collapsed or looked thin. Anything dense added
later should follow the same pattern rather than trusting a static guess.

### What the frame budget is actually limited by

Worth knowing before optimising anything else here. Per frame, measured:

| | ISR on core 0 | ISR on core 1 |
|---|---|---|
| clear | 12.2 ms | 10.9 ms |
| draw | 3.0 to 4.1 ms | 2.8 to 3.7 ms |
| present, idle | 0.9 to 1.7 ms | 2.0 to 3.3 ms |

Moving the interrupt off the drawing core bought about 1.3 ms, roughly 8% of
the frame, not the half a core its 26 MB/s of memcpy might suggest. The work is
memory bandwidth, not CPU cycles, which is the same thing the cache line
experiment showed: one store per 64 byte line costs as much as sixteen.

That also settles the obvious next idea. Handing the bounce fill to a GDMA
channel would consume the same PSRAM bandwidth and save only that 1.3 ms of CPU,
which moving cores already recovered. It would cost `no_fb` mode (the
`on_bounce_empty` hook is only consulted when the driver owns no frame buffer),
a hand written buffer swap, and a hard deadline in place of a synchronous copy.
Not worth it.

So filters integrate at 250 Hz while the display runs at 58.5 Hz, instead of
both being pinned to the frame rate. State shared between the two is guarded by
`demo_lock()` / `demo_unlock()`, a spinlock, held only long enough to copy a
struct.

Two filters had a fixed blend coefficient tuned for the old loop rate, which
would have become four to eight times twitchier at 250 Hz. `intuitive_cube` and
`gravity_locked_cube` now derive their coefficient from dt against a fixed time
constant, so they feel the way the originals did.

### i2c_master_probe is not safe alongside other bus traffic

Worth knowing if you add anything that scans the bus. `i2c_master_probe()`
builds its operation list on the stack and publishes a pointer to it in the
shared bus handle, and it reprograms the bus timing:

```c
i2c_operation_t i2c_ops[] = { ... };        /* stack */
bus_handle->i2c_trans = (i2c_transaction_t) { .ops = i2c_ops, ... };
```

Once it returns and drops the bus mutex that pointer dangles. With a single
user of the bus nothing notices. With the sensor task reading at 250 Hz on the
other core, the next transaction picks up the stale pointer and the firmware
panics with StoreProhibited on a recycled stack address. The `diagnostic` demo
therefore calls `demo_sensor_pause(true)` around its scan. Ordinary device
transactions are properly serialised and need no such care.

## Console wiring

The board has two USB-C sockets:

* **UART** goes through a CH343 bridge to UART0. It enumerates whatever the
  firmware is doing, so this is the one esptool talks to and the one configured
  as the primary console. One cable flashes the board, shows the log and drives
  the demo CLIs.
* **USB** is the ESP32-S3's native USB. It only enumerates if the running
  firmware brings it up, which is why the factory demo shows nothing there. It
  is wired up as the secondary console, so log output appears there too, but it
  is output only.

Swap `CONFIG_ESP_CONSOLE_UART_DEFAULT` for `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`
in `sdkconfig.defaults` to invert that (and delete `sdkconfig` so the defaults
are re-read).

Note that `CONFIG_ESPTOOLPY_FLASHMODE_QIO` cannot be honoured on this part:
octal PSRAM and quad flash share data lines, so Kconfig falls back to DIO.

## Status

Builds clean with ESP-IDF v5.3.2, no warnings from any of the sources here, and
runs on real hardware:

* ST7701S comes up at 480x480 with both PSRAM frame buffers
* CST820 answers (chip 0xb7, project 0x98, firmware 0x03)
* QMI8658 answers at 0x6b (revision 0x7c) and streams live data
* the launcher, the ESC exit path and demo selection over the console all work

Also confirmed by eye: the cube renders well formed and centred, and the picture
is steady once the panel runs in bounce buffer mode. Touch selects demos from
the launcher.

The IMU frame was corrected against measurements. Flat with the screen up now
reads (+0.05, -0.06, +0.98) g, matching the convention the demos expect. The
board sits about 3 degrees off level on a desk, which shows up as a small
resting tilt.

**Open item.** The frame still has one unresolved 180 degree spin about Z, which
decides whether tilting the board right leans the cube right or left. Both
candidates are right-handed, so the fusion filters are correct either way and
only the on-screen direction changes. To settle it, hold the board with its
right-hand edge raised and read the accelerometer: the documented convention is
"tilt right is +X", so a positive ax confirms the current mapping and a negative
ax means switching both `BOARD_IMU_*_SIGN` lines to `{ -1.0f, 1.0f, -1.0f }`.

The gyro shows a sizeable zero-rate offset at rest, about (-5.7, -3.7, +0.3)
dps. Every fusion demo calls `qmi8658_calibrate()` and removes it; `axis_test`
deliberately does not, so the raw offset stays visible there.

Known cost, not yet addressed: the cube demos run at roughly 10 to 12 fps
against their 20 fps target, because each frame clears all 450 KB of the buffer
in PSRAM and then waits for VSYNC. Clearing only the region the cube occupies
would roughly double it.
