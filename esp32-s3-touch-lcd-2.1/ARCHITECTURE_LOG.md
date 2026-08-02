# Architecture log

How the display and sensor architecture got to its present shape, in the order
it happened. [PORTING_NOTES.md](PORTING_NOTES.md) records the conclusions; this
records the route, because the route is most of the lesson.

The starting point was the RP2350 design: one firmware per demo, each with a
static 115 KB frame buffer in SRAM, each drawing pixels immediately into it,
each pacing itself with `sleep_ms()`. All four of those choices are gone. None
of the demos were rewritten.

## 1. The port, written blind (`52654a9`)

No compiler and no board were available, so every decision came from datasheets
and Waveshare's sample.

ESP-IDF links one application per project, so the twelve separate `.uf2` images
became one firmware with a runtime launcher. A 480x480 RGB565 frame is 450 KB,
far too big to be a static array, so the frame buffer moved to PSRAM: two of
them, double buffered, with the `gfx_*` calls writing pixels straight in. The
demos kept the sleeps they had inherited.

It compiled on the first attempt, which turned out to prove very little.

## 2. First contact with hardware (`53f40b5`)

Three things only the board could say.

The buzzer sits on EXIO8 of the IO expander and is active high, and the expander
powers up with every output high, so the port's blanket initialisation had it
sounding continuously. The idle pattern now parks that pin low, and does so
before the pins become outputs.

The board has two USB-C sockets. The native one only enumerates if the running
firmware brings it up, which the factory image does not, so nothing appeared on
the host at all. The console moved to UART0 behind the CH343 bridge, and one
cable now flashes, logs and drives the CLIs.

Flash is 16 MB, not the conservative 8 MB assumed.

## 3. The picture rolled sideways (`17317d0`)

The first architecture change forced by a symptom rather than a design.

The LCD's DMA was streaming pixels straight out of PSRAM and losing the bus to
the CPU, which cleared and redrew all 450 KB every frame. A starved pixel FIFO
starts its line a few pixels late, and the error accumulates frame over frame.

Bounce buffers fixed it: the DMA reads two 9.6 KB buffers in internal SRAM and
the driver refills them from PSRAM in the background, so CPU traffic no longer
reaches the panel.

## 4. The IMU frame (`e233283`)

Lying flat with the screen up, the sensor read `az = -0.97 g`, so its +Z points
into the back of the board. Corrected with a 180 degree rotation about X, which
inverts two axes. Negating Z alone would have left a mirrored, left handed frame
and the fusion filters would have wound yaw the wrong way.

## 5. Rendering paced by the panel (`aa09bbe`)

Reading the driver turned up something the design had assumed away:
`esp_lcd_panel_draw_bitmap()` only records which buffer to use next. It does not
wait. The panel keeps reading the old buffer until the current frame ends, which
meant the demos' inherited sleeps had been providing safety rather than pacing,
and only by accident.

Two premises needed correcting before the fix could be built. The RGB peripheral
free-runs off its own timing generator, so nothing triggers the DMA per frame.
And the ISR cannot do the drawing. So the shape is: the interrupt signals, the
task draws.

`on_bounce_frame_finish` fires the moment the outgoing buffer has been fully
read and the incoming one adopted. It gives a semaphore,
`LCD_2IN1_Display()` waits on it, and every sleep in every render loop was
deleted.

**15 fps to 30 fps**, locked to exactly half the panel rate, because per frame
work was 18.4 ms against a 17.1 ms frame.

## 6. What memset is doing (`2e58596`)

The question that reframed the problem.

Measured on the board, over a 450 KB region of PSRAM:

| pass | time | rate |
|---|---|---|
| `memset`, write only | 14,430 us | 30.5 MB/s |
| read only | 8,850 us | 49.7 MB/s |
| read and write | 14,883 us | 29.5 MB/s |
| one store per 64 byte line | 15,199 us | same as writing all of it |

Writing costs the same as reading and writing, because the cache fetches every
line before overwriting it. And writing a sixteenth of the data costs the same
as writing all of it. **The cost is cache lines touched, not bytes written.**

That made the panel being round worth something: the corners of the 480x480
rectangle have no pixels behind them, and skipping them is a fifth of the area.
The clear went from 15 ms to 12.2 ms, and that 1.3 ms was exactly enough to stop
missing the frame boundary. **30 fps to 58.5 fps**, the panel's own rate.

Relocating the status overlays to make that safe revealed that the corner
anchored text inherited from the RP2350 had been invisible on the 1.28" round
panel too.

## 7. Two cores (`76c82a0`)

Sensor acquisition and the demos' filters moved to core 1, publishing samples at
the sensor's 250 Hz rather than at whatever the frame rate happened to be.

This exposed a latent bug in ESP-IDF. `i2c_master_probe()` builds its operation
list on the stack and publishes a pointer to it in the shared bus handle, so once
it returns that pointer dangles. Harmless with a single user of the bus; with a
250 Hz reader on the other core, the next transaction picks up the stale pointer
and panics. The bus scan now quiesces the sensor task first.

Two filters had a fixed blend coefficient tuned for the old loop rate, which at
250 Hz would have made them four to eight times twitchier. Both now derive it
from dt against a fixed time constant.

## 8. Which core takes the interrupt (`dd5512a`)

The bounce ISR copies 450 KB per frame, about 26 MB/s, which suggested it was
costing something like half a core. Moving it to core 1 recovered 1.3 ms, about
8% of the frame.

That measurement also settled whether the bounce fill should be handed to a GDMA
channel: the prize would have been exactly the CPU time this recovered, because
the copy is limited by memory bandwidth and not by cycles, and DMA consumes the
same bandwidth.

## 9. Drawing recorded, not painted (`4d9323c`)

`gfx_*` stopped painting pixels and started appending to a display list, which a
scanline compositor then turned into rows: each row built in internal SRAM from
the background up, written to PSRAM once. Every pixel written exactly once with
its final value, one mechanism for all content.

The API did not change, so no demo was touched. Content no primitive describes,
such as the gradient in `display_test`, registers a row painter.

Performance: a wash, within half a millisecond either way. Composing still wrote
every visible pixel, which was always the 11 ms; all that moved was the scattered
per pixel drawing, from PSRAM into SRAM. Recording, though, dropped to 0.05 to
0.6 ms, and the row based renderer was the prerequisite for the step after.

## 10. No frame buffer (`69bcd12`)

`no_fb` mode. Nothing is stored at frame size at all: the panel's interrupt calls
`on_bounce_empty` when its DMA has drained a bounce buffer, and those ten rows
are composed straight into it. The double buffering moved to the display lists,
two of about 15 KB in SRAM in place of two 450 KB frames in PSRAM.

The first flash panicked with a coprocessor exception. Xtensa forbids the FPU in
an interrupt handler, and the rasterisers were full of floats.
`CONFIG_FREERTOS_FPU_IN_ISR` exists but is ESP32 only, not S3, so the compose
path became integer throughout, including an integer square root for circles and
half row arithmetic for line interpolation.

| | before | after |
|---|---|---|
| render core, per frame | 13.9 ms | 0.03 to 0.5 ms |
| PSRAM free | 6903 KB | 7803 KB |
| PSRAM bandwidth left for the app | 30.5 MB/s | 48.6 MB/s |

## The same frame, restructured six times

`buffered_cube`, milliseconds:

| | clear | draw | compose | wait | total |
|---|---|---|---|---|---|
| sleep paced | 14.6 | 3.8 | | 15.2 | 33.6 |
| event driven | 14.6 | 3.8 | | 14.9 | 33.3 |
| corner skip | 12.2 | 3.8 | | 1.0 | 17.1 |
| ISR on core 1 | 10.9 | 3.7 | | 3.1 | 17.1 |
| display list | | 0.1 | 13.8 | 3.1 | 17.1 |
| no frame buffer | | 0.1 | in ISR | 17.0 | 17.1 |

The total stops moving at the third row because the panel sets it. Everything
after that is redistribution: work leaving the render core, then leaving the
memory bus.

## What the measurements taught

Three predictions made along the way were wrong, all in the same direction.

* Undraw would make the clear "disappear, under 1 ms". Cache line granularity
  put it nearer 6 ms, because a diagonal line touches a new line every row.
* The bounce ISR was "roughly half a core". It was 8% of the frame.
* The display list would save 1.5 to 2 ms. It saved nothing measurable.

Each time the reasoning was about CPU work when the answer was the memory bus.
The turn came from one question, what `memset` is actually doing, because
answering it properly required the experiment that made the bus visible. After
that the remaining decisions were straightforward, including the ones that were
decisions not to build something.

## What never changed

The twelve demos. The `gfx_*` drawing API. The serial command grammars, the
23 field CSV stream, the injection protocol, and the host tools that drive them.

The drawing interface written blind, before the board was ever powered on,
survived four rewrites of everything beneath it.
