# Rolling back

Everything after `e233283` was verified by measurement but **not by eye**. If
something looks wrong on the panel, this maps what you see to the smallest
change that could have caused it, and says which rollbacks actually apply
cleanly, because not all of them do.

## The last build that was looked at and approved

Tag **`visual-ok`** (`e233283`). At that point the cube rendered well formed and
centred, with no drift.

```bash
git checkout visual-ok
```

What that state has: two frame buffers in PSRAM, immediate mode drawing, sleeps
for pacing, bounce buffers, the corrected IMU frame. What it lacks: everything
after, including the frame rate work.

## What you see, and what probably caused it

| Symptom | Most likely cause | What to do |
|---|---|---|
| Torn rows, flickering bands, strips of stale or garbage content | no frame buffer, composing in the interrupt (`69bcd12`) | `git revert 69bcd12`, applies cleanly |
| Lines gappy, too thick or too thin; circles lumpy; text malformed | the scanline rasteriser (`4d9323c`) | `git checkout render-1-framebuffer` |
| Text clipped by the bezel, or overlapping | overlay row positions (`2e58596`) | edit `DEMO_ROW_TOP` / `DEMO_ROW_BOTTOM` in `demo_common.h`. Do not revert, the old positions were invisible |
| The picker scrolls too far, snaps too eagerly, or feels wrong | the carousel (`bfab8c9`) | tune `CAROUSEL_FRICTION`, `CAROUSEL_SNAP`, `CAROUSEL_PITCH` at the top of `main.c`, or `git revert bfab8c9`, applies cleanly |
| Cube leans the opposite way to how you tilt the board | the IMU frame, unresolved 180 degree spin about Z | set both `BOARD_IMU_*_SIGN` lines in `board_config.h` to `{ -1.0f, 1.0f, -1.0f }` |
| Picture walks sideways | bounce buffers (`17317d0`) | that would be a regression of something already fixed, worth reporting rather than reverting |
| Demos run visibly slow or stutter | frame pacing (`aa09bbe`) or the display list | check the serial stats: `record` should be under 1 ms and `total` about 17 ms |

## Which rollbacks are clean

Tested by actually applying them, not assumed:

| Rollback | Result |
|---|---|
| `git revert bfab8c9` | **clean.** Removes the carousel, restores the list picker |
| `git revert 69bcd12` | **clean.** Restores the two PSRAM frame buffers, keeps the display list |
| `git revert 4d9323c` | conflicts in `gfx.c`, even after reverting `69bcd12` first |
| `git revert 2e58596` | conflicts across five files |
| `git revert aa09bbe` | conflicts in the display driver |

Anything below the display list has been rewritten by later work, so use a tag
checkout rather than a revert to get back there.

## Tags

| Tag | Commit | State |
|---|---|---|
| `visual-ok` | `e233283` | Last build confirmed by eye. Frame buffers, sleeps, immediate mode |
| `render-1-framebuffer` | `dd5512a` | Frame buffers, immediate mode, event paced, corner skip clear, two cores. 58.5 fps |
| `render-2-displaylist` | `4d9323c` | Display list composed into frame buffers |
| `render-3-nofb` | `69bcd12` | No frame buffer, composed in the interrupt. Current renderer |

## Building and flashing a rolled back state

There is no ESP-IDF installed locally; the toolchain runs in Docker. From the
repository root, with Docker Desktop running:

```bash
docker run --rm -v "C:\Users\murr2k\projects\RP2350\esp32-s3-touch-lcd-2.1:/project" \
    -w /project espressif/idf:v5.3.2 \
    sh -c "git config --global --add safe.directory '*'; idf.py build"
```

Then flash over the **UART** socket:

```bash
cd esp32-s3-touch-lcd-2.1/build
python -m esptool --chip esp32s3 -p COM3 -b 460800 \
    --before default_reset --after hard_reset write-flash "@flash_args"
```

Delete `sdkconfig` and rebuild if you have moved between states that changed
`sdkconfig.defaults`, since the defaults are only read when it is absent.
`render-3-nofb` turned `CONFIG_LCD_RGB_ISR_IRAM_SAFE` off, and the states before
it had it on.

## Getting back to the tip

```bash
git checkout esp32-s3-touch-lcd-2.1
```
