# ESP32-S3-Touch-LCD-2.1 LCD & IMU Demo Collection

A work-alike port of the [RP2350-LCD-1.28 demo collection](../README.md) to the
Waveshare ESP32-S3-Touch-LCD-2.1: a 2.1" round 480x480 touch display with the
same QMI8658 6-axis IMU behind it.

Same demos, same serial protocols, same host-side test tools. See
[PORTING_NOTES.md](PORTING_NOTES.md) for what had to change and why, and
[ARCHITECTURE_LOG.md](ARCHITECTURE_LOG.md) for how the display and sensor
architecture arrived at its present shape, measurement by measurement.

## Author

Murray Kopit (murr2k@gmail.com)
License: MIT

## Hardware

[Waveshare ESP32-S3-Touch-LCD-2.1](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.1):

- **MCU**: ESP32-S3R8, dual Xtensa LX7 @ 240 MHz
- **Memory**: 512 KB SRAM, 8 MB octal PSRAM, 8 or 16 MB flash
- **Display**: 2.1" round IPS, 480x480, ST7701S, 16-bit parallel RGB
- **Touch**: CST820 capacitive controller, single point plus gestures
- **IMU**: QMI8658, 3-axis accelerometer and 3-axis gyroscope, I2C
- **Also on board**: PCF85063 RTC, microSD slot, TCA9554 IO expander, buzzer on
  expander pin EXIO8, battery charger and divider
- **Interfaces**: two USB Type-C sockets (see below), GPIO header, JST battery
  connector

## Demos

One firmware holds all of them. Pick one by tapping it on the screen or by
pressing its key on the serial console.

| Key | Demo | What it does |
|-----|------|--------------|
| 1 | `display_test` | Colour fields, gradient, geometry, text, backlight sweep |
| 2 | `buffered_cube` | Spinning wireframe cube, no sensors involved |
| 3 | `finger_cube` | Spin a cube with your finger, flick it to throw it |
| 4 | `intuitive_cube` | Cube follows the board's tilt, accelerometer only |
| 5 | `gravity_locked_cube` | Cube stays level while the board turns |
| 6 | `madgwick_cube` | 6-DOF Madgwick orientation with a body axis triad |
| 7 | `configurable_cube` | Interactive cube, full axis and filter CLI |
| 8 | `rotation_test` | Per-axis gyro dials, the axis calibration tool |
| 9 | `axis_test` | Live accelerometer and gyro vectors with raw counts |
| a | `kalman_6dof` | Quaternion Kalman filter with the CSV debug stream |
| b | `kalman_6dof_config` | Kalman filter with axis CLI and data injection |
| c | `touch_test` | Draw on the panel, watch coordinates and gestures |
| d | `diagnostic` | I2C scan and hardware report |
| e | `rain` | Neopixel rain screensaver, also runs itself when idle |

To leave a demo and come back to the menu: **hold a finger near the top of the
screen** for about half a second, or press **ESC** (or `~`) on the console.

## The clock

The picker shows the time of day, read from the PCF85063 on the I2C bus. There
is no backup cell fitted, so the part comes up after every power cycle with its
oscillator stop flag set and no idea what time it is. The firmware therefore
joins a network at boot, asks an NTP server, sets the clock and **takes the
radio straight back down**. It shows `--:--:--` until that lands, and the status
line along the bottom says where it has got to.

Taking the radio down again is not tidiness. The panel has no frame buffer and
composes each row inside an interrupt against a deadline set by the pixel clock,
and with XIP enabled the WiFi stack executes from the same PSRAM the composer is
reading. Staying associated would put an unpredictable second consumer on that
bandwidth permanently, for a reading taken once. Measured across the fetch:
composition never moved from its usual 203 to 267 us against a 448 us trim
threshold, and no rows were ever dropped. What did move was the render loop,
from about 250 us a frame to 750 us while the radio was up. That is the cost,
and it is why it does not stay up.

To point it at your own network, copy the example and fill it in:

```sh
cp main/wifi_secrets.h.example main/wifi_secrets.h
```

`wifi_secrets.h` is in `.gitignore`, because this repository is public and a
password committed once stays in the history whatever you do to it afterwards.
Without the file the firmware still builds and runs; it reports `TIME no net`
and leaves the clock unset.

The zone is one line at the top of `main/net_time.c`, a POSIX TZ string. It ships
set to US Pacific, and the two rules on the end are the summer time changeover,
so the clock stays right across it without anyone touching anything.

## Build environment

### Prerequisites

- Ubuntu 22.04, WSL2 or macOS
- **ESP-IDF v5.3 or newer** (v5.2 will not work: the port uses the current
  `i2c_master` driver API)

### Install ESP-IDF

```bash
./tools/install_esp_idf.sh
# then, in every new shell:
. ~/esp/esp-idf/export.sh
```

Or by hand:

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.3.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3 && . ./export.sh
```

### Build

```bash
cd esp32-s3-touch-lcd-2.1
idf.py set-target esp32s3
idf.py build
```

or `./tools/build.sh`, which sources the toolchain for you.

Output: `build/esp32s3_lcd_demo.bin`.

### Which USB socket

The board has two USB-C sockets and they are not interchangeable:

| Socket | Goes to | Use it for |
|--------|---------|------------|
| **UART** | CH343 bridge to UART0 | flashing, logs, and the demo CLIs |
| **USB** | ESP32-S3 native USB | log output only (secondary console) |

**Use the UART socket.** It enumerates as a COM port no matter what firmware is
running, which is why esptool can always reach it. The USB socket only appears
on the host if the running firmware brings the native USB up, so a board sitting
in an unknown state shows nothing there.

A charge-only USB-C cable will power the board and light the display while
carrying no data. If no port appears, suspect the cable first.

### Flash

The ESP32-S3 has no UF2 bootloader drive. It enumerates as a serial port and
esptool writes over it:

```bash
idf.py -p COM3 flash monitor             # Windows, CH343 bridge
idf.py -p /dev/ttyUSB0 flash monitor     # Linux
./tools/flash.sh /dev/ttyUSB0            # same thing with the toolchain sourced
```

esptool drives the auto-reset lines, so the board does not need to be put into
download mode by hand. If it does not answer, hold **BOOT**, tap **RESET**,
release **BOOT**, then flash again.

Leave the monitor with `Ctrl-]`.

## Serial console

115200 8N1 on the **UART** socket. Any terminal works: `idf.py monitor`,
minicom, screen, PuTTY.

```bash
minicom -D /dev/ttyUSB0 -b 115200
screen /dev/ttyUSB0 115200
```

At the launcher: press a demo key, `?` to reprint the menu.

### configurable_cube CLI

Press `m` for the menu.

**Axis controls**
- `1`-`6`: toggle axis inversions (1=AX 2=AY 3=AZ 4=GX 5=GY 6=GZ)
- `q`/`w`/`s`: swap accelerometer axes (X<->Y, X<->Z, Y<->Z)
- `a`/`z`/`x`: swap gyroscope axes (X<->Y, X<->Z, Y<->Z)

**Display controls**
- `d`: sensor data overlay, `c`: cube, `e`: axes, `p`: pause

**Filter controls**
- `+`/`-`: Madgwick beta, `r`: reset orientation, `b`: recalibrate gyro bias

**Information**
- `v`: current sensor values, `m`: menu, `i`: axis configuration

### kalman_6dof CLI

- `r` reset orientation (bias preserved), `m` display mode, `h` help
- `+`/`-` R_measure, `q`/`a` Q_angle
- `d` toggle the CSV debug stream, bracketed by `DEBUG_START` / `DEBUG_STOP`
- `p` print parameters
- `P<X>=<value>` set a parameter: `PK=` gain, `PI=` integral, `PF=` filter alpha,
  `PB=x,y,z` gyro bias

### kalman_6dof_config CLI

Adds runtime axis configuration and data injection, the interface the
regression harness drives:

- `AG<n>=<+-1>`, `AA<n>=<+-1>`, `AS<n>=<scale>`, `AW<n>=<weight>`, `AMAP=a,b,c`
- `TX=30`, `TY=20`, `TZ=10`, `TA=1`, `TB=1`, `TC=1`, `TM=45`, `T0=0`
- `I<ax>,<ay>,<az>,<gx>,<gy>,<gz>` injects one sample and echoes a `CSV,` line
- `r`, `d`, `p`, `h`

## Host tools

The Python tools in the repository root talk to a TCP socket on port 9999
rather than to a serial port, so start the bridge first:

```bash
python tools/serial_bridge.py --port /dev/ttyACM0
# then, from the repository root
python regression_test.py
python monitor_axes.py
python test_injection.py
```

The bridge autodetects an Espressif USB Serial/JTAG port when `--port` is
omitted.

## Project layout

```
esp32-s3-touch-lcd-2.1/
├── CMakeLists.txt              ESP-IDF project
├── sdkconfig.defaults          PSRAM, flash, console and FreeRTOS settings
├── partitions.csv
├── components/
│   ├── board/                  pin map, I2C, IO expander, backlight, battery
│   ├── display/                ST7701S + RGB panel, drawing helpers, 5x7 font
│   └── sensors/                QMI8658 IMU, CST820 touch
├── main/
│   ├── main.c                  launcher, menu, demo registry
│   ├── demo_common.[ch]        shared geometry, filters and launcher services
│   └── demos/                  one file per demo
└── tools/                      build, flash, ESP-IDF install, serial bridge
```

## Technical details

### Display

- 480x480 RGB565, 16-bit parallel RGB at a 16 MHz pixel clock
- Two 450 KB frame buffers in PSRAM, flipped on present, so no tearing
- ST7701S brought up over 9-bit SPI before the RGB interface starts
- Chip select and reset go through the TCA9554 at I2C address 0x20

Screen drift on RGB panels comes from the PSRAM bus stalling while the cache
fetches from flash. `sdkconfig.defaults` avoids it by keeping code and constants
in PSRAM (`CONFIG_SPIRAM_FETCH_INSTRUCTIONS`, `CONFIG_SPIRAM_RODATA`). The other
cure is a bounce buffer: see the comment in `components/display/lcd_2in1.c`.

### IMU

- I2C address 0x6B (0x6A is probed as a fallback), WHO_AM_I returns 0x05
- Accelerometer +/-2 g, 16384 LSB/g
- Gyroscope +/-256 dps, 128 LSB/dps
- Both at 250 Hz, read as a single 12-byte burst

### Madgwick filter

- Beta 0.1 by default, adjustable at runtime
- Unit quaternion output, Euler angles derived for display

## Troubleshooting

**Black screen**
- Check that the TCA9554 answers at 0x20: run `diagnostic`, which scans the bus.
  Nothing works if the expander is missing, since it holds the panel in reset.
- Confirm PSRAM is detected. Without it the frame buffers cannot be allocated
  and `LCD_2IN1_Init()` fails. `diagnostic` prints the size.

**Picture drifts sideways or tears**
- Keep `CONFIG_SPIRAM_FETCH_INSTRUCTIONS` and `CONFIG_SPIRAM_RODATA` on, or
  enable the bounce buffer.

**IMU not responding**
- `diagnostic` shows every device on the bus. Expect 0x15 (touch), 0x20
  (expander), 0x51 (RTC) and 0x6B (IMU).

**Touch not responding**
- The CST820 sleeps if it is not kept awake. `cst820_init()` disables auto sleep;
  if the controller was reset by something else, leave the demo and come back.

**Console input ignored**
- Keyboard input only works on the **UART** socket. The native USB socket is
  configured as a secondary console, which is output only.

**Buzzer sounds continuously**
- EXIO8 on the IO expander is the buzzer and it is active high, while the
  expander's output register powers up as 0xFF. `DEV_Module_Init()` loads
  `BOARD_EXIO_IDLE_STATE` before switching the pins to outputs for exactly this
  reason. Anything that writes 0xFF to the expander will start the tone.

**Wrong axis directions**
- Expected on a new board. Run `rotation_test`, then `configurable_cube`, then
  write the result into `BOARD_IMU_*` in
  `components/board/include/board_config.h`.

## Dependencies

- [ESP-IDF v5.3+](https://github.com/espressif/esp-idf)
- Waveshare's ST7701S init sequence for this panel
- Python 3 with pyserial for the host tools

## References

- [Waveshare ESP32-S3-Touch-LCD-2.1 wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.1)
- [ESP-IDF RGB LCD driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/lcd/rgb_lcd.html)
- [QMI8658 datasheet](https://www.waveshare.com/w/upload/1/18/Qmi8658a.pdf)
- [Madgwick filter paper](https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf)

## License

MIT, see [LICENSE](../LICENSE).
