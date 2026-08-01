# ESP32-S3-Touch-LCD-2.1 LCD & IMU Demo Collection

A work-alike port of the [RP2350-LCD-1.28 demo collection](../README.md) to the
Waveshare ESP32-S3-Touch-LCD-2.1: a 2.1" round 480x480 touch display with the
same QMI8658 6-axis IMU behind it.

Same demos, same serial protocols, same host-side test tools. See
[PORTING_NOTES.md](PORTING_NOTES.md) for what had to change and why.

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
- **Also on board**: PCF85063 RTC, microSD slot, TCA9554 IO expander,
  battery charger and divider
- **Interfaces**: USB Type-C (native USB), GPIO header, JST battery connector

## Demos

One firmware holds all of them. Pick one by tapping it on the screen or by
pressing its key on the serial console.

| Key | Demo | What it does |
|-----|------|--------------|
| 1 | `display_test` | Colour fields, gradient, geometry, text, backlight sweep |
| 2 | `buffered_cube` | Spinning wireframe cube, no sensors involved |
| 3 | `intuitive_cube` | Cube follows the board's tilt, accelerometer only |
| 4 | `gravity_locked_cube` | Cube stays level while the board turns |
| 5 | `madgwick_cube` | 6-DOF Madgwick orientation with a body axis triad |
| 6 | `configurable_cube` | Interactive cube, full axis and filter CLI |
| 7 | `rotation_test` | Per-axis gyro dials, the axis calibration tool |
| 8 | `axis_test` | Live accelerometer and gyro vectors with raw counts |
| 9 | `kalman_6dof` | Quaternion Kalman filter with the CSV debug stream |
| a | `kalman_6dof_config` | Kalman filter with axis CLI and data injection |
| b | `touch_test` | Draw on the panel, watch coordinates and gestures |
| c | `diagnostic` | I2C scan and hardware report |

To leave a demo and come back to the menu: **hold a finger near the top of the
screen** for about half a second, or press **ESC** (or `~`) on the console.

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

### Flash

The ESP32-S3 has no UF2 bootloader drive. It enumerates as a serial port and
esptool writes over it:

```bash
idf.py -p /dev/ttyACM0 flash monitor     # Linux
idf.py -p COM7 flash monitor             # Windows
./tools/flash.sh /dev/ttyACM0            # same thing with the toolchain sourced
```

If the board does not answer, hold **BOOT**, tap **RESET**, release **BOOT** to
drop it into the ROM bootloader, then flash again.

Leave the monitor with `Ctrl-]`.

## Serial console

115200 8N1 over the USB-C connector (the rate is nominal, it is a USB CDC
device). Any terminal works: `idf.py monitor`, minicom, screen, PuTTY.

```bash
minicom -D /dev/ttyACM0 -b 115200
screen /dev/ttyACM0 115200
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
- The launcher installs the USB Serial/JTAG driver for non blocking reads. If
  your terminal sends only on Enter, that is fine: the demos read characters as
  they arrive.

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
