#!/usr/bin/env python3
"""Bridge the board's USB serial port to a TCP socket.

The host-side test tools in the repository root (regression_test.py,
monitor_axes.py, test_injection.py, ...) all speak to a TCP socket on port 9999
rather than to a serial port directly, because the original setup ran the tools
inside WSL while the board enumerated on Windows. This is the same bridge as
serial_bridge_fixed.py, with the port configurable instead of hardcoded.

    python tools/serial_bridge.py --port COM7          # Windows
    python tools/serial_bridge.py --port /dev/ttyACM0  # Linux
    python tools/serial_bridge.py                      # autodetect

The ESP32-S3 exposes a USB Serial/JTAG CDC port, so the baud rate is ignored by
the hardware but pyserial still wants a number.
"""

import argparse
import select
import signal
import socket
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

RUNNING = True


def autodetect() -> str:
    """Pick the first port that looks like an Espressif USB Serial/JTAG device."""
    candidates = list(list_ports.comports())
    for port in candidates:
        # 303a is the Espressif vendor id; 1001 is the USB Serial/JTAG function.
        if port.vid == 0x303A:
            return port.device
    if candidates:
        return candidates[0].device
    sys.exit("No serial ports found. Pass --port explicitly.")


def pump(source, sink, name, closed):
    """Move bytes from source() to sink() until either end goes away."""
    try:
        while RUNNING and not closed.is_set():
            data = source()
            if data:
                sink(data)
            else:
                time.sleep(0.001)
    except Exception as exc:  # noqa: BLE001 - the bridge should report and move on
        if RUNNING:
            print(f"{name} stopped: {exc}")
    finally:
        closed.set()


def main() -> int:
    global RUNNING

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, autodetected when omitted")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--tcp-host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=9999)
    args = parser.parse_args()

    port = args.port or autodetect()
    print(f"Serial: {port} @ {args.baud}")

    ser = serial.Serial(port, args.baud, timeout=0)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.tcp_host, args.tcp_port))
    server.listen(1)
    print(f"Listening on {args.tcp_host}:{args.tcp_port}, Ctrl-C to stop")

    def shutdown(_sig, _frame):
        global RUNNING
        RUNNING = False
        try:
            server.close()
        except OSError:
            pass
        try:
            ser.close()
        except OSError:
            pass
        print("\nBridge closed")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    while RUNNING:
        client, address = server.accept()
        print(f"Client connected from {address[0]}")
        closed = threading.Event()

        def from_serial():
            return ser.read(ser.in_waiting or 1)

        def from_client():
            # Blocking socket plus a poll, so sendall() on the other thread
            # never has to deal with a partially written buffer.
            readable, _, _ = select.select([client], [], [], 0.01)
            if not readable:
                return b""
            data = client.recv(4096)
            if not data:
                raise ConnectionResetError("client closed the connection")
            return data

        threads = [
            threading.Thread(target=pump,
                             args=(from_serial, client.sendall, "serial->tcp", closed),
                             daemon=True),
            threading.Thread(target=pump,
                             args=(from_client, ser.write, "tcp->serial", closed),
                             daemon=True),
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        client.close()
        print("Client disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
