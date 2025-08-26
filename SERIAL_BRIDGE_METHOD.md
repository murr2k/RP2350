# Serial Bridge Method for RP2350 Communication

## Overview
Direct serial communication from WSL to the RP2350 often hangs or fails due to USB passthrough limitations. The TCP bridge method provides a reliable alternative by running a Python bridge on the Windows side that forwards serial data over TCP.

## Setup

### 1. Windows Side - Serial Bridge Script
Location: `C:\Users\Public\serial_bridge.py`

```python
#!/usr/bin/env python3
import serial
import socket
import threading
import time
import signal
import sys

# Configuration
SERIAL_PORT = 'COM6'  # Adjust to your device's COM port
BAUD_RATE = 115200
TCP_HOST = '0.0.0.0'
TCP_PORT = 9999

# Global flags for clean shutdown
running = True
client_socket = None
server_socket = None
serial_port = None

def signal_handler(sig, frame):
    """Handle Ctrl-C for clean shutdown"""
    global running
    print("\n\nShutting down...")
    running = False
    
    # Close connections
    try:
        if client_socket:
            client_socket.close()
    except:
        pass
    
    try:
        if server_socket:
            server_socket.close()
    except:
        pass
    
    try:
        if serial_port:
            serial_port.close()
    except:
        pass
    
    print("Goodbye!")
    sys.exit(0)

def serial_to_tcp(ser, client):
    """Forward serial data to TCP client"""
    global running
    try:
        while running:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                client.send(data)
            time.sleep(0.001)
    except Exception as e:
        if running:
            print(f"Serial->TCP error: {e}")

def tcp_to_serial(ser, client):
    """Forward TCP data to serial"""
    global running
    try:
        client.settimeout(0.1)  # Non-blocking with timeout
        while running:
            try:
                data = client.recv(1024)
                if not data:
                    break
                ser.write(data)
            except socket.timeout:
                continue  # Normal timeout, continue loop
    except Exception as e:
        if running:
            print(f"TCP->Serial error: {e}")

def main():
    global running, client_socket, server_socket, serial_port
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("Serial-TCP Bridge")
    print("=" * 50)
    print(f"Serial Port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"TCP Server: {TCP_HOST}:{TCP_PORT}")
    print("Press Ctrl+C to exit")
    print("=" * 50)
    
    try:
        # Open serial port
        serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✓ Serial port {SERIAL_PORT} opened")
    except Exception as e:
        print(f"✗ Failed to open serial port: {e}")
        print("\nTroubleshooting:")
        print("1. Check the COM port number in Device Manager")
        print("2. Ensure no other program is using the port")
        print("3. Verify the device is connected")
        sys.exit(1)
    
    try:
        # Create TCP server
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(1.0)  # Timeout for accept() to check running flag
        server_socket.bind((TCP_HOST, TCP_PORT))
        server_socket.listen(1)
        print(f"✓ TCP server listening on {TCP_HOST}:{TCP_PORT}")
    except Exception as e:
        print(f"✗ Failed to create TCP server: {e}")
        serial_port.close()
        sys.exit(1)
    
    print("\n" + "=" * 50)
    print("Bridge is running!")
    print("=" * 50 + "\n")
    
    while running:
        try:
            print("Waiting for connection... (Press Ctrl+C to exit)")
            
            # Accept with timeout to allow checking running flag
            try:
                client_socket, addr = server_socket.accept()
            except socket.timeout:
                continue  # Check running flag and try again
            
            if not running:
                break
                
            print(f"✓ Connection from {addr}")
            
            # Create bidirectional threads
            t1 = threading.Thread(target=serial_to_tcp, args=(serial_port, client_socket))
            t2 = threading.Thread(target=tcp_to_serial, args=(serial_port, client_socket))
            
            t1.daemon = True
            t2.daemon = True
            t1.start()
            t2.start()
            
            # Wait for threads to finish or interrupt
            while t1.is_alive() and t2.is_alive() and running:
                time.sleep(0.1)
            
            # Close client connection
            try:
                client_socket.close()
            except:
                pass
            client_socket = None
            
            if running:
                print("✗ Client disconnected")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            if running:
                print(f"Error in main loop: {e}")
    
    # Cleanup
    signal_handler(None, None)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        signal_handler(None, None)
```

### 2. Start the Bridge

**On Windows (PowerShell or CMD):**
```bash
cd C:\Users\Public
python serial_bridge.py
```

Output should show:
```
==================================================
Serial-TCP Bridge
==================================================
Serial Port: COM6 @ 115200 baud
TCP Server: 0.0.0.0:9999
Press Ctrl+C to exit
==================================================
✓ Serial port COM6 opened
✓ TCP server listening on 0.0.0.0:9999

==================================================
Bridge is running!
==================================================

Waiting for connection... (Press Ctrl+C to exit)
```

**To stop the bridge:**
- Press `Ctrl+C` in the terminal
- The bridge will cleanly close all connections and exit

### 3. Connect from WSL

The bridge is accessible at `192.168.64.1:9999` (WSL's gateway IP).

## Usage Examples

### Check Connection
```bash
nc -w 1 192.168.64.1 9999 < /dev/null && echo "Bridge active" || echo "Bridge not running"
```

### Send Single Command
```bash
echo "r" | nc -w 1 192.168.64.1 9999
```

### Enable Debug Stream
```bash
(echo "d"; sleep 2) | nc -w 3 192.168.64.1 9999 | head -20
```

### Interactive Session
```bash
nc 192.168.64.1 9999
# Type commands interactively
# Ctrl+C to exit
```

### Test Gyro Axes
```bash
# Test X axis
(echo "TX=30"; sleep 3; echo "T0=0") | nc -w 5 192.168.64.1 9999 | grep CSV

# Test Y axis  
(echo "TY=30"; sleep 3; echo "T0=0") | nc -w 5 192.168.64.1 9999 | grep CSV

# Test Z axis
(echo "TZ=30"; sleep 3; echo "T0=0") | nc -w 5 192.168.64.1 9999 | grep CSV
```

### Monitor Orientation
```bash
(echo "r"; sleep 0.5; echo "d"; sleep 5; echo "d") | nc -w 7 192.168.64.1 9999 | \
    grep CSV | awk -F, '{print "P:"$9" R:"$10" Y:"$11}' | tail -20
```

### Apply Configuration
```bash
# Remap axes
echo "AMAP=1,0,2" | nc -w 1 192.168.64.1 9999

# Invert gyro axis
echo "AG0=-1" | nc -w 1 192.168.64.1 9999

# Scale gyro axis
echo "AS0=2.0" | nc -w 1 192.168.64.1 9999
```

## Python Scripts Using Bridge

### Basic Connection Test
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.64.1', 9999))
sock.send(b"r\n")  # Reset command
response = sock.recv(1024)
print(response.decode())
sock.close()
```

### Continuous Monitoring
```python
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.64.1', 9999))
sock.settimeout(0.1)

# Enable debug stream
sock.send(b"d\n")
time.sleep(0.5)

# Monitor for 10 seconds
start = time.time()
while time.time() - start < 10:
    try:
        data = sock.recv(1024).decode('utf-8', errors='ignore')
        for line in data.split('\n'):
            if 'CSV' in line:
                print(line)
    except socket.timeout:
        continue

# Disable debug stream
sock.send(b"d\n")
sock.close()
```

## Advantages

1. **Reliability**: No USB passthrough issues or serial port hanging
2. **Flexibility**: Works from any WSL instance or even remote machines
3. **Multiple Connections**: Can reconnect without device issues
4. **Debugging**: Can see connection status on Windows side
5. **Performance**: TCP buffering handles high-speed data better

## Troubleshooting

### Bridge Won't Start
- Check COM port number in Device Manager
- Ensure no other program is using the COM port
- Install pyserial: `pip install pyserial`

### Can't Connect from WSL
- Check Windows Firewall isn't blocking port 9999
- Verify gateway IP: `ip route | grep default`
- Ensure bridge shows "Waiting for connection..."

### Data Not Flowing
- Check device is powered and running
- Verify baud rate matches (115200)
- Try reset command: `echo "r" | nc -w 1 192.168.64.1 9999`

## Notes

- The bridge maintains the connection until either side disconnects
- Multiple WSL clients cannot connect simultaneously (one at a time)
- The bridge automatically reconnects when a new client connects
- No need to restart the bridge when flashing new firmware
- The device stays attached to Windows (no usbipd needed for serial)