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