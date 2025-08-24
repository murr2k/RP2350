#!/bin/bash

# USB Serial Setup Script for WSL -> Windows COM Port Communication
# This script helps set up USB serial communication for RP2350 development in WSL

echo "=== RP2350 USB Serial Setup for WSL ==="

# Function to check if running in WSL
check_wsl() {
    if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null ; then
        echo "✓ Running in WSL environment"
        return 0
    else
        echo "✗ Not running in WSL - this script is for WSL environments"
        return 1
    fi
}

# Function to install required packages
install_packages() {
    echo "Installing required packages..."
    
    # Update package list
    sudo apt update
    
    # Install serial communication tools
    sudo apt install -y \
        minicom \
        screen \
        picocom \
        setserial \
        python3-serial \
        usbutils \
        socat
    
    echo "✓ Packages installed"
}

# Function to set up udev rules
setup_udev_rules() {
    echo "Setting up udev rules for RP2350..."
    
    # Create udev rule for RP2350
    sudo tee /etc/udev/rules.d/99-rp2350.rules > /dev/null <<EOF
# RP2350 USB Serial
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666", GROUP="dialout", SYMLINK+="rp2350"

# RP2350 in bootloader mode
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666", GROUP="plugdev"
EOF

    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    echo "✓ Udev rules configured"
}

# Function to add user to dialout group
setup_user_permissions() {
    echo "Setting up user permissions..."
    
    # Add user to dialout and plugdev groups
    sudo usermod -a -G dialout,plugdev $USER
    
    echo "✓ User added to dialout and plugdev groups"
    echo "⚠  You may need to log out and log back in for group changes to take effect"
}

# Function to create connection script
create_connection_script() {
    echo "Creating connection scripts..."
    
    # Create directory for scripts
    mkdir -p ~/bin
    
    # Create minicom connection script
    cat > ~/bin/connect_rp2350.sh <<EOF
#!/bin/bash

# RP2350 Connection Script
# Usage: connect_rp2350.sh [port] [baudrate]

PORT=\${1:-/dev/rp2350}
BAUDRATE=\${2:-115200}

echo "Connecting to RP2350 on \$PORT at \$BAUDRATE baud..."
echo "Press Ctrl+A then X to exit minicom"

# Check if device exists
if [ ! -e "\$PORT" ]; then
    echo "Device \$PORT not found!"
    echo "Available devices:"
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
    echo ""
    echo "In Windows, check Device Manager for the COM port number"
    echo "The device might appear as /dev/ttyACM0 or similar"
    exit 1
fi

# Connect using minicom
minicom -D \$PORT -b \$BAUDRATE
EOF

    # Create screen connection script
    cat > ~/bin/screen_rp2350.sh <<EOF
#!/bin/bash

# RP2350 Screen Connection Script
PORT=\${1:-/dev/rp2350}
BAUDRATE=\${2:-115200}

echo "Connecting to RP2350 using screen..."
echo "Press Ctrl+A then K then Y to exit"

screen \$PORT \$BAUDRATE
EOF

    # Create picocom connection script
    cat > ~/bin/picocom_rp2350.sh <<EOF
#!/bin/bash

# RP2350 Picocom Connection Script
PORT=\${1:-/dev/rp2350}
BAUDRATE=\${2:-115200}

echo "Connecting to RP2350 using picocom..."
echo "Press Ctrl+A then Ctrl+X to exit"

picocom -b \$BAUDRATE \$PORT
EOF

    # Make scripts executable
    chmod +x ~/bin/*.sh
    
    echo "✓ Connection scripts created in ~/bin/"
}

# Function to create WSL USB bridge setup
create_usb_bridge_info() {
    cat > ~/rp2350_usb_bridge.md <<EOF
# RP2350 USB Bridge Setup for WSL

## Option 1: Using usbipd-win (Recommended)

1. Install usbipd-win on Windows:
   - Download from: https://github.com/dorssel/usbipd-win/releases
   - Or use winget: \`winget install usbipd\`

2. In Windows PowerShell (as Administrator):
   \`\`\`powershell
   # List USB devices
   usbipd wsl list
   
   # Find your RP2350 device (usually Raspberry Pi Pico)
   # Bind the device (replace X-X with your device bus ID)
   usbipd bind --busid X-X
   
   # Attach to WSL
   usbipd wsl attach --busid X-X
   \`\`\`

3. In WSL:
   \`\`\`bash
   # Check if device is available
   ls -la /dev/ttyACM*
   
   # Connect to device
   ./connect_rp2350.sh /dev/ttyACM0
   \`\`\`

## Option 2: Using COM Port Bridge

If direct USB passthrough doesn't work, you can bridge the Windows COM port:

1. In Windows, note the COM port (e.g., COM3) from Device Manager

2. Use socat to bridge (run this in WSL):
   \`\`\`bash
   # Install socat if not already installed
   sudo apt install socat
   
   # Bridge Windows COM port to Unix socket
   socat pty,link=/tmp/rp2350,raw,echo=0 exec:'/mnt/c/Windows/System32/mode.com COM3 BAUD=115200 PARITY=n DATA=8 STOP=1'
   
   # In another terminal, connect to the bridge
   ./connect_rp2350.sh /tmp/rp2350
   \`\`\`

## Option 3: Using Windows Terminal

You can also connect directly from Windows using:
- PuTTY
- Arduino Serial Monitor  
- Windows Terminal with PowerShell

## Troubleshooting

- If device not found, check Windows Device Manager for COM port
- Ensure RP2350 is in the correct mode (not bootloader mode)
- Try different USB cables and ports
- Check Windows driver installation

## Useful Commands

\`\`\`bash
# List USB devices in WSL
lsusb

# List serial devices
ls -la /dev/tty*

# Check udev rules
udevadm info -a -n /dev/ttyACM0

# Monitor USB device connections
sudo udevadm monitor
\`\`\`
EOF

    echo "✓ USB bridge setup guide created: ~/rp2350_usb_bridge.md"
}

# Function to test setup
test_setup() {
    echo "Testing setup..."
    
    # Check if groups are correct
    if groups $USER | grep -q dialout; then
        echo "✓ User is in dialout group"
    else
        echo "⚠  User not in dialout group - may need to log out/in"
    fi
    
    # Check if scripts exist
    if [ -f ~/bin/connect_rp2350.sh ]; then
        echo "✓ Connection scripts created"
    else
        echo "✗ Connection scripts missing"
    fi
    
    # Check for serial devices
    echo "Current serial devices:"
    ls -la /dev/ttyACM* /dev/ttyUSB* /dev/rp2350 2>/dev/null || echo "No serial devices found"
    
    echo ""
    echo "Setup completed! Next steps:"
    echo "1. Connect your RP2350 device to USB"
    echo "2. Flash firmware with USB serial enabled"
    echo "3. Use: ~/bin/connect_rp2350.sh to connect"
    echo "4. Read ~/rp2350_usb_bridge.md for WSL USB setup"
}

# Main execution
main() {
    echo "Starting RP2350 USB Serial setup..."
    echo ""
    
    if ! check_wsl; then
        exit 1
    fi
    
    install_packages
    setup_udev_rules  
    setup_user_permissions
    create_connection_script
    create_usb_bridge_info
    test_setup
    
    echo ""
    echo "=== Setup Complete ==="
    echo "You may need to logout and login again for group changes to take effect."
}

# Run main function
main "$@"