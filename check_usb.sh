#!/bin/bash
# Check USB device status for RP2350

echo "USB Device Check for RP2350"
echo "==========================="
echo ""

echo "1. USB devices in WSL:"
echo "-----------------------"
lsusb 2>/dev/null | grep -E "2e8a|Boot|Serial" || echo "No RP devices found"
echo ""

echo "2. Check with picotool:"
echo "-----------------------"
echo "pass" | sudo -S picotool info 2>&1 | grep -v password || echo "Picotool cannot access device"
echo ""

echo "3. Check serial devices:"
echo "------------------------"
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
echo ""

echo "4. Check dmesg for recent USB events:"
echo "--------------------------------------"
echo "pass" | sudo -S dmesg | tail -20 | grep -E "usb|USB|tty" || echo "No recent USB events"
echo ""

echo "Windows-side commands to attach device:"
echo "----------------------------------------"
echo "In PowerShell (Admin):"
echo "  usbipd list                    # List all USB devices"
echo "  usbipd bind --busid X-X         # Bind device (one time)"
echo "  usbipd attach --wsl --busid X-X # Attach to WSL"
echo ""
echo "To enter BOOTSEL mode:"
echo "  Hold BOOTSEL button while pressing RESET"
echo ""
echo "To flash without BOOTSEL (if supported):"
echo "  sudo picotool reboot -u         # Reboot to BOOTSEL"
echo "  sudo picotool load file.uf2 -f  # Flash file"
echo "  sudo picotool reboot            # Reboot normally"