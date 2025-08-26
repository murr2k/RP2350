#!/bin/bash
# Quick flash script for RP2350 at BUSID 2-4

echo "RP2350 Quick Flash Script"
echo "========================"
echo ""

# Check arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 <uf2_file>"
    echo "Example: $0 kalman_6dof_config.uf2"
    exit 1
fi

UF2_FILE=$1

# Check file exists
if [ ! -f "$UF2_FILE" ]; then
    echo "Error: File $UF2_FILE not found!"
    exit 1
fi

echo "Target file: $UF2_FILE"
echo ""

# Instructions for Windows
echo "WINDOWS SIDE INSTRUCTIONS:"
echo "=========================="
echo "1. Your device is at BUSID 2-4"
echo ""
echo "2. Attach to WSL (PowerShell Admin):"
echo "   usbipd attach --wsl --busid 2-4"
echo ""
echo "3. For BOOTSEL mode (if needed):"
echo "   - Hold BOOTSEL button"
echo "   - Press RESET while holding BOOTSEL"
echo "   - Device should show as 'RP2 Boot'"
echo "   - Then: usbipd attach --wsl --busid 2-4"
echo ""
echo "Press Enter when device is attached..."
read

# Check device in WSL
echo ""
echo "Checking device status..."
echo "-------------------------"

# Check if visible
if lsusb | grep -q "2e8a"; then
    echo "✓ RP2350 device found in WSL"
    
    # Check picotool access
    if echo "pass" | sudo -S picotool info 2>&1 | grep -q "RP2"; then
        echo "✓ Device accessible with picotool"
        
        # Check if in BOOTSEL mode
        if echo "pass" | sudo -S picotool info 2>&1 | grep -q "BOOTSEL"; then
            echo "✓ Device is in BOOTSEL mode"
            echo ""
            echo "Flashing $UF2_FILE..."
            echo "pass" | sudo -S picotool load "$UF2_FILE" -f
            
            if [ $? -eq 0 ]; then
                echo ""
                echo "✓ Flash successful!"
                echo "Rebooting device..."
                echo "pass" | sudo -S picotool reboot
                echo ""
                echo "Device should now be running the new firmware!"
            else
                echo "✗ Flash failed"
            fi
        else
            echo "⚠ Device not in BOOTSEL mode"
            echo ""
            echo "Attempting to reboot into BOOTSEL mode..."
            echo "pass" | sudo -S picotool reboot -u 2>/dev/null
            
            if [ $? -eq 0 ]; then
                echo "Device rebooting to BOOTSEL..."
                sleep 3
                echo "Flashing..."
                echo "pass" | sudo -S picotool load "$UF2_FILE" -f
                if [ $? -eq 0 ]; then
                    echo "✓ Flash successful!"
                    echo "pass" | sudo -S picotool reboot
                else
                    echo "✗ Flash failed"
                fi
            else
                echo ""
                echo "Could not auto-reboot to BOOTSEL."
                echo "Please manually enter BOOTSEL mode:"
                echo "1. Hold BOOTSEL button"
                echo "2. Press RESET"
                echo "3. Release both"
                echo "4. Re-run this script"
            fi
        fi
    else
        echo "✗ Device not accessible with picotool"
        echo ""
        echo "Try entering BOOTSEL mode manually:"
        echo "1. Hold BOOTSEL button"
        echo "2. Press RESET"
        echo "3. Release both"
        echo "4. In Windows: usbipd attach --wsl --busid 2-4"
        echo "5. Re-run this script"
    fi
else
    echo "✗ No RP2350 device found in WSL"
    echo ""
    echo "Make sure you ran in Windows PowerShell (Admin):"
    echo "  usbipd attach --wsl --busid 2-4"
fi

echo ""
echo "To detach from WSL (return to Windows):"
echo "  usbipd detach --busid 2-4"