#!/bin/bash

# RP2350-LCD-1.28 Build Script
# This script builds the project using CMake and the Pico SDK

set -e  # Exit on any error

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
TARGET_NAME="rp2350_lcd_demo"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if Pico SDK is available
check_pico_sdk() {
    print_status "Checking Pico SDK..."
    
    if [ -z "$PICO_SDK_PATH" ]; then
        print_error "PICO_SDK_PATH environment variable is not set!"
        print_status "Please install Pico SDK and set PICO_SDK_PATH"
        print_status "Example: export PICO_SDK_PATH=/path/to/pico-sdk"
        exit 1
    fi
    
    if [ ! -d "$PICO_SDK_PATH" ]; then
        print_error "Pico SDK directory not found: $PICO_SDK_PATH"
        exit 1
    fi
    
    if [ ! -f "$PICO_SDK_PATH/pico_sdk_init.cmake" ]; then
        print_error "Invalid Pico SDK directory: $PICO_SDK_PATH"
        print_status "Missing pico_sdk_init.cmake file"
        exit 1
    fi
    
    print_success "Pico SDK found at: $PICO_SDK_PATH"
}

# Function to create build directory
setup_build_dir() {
    print_status "Setting up build directory..."
    
    if [ "$1" == "clean" ]; then
        print_status "Cleaning existing build directory..."
        rm -rf "$BUILD_DIR"
    fi
    
    mkdir -p "$BUILD_DIR"
    print_success "Build directory ready: $BUILD_DIR"
}

# Function to configure CMake
configure_cmake() {
    print_status "Configuring CMake..."
    
    cd "$BUILD_DIR"
    
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DPICO_BOARD=pico2 \
          -DPICO_SDK_PATH="$PICO_SDK_PATH" \
          "$PROJECT_DIR"
    
    if [ $? -eq 0 ]; then
        print_success "CMake configuration completed"
    else
        print_error "CMake configuration failed"
        exit 1
    fi
}

# Function to build the project
build_project() {
    print_status "Building project..."
    
    cd "$BUILD_DIR"
    
    # Build with multiple cores if available
    CORES=$(nproc 2>/dev/null || echo 4)
    print_status "Building with $CORES cores..."
    
    make -j$CORES
    
    if [ $? -eq 0 ]; then
        print_success "Build completed successfully"
    else
        print_error "Build failed"
        exit 1
    fi
}

# Function to show build results
show_results() {
    print_status "Build Results:"
    echo "----------------------------------------"
    
    if [ -f "$BUILD_DIR/${TARGET_NAME}.elf" ]; then
        print_success "ELF file: $BUILD_DIR/${TARGET_NAME}.elf"
        
        # Show memory usage
        if command -v arm-none-eabi-size &> /dev/null; then
            echo ""
            print_status "Memory usage:"
            arm-none-eabi-size "$BUILD_DIR/${TARGET_NAME}.elf"
        fi
    fi
    
    if [ -f "$BUILD_DIR/${TARGET_NAME}.uf2" ]; then
        print_success "UF2 file: $BUILD_DIR/${TARGET_NAME}.uf2"
        
        # Show file size
        UF2_SIZE=$(ls -lh "$BUILD_DIR/${TARGET_NAME}.uf2" | awk '{print $5}')
        print_status "UF2 file size: $UF2_SIZE"
        
        echo ""
        print_status "To flash to RP2350:"
        print_status "1. Hold BOOTSEL button while connecting USB"
        print_status "2. Copy ${TARGET_NAME}.uf2 to the RPI-RP2 drive"
        print_status "3. Or use: cp $BUILD_DIR/${TARGET_NAME}.uf2 /media/\$USER/RPI-RP2/"
    fi
    
    if [ -f "$BUILD_DIR/${TARGET_NAME}.bin" ]; then
        print_success "BIN file: $BUILD_DIR/${TARGET_NAME}.bin"
    fi
    
    if [ -f "$BUILD_DIR/${TARGET_NAME}.hex" ]; then
        print_success "HEX file: $BUILD_DIR/${TARGET_NAME}.hex"
    fi
    
    echo "----------------------------------------"
}

# Function to show help
show_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  clean     - Clean build directory before building"
    echo "  config    - Only configure CMake (don't build)"
    echo "  help      - Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  PICO_SDK_PATH - Path to Pico SDK (required)"
    echo ""
    echo "Examples:"
    echo "  $0              # Normal build"
    echo "  $0 clean        # Clean build"
    echo "  $0 config       # Configure only"
}

# Main function
main() {
    print_status "RP2350-LCD-1.28 Build Script"
    print_status "Project: $PROJECT_DIR"
    echo ""
    
    case "${1:-}" in
        help|--help|-h)
            show_help
            exit 0
            ;;
        clean)
            check_pico_sdk
            setup_build_dir clean
            configure_cmake
            build_project
            show_results
            ;;
        config)
            check_pico_sdk
            setup_build_dir
            configure_cmake
            print_success "Configuration complete. Run 'make' in build directory to build."
            ;;
        "")
            check_pico_sdk
            setup_build_dir
            configure_cmake
            build_project
            show_results
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"