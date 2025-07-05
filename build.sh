#!/bin/bash

# Build script for Pico Multi-App Firmware
# Creates pico_multi.uf2 with all apps integrated

set -e

echo "==================================="
echo "Building Pico Multi-App Firmware"
echo "==================================="
echo "Target: Raspberry Pi Pico 2 (RP2350)"
echo "Features:"
echo "  • Multi-app firmware with DIP switch selection"
echo "  • Dynamic USB serial numbers (PICO_000, PICO_001, etc.)"
echo "  • Apps: motor, tempctrl, tempmon, imu, lidar, rfswitch"
echo "  • JSON-based command protocol with cJSON"
echo "==================================="

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Set PICO_SDK_PATH if not already set
if [ -z "$PICO_SDK_PATH" ]; then
    if [ -d "../pico-sdk" ]; then
        export PICO_SDK_PATH="$(realpath ../pico-sdk)"
        echo "Using local Pico SDK: $PICO_SDK_PATH"
    elif [ -d "pico-sdk" ]; then
        export PICO_SDK_PATH="$(realpath pico-sdk)"
        echo "Using local Pico SDK: $PICO_SDK_PATH"
    else
        echo "ERROR: PICO_SDK_PATH not set and no local pico-sdk directory found!"
        echo "Please set PICO_SDK_PATH or place the pico-sdk in the project directory."
        exit 1
    fi
else
    echo "Using PICO_SDK_PATH: $PICO_SDK_PATH"
fi

# Configure with CMake for Pico 2 (RP2350)
echo "Configuring with CMake for Pico 2..."
PICO_BOARD=pico2 cmake ..

# Build the project
echo "Building project..."
PICO_BOARD=pico2 make -j$(nproc)

# Check if the output file was created
if [ -f "pico_multi.uf2" ]; then
    echo "==================================="
    echo "✅ Build successful!"
    echo "Output: build/pico_multi.uf2"
    echo ""
    echo "Ready to flash to Pico 2!"
    echo "DIP switch combinations:"
    echo "  000 - Motor app (APP_MOTOR)"
    echo "  001 - Temperature controller (APP_TEMPCTRL)"
    echo "  010 - Temperature monitor (APP_TEMPMON)" 
    echo "  011 - IMU sensor (APP_IMU)"
    echo "  100 - Lidar sensor (APP_LIDAR)"
    echo "  101 - RF switch control (APP_RFSWITCH)"
    echo "==================================="
    
    # Show file size
    ls -lh pico_multi.uf2
else
    echo "❌ Build failed - pico_multi.uf2 not found"
    exit 1
fi