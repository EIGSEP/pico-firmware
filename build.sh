#!/bin/bash

# Build script for Pico Multi-App Firmware
# Creates pico_multi.uf2 with all apps integrated

set -e

echo "==================================="
echo "Building Pico Multi-App Firmware"
echo "==================================="

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build

# Check if CMAKE_TOOLCHAIN_FILE is set
if [ -z "$PICO_SDK_PATH" ]; then
    echo "WARNING: PICO_SDK_PATH not set. Assuming SDK is in PATH."
fi

# Configure with CMake
echo "Configuring with CMake..."
cmake ..

# Build the project
echo "Building project..."
make -j$(nproc)

# Check if the output file was created
if [ -f "pico_multi.uf2" ]; then
    echo "==================================="
    echo "✅ Build successful!"
    echo "Output: build/pico_multi.uf2"
    echo "==================================="
    
    # Show file size
    ls -lh pico_multi.uf2
else
    echo "❌ Build failed - pico_multi.uf2 not found"
    exit 1
fi