#!/bin/bash

# Build script for Pico C switch network

# Check if PICO_SDK_PATH is set
if [ -z "$PICO_SDK_PATH" ]; then
    echo "Error: PICO_SDK_PATH environment variable is not set"
    echo "Please set it to the location of your Pico SDK installation"
    echo "Example: export PICO_SDK_PATH=/path/to/pico-sdk"
    exit 1
fi

# Create build directory
mkdir -p build
cd build

# Run cmake
cmake ..

# Build the project
make -j$(nproc)

echo "Build complete! The UF2 file is located at: build/switch_network.uf2"
echo "To flash to Pico:"
echo "1. Hold BOOTSEL button while connecting Pico to USB"
echo "2. Copy build/switch_network.uf2 to the RPI-RP2 drive"