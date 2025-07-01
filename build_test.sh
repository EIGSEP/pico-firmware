#!/bin/bash

# Build script for DIP switch test harness

set -e

echo "==================================="
echo "Building DIP Switch Test Harness"
echo "==================================="

# Check if test build directory exists
if [ ! -d "build_test" ]; then
    echo "Creating test build directory..."
    mkdir build_test
fi

cd build_test

# Configure with CMake using the test CMakeLists
echo "Configuring test with CMake..."
cmake -f ../test_dip_CMakeLists.txt ..

# Build the test project
echo "Building test project..."
make -j$(nproc)

# Check if the output file was created
if [ -f "test_dip.uf2" ]; then
    echo "==================================="
    echo "✅ Test build successful!"
    echo "Output: build_test/test_dip.uf2"
    echo "==================================="
    
    # Show file size
    ls -lh test_dip.uf2
    
    echo ""
    echo "Usage:"
    echo "1. Flash test_dip.uf2 to your Pico"
    echo "2. Connect via serial (screen /dev/ttyACM0 115200)"
    echo "3. Test different DIP switch combinations"
else
    echo "❌ Test build failed - test_dip.uf2 not found"
    exit 1
fi