#!/bin/bash

# Clean script for Pico Multi-App Firmware
# Removes build artifacts

echo "Cleaning build artifacts..."

if [ -d "build" ]; then
    rm -rf build
    echo "✅ Build directory removed"
else
    echo "ℹ️  No build directory found"
fi

echo "Clean complete."