#!/usr/bin/env python3
"""
Example usage of the picohost library with various Pico devices.
"""

import time
from picohost import PicoDevice, PicoMotor, PicoRFSwitch, PicoPeltier


def example_basic_device():
    """Example of using the base PicoDevice class."""
    print("=== Basic Device Example ===")
    
    # Find available Pico devices
    ports = PicoDevice.find_pico_ports()
    if not ports:
        print("No Pico devices found!")
        return
    
    print(f"Found {len(ports)} Pico device(s): {ports}")
    
    # Connect to the first device using context manager
    with PicoDevice(ports[0]) as device:
        print(f"Connected to {device.port}")
        
        # Send a custom command
        device.send_command({"cmd": "status"})
        
        # Wait for response
        response = device.wait_for_response(timeout=2.0)
        if response:
            print(f"Got response: {response}")
        
        # Monitor for 5 seconds
        print("Monitoring for 5 seconds...")
        time.sleep(5)


def example_motor_control():
    """Example of motor control."""
    print("\n=== Motor Control Example ===")
    
    # Assuming motor device is on /dev/ttyACM0
    port = "/dev/ttyACM0"
    
    try:
        with PicoMotor(port) as motor:
            print(f"Connected to motor controller")
            
            # Move motors in a square pattern
            moves = [
                (1000, 0),    # Right
                (0, 1000),    # Up
                (-1000, 0),   # Left
                (0, -1000),   # Down
            ]
            
            for az, el in moves:
                print(f"Moving: azimuth={az}, elevation={el}")
                motor.move(az, el)
                time.sleep(2)
                
    except Exception as e:
        print(f"Motor control error: {e}")


def example_temperature_control():
    """Example of temperature control with callbacks."""
    print("\n=== Temperature Control Example ===")
    
    # Custom response handler
    def handle_temp_response(data):
        if data.get('status') == 'update':
            temp1 = data.get('temp1', 0)
            target1 = data.get('target1', 0)
            print(f"Channel 1: {temp1:.1f}°C (target: {target1:.1f}°C)")
    
    # Find Peltier device
    ports = PicoPeltier.find_pico_ports()
    if not ports:
        print("No Peltier device found!")
        return
    
    with PicoPeltier(ports[0]) as peltier:
        print(f"Connected to Peltier controller")
        
        # Set custom handler
        peltier.set_response_handler(handle_temp_response)
        
        # Set temperature to 25°C on channel 1
        print("Setting channel 1 to 25°C...")
        peltier.set_temperature(25.0, channel=1)
        peltier.enable(channel=1)
        
        # Monitor for 10 seconds
        print("Monitoring temperature for 10 seconds...")
        time.sleep(10)
        
        # Disable control
        peltier.disable(channel=1)
        print("Temperature control disabled")


def example_rf_switch():
    """Example of RF switch control."""
    print("\n=== RF Switch Example ===")
    
    # Manual connection without context manager
    switch = PicoRFSwitch("/dev/ttyACM1")  # Adjust port as needed
    
    if switch.connect():
        print("Connected to RF switch")
        switch.start()
        
        try:
            # Cycle through switch states
            for state in range(8):
                print(f"Setting switch state to {state}")
                switch.set_switch_state(state)
                time.sleep(1)
                
        finally:
            switch.disconnect()
            print("Disconnected")
    else:
        print("Failed to connect to RF switch")


def example_custom_protocol():
    """Example of implementing a custom protocol."""
    print("\n=== Custom Protocol Example ===")
    
    class CustomDevice(PicoDevice):
        """Custom device with specific protocol."""
        
        def get_info(self):
            """Request device information."""
            return self.send_command({"cmd": "info"})
        
        def set_parameter(self, name, value):
            """Set a device parameter."""
            return self.send_command({
                "cmd": "set_param",
                "name": name,
                "value": value
            })
    
    # Use the custom device
    device = CustomDevice("/dev/ttyACM0")
    if device.connect():
        device.start()
        
        # Custom response handler
        def info_handler(data):
            if data.get('cmd') == 'info':
                print(f"Device info: {data}")
        
        device.set_response_handler(info_handler)
        
        # Use custom methods
        device.get_info()
        device.set_parameter("gain", 1.5)
        
        time.sleep(2)
        device.disconnect()


if __name__ == "__main__":
    # Run examples based on what's available
    print("Picohost Library Examples\n")
    
    # Always run basic example
    example_basic_device()
    
    # Run other examples if you have the hardware
    # Uncomment the examples you want to run:
    
    # example_motor_control()
    # example_temperature_control()
    # example_rf_switch()
    # example_custom_protocol()