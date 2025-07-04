#!/usr/bin/env python3
"""
Test script for Peltier temperature control system.
Allows setting target temperatures and monitoring sensor readings.
"""

import argparse
import serial
import json
import time
import sys
from datetime import datetime
import threading
from serial.tools import list_ports

# USB IDs for Raspberry Pi Pico
PICO_VID = 0x2E8A
PICO_PID_CDC = 0x0009


def find_pico_ports():
    """Find all connected Pico devices in CDC mode."""
    ports = []
    for info in list_ports.comports():
        if info.vid == PICO_VID and info.pid == PICO_PID_CDC:
            ports.append(info.device)
    return ports


class PeltierController:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.reader_thread = None
        self.last_status = {}
        
    def connect(self):
        """Connect to the Pico device."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the device."""
        self.stop_monitoring()
        if self.ser:
            self.ser.close()
            self.ser = None
    
    def send_command(self, cmd_dict):
        """Send a JSON command to the device."""
        if not self.ser:
            print("Not connected")
            return False
        
        try:
            json_str = json.dumps(cmd_dict)
            self.ser.write((json_str + '\n').encode())
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def read_line(self):
        """Read a line from the serial port."""
        if not self.ser:
            return None
        
        try:
            line = self.ser.readline().decode().strip()
            if line:
                return line
        except:
            pass
        return None
    
    def parse_response(self, line):
        """Parse JSON response from the device."""
        try:
            data = json.loads(line)
            return data
        except:
            return None
    
    def monitor_thread(self):
        """Background thread to read and display status updates."""
        while self.running:
            line = self.read_line()
            if line:
                data = self.parse_response(line)
                if data:
                    self.handle_response(data)
    
    def handle_response(self, data):
        """Handle a parsed response from the device."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        if data.get('status') == 'update':
            # Temperature control status update
            self.last_status = data
            print(f"\n[{timestamp}] Status Update:")
            print(f"  Channel 1: {data.get('temp1', 'N/A'):.2f}°C → {data.get('target1', 'N/A'):.2f}°C " +
                  f"(drive: {data.get('drive1', 0):.3f}, enabled: {data.get('enabled1', False)})")
            print(f"  Channel 2: {data.get('temp2', 'N/A'):.2f}°C → {data.get('target2', 'N/A'):.2f}°C " +
                  f"(drive: {data.get('drive2', 0):.3f}, enabled: {data.get('enabled2', False)})")
            
        elif 'sensors' in data:
            # Temperature monitor status update
            self.last_status = data
            print(f"\n[{timestamp}] Sensor Readings:")
            sensors = json.loads(data['sensors']) if isinstance(data['sensors'], str) else data['sensors']
            for sensor in sensors:
                valid = "✓" if sensor.get('valid', False) else "✗"
                temp = sensor.get('temperature', 0)
                print(f"  {sensor.get('id', 'Unknown')}: {temp:.2f}°C [{valid}]")
        
        elif data.get('error'):
            print(f"\n[{timestamp}] ERROR: {data['error']}")
        
        elif data.get('status') == 'initialized':
            print(f"\n[{timestamp}] Device initialized - {data.get('sensor_count', 0)} sensors found")
    
    def start_monitoring(self):
        """Start the background monitoring thread."""
        if not self.running:
            self.running = True
            self.reader_thread = threading.Thread(target=self.monitor_thread)
            self.reader_thread.daemon = True
            self.reader_thread.start()
    
    def stop_monitoring(self):
        """Stop the background monitoring thread."""
        self.running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=1)
    
    # Command functions
    def set_temperature(self, temperature, channel=0):
        """Set target temperature for specified channel(s)."""
        cmd = {
            "cmd": "set_temp",
            "temperature": float(temperature),
            "channel": int(channel)
        }
        self.send_command(cmd)
        print(f"Set channel {channel} target to {temperature}°C")
    
    def set_hysteresis(self, hysteresis, channel=0):
        """Set hysteresis band for specified channel(s)."""
        cmd = {
            "cmd": "set_hysteresis",
            "hysteresis": float(hysteresis),
            "channel": int(channel)
        }
        self.send_command(cmd)
        print(f"Set channel {channel} hysteresis to ±{hysteresis}°C")
    
    def enable_control(self, channel=0):
        """Enable temperature control for specified channel(s)."""
        cmd = {
            "cmd": "enable",
            "channel": int(channel)
        }
        self.send_command(cmd)
        print(f"Enabled channel {channel}")
    
    def disable_control(self, channel=0):
        """Disable temperature control for specified channel(s)."""
        cmd = {
            "cmd": "disable",
            "channel": int(channel)
        }
        self.send_command(cmd)
        print(f"Disabled channel {channel}")


def interactive_mode(controller):
    """Run an interactive command-line interface."""
    print("\nPeltier Temperature Control Test")
    print("================================")
    print("Commands:")
    print("  temp <value> [channel]  - Set target temperature (channel: 0=both, 1/2=individual)")
    print("  hyst <value> [channel]  - Set hysteresis band")
    print("  enable [channel]        - Enable temperature control")
    print("  disable [channel]       - Disable temperature control")
    print("  status                  - Show last status")
    print("  quit                    - Exit")
    print()
    
    controller.start_monitoring()
    
    while True:
        try:
            cmd_input = input("> ").strip().lower()
            if not cmd_input:
                continue
            
            parts = cmd_input.split()
            cmd = parts[0]
            
            if cmd == 'quit' or cmd == 'exit':
                break
            
            elif cmd == 'temp' and len(parts) >= 2:
                temp = float(parts[1])
                channel = int(parts[2]) if len(parts) > 2 else 0
                controller.set_temperature(temp, channel)
            
            elif cmd == 'hyst' and len(parts) >= 2:
                hyst = float(parts[1])
                channel = int(parts[2]) if len(parts) > 2 else 0
                controller.set_hysteresis(hyst, channel)
            
            elif cmd == 'enable':
                channel = int(parts[1]) if len(parts) > 1 else 0
                controller.enable_control(channel)
            
            elif cmd == 'disable':
                channel = int(parts[1]) if len(parts) > 1 else 0
                controller.disable_control(channel)
            
            elif cmd == 'status':
                if controller.last_status:
                    print("\nLast status:")
                    print(json.dumps(controller.last_status, indent=2))
                else:
                    print("No status received yet")
            
            else:
                print("Unknown command. Type 'quit' to exit.")
                
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except ValueError as e:
            print(f"Invalid value: {e}")
        except Exception as e:
            print(f"Error: {e}")
    
    controller.stop_monitoring()


def main():
    parser = argparse.ArgumentParser(description='Test Peltier temperature control system')
    parser.add_argument('-p', '--port', help='Serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--list', action='store_true', help='List available Pico devices')
    
    # Quick command options
    parser.add_argument('--temp', type=float, help='Set target temperature')
    parser.add_argument('--channel', type=int, default=0, help='Channel (0=both, 1/2=individual)')
    parser.add_argument('--enable', action='store_true', help='Enable control')
    parser.add_argument('--disable', action='store_true', help='Disable control')
    parser.add_argument('--monitor', action='store_true', help='Monitor only (no interactive mode)')
    
    args = parser.parse_args()
    
    if args.list:
        ports = find_pico_ports()
        if ports:
            print("Found Pico devices:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No Pico devices found")
        return
    
    # Find port if not specified
    if not args.port:
        ports = find_pico_ports()
        if not ports:
            print("No Pico devices found. Please check connection.")
            return
        elif len(ports) > 1:
            print("Multiple Pico devices found. Please specify port with -p:")
            for port in ports:
                print(f"  {port}")
            return
        else:
            args.port = ports[0]
            print(f"Using port: {args.port}")
    
    # Create controller
    controller = PeltierController(args.port, args.baud)
    
    if not controller.connect():
        return
    
    try:
        # Execute quick commands if specified
        if args.temp is not None:
            controller.set_temperature(args.temp, args.channel)
            time.sleep(0.1)
        
        if args.enable:
            controller.enable_control(args.channel)
            time.sleep(0.1)
        
        if args.disable:
            controller.disable_control(args.channel)
            time.sleep(0.1)
        
        # Run appropriate mode
        if args.monitor:
            print("Monitoring... Press Ctrl+C to stop")
            controller.start_monitoring()
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopping...")
        elif args.temp is None and not args.enable and not args.disable:
            # No quick commands, run interactive mode
            interactive_mode(controller)
        else:
            # Quick command mode - monitor for a few seconds to see response
            controller.start_monitoring()
            time.sleep(3)
    
    finally:
        controller.disconnect()
        print("Disconnected")


if __name__ == '__main__':
    main()