import machine, onewire, ds18x20

DATA_PINS = [26, 27]  

total_devices = 0

for pin_num in DATA_PINS:
    print(f"\nScanning pin {pin_num}:")
    
    ow = onewire.OneWire(machine.Pin(pin_num))
    ds = ds18x20.DS18X20(ow)
    
    roms = ds.scan()
    print(f"Found {len(roms)} device(s) on pin {pin_num}:")
    
    for rom in roms:
        print("  " + "".join("{:02x}".format(b) for b in rom)) # prints each ROM code as hexadecimal bytes
    
    total_devices += len(roms)

print(f"\nTotal devices found across all pins: {total_devices}")
