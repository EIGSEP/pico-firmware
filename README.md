# Switch Network Control

[![codecov](https://codecov.io/gh/EIGSEP/switchNW/graph/badge.svg?token=9Z70IE4AQ8)](https://codecov.io/gh/EIGSEP/switchNW)

This repository holds a package and scripts for controlling the switch network EIGSEP will use for automatic calibration.

**Installation**
```
pip install <path/to/switchNW>
```

**Getting Started**

There are two interfaces to work with here. One is the Pi Pico and one is the LattePanda sending the commands. The Pi Pico is connected to the switch network and is responsible for controlling the GPIO pins that switch the relays. The LattePanda is connected to the Pi Pico via USB serial and sends commands to it. 

**Important**: The SwitchNetwork package and the scripts/main.py must be copied to the Pi Pico and live in the same directory.

From a terminal, copy the package and the script to the Pico root:
```shell
mpremote devs  # list the devices to see where the Pico is connected
# replace /dev/ttyACM0 with the output of the previous command
mpremote connect /dev/ttyACM0 fs cp -r src/switch_network :/switch_network
mpremote connect /dev/ttyACM0 fs cp -r scripts/main.py :/main.py  # script to run
mpremote connect /dev/ttyACM0 reset   # rests the pico, so main.py stars automatically
```

Now main.py is running on the Pico and will start automatically on boot.


For a three-switch network, one would change the main.py to use the right pin numbers in the list at the top. For example:
```
pins = [0,1,2]
```
Then, when initializing the SwitchNetwork Object, first set the correct paths to each object in a dictionary, the correct pins for the gpios argument, and the correct /dev/ttyACM* path for the USB serial.

**Resources**: See EIGSEP memo in preparation. 

