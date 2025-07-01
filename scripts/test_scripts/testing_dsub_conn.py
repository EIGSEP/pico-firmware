'''dev script for switching through all switches.'''
from switch_network import SwitchNetwork
import numpy as np

PATHS = {
            'one'   : '1000000',
            'two'   : '0100000',
            'three' : '0010000',
            'four'  : '0001000',
            'five'  : '0000100',
            'six'   : '0000010',
            'seven' : '0000001',
            'all'   : '1111111',
            'none'  : '0000000'
        }

GPIOS = [0,1,2,3,4,6,7]

snw = SwitchNetwork(serport='/dev/ttyACM0', gpios=GPIOS, paths=PATHS)
input(snw.state)
for key, inst in PATHS.items():
    input(f'switching to {key}. press Enter to proceed.')
    snw.switch(key, verbose=True)
