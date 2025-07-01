'''script for testing if the switch paths were correct (tested with a multimeter).'''
from switch_network import SwitchNetwork
import numpy as np

snw = SwitchNetwork(serport='/dev/ttyACM0')
for key, inst in snw.paths.items():
    input(f'press Enter to proceed.')
    snw.switch(key, verbose=True)
