'''This script is for development, allows me to test the vna/switch compatibility and take data. Need a filename to write test data to, and you can configure how the script runs by changing the variables defined at the top. Writes and plots data.

AUTO: whether or not to use switch network to switch automatically.
FSTART, FSTOP : frequency ranges. 
'''

import sys
from cmt_vna import VNA
import matplotlib.pyplot as plt
import numpy as np
from switch_network import SwitchNetwork
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from cmt_vna import calkit as cal
import time 

FSTART, FSTOP = 250E6, 1.8E9
#FSTART, FSTOP = 50E6, 250e6 

AUTO = True

SERPORT = '/dev/ttyACM0'

try:
    assert len(sys.argv) == 2
except AssertionError:
    print('You must have a filename.')

filename = sys.argv[-1]

vna = VNA()
snw = SwitchNetwork(serport = SERPORT)

freqs = vna.setup(npoints=100, fstart=FSTART, fstop=FSTOP)
if AUTO:
    vna.add_OSL(snw=snw)
    snw.switch('VNAANT')

elif not AUTO:
    vna.add_OSL(snw=None)
    input('Switch to antenna now.')

antenna = vna.measure_S11()
snw.powerdown()

kit = cal.S911T(freq_Hz = freqs)
stds_meas = vna.data['vna']
sparams = {'vna': kit.sparams(stds_meas=stds_meas)}

antenna_cal = cal.calibrate(gammas=np.array([antenna]), sprms_dict=sparams)
np.savez(filename, freqs=freqs, antenna=antenna_cal)

plt.ion()
fig, ax = plt.subplots(2,1, figsize=(8,8), sharex = True)

ax[0].plot(freqs/1e6, np.log10(20*np.abs(vna.data['vna'].T))) 
ax[0].legend()

ax[1].plot(freqs/1e6, 20*np.log10(np.abs(antenna_cal[0])))
ax[1].set_xlabel('Frequency [MHz]')

plt.show(block=True)
