from picohost import PicoPotentiometer
import numpy as np
import time 

pot = PicoPotentiometer(port='/dev/ttyACM0')

print(pot.last_status)

pot.calibrate()

vs = []
while True:
    try:
        v = pot.last_status['pot0_voltage']
        vs.append(v)
        time.sleep(0.5)
        print(v)
    except:
        vs = np.array(vs)
        np.savez('testing_pot_vs.npz', vs=vs, cal_values=pot.cal0_values)
        break
