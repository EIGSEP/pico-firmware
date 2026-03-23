import numpy as np
import time
import matplotlib.pyplot as plt
from picohost import PicoDevice

ps = PicoDevice('/dev/ttyACM0')
data = []
x = []
count = 0
time.sleep(2)

plt.ion()
fig, ax = plt.subplots()
scat = ax.scatter(x, data, color='black')
while True:
    try:
        d = ps.last_status['distance_m']
        data.append(d)
        x.append(count)
        scat.set_offsets(np.array([x, data]).T)
        ax.set_xlim(0, count+1)
        ax.set_ylim(0, max(data)+1) 
        count+= 1 
        fig.canvas.draw_idle()
        plt.pause(0.05)
        time.sleep(1)
    except KeyboardInterrupt:
        np.savez('testing_lidar_data_nopullup.npz', data=np.array(data))
        print("All Done!")
        break
