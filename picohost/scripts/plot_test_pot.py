import numpy as np
import time
import matplotlib.pyplot as plt
from picohost import PicoDevice

ps = PicoDevice('/dev/ttyACM0')
data0 = []
data1 = []
x = []
count = 0
time.sleep(2)

plt.ion()
fig, ax = plt.subplots()
scat = ax.scatter(x, data0, color='cornflowerblue', label='10k')
scat1 = ax.scatter(x, data1, color='black', label='100k')
plt.legend()
while True:
    try:
        d = ps.last_status
        data0.append(d['pot0_voltage'])
        data1.append(d['pot1_voltage'])
        x.append(count)
        scat.set_offsets(np.array([x, data0]).T)
        scat1.set_offsets(np.array([x, data1]).T)
        ax.set_xlim(0, count+1)
        ax.set_ylim(0, max(data0)+1) 
        count+= 1 
        fig.canvas.draw_idle()
        plt.pause(0.05)
        time.sleep(1)
    except KeyboardInterrupt:
        np.savez('pot_plot_data_diff_res1.npz', data0=np.array(data0), data1 = np.array(data1))
        print("All Done!")
        break
