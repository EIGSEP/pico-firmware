import numpy as np
from picohost import PicoDevice
import time 

ps = PicoDevice('/dev/ttyACM0')
time.sleep(2)
data = ps.last_status
data = {key:[data] for key, data in enumerate(data.items())}
while True:
    try:
        time.sleep(1)
        nu_data = ps.last_status
        for key in data.keys():
            data[key] = data[key].append(nu_data[key]) 
        print(data)
    except KeyboardInterrupt:
        break 
