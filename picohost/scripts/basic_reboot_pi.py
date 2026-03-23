import pigpio
import time 
import subprocess

BOOTSEL_PIN = 18
RESET_PIN = 17

pi = pigpio.pi()

#set the pins
pi.set_mode(RESET_PIN, pigpio.OUTPUT)
pi.set_mode(BOOTSEL_PIN, pigpio.OUTPUT)

print('Unplugging pico')
pi.write(RESET_PIN, 1)

print('pressing bootsel')
pi.write(BOOTSEL_PIN, 1)

time.sleep(5)
print('plugging pico back in')
pi.write(RESET_PIN, 0)

time.sleep(5)
print('releasing bootsel')
pi.write(BOOTSEL_PIN,0)

subprocess.run([
    "cp",
    "/home/eigsep/Downloads/RPI_PICO2-20251209-v1.27.0.uf2",
    "/media/eigsep/RP2350/RPI_PICO2-20251209-v1.27.0.uf2"
], check=True)

