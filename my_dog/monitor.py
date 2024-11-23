import odrive
from odrive.enums import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

print("Finding odrv...")
odrv0 = odrive.find_any()
time.sleep(1)
fig, ax = plt.subplots()

max_x = 5
max_rand = 1

x = np.arange(0, max_x)
ax.set_ylim(-1, 1)
line, = ax.plot(x, [0, 0, 0, 0, 0])

def init():  # give a clean slate to start
    line.set_ydata([] * len(x))
    return line,

def animate(i):  # update the y values (every 1000ms)
    data = []
    data.append(odrv0.axis0.encoder.pos_estimate)
    print(odrv0.axis0.controller.pos_setpoint)
    line.set_ydata(data)

    return line,

ani = animation.FuncAnimation(fig, animate, init_func=init, interval=10, blit=True, save_count=20)

plt.show()