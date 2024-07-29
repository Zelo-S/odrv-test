import time
import odrive
from odrive.enums import *
import matplotlib.pyplot as plt

print("Finding odrv...")
odrv0 = odrive.find_any()

# print(INPUT_MODE_POS_FILTER) # 3
# print(INPUT_MODE_TRAP_TRAJ) # 5
# print(CONTROL_MODE_VELOCITY_CONTROL) # 2
# print(INPUT_MODE_VEL_RAMP) # 2 
# print(CONTROL_MODE_TORQUE_CONTROL) # 1

time.sleep(1)
pos = odrv0.axis1.controller.input_pos
print("Setting back to zero pos...")
while pos < 0:
    pos += 0.01
    odrv0.axis1.controller.input_pos = pos
    time.sleep(0.01)
odrv0.axis1.controller.input_pos = 0
time.sleep(2)

position_history = []
power_history = []
current_history = []

TIME_STEP = 0.01
GOAL_ANGULAR_VEL = 1


while pos <= 2:
    pos += (TIME_STEP * GOAL_ANGULAR_VEL) / 0.628
    odrv0.axis1.controller.input_pos = -1 * pos
    position_history.append(odrv0.axis1.controller.input_pos)
    power_history.append(odrv0.axis1.controller.mechanical_power)
    time.sleep(TIME_STEP)

import numpy as np
x = np.arange(0, len(position_history))
plt.plot(x, position_history)
plt.show()
x = np.arange(0, len(power_history))
plt.plot(x, power_history)
plt.show()