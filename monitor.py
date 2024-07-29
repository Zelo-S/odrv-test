import odrive
from odrive.enums import *
import time

print("Finding odrv...")
odrv0 = odrive.find_any()

print(odrv0.axis1.controller.config.input_mode)
print(odrv0.axis1.controller.config.control_mode)

import matplotlib.pyplot as plt
from odrive.utils import start_liveplotter
avg_I = 0
start_liveplotter(lambda: [odrv0.axis1.motor.current_control.Iq_measured])
time.sleep(100000)
# avg_I += odrv0.axis1.motor.current_control.Iq_measured
# print("Average I: ", avg_I)
plt.close('all')