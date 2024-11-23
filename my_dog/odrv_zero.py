import time
import odrive
from odrive.enums import *

print("Finding odrv...")
odrv0 = odrive.find_any()

print("setting to 0...")
time.sleep(1)

pos = odrv0.axis1.controller.input_pos
print(pos)
while pos < 0:
    pos += 0.01
    odrv0.axis1.controller.input_pos = pos
    time.sleep(0.01)