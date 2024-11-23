import time
import odrive
from odrive.enums import *

print("Finding odrv...")
odrv0 = odrive.find_any()

print("setting to 0...")
odrv0.axis1.controller.input_pos = 0

i = 0

while i < 10:
    odrv0.axis1.controller.input_pos = i
    i += 0.1
    time.sleep(0.1)

time.sleep(0.5)    

while i >= 0:
    odrv0.axis1.controller.input_pos = i
    i -= 0.1
    time.sleep(0.1)