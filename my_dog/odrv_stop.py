import time
import odrive
from odrive.enums import *

print("Finding odrv...")
odrv0 = odrive.find_any()

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.requested_state = AXIS_STATE_IDLE