import time
import odrive
from odrive.enums import *

print("Finding odrv...")
odrv0 = odrive.find_any()
# Calibrate motor and wait for it to finish
print("starting calibration...")
time.sleep(1)
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

print("setting params axis 0...")
odrv0.axis0.encoder.set_linear_count(0)
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("setting params axis 1...")
odrv0.axis1.encoder.set_linear_count(0)
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL