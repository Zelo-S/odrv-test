import numpy as np
import os
import time
import can
import struct
from enum import Enum

class Joint(Enum): #Axis ID
    FL_hip = 0
    FL_upper = 1
    FL_lower = 2
    FR_hip = 3
    FR_upper = 4
    FR_lower = 5
    BL_hip = 6
    BL_upper = 7
    BL_lower = 8
    BR_hip = 9 # doesn't work
    BR_upper = 10 # doesn't work
    BR_lower = 11

from live_plotter import FastLivePlotter

live_plotter = FastLivePlotter(legends=[["point", "setpoint"]], ylims=[(-1.5, 1.5)])
# live_plotter2 = FastLivePlotter(legends=[["current", "current"]], ylims=[(-9, 9)])


live_plotter.plot(
    y_data_list=[np.stack([np.asarray([1,4,5,6]), np.asarray([9,8,1,3])], axis=1)]
)

def fetch_encoder():
    setpoint_list = [0]
    pos_list = [0]
    vel_list = [0]

    with can.interface.Bus("can1", interface="socketcan") as bus:
        while not (bus.recv(timeout=0) is None): pass
        
        for msg in bus:
            if msg.arbitration_id == (Joint.BR_upper.value << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                print("BR_upper is:", round(time.time(), 2), error, state, result, traj_done)
            
            if msg.arbitration_id == (Joint.BR_upper.value << 5 | 0x09):
                pos, vel = struct.unpack('ff', msg.data)
                pos_list.append(pos)
                setpoint_list.append(0)

                """
                live_plotter.plot(
                    y_data_list=[np.stack([np.asarray(pos_list), np.asarray(setpoint_list)], axis=1)]
                )
"""
            time.sleep(1) 

fetch_encoder()

"""
# Configure the CAN interface
bus = can.interface.Bus(channel='can0', interface='socketcan')


try:
    while True:
        message = bus.recv(timeout=1.0)
        if message is not None:
            print(f"Recv message: {message}")
            # Check if the message is an encoder feedback message
            if message.arbitration_id == 0x09:  # Example ID for encoder feedback
                # Unpack the data (assuming 4 bytes for position and 4 bytes for velocity)
                pos, vel = struct.unpack('<ff', message.data)
                print(f"Encoder Feedback - Position: {pos}, Velocity: {vel}")

except KeyboardInterrupt:
    print("Stopped listening for CAN messages.")

finally:
    # Shutdown the CAN bus
    bus.shutdown()
"""
