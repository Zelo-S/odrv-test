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

def calib_test():
    with can.interface.Bus("can0", interface="socketcan") as bus:
        while not (bus.recv(timeout=0) is None): pass

        a = Joint.FR_upper.value
        b = Joint.FR_lower.value

        bus.send(can.Message(
            arbitration_id=(Joint.FL_hip.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        bus.send(can.Message(
            arbitration_id=(Joint.FL_upper.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        bus.send(can.Message(
            arbitration_id=(Joint.FL_lower.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        bus.send(can.Message(
            arbitration_id=(Joint.FR_hip.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        bus.send(can.Message(
            arbitration_id=(Joint.FR_upper.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        bus.send(can.Message(
            arbitration_id=(Joint.FR_lower.value << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 3),
            is_extended_id=False
        ))

        joint_values = [Joint.BL_upper.value, Joint.BL_lower.value, Joint.BL_hip.value, Joint.BR_upper.value, Joint.BR_lower.value, Joint.BR_hip.value]
        mapping = { joint_values[0]: "BL_upper",
                    joint_values[1]: "BL_lower",
                    joint_values[2]: "BL_hip",
                    joint_values[3]: "BR_upper",
                    joint_values[4]: "BR_lower",
                    joint_values[5]: "BR_hip" }

        
        """
        for msg in bus:
            if msg.arbitration_id == (a << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                print(a, round(time.time(), 2), error, state, result, traj_done)
            if msg.arbitration_id == (b << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                print(b, round(time.time(), 2), error, state, result, traj_done)
        """
calib_test()