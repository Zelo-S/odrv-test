import os
import sys
import odrive
import can
import struct
import socket
import time
import numpy as np
from enum import Enum
import math


# can_front = can.interface.Bus(channel='can0', bustype='socketcan')

class Commands(Enum): #ODrive CAN cmd_id
    EMERGENCY_STOP = 0x002
    MOTOR_ERROR = 0x003
    ENCODER_ERROR = 0x004
    AXIS_REQUESTED_STATE = 0x007
    ENCODER_COUNT = 0x00A
    INPUT_POS = 0x00C
    REBOOT = 0x016
    VBUS_VOLTAGE = 0x017
    CLEAR_ERRORS = 0x018
    LINEAR_COUNT = 0x019
    CONTROL_MODE = 0x00B

class State(Enum): #ODrive CAN data
    IDLE = 1
    FULL_CALIBRATION_SEQUENCE = 3
    CLOSED_LOOP_CONTROL = 8


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
    BR_hip = 9
    BR_upper = 10
    BR_lower = 11

bus0 = can.interface.Bus("can0", interface="socketcan")
bus1 = can.interface.Bus("can1", interface="socketcan")

L1 = 165
L2 = 165

def send_CAN(node_id, cmd_id, data=[], data_format=""):
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass
    arbitration_id = (node_id << 5) | cmd_id
    data_can = struct.pack(data_format, *data)
    message = can.Message(arbitration_id=arbitration_id, data=data_can, is_extended_id=False)
    bus0.send(message)
    bus1.send(message)

def q1q2_from_xy(x, y):
    xD_mag = math.sqrt(x*x + y*y)
    c2 = (xD_mag * xD_mag - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1:
        return None
    if c2 == 1:
        return math.atan2(y, x), 0, math.atan2(y, x), 0
    q2_1 = 1 * math.acos( c2 )
    q2_2 = -1 * math.acos( c2 )
    theta = math.atan2(y, x)
    q1_1 = theta - math.atan2(L2 * math.sin(q2_1), L1 + L2 * math.cos(q2_1))
    q1_2 = theta - math.atan2(L2 * math.sin(q2_2), L1 + L2 * math.cos(q2_2))
    return q1_1, q2_1, q1_2, q2_2

def set_stand_start():
    send_CAN(Joint.FL_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.FL_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.BL_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.BL_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.FR_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.FR_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.BR_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.BR_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    
    L1 = 165
    L2 = 165

    t_ = 0
    time_start = time.time()
    time_duration_path_1 = 3
    time_duration_path_2 = 3
    final_pos_path_2 = L1 + L2
    final_standing_height = 200

    time_step = 1e-3

    max_q1 = -10000
    min_q1 = 10000
    max_q2 = -10000
    min_q2 = 10000
    
    q1_1, q2_1, q1_2, q2_2 = q1q2_from_xy(L1 + L2 - 50, 0)

    max_q1 = max(q1_2, max_q1)
    min_q1 = min(q1_2, min_q1)
    max_q2 = max(q2_2, max_q2)
    min_q2 = min(q2_2, min_q2)
    
    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_lower axis should turn +1

    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_lower axis should turn +1

    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_lower axis should turn +1

    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 0, 0], data_format="<fhh") # BR_lower axis should turn +1

                
class Tunas(Enum):
    POS_GAIN = 0x01a
    VEL_GAIN = 0x01b

def tuning_pos(pos_gain_value):
    pos_gain_value = float(pos_gain_value)
    # print(pos_gain_value, type(pos_gain_value))
    send_CAN(Joint.BR_upper.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.BR_lower.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.BR_hip.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")

    send_CAN(Joint.BL_upper.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.BL_lower.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.BL_hip.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")

    send_CAN(Joint.FR_upper.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.FR_lower.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.FR_hip.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    
    send_CAN(Joint.FL_upper.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.FL_lower.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")
    send_CAN(Joint.FL_hip.value, Tunas.POS_GAIN.value, [pos_gain_value], data_format="<f")


def tuning_vel(vel_gain_value, vel_integrator_gain_value):
    vel_gain_value = float(vel_gain_value)
    vel_integrator_gain_value = float(vel_integrator_gain_value)
    send_CAN(Joint.BR_upper.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.BR_lower.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.BR_hip.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")

    send_CAN(Joint.BL_upper.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.BL_lower.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.BL_hip.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")

    send_CAN(Joint.FR_upper.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.FR_lower.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.FR_hip.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")

    send_CAN(Joint.FL_upper.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.FL_lower.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")
    send_CAN(Joint.FL_hip.value, Tunas.VEL_GAIN.value, [vel_gain_value, vel_integrator_gain_value], data_format="<ff")

def haruto():
    
    tuning_pos(33)
    tuning_vel(0.009, 0.045)

    """
    send_CAN(Joint.FL_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.FL_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.BL_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.BL_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.FR_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.FR_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")

    send_CAN(Joint.BR_upper.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    send_CAN(Joint.BR_lower.value, Commands.CONTROL_MODE.value, [odrive.enums.ControlMode.POSITION_CONTROL, odrive.enums.InputMode.PASSTHROUGH], data_format="<ii")
    """
    T = 0
    b = L1 + L2 - 50
    a = -80
    
    max_q1 = 0
    min_q1 = 0
    max_q2 = 0
    min_q2 = 0

    while True:
        while T < 1:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            x = -a * T + a + b
            y = 0
            q1_1, q2_1, q1_2, q2_2 = q1q2_from_xy(x, y)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            max_q1 = max(math.degrees(q1_2), max_q1)
            min_q1 = min(math.degrees(q1_2), min_q1)
            max_q2 = max(math.degrees(q2_2), max_q2)
            min_q2 = min(math.degrees(q2_2), min_q2)
            
            print(f"Up: {x}, {y}")
            T += 3e-2
            time.sleep(0.03)
        while T >= 0:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            x = -a * T + a + b
            y = 0
            q1_1, q2_1, q1_2, q2_2 = q1q2_from_xy(x, y)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            max_q1 = max(math.degrees(q1_2), max_q1)
            min_q1 = min(math.degrees(q1_2), min_q1)
            max_q2 = max(math.degrees(q2_2), max_q2)
            min_q2 = min(math.degrees(q2_2), min_q2)

            print(f"Down: {x}, {y}")
            T -= 3e-2
            time.sleep(0.03)
            # print("Path 2", T)

def one_at_a_time_updown():
    
    tuning_pos(33)
    tuning_vel(0.009, 0.045)

    T = 0
    b = L1 + L2 - 50
    a = -100
    
    max_q1 = 0
    min_q1 = 0
    max_q2 = 0
    min_q2 = 0

    while True:
        for i in range(4):
            while T < 1:
                q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
                x = a * T + b
                y = 0
                q1_1, q2_1, q1_2, q2_2 = q1q2_from_xy(x, y)

                if i == 0:
                    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 1:
                    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 2:
                    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 3:
                    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                max_q1 = max(math.degrees(q1_2), max_q1)
                min_q1 = min(math.degrees(q1_2), min_q1)
                max_q2 = max(math.degrees(q2_2), max_q2)
                min_q2 = min(math.degrees(q2_2), min_q2)
                
                print(f"Up: {x}, {y}")
                T += 3e-2
                time.sleep(0.0008)
            while T >= 0:
                q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
                x = a * T + b
                y = 0
                q1_1, q2_1, q1_2, q2_2 = q1q2_from_xy(x, y)
                
                if i == 0:
                    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 1:
                    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 2:
                    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 3:
                    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                max_q1 = max(math.degrees(q1_2), max_q1)
                min_q1 = min(math.degrees(q1_2), min_q1)
                max_q2 = max(math.degrees(q2_2), max_q2)
                min_q2 = min(math.degrees(q2_2), min_q2)

                print(f"Down: {x}, {y}")
                T -= 3e-2
                time.sleep(0.0008)
                # print("Path 2", T)
            time.sleep(1)

one_at_a_time_updown()