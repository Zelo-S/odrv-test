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
import fetchtraj

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
        return 0, 0, 0, 0
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

def f_x(a, b, x):
    return b * np.cos(a * x)

def cosine_wave_single_steps():
    
    tuning_pos(33)
    tuning_vel(0.009, 0.045)

    T = 0
    offset = L1 + L2 - 80
    a = 0.03*np.pi
    b = 100

    while True:
        T = 0
        while 0 <= T < 0.98:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            y_FL = (-np.pi / a) * (T - 0) + np.pi / (2 * a)
            x_FL = -1 * f_x(a, b, -np.pi*(T - 0)/a + np.pi/(2*a)) + offset

            y_FR = np.pi*(T-2)/(3*a) + np.pi/(2*a)
            x_FR = offset

            y_BL = -1 * (np.pi*(T-3)/(3*a) + np.pi/(2*a))
            x_BL = offset

            y_BR = -1 * (np.pi*(T-1)/(3*a) + np.pi/(2*a))
            x_BR = offset

            _, _, FL_q1_2, FL_q2_2 = q1q2_from_xy(x_FL, y_FL)
            _, _, FR_q1_2, FR_q2_2 = q1q2_from_xy(x_FR, y_FR)
            _, _, BL_q1_2, BL_q2_2 = q1q2_from_xy(x_BL, y_BL)
            _, _, BR_q1_2, BR_q2_2 = q1q2_from_xy(x_BR, y_BR)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * FL_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * FL_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [BL_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [BL_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [FR_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [FR_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * BR_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * BR_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            T += 3e-2
            time.sleep(0.0008)
        while 0.98 <= T < 1:
            T += 3e-2
            time.sleep(0.5)
        while 1 <= T < 1.98:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            y_FL = np.pi*(T-4)/(3*a) + np.pi/(2*a)
            x_FL = offset

            y_FR = np.pi*(T-2)/(3*a) + np.pi/(2*a)
            x_FR = offset

            y_BL = -1 * (np.pi*(T-3)/(3*a) + np.pi/(2*a))
            x_BL = offset

            y_BR = -1 * ((-np.pi / a) * (T - 1) + np.pi / (2 * a))
            x_BR = -1 * f_x(a, b, -np.pi*(T - 1)/a + np.pi/(2*a)) + offset

            _, _, FL_q1_2, FL_q2_2 = q1q2_from_xy(x_FL, y_FL)
            _, _, FR_q1_2, FR_q2_2 = q1q2_from_xy(x_FR, y_FR)
            _, _, BL_q1_2, BL_q2_2 = q1q2_from_xy(x_BL, y_BL)
            _, _, BR_q1_2, BR_q2_2 = q1q2_from_xy(x_BR, y_BR)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * FL_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * FL_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [BL_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [BL_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [FR_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [FR_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * BR_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * BR_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            T += 3e-2
            time.sleep(0.0008)
        while 1.98 <= T < 2:
            T += 3e-2
            time.sleep(0.5)
        while 2 <= T < 2.98:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            y_FL = np.pi*(T-4)/(3*a) + np.pi/(2*a)
            x_FL = offset

            y_FR = (-np.pi / a) * (T - 2) + np.pi / (2 * a)
            x_FR = -1 * f_x(a, b, -np.pi*(T - 2)/a + np.pi/(2*a)) + offset

            y_BL = -1 * (np.pi*(T-3)/(3*a) + np.pi/(2*a))
            x_BL = offset

            y_BR = -1 * (np.pi*(T-5)/(3*a) + np.pi/(2*a))
            x_BR = offset

            _, _, FL_q1_2, FL_q2_2 = q1q2_from_xy(x_FL, y_FL)
            _, _, FR_q1_2, FR_q2_2 = q1q2_from_xy(x_FR, y_FR)
            _, _, BL_q1_2, BL_q2_2 = q1q2_from_xy(x_BL, y_BL)
            _, _, BR_q1_2, BR_q2_2 = q1q2_from_xy(x_BR, y_BR)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * FL_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * FL_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [BL_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [BL_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [FR_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [FR_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * BR_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * BR_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            T += 3e-2
            time.sleep(0.0008)
        while 2.98 <= T < 3:
            T += 3e-2
            time.sleep(0.5)
        while 3 <= T < 3.98:
            q1_1, q2_1, q1_2, q2_2 = 0, 0, 0, 0
            y_FL = np.pi*(T-4)/(3*a) + np.pi/(2*a)
            x_FL = offset

            y_FR = np.pi*(T-6)/(3*a) + np.pi/(2*a)
            x_FR = offset

            y_BL = -1 * ((-np.pi / a) * (T - 3) + np.pi / (2 * a))
            x_BL = -1 * f_x(a, b, -np.pi*(T - 3)/a + np.pi/(2*a)) + offset

            y_BR = -1 * (np.pi*(T-5)/(3*a) + np.pi/(2*a))
            x_BR = offset

            _, _, FL_q1_2, FL_q2_2 = q1q2_from_xy(x_FL, y_FL)
            _, _, FR_q1_2, FR_q2_2 = q1q2_from_xy(x_FR, y_FR)
            _, _, BL_q1_2, BL_q2_2 = q1q2_from_xy(x_BL, y_BL)
            _, _, BR_q1_2, BR_q2_2 = q1q2_from_xy(x_BR, y_BR)

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * FL_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * FL_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [BL_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [BL_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [FR_q1_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [FR_q2_2 * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [-1 * BR_q1_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [-1 * BR_q2_2 * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            T += 3e-2
            time.sleep(0.0008)
        while 3.98 <= T < 4:
            T += 3e-2
            time.sleep(0.5)
        T -= 4

def one_leg_up_down():
    """
        One leg per time, up down
        Orientation: +Z = near hip, -Z = near end-effector, trajectory generation described by the following bot:

        # Requires roboticstoolbox-python, ideally on python==3.10
        ERobot: , 2 joints (RR)
        ┌──────┬────────┬───────┬────────┬─────────────────────┐
        │ link │  link  │ joint │ parent │ ETS: parent to link │
        ├──────┼────────┼───────┼────────┼─────────────────────┤
        │    0 │ link0  │     0 │ BASE   │ Ry(q0)              │
        │    1 │ link1  │     1 │ link0  │ tz(-0.165) ⊕ Ry(q1) │
        │    2 │ @link2 │       │ link1  │ tz(-0.165)          │
        └──────┴────────┴───────┴────────┴─────────────────────┘
        
    """
    traj = fetchtraj.MyTraj()
    trajarr = traj.fetch_traj_delta_150()
    while True:
        for i in range(4):
            for q in trajarr:
                upper_q = q[0] # is a negative value so -y,
                lower_q = q[1] # is a positive value so +y, 

                if i == 0:
                    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [upper_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [lower_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 1:
                    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [upper_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [lower_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 2:
                    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [-1 * upper_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [-1 * lower_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 3:
                    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [-1 * upper_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [-1 * lower_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                time.sleep(0.00008)
            time.sleep(2)

def two_leg_up_down(): # NOTE: Very stable
    """
        Two leg per time, diagonally aligned, up down
        Orientation: +Z = near hip, -Z = near end-effector, trajectory generation described by the following bot:

        # Requires roboticstoolbox-python, ideally on python==3.10
        ERobot: , 2 joints (RR)
        ┌──────┬────────┬───────┬────────┬─────────────────────┐
        │ link │  link  │ joint │ parent │ ETS: parent to link │
        ├──────┼────────┼───────┼────────┼─────────────────────┤
        │    0 │ link0  │     0 │ BASE   │ Ry(q0)              │
        │    1 │ link1  │     1 │ link0  │ tz(-0.165) ⊕ Ry(q1) │
        │    2 │ @link2 │       │ link1  │ tz(-0.165)          │
        └──────┴────────┴───────┴────────┴─────────────────────┘
        
    """
    traj = fetchtraj.MyTraj()
    trajarr = traj.fetch_traj_delta_50()
    while True:
        for i in range(2):
            for q in trajarr:
                upper_q = q[0] # is a negative value so -y,
                lower_q = q[1] # is a positive value so +y, 

                if i == 0:
                    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [upper_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [lower_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [upper_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [lower_q * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

                elif i == 1:
                    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [-1 * upper_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [-1 * lower_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [-1 * upper_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
                    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [-1 * lower_q * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

                time.sleep(0.0003)
            time.sleep(0.5)


def slow_walk():
    """
    Follows a trajectory described by: ... 
    """

    traj_generator = fetchtraj.TrajFetch(
        all_filepath="traj_gen/trajectories/slow_walk_all_legs.npy"
    )
    
    all_arr = traj_generator.fetch_all()
    
    FR_gait = all_arr[0]
    BL_gait = all_arr[1]
    FL_gait = all_arr[2]
    BR_gait = all_arr[3]

    while True:
        for FRq, BLq, FLq, BRq in zip(FR_gait, BL_gait, FL_gait, BR_gait):

            send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [FRq[0] * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [FRq[1] * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [-1 * FLq[0] * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [-1 * FLq[1] * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [-1 * BLq[0] * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [-1 * BLq[1] * (9 / (2 * math.pi)), 4, 0], data_format="<fhh") # BR_lower axis should turn +1

            send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [BRq[0] * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_upper axis should turn -1
            send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [BRq[1] * (9 / (2 * math.pi)),      4, 0], data_format="<fhh") # BR_lower axis should turn +1
            
            time.sleep(0.003)

        time.sleep(1)

slow_walk()