import math
import matplotlib.pyplot as plt
import sys
import odrive
import threading
import can
import struct
import socket
import time
import numpy as np
from odrive.enums import *
from enum import Enum

import numpy as np
from flask import Flask, render_template, send_from_directory, request
import random

app = Flask(__name__)

bus0 = can.interface.Bus("can0", interface="socketcan")
bus1 = can.interface.Bus("can1", interface="socketcan")

@app.route("/")
def root():
    return send_from_directory('./client/dist', 'index.html')

@app.route("/<path:path>")
def assets(path):
    return send_from_directory("./client/dist", path)

@app.route("/press_button")
def button_press():
    count = np.random.randint(0, 5)
    return "Buston pressed! " + str(count)

@app.route("/post/speed", methods=["POST"])
def post():
    posted_json = request.get_json()
    speed = posted_json['speed_']
    return f"Got speed value of {speed}"

@app.route("/post/idle", methods=["POST"])
def idle():
    print("Idling...")
    set_idle()
    return ""

@app.route("/post/calibration", methods=["POST"])
def calibration():
    print("Calibrating...")
    calibration()
    return f""

@app.route("/post/zero", methods=["POST"])
def zero():
    print("Zeroing...")
    set_lc_zero()
    return ""

@app.route("/post/clc", methods=["POST"])
def closed_loop():
    set_CLC()
    return ""

@app.route("/post/home", methods=["POST"])
def home():
    print("Homing...")
    return_home()
    return ""
    
@app.route("/post/pos_gain", methods=["POST"])
def pos_gain():
    print("Setting pos gain...")
    posted_json = request.get_json()
    pos_gain = posted_json['pos_gain']
    tuning_pos(pos_gain_value=pos_gain)
    return f""

@app.route("/post/vel_gain", methods=["POST"])
def vel_gain():
    print("Setting vel gain...")
    posted_json = request.get_json()
    vel_gain = posted_json['vel_gain']
    vel_integrator_gain = posted_json['vel_integrator_gain']
    tuning_vel(vel_gain_value=vel_gain, vel_integrator_gain_value=vel_integrator_gain)
    return f""

@app.route("/post/set_pos", methods=["POST"])
def set_pos():
    print("Setting pos...")
    posted_json = request.get_json()
    set_pos = posted_json['set_pos']
    return f""
    
#############

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

def send_CAN(node_id, cmd_id, data=[], data_format=""):
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass
    arbitration_id = (node_id << 5) | cmd_id
    data_can = struct.pack(data_format, *data)
    message = can.Message(arbitration_id=arbitration_id, data=data_can, is_extended_id=False)
    bus0.send(message)
    bus1.send(message)

"""
def odrive_getPos(node_id):
    while(1):
        msg = can_bus.recv()
        if(msg.arbitration_id == (node_id << 5) + 0x9):
            data = struct.unpack('ff', msg.data)
            return data[0]
"""

coords_y = []

def calibration():
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass
    
    send_CAN(Joint.FL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")

    send_CAN(Joint.FR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")

    send_CAN(Joint.BL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")

    send_CAN(Joint.BR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")
    send_CAN(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], data_format="<I")

def set_idle():
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass

    send_CAN(Joint.FL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="h")
    send_CAN(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="h")
    send_CAN(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="h")

    send_CAN(Joint.FR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")

    send_CAN(Joint.BL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")

    send_CAN(Joint.BR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")
    send_CAN(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")

def set_lc_zero(): # set linear-counts to 0
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass

    tuning_pos(33)
    tuning_vel(0.009, 0.045)

    send_CAN(Joint.FL_hip.value,   Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.FL_upper.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.FL_lower.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")

    send_CAN(Joint.FR_hip.value,   Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.FR_upper.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.FR_lower.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")

    send_CAN(Joint.BL_hip.value,   Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.BL_upper.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.BL_lower.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")

    send_CAN(Joint.BR_hip.value,   Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.BR_upper.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")
    send_CAN(Joint.BR_lower.value, Commands.LINEAR_COUNT.value, [0], data_format="<I")

def set_CLC():
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass

    send_CAN(Joint.FL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="h")
    send_CAN(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="h")
    send_CAN(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="h")

    send_CAN(Joint.FR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")

    send_CAN(Joint.BL_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")

    send_CAN(Joint.BR_hip.value,   Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")

def return_home():
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass

    send_CAN(Joint.FL_upper.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.FL_lower.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_lower axis should turn +1
    send_CAN(Joint.FL_hip.value,   Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh")

    send_CAN(Joint.BL_upper.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.BL_lower.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_lower axis should turn +1
    send_CAN(Joint.BL_hip.value,   Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh")

    send_CAN(Joint.FR_upper.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.FR_lower.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_lower axis should turn +1
    send_CAN(Joint.FR_hip.value,   Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh")

    send_CAN(Joint.BR_upper.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_upper axis should turn -1
    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh") # BR_lower axis should turn +1
    send_CAN(Joint.BR_hip.value,   Commands.INPUT_POS.value, [0, 0, 0], data_format="<fhh")

def set_position(set_pos):
    set_pos = float(set_pos)
    send_CAN(Joint.BR_upper.value,   Commands.INPUT_POS.value, [set_pos, 0, 0], data_format="<fhh")

def set_velocity(set_pos):
    set_pos = float(set_pos)
    send_CAN(Joint.BR_upper.value,   Commands.INPUT_POS.value, [set_pos, 0, 0], data_format="<fhh")


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

#############

if __name__ == "__main__":
    app.run(debug=True)