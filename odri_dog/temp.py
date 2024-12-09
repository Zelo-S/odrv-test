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

class Tunas(Enum):
    POS_GAIN = 0x01a
    VEL_GAIN = 0x01b

def send_CAN(node_id, cmd_id, data=[], data_format=""):
    while not (bus0.recv(timeout=0) is None): pass
    while not (bus1.recv(timeout=0) is None): pass
    arbitration_id = (node_id << 5) | cmd_id
    data_can = struct.pack(data_format, *data)
    message = can.Message(arbitration_id=arbitration_id, data=data_can, is_extended_id=False)
    bus0.send(message)
    bus1.send(message)

def init():
    send_CAN(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], data_format="<I")
    send_CAN(Joint.BR_lower.value, Tunas.POS_GAIN.value, [33], data_format="<f")
    send_CAN(Joint.BR_lower.value, Tunas.VEL_GAIN.value, [float(0.009), float(0.045)], data_format="<ff")

def stop():
    send_CAN(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], data_format="<I")

def run():
    setpoint = 0
    while setpoint < 2.5:
        send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [setpoint, 0, 0], data_format="<fhh")
        time.sleep(4)
        setpoint += 0.5
    while setpoint >= 0:
        send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [setpoint, 0, 0], data_format="<fhh")
        time.sleep(4)
        setpoint -= 0.5
    stop()
        
def press(final_pos):
    setpoint = 0
    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [setpoint, 0, 0], data_format="<fhh")
    while setpoint >= final_pos:
        send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [setpoint, 0, 0], data_format="<fhh")
        time.sleep(0.1)
        setpoint -= 0.01
    time.sleep(5)
    send_CAN(Joint.BR_lower.value, Commands.INPUT_POS.value, [setpoint, 0, 0], data_format="<fhh")


init()
time.sleep(1)
offset = 0
for i in range(5):
    press(-0.25 + offset)
    offset -= 0.05
time.sleep(2)
stop()
# run()
# stop()