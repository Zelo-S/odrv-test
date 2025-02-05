import numpy as np
import os
import time
import can
import struct
from enum import Enum
from collections import deque
import blinker

from random import randint
import threading

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

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

pos_vals = []
setpoint_vals = []
timestamps = []

def canbus_thread():
    global pos_vals
    global timestamps
    start_time = time.time()
    with can.interface.Bus("can1", interface="socketcan") as bus:
        while True:
            msg = bus.recv(timeout=1)
            if msg is None:
                continue
            
            if msg.arbitration_id == (Joint.BR_upper.value << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))

            if msg.arbitration_id == (Joint.BR_upper.value << 5 | 0x09):
                pos, vel = struct.unpack('ff', msg.data)
                pos_vals.append(pos)
                setpoint_vals.append(setpoint_vals)
                timestamps.append(round(time.time() - start_time, 1))

can_thr = threading.Thread(target=canbus_thread)
can_thr.start()
                        
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        print("init main graph")
        # Temperature vs time dynamic plot
        self.plot_graph = pg.PlotWidget()
        self.setCentralWidget(self.plot_graph)
        self.plot_graph.setBackground("w")
        pen = pg.mkPen(color=(255, 0, 0))
        self.plot_graph.setTitle("Encoder Pos", color="b", size="10pt")
        styles = {"color": "red", "font-size": "18px"}
        self.plot_graph.setLabel("left", "pos", **styles)
        self.plot_graph.setLabel("bottom", "t", **styles)
        self.plot_graph.addLegend()
        self.plot_graph.showGrid(x=True, y=True)
        self.plot_graph.setYRange(-2, 2)

        self.time = []

        self.line = self.plot_graph.plot(
            self.time,
            [],
            name="Pos",
            pen=pen
        )

        self.start_time = round(time.time(), 1)
        self.window_length = 50

        # Add a timer to simulate new temperature measurements
        self.timer = QtCore.QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()
        

    def update_plot(self):
        self.line.setData(timestamps, pos_vals)

app = QtWidgets.QApplication([])
main = MainWindow()
main.show()
app.exec()