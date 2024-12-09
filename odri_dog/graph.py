import numpy as np
import odrive
import time
from live_plotter import LivePlotter
import matplotlib.pyplot as plt

from live_plotter import FastLivePlotter
live_plotter = FastLivePlotter(legends=[["point", "setpoint"]], ylims=[(-0.5, 0)])
live_plotter2 = FastLivePlotter(legends=[["current", "current"]], ylims=[(-8,8)])
odrv0 = odrive.find_any()

def plot():
    pos_list = []
    setpoint_list = []
    current_list = []

    while True:
        # Retrieve data from the ODrive
        pos = odrv0.axis0.encoder.pos_estimate
        setpoint = odrv0.axis0.controller.pos_setpoint
        current = odrv0.axis0.motor.current_control.Iq_measured
        
        pos_list.append(pos)
        setpoint_list.append(setpoint)
        current_list.append(current)

        y_data_1 = np.stack([np.array(pos_list), np.array(setpoint_list)], axis=1)
        y_data_2 = np.stack([np.array(current_list), np.array(current_list)], axis=1)

        live_plotter.plot(
            y_data_list=[y_data_1]
        )
        live_plotter2.plot(
            y_data_list=[y_data_2]
        )

        # Control the update rate
        time.sleep(0.01)  # Adjust the sleep time as needed
    
plot()