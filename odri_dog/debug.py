import odrive
from odrive.utils import start_liveplotter

odrv0 = odrive.find_any()

import threading
import matplotlib.pyplot as plt
import numpy as np

active_data = []

def fetch_data():
    global active_data
    while True:
        set_pos = odrv0.axis1.encoder.pos_estimate
        active_data.append(set_pos)

fetch_data_T = threading.Thread(target=fetch_data)
fetch_data_T.daemon = True
fetch_data_T.start()

# plt.figure()
# plt.plot(np.arange(len(active_data)), active_data)
# plt.show()
# def update_data(i):
print(active_data)
print("Done")