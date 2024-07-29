import odrive
import matplotlib.pyplot as plt

print("Finding odrv...")
odrv0 = odrive.find_any()

while True:
    pos_estimate = odrv0.axis1.controller
    print(pos_estimate)
    plt.pause(0.05)

plt.show()
