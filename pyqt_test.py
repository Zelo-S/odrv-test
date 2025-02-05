import time
from random import randint
import threading

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

some_data = []
def thing():
    res = 0
    while True:
        res += 1
        print(res)
        time.sleep(0.2)
        some_data.append(res)
        
thr = threading.Thread(target=thing, daemon=True)
thr.start()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Temperature vs time dynamic plot
        self.plot_graph = pg.PlotWidget()
        self.setCentralWidget(self.plot_graph)
        self.plot_graph.setBackground("w")
        pen = pg.mkPen(color=(255, 0, 0))
        self.plot_graph.setTitle("Temperature vs Time", color="b", size="20pt")
        styles = {"color": "red", "font-size": "18px"}
        self.plot_graph.setLabel("left", "Temperature (°C)", **styles)
        self.plot_graph.setLabel("bottom", "Time (min)", **styles)
        self.plot_graph.addLegend()
        self.plot_graph.showGrid(x=True, y=True)
        self.plot_graph.setYRange(0, 100)

        self.time = []

        self.line = self.plot_graph.plot(
            self.time,
            [],
            name="Temperature Sensor",
            pen=pen,
            symbol="+",
            symbolSize=15,
            symbolBrush="b",
        )

        self.start_time = round(time.time(), 1)

        # Add a timer to simulate new temperature measurements
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        print(f"Data is: {some_data}")
        self.time.append(round(time.time(), 1) - self.start_time)
        self.time = self.time[-20:]
        print(self.time[-20:])
        self.plot_graph.setYRange(0, max(some_data[-20:]))
        self.line.setData(self.time, some_data[-20:])



app = QtWidgets.QApplication([])
main = MainWindow()
main.show()
app.exec()
