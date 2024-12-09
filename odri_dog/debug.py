import numpy as np
from live_plotter import LivePlotter, FastLivePlotter

live_plotter = FastLivePlotter(
    xlabels=["x"],
    ylabels=["y"],
    ylims=[(-2, 2)],
    legends=[["sin", "cos"]],
)
live_plotter2 = FastLivePlotter(
    xlabels=["x"],
    ylabels=["y"],
    ylims=[(-2, 2)],
    legends=["cos"],
)

new_x_data = []
i = 0
while True:
    new_x_data.append(i)
    y_data = np.stack([np.sin(new_x_data), np.cos(new_x_data)], axis=1)
    y_data2 = np.stack([np.cos(new_x_data), np.cos(new_x_data)], axis=1)

    live_plotter.plot(
        y_data_list=[y_data],
    )

    live_plotter2.plot(
        y_data_list=[y_data2]
    )
    i += 1