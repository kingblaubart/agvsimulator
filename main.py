from God import God
from SpaceFree2D_OpenGL import SpaceFree2DOpenGL
import json
import imageio
import matplotlib.pyplot as plt
import numpy as np
import Lib as lib
import Config as cfg

def start_simulation():
    parameters = json.load(open("OneCar.json"))
    parameters = cfg.parameters
    g = God(parameters)
    g.file_read()
    g.simulate()

    for data in lib.data:
        time, obj, x, y, v, dir = data
        # print(f"{float(time):< 6.4}    {obj:<10}     {float(x):< 6.4}     {float(y):< 10.3}      {float(dir): < 3.3}")

    t = np.asarray(g.cars[0].planner.t_equi_in_t)

    for car in g.cars:
        plt.plot(car.distances)
    plt.show()

    if parameters["animation"]:
        s = SpaceFree2DOpenGL(g)
        s.create_space()


if __name__ == "__main__":
    start_simulation()
