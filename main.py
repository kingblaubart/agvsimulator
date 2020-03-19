from God import God
from SpaceFree2D_OpenGL import SpaceFree2DOpenGL
import json
import imageio
import matplotlib.pyplot as plt
import numpy as np
import Config as cfg
import multiprocessing

def start_simulation():
    parameters = json.load(open("Ackermann.json"))
    #parameters = cfg.parameters
    g = God(parameters)
    g.file_read()
    g.simulate()

    # for data in lib.data:
    #     time, obj, x, y, v, dir = data
    #     # print(f"{float(time):< 6.4}    {obj:<10}     {float(x):< 6.4}     {float(y):< 10.3}      {float(dir): < 3.3}")
    p1 = multiprocessing.Process(target=plots, args=(g,))
    p1.start()
    if parameters["animation"]:
        s = SpaceFree2DOpenGL(g)
        s.create_space()


def plots(g):
    #t = g.cars[0].planner.t_equi_in_t
    plt.style.use('seaborn')

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(nrows=5, ncols=1)

    for car in g.cars:
        if not car.ghost:
            ax1.plot(car.lat_distances)
    ax1.set_title('Lateral Distance')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Meter')
    for car in g.cars:
        if not car.ghost:
            ax2.plot(car.distances)
    ax2.set_title('Distance')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Meter')
    for car in g.cars:
        if not car.ghost:
            ax3.plot(car.controller.vel)
    ax3.set_title('Vel_c')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('m/s')
    plt.tight_layout()

    for car in g.cars:
        if not car.ghost:
            ax4.plot(car.planner.a_from_v_equi_in_t)
    ax4.set_title('Planned Acceleration')
    ax4.set_xlabel('Time')
    ax4.set_ylabel('m/s^2')
    for car in g.cars:
        if not car.ghost:
            ax5.plot(car.controller.acc)
    ax5.set_title('Controller Output')
    ax5.set_xlabel('Time')
    ax5.set_ylabel('m/s^2')
    plt.show()



if __name__ == "__main__":
    start_simulation()
