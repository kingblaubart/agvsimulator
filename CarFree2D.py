from Path import Path
from math import sqrt, cos, sin, tan, pi
from Point import Point
from newPathPlanner import PathPlanner
from ControllerFree2D import Controller
import numpy as np
from Event import Event
import Lib as lib


class CarFree2D:
    def __init__(self, id: int, spawn_x, spawn_y, start, start_dir, end_dir, size_x, size_y, angle, max_vel, max_acc,
                 color, min_latency, max_latency, errorrate):
        self.ghost = False
        # PHYSICAL PROPERTIES
        self.color = color                          # color code
        self.id = id                                # unique car id
        self.spawn = [spawn_x, spawn_y]             # spawnpoint
        self.destination = 0
        self.position_x = [spawn_x]                          # not used for EventQueue
        self.position_y = [spawn_y]
        self.direction = start_dir                      # direction of the car
        self.start_direction = angle                # start-direction of the car
        self.last_position = [spawn_x, spawn_y]     # position after last control input
        self.length = size_x                        # in m
        self.width = size_y                         # in m
        self.start_dir = start_dir
        self.end_dir = end_dir
        self.end = None
        self.min_latency = min_latency
        self.max_latency = max_latency
        self.errorrate = errorrate
        self.start_time = start
        self.wheelbase = self.length - 0.5 * self.width
        # VELOCITY
        self.last_velocity = 0
        self.last_velocity_x = 0                      # velocity after last control input
        self.last_velocity_y = 0
        self.velocity = []                          # not used for EventQueue
        self.max_velocity = max_vel                 # absolute limit of velocity
        # ACCELERATION
        self.acceleration = 0                       # acceleration given by last control input
        self.acceleration_y = 0
        self.acceleration_x = 0
        self.max_acceleration = max_acc             # absolute limit of acceleration
        self.acceleration_x_c = 0
        self.acceleration_y_c = 0
        self.acceleration_controller = 0
        # STEERING
        self.steering = 0                           # steering angle given by last control input
        self.direction = 0                          # direction angle given by last control input
        self.steering_control = 0
        # PATH
        self.shape = []                             # shape of the planned path (without exact timestamp)
        self.path = Path(self.spawn)                # "shape" with exact timestamp
        self.waypoints = []                         # list for the given points with the
        self.planner = None
        # CONTROLS
        self.time_last_control = start - lib.dt                  # timestamp of last control input
        self.time_last_update = start - lib.dt                  # timestamp of last given acceleration
        self.time_last_step = start - lib.dt
        self.stop = False                           # False: car drives, True: car stops (or will stop within the next time (t < ts)
        self.stop_time = 0                          # timestamp of stop (velocitiy 0 reached)
        self.controller = Controller(self, lib.k_p, lib.k_d)
        # DEBUGGING
        self.debugging1 = []
        self.debugging2 = []
        self.debugging3 = []
        self.debugging4 = []
        self.dc_pos = []
        dim = len(lib.statespace.A)
        self.old_state = [np.zeros(dim).reshape(-1, 1), np.zeros(dim).reshape(-1, 1)]
        self.state = [np.zeros(dim), np.zeros(dim)]
        self.full_arc = 0
        self.counter = 0
        self.min_dist = self.make_min_dist()
        self.distances = []

    # GETTER
    # currenttly not used
    def get_speed(self, absolute):  # absolute is boolean - true returns absolute value, false vectorial speed
        if absolute:
            return sqrt(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
        else:
            return self.velocity

    # currenttly not used
    def get_acceleration(self):
        return self.acceleration

    def set_first_position(self):
        self.position_x = [self.spawn[0]]
        self.position_y = [self.spawn[1]]
        self.last_position = self.spawn

        # sets waypoint (given by *.json file) for the car
    def set_waypoint(self, x, y):
        p = Point(x, y)
        self.path.add(p)
        self.waypoints.append(p)
        # raise Exception('The point (' + str(p.x) + '|' + str(p.y) + ') is too far away. Skipped.')

    def make_min_dist(self):
        edges = [[self.spawn[0] - self.length / 2, self.spawn[1] - self.width / 2],
                 [self.spawn[0] + self.length / 2, self.spawn[1] - self.width / 2],
                 [self.spawn[0] + self.length / 2, self.spawn[1] + self.width / 2],
                 [self.spawn[0] - self.length / 2, self.spawn[1] + self.width / 2]]
        out = lib.dist(self.spawn, edges[0])
        for e in edges:
            dist = lib.dist(self.spawn, e)
            if dist > out:
                out = dist
        return out

    # creates spline for the given waypoints (from the *.json file)
    def create_spline(self):
        self.planner = PathPlanner(self, self.max_velocity, self.max_acceleration)
        self.planner.make_path(self.path.points)
        self.write_path()
        if not self.ghost:
            pass
            #self.make_controls()
            self.make_controls()

        self.planner.t_equi_in_t = (np.array(self.planner.t_equi_in_t) + self.start_time).tolist()
        self.stop_time = self.planner.t_equi_in_t[-1]

    def write_path(self):
        for point in self.planner.path_from_v_equi_in_t:
            self.shape.append([np.real(point), np.imag(point)])
        pass

    def test_dc_motor(self, ax, ay):

        self.acceleration_x = ax
        self.acceleration_y = ay
        self.counter += 1
        # x direction:
        self.state[0] = lib.statespace.A.A.dot(self.old_state[0]) + lib.statespace.B.A.dot(ax)
        x = lib.statespace.C.A.dot(self.old_state[0]) + lib.statespace.D.A.dot(ax)
        # y direction:
        self.state[1] = lib.statespace.A.A.dot(self.old_state[1]) + lib.statespace.B.A.dot(ay)
        y = lib.statespace.C.A.dot(self.old_state[1]) + lib.statespace.D.A.dot(ay)

        self.old_state = self.state
        return x[0][0], y[0][0]

    def steer(self, t, ax, ay, stop):
        t = round(t, 5)
        ax -= self.acceleration_x_c
        ay -= self.acceleration_y_c
        self.acceleration_x = ax
        self.acceleration_y = ay
        lp = self.last_position[:]

        # x direction
        self.state[0] = lib.statespace.A.A.dot(self.old_state[0]) + lib.statespace.B.A.dot(ax)
        x = lib.statespace.C.A.dot(self.old_state[0]) + lib.statespace.D.A.dot(ax)
        x = x[0][0]
        # y direction:
        self.state[1] = lib.statespace.A.A.dot(self.old_state[1]) + lib.statespace.B.A.dot(ay)
        y = lib.statespace.C.A.dot(self.old_state[1]) + lib.statespace.D.A.dot(ay)
        y = y[0][0]
        self.position_x.append(self.spawn[0] + x)
        self.position_y.append(self.spawn[1] + y)
        self.last_position = [self.spawn[0] + x, self.spawn[1] + y]

        self.old_state = self.state

        if stop:
            self.stop_time = t
        else:
            comp_vel = complex(self.last_velocity_x, self.last_velocity_y)
            self.direction = np.angle([comp_vel])[0]
        self.stop = stop
        try:
            self.direction = lib.angle(Point(self.position_x[-2], self.position_y[-2]), Point(self.position_x[-1], self.position_y[-1]))
        except IndexError:
            pass

        self.time_last_step = t

    def control(self, t, ax, ay):
        self.acceleration_x_c = ax
        self.acceleration_y_c = ay

    # CONVERTING CONTROL_PREP INTO CONTROLS FOR CAR
    # [timestamp, estimated x, estimated y, abs(acceleration), direction to drive]
    def make_controls(self):
        stop_time = self.planner.t_equi_in_t[-1]
        stop = False
        t = self.start_time
        for acc in self.planner.acceleration_from_v_equi_in_t:
            if acc == self.planner.acceleration_from_v_equi_in_t[-1]:
                stop = True
            ev = Event(t, self, (t, self, np.real(acc), np.imag(acc), stop, "steering"), lambda: lib.eventqueue.car_steering)
            lib.eventqueue.add_event(ev)
            t += lib.pt
        # fills the self.controls list with acceleration values
        self.controller.set_path(self.planner.path_from_v_equi_in_t)

    # used with EventQueue
    # returns position of the car at specific time t
    # also more information available (but certainly not needed),f.e. acceleration, direction, steering, angle, ...
    def get_data(self, t):
        if self.ghost:
            try:
                if (t-self.start_time) > 0:
                    index = int((t - self.start_time) / lib.pt)
                else:
                    index = 0
                point = self.planner.path_from_v_equi_in_t[index]
                vel = self.planner.velocity_from_v_equi_in_t[index]
                dir = np.angle([vel])
            except IndexError:
                point = self.planner.path_from_v_equi_in_t[-1]
                vel = self.planner.velocity_from_v_equi_in_t[-1]
                vel_dir = self.planner.velocity_from_v_equi_in_t[-2]
                dir = np.angle([vel_dir])

            vel = abs(vel)
            x = point.real
            y = point.imag

        else:
            if not self.stop:
                dt = (t - self.time_last_control)

            else:
                dt = (self.stop_time - self.time_last_control)

            x = self.last_position[0]
            y = self.last_position[1]

            dir = round(self.direction, 5)

            vel = self.last_velocity

        if t == 0:
            dir = self.start_direction
        return [t, self.id, round(x, 4), round(y, 4), vel, dir]
