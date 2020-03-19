from Path import Path
from math import sqrt, cos, sin, tan, pi, atan2
from Point import Point
from newPathPlanner import PathPlanner
from ControllerAckermann import Controller
import numpy as np
from Event import Event
from Channel import Channel
import Lib as lib


class CarAckermann:
    def __init__(self, id: int, spawn_x, spawn_y, start, start_dir, end_dir, size_x, size_y, angle, max_vel, max_acc,
                 max_steering_angle, color, min_latency, max_latency, errorrate, seg_len, channel_properties):
        self.ghost = False
        # PHYSICAL PROPERTIES
        self.color = color                          # color code
        self.id = id                                # unique car id
        self.spawn = [spawn_x, spawn_y]             # spawnpoint
        self.destination = 0

        self.position_x = [spawn_x]                 # not used for EventQueue
        self.position_y = [spawn_y]
        self.direction = start_dir                  # direction of the car
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
        self.rearbase = 0.5*self.width
        self.wheelangles = [0, 0]
        self.stored_a = []

        # VELOCITY
        self.last_velocity = 0
        self.velocity = []                          # not used for EventQueue
        self.max_velocity = max_vel                 # absolute limit of velocity
        # ACCELERATION
        self.acceleration = 0                       # acceleration given by last control input
        self.max_acceleration = max_acc             # absolute limit of acceleration
        self.acceleration_controller = 0
        self.acc_debug = []
        self.emergency_brake = False
        # STEERING
        self.steering = 0                           # steering angle given by last control input
        self.direction = 0                          # direction angle given by last control input
        self.steering_control = 0
        self.max_steering_angle = max_steering_angle
        # PATH
        self.shape = []                             # shape of the planned path (without exact timestamp)
        self.path = Path(self.spawn)                # "shape" with exact timestamp
        self.path_buffer = []
        self.segment_length = seg_len               # length of segments (path is divided in segments which are then
                                                    # sent to the AGV
        self.last_segment_time = 0                  # start-time of the last received path-segment
        self.waypoints = []                         # list for the given points with the
        self.planner = None
        # CONTROLS
        self.time_last_control = start - lib.dt                  # timestamp of last control input
        self.time_last_update = start - lib.dt                  # timestamp of last given acceleration
        self.time_last_step = start - lib.dt
        self.stop_time = 0                          # timestamp of stop (velocity 0 reached)
        self.controller = Controller(self, lib.k_p, lib.k_d)
        dim = len(lib.statespace.A)
        self.old_state = np.zeros(dim).reshape(-1, 1)
        self.state = None
        self.full_arc = 0
        self.min_dist = self.make_min_dist()
        self.distances = []
        self.lat_distances = []
        #CHANNEL
        self.channel = Channel(self.errorrate, channel_properties)
        # DEBUGGING
        self.debugging = []
        self.arc_debug = []

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

    # used for collision control
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
        self.stop_time = self.planner.t_equi_in_t[-1]
        self.write_path()
        self.planner.t_equi_in_t = (np.array(self.planner.t_equi_in_t) + self.start_time).tolist()
        self.update_path()

    def write_path(self):
        for point in self.planner.path_from_v_equi_in_t:
            self.shape.append([np.real(point), np.imag(point)])

    def steer(self, t, a, angle):
        if self.emergency_brake:
            a = -self.max_acceleration          # braking as fast as possible
        else:
            a += self.acceleration_controller   # controlling the current acceleration
        self.acc_debug.append(a)
        angle = (angle + self.steering_control)
        self.debugging.append(angle)
        # angle = min(self.max_steering_angle, angle)
        # angle = max(-self.max_steering_angle, angle)
        if (self.last_velocity <= 0) & self.emergency_brake:
            self.stop_time = t

        if t < self.stop_time:
            start_angle = self.direction - np.pi/2
            pos = np.array(self.last_position) - np.array([0, 0.5*cos(self.direction) * self.wheelbase])

            # DC-Motor
            self.state = lib.statespace.A.dot(self.old_state) + lib.statespace.B.dot(a)
            new_arc = (lib.statespace.C.dot(self.old_state) + lib.statespace.D.dot(a))[0, 0]

            # State Transition
            self.old_state = self.state

            # Saving attributes
            arc = new_arc - self.full_arc
            self.full_arc = new_arc
            self.last_velocity = arc / lib.pt
            self.velocity.append(self.last_velocity)
            self.arc_debug.append(arc)

            try:
                radius = self.wheelbase / tan(angle)
                self.wheelangles = [atan2(self.wheelbase, radius - 0.5 * self.rearbase), atan2(self.wheelbase, radius + 0.5*self.rearbase)]
                phi = arc / radius
                center_of_turning_cycle = np.array(pos) - np.array([radius, 0])
                point = np.array([radius * cos(phi), radius * sin(phi)])

                point = point + center_of_turning_cycle - pos

                point = np.dot(np.array([[np.cos(start_angle), -np.sin(start_angle)], [np.sin(start_angle), np.cos(start_angle)]]), point.reshape(-1, 1))
                point = point + np.array(pos).reshape(-1, 1)
                self.direction = (phi + self.direction) % (2*pi)
                self.last_position = (point.reshape(1, -1)+np.array([0, 0.5*cos(self.direction) * self.wheelbase])).tolist()[0]

                x = self.last_position[0]
                y = self.last_position[1]
                self.position_x.append(x)
                self.position_y.append(y)

            # in case of a straight movement:
            except ZeroDivisionError:
                x = self.last_position[0] + arc * cos(self.direction)
                y = self.last_position[1] + arc * sin(self.direction)
                self.last_position = [x, y]
                self.position_x.append(x)
                self.position_y.append(y)
                self.wheelangles = [0, 0]

    def control(self, acc):
        self.acceleration_controller = acc

    def make_controls(self):
        t = self.last_segment_time

        for i in range(len(self.stored_a)):
            acc = self.stored_a[i]
            prio = 1    # didsdsdsdfs
            ev = Event(t, self, (round(t, 7), self, acc, "steering"), lambda: lib.eventqueue.car_steering_ackermann, prio)
            lib.eventqueue.add_event(ev)
            t += lib.pt

    # used with EventQueue
    # returns position of the car at specific time t
    # also more information available (but certainly not needed),f.e. acceleration, direction, steering, angle, ...
    def get_data(self, t):
        if self.ghost:
            try:
                if (t-self.start_time) > 0:
                    index = round((t - self.start_time) / lib.pt)
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
            wheel_angles = [0, 0]
            vel = abs(vel)
            x = point.real
            y = point.imag

        else:
            x = self.last_position[0]
            y = self.last_position[1]

            x = self.position_x[-1]
            y = self.position_y[-1]

            dir = round(self.direction, 5)

            vel = self.last_velocity

            wheel_angles = self.wheelangles

        if t == 0:
            dir = self.start_direction
        return [t, self.id, round(x, 7), round(y, 7), vel, wheel_angles, dir]

    def update_path(self):
        print('update', self.last_segment_time)
        self.stored_a = self.planner.a_from_v_equi_in_t[round(self.last_segment_time/lib.pt): round((self.last_segment_time + self.segment_length) / lib.pt)]
        # fills the self.controls list with acceleration values
        self.controller.set_path(self.planner.path_from_v_equi_in_t)
        self.make_controls()
        t = self.last_segment_time
        self.last_segment_time += self.segment_length

        if self.last_segment_time < self.stop_time:
            ev = Event(self.last_segment_time - self.segment_length/3, self, (t, self,), lambda: lib.eventqueue.update_path)
            lib.eventqueue.add_event(ev)

    def brake(self, t):
        for e in lib.eventqueue.events:
            if e.object == self:
                del e
        self.emergency_brake = True
