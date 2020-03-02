import Lib as lib
from math import sqrt, atan, pi, cos, sin, tanh
import numpy as np


class Controller:
    def __init__(self, car, kp, kd):
        self.k_p = kp
        self.k_d = kd
        self.k = 0.1
        self.k_i = 0
        self.car = car
        self.path = [None]
        self.stop_time = 0
        self.last_delta_v = 0
        self.integral = 0
        self.acc = []
        self.throttle = []
        self.vel = []

    def control(self, t):
        act_data = None
        for data in lib.data[::-1]:
            if self.car.id == data[1]:
                act_data = data
                break
        drc = act_data[-1]
        vel = act_data[-3]
        if not t > self.stop_time:
            if t > self.car.start_time:
                try:
                    index = int((t - self.car.start_time) / lib.pt)
                    planned_velocity = self.car.planner.v_equi_in_t[index]
                    point = self.path[index]

                except IndexError:
                    planned_velocity = self.car.planner.v_equi_in_t[-1]
                    point = self.path[-1]
            else:
                planned_velocity = 0
                point = self.path[0]
                planned_direction = self.car.start_dir
            x = point.real
            y = point.imag
            x_act = act_data[2]+0.5*cos(drc)*self.car.wheelbase
            y_act = act_data[3]+0.5*sin(drc)*self.car.wheelbase

            #################################################################################
            # Velocity Controlling
            #################################################################################
            position = complex(x_act, y_act)

            distances = np.absolute(np.array(self.path) - position)

            index_of_min_distance = np.argmin(distances)

            # Get nearest point
            point = self.path[index_of_min_distance]
            prev_point = self.path[index_of_min_distance-1]
            try:
                next_point = self.path[index_of_min_distance+1]
            except IndexError:
                next_point = self.path[index_of_min_distance]

            # calculate parameters
            if np.absolute(prev_point - position) > np.absolute(next_point - position):
                a = -(point.imag - next_point.imag)
                b = -(next_point.real - point.real)
                c = -(point.real * next_point.imag - next_point.real * point.imag)
            else:
                a = +(point.imag - prev_point.imag)
                b = +(prev_point.real - point.real)
                c = +(point.real * prev_point.imag - prev_point.real * point.imag)

            # crosstrack-error
            try:
                crosstrack_error = (a * x_act + b*y_act + c) / (sqrt(a**2 + b**2))
                self.car.lat_distances.append(crosstrack_error)
                steering = self.lateral_control(t, a, b, crosstrack_error, drc, vel)

            except ZeroDivisionError:
                steering = 0
                crosstrack_error = 0

            delta_v = planned_velocity - vel
            integral = self.integral + delta_v * lib.ct
            derivate = (delta_v - self.last_delta_v)

            self.last_delta_v = delta_v
            self.integral = integral

            acc = max(min(self.k_p * delta_v + self.k_i * integral + self.k_d * derivate, self.car.max_acceleration), -self.car.max_acceleration)

            #####################################################################################
            # Position Controlling
            #####################################################################################
            # distance = sqrt((y-act_data[3])**2 + (x-act_data[2])**2)
            dist_vec = complex(x, y) - complex(act_data[2], act_data[3])

            distance = np.linalg.norm(dist_vec)
            self.car.distances.append(distance)

            # decider = (planned_direction - drc + pi/2) % (2*pi)
            decider = (np.angle(dist_vec) - drc + pi/2) % (2 * pi)

            throttle = int(decider/pi) == 0
            self.throttle.append(throttle)
            try:
                vel_c = (self.car.distances[-1] - self.car.distances[-2]) / lib.ct
            except IndexError:
                vel_c = 0
            # print(vel_c * lib.ct)
            self.vel.append(vel_c)
            if distance > 0.4:
                pass
                #print('Kp', self.k_p)
            acc = max(min(self.k_p * distance + self.k_d * vel_c, self.car.max_acceleration), -self.car.max_acceleration)
            #acc = self.k_p * distance + self.k_d * vel_c
            acc = acc * (throttle - 0.5) * 2

            # correct angle
            if steering > 0:
                steering = steering % (2*pi)
            else:
                steering = steering % (-2*pi)
            if steering > pi:

                steering = 2 * pi - steering
            if steering < -pi:
                steering = 2*pi + steering
            if steering > pi:
                steering = steering - pi
            if steering < -pi:
                steering = pi + steering

            if steering > pi/2:
                steering = steering - pi

            if steering < -pi/2:
                steering = steering + pi
            #steering = max(min(steering, -0.7), 0.7)
            lib.debug_list.append([t, self.car, position, steering, a, b, c, point, crosstrack_error, dist_vec, complex(x, y), throttle])
        else:
            acc = steering = 0
            distance = throttle = 0
        try:
            int(steering)
        except ValueError:
            steering = 0

        self.acc.append(acc)
        return acc, steering

    def lateral_control(self, t, a, b, error, drc, vel):
        k = 30   # controller parameter
        if not vel == 0:
            cross_track_steering = atan(k * error / vel)  # Note: Difference in Implementation, instead of v --> k_v + v
        else:
            cross_track_steering = 0

        heading_error = atan(-a / b) - drc
        return cross_track_steering + heading_error

    def set_path(self, path):
        if self.path[0] == None:
            self.path = path
        else:
            np.append(self.path, path)
        self.stop_time = self.car.planner.t_equi_in_t[-1]
