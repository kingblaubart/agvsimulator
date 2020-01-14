import Lib as lib
from math import sqrt, atan, pi


class Controller:
    def __init__(self, car, kp, kd):
        self.k_p = kp
        self.k_d = kd
        self.k = 0.1
        self.car = car
        self.path = None
        self.stop_time = 0
        self.last_x_delta = 0
        self.last_y_delta = 0

    def control(self, t):
        act_data = None
        for data in lib.data[::-1]:
            if self.car.id == data[1]:
                act_data = data
                break
        drc = act_data[-1]
        vel = act_data[-2]
        if not t > self.stop_time:
            if t > self.car.start_time:
                # try:
                #     index = int((t - self.car.start_time) / lib.dt)
                #     point = self.path[index]
                #     next_point = self.path[index+1]
                #     planned_direction = self.car.planner.directions[index]
                # except IndexError:
                #     next_point = point = self.path[-1]
                #     planned_direction = self.car.planner.directions[-1]
                try:
                    index = int((t - self.car.start_time) / lib.dt)
                    point = self.path[index]
                    x = point.real
                    y = point.imag

                    planned_direction = self.car.planner.directions[index]
                    x_delta = act_data[2] - x
                    y_delta = act_data[3] - y
                    distance = sqrt(x_delta**2 + y_delta**2)
                    index += int((distance/vel)/lib.dt)
                    next_point = self.path[index]
                except IndexError:
                    next_point = point = self.path[-1]
                    planned_direction = self.car.planner.directions[-1]
                    x = point.real
                    y = point.imag
                    x_delta = act_data[2] - x
                    y_delta = act_data[3] - y


            else:
                x, y = self.car.spawn
                next_point = self.path[1]
                planned_direction = self.car.start_direction
                x_delta = act_data[2] - x
                y_delta = act_data[3] - y

            x_next = next_point.real
            y_next = next_point.imag

            a = y - y_next
            b = x_next - x
            c = x * y_next - x_next * y


            vx = (x_delta - self.last_x_delta) / lib.ct
            vy = (y_delta - self.last_y_delta) / lib.ct

            try:
                crosstrack_error = (a * act_data[2] + b*act_data[3] + c) / (sqrt(a**2 + b**2))

                steering = self.lateral_control(t, a, b, crosstrack_error, drc, vel)
            except ZeroDivisionError:
                steering = 0

            distance = sqrt(x_delta**2 + y_delta**2)
            self.car.distances.append(distance)

            decider = (planned_direction - drc + pi/2) % (2*pi)

            throttle = int(decider/pi) == 0

            # ax = lib.k_p * x_delta + lib.k_d * vx
            # ay = lib.k_p * y_delta + lib.k_d * vy
            try:
                vel_c = (self.car.distances[-1] - self.car.distances[-2]) / lib.ct
            except IndexError:
                vel_c = 0

            acc = lib.k_p * distance + lib.k_d * vel_c

            self.last_x_delta = x_delta
            self.last_y_delta = y_delta
            #steering = 0
            
        else:
            acc = steering = 0
        return acc, steering

    def lateral_control(self, t, a, b, error, drc, vel):
        k = 0.7  # controller parameter
        cross_track_steering = atan(k * error / vel)  # Note: Difference in Implementation, instead of v --> k_v + v
        heading_error = atan(-a / b) - drc
        return cross_track_steering + heading_error

    def set_path(self, path):
        self.path = path.tolist()
        self.stop_time = self.car.planner.t_equi_in_t[-1]
