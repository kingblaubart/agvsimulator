from CarFree2D import CarFree2D
from Event import Event
import Lib as lib
import numpy as np
from math import atan, cos, sin, tan
import copy
import random


class EventQueue:

    def __init__(self, god):
        self.events = []
        self.last_index = 0
        self.god = god
        self.last_get_data = 0
        self.last_get_vis_data = 0
        self.last_coll_control = 0
        self.last_control = 0
        self.counter = []
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.check_for_collision))
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.get_data))
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.get_vis_data))
        self.successes = []
        self.errors = []
        try:
            log = open("connection.log", "w")
            log.write("Connection Information:\n\n")
            log.close()
            self.log = open("connection.log", "a")

        except:
            print("A log-file could not be generated")

    def add_event(self, event: Event):
        not_inserted = True

        # iterates the current events starting with the last one
        for e in self.events[::-1]:
            if event.time >= e.time:
                index = self.events.index(e)
                if self.events[index-1].time != event.time:
                    self.events.insert(index+1, event)
                    not_inserted = False
                    break

        # for e in self.events[::-1]:
        #     if event.time >= e.time:
        #         index = self.events.index(e)
        #         if event.time == e.time:
        #             if event.priority < e.priority:
        #                 self.events.insert(index, event)
        #                 not_inserted = False
        #                 break
        #         else:
        #             self.events.insert(index+1, event)
        #             not_inserted = False
        #             break

        # if the event is not inserted yet, it has to be inserted as the first
        if not_inserted:
            self.events.insert(0, event)

        # Add get_data
        difference = event.time - self.last_get_data
        to_add = round(difference / lib.dt)
        for i in range(to_add):
            self.last_get_data = round(self.last_get_data + lib.dt, 7)
            data_event = Event(self.last_get_data, None, (self.last_get_data,), lambda: lib.eventqueue.get_data, 2)
            lib.eventqueue.add_event(data_event)

        # Add get_vis_data
        difference = event.time - self.last_get_vis_data
        vdt = 1/lib.fps
        to_add = int(difference * lib.fps)
        # if to_add != round(difference * lib.fps):
        #     print(self.last_get_vis_data * lib.fps, to_add, difference * lib.fps, round(difference * lib.fps))
        for i in range(to_add):
            self.last_get_vis_data = round(self.last_get_vis_data + vdt, 7)
            vis_data_event = Event(self.last_get_vis_data, None, (self.last_get_vis_data,), lambda: lib.eventqueue.get_vis_data, 3)
            lib.eventqueue.add_event(vis_data_event)

        # Add check_for_collision - deactivated at the moment for runtime reasons
        difference = event.time - self.last_coll_control
        to_add = round(difference / lib.coll_det_freq)
        for i in range(to_add):
            self.last_coll_control = round(self.last_coll_control + lib.coll_det_freq, 7)
            coll_event = Event(self.last_coll_control, None, (self.last_coll_control,), lambda: lib.eventqueue.check_for_collision, 5)
            # lib.eventqueue.add_event(coll_event)

        # Add control
        difference = event.time - self.last_control
        to_add = round(difference / lib.ct)
        # print(to_add, difference / lib.ct)
        for i in range(to_add):
            self.last_control = round(self.last_control + lib.ct, 7)
            control_event = Event(self.last_control+1e-9, None, (self.last_control,), lambda: lib.eventqueue.control, 4)
            lib.eventqueue.add_event(control_event)

    def exe(self, x, y):
        x(*y)

    # INCLUDED FUNCTIONS
    # linking functions to "real" function

    # creates the paths of the cars
    def create_spline(self, car):
        car = self.god.cars[car.id]
        print(car.id)
        car.create_spline()
        # if the latency differs from 0 a copy of the car is created which has no
        # latency and is displayed transparently

        car_copy = copy.deepcopy(car)
        car_copy.ghost = True
        car_copy.id = str(car_copy.id) + ' Ghost'

        self.god.cars.append(car_copy)
        lib.carList.append(car_copy)

    # gives the cars their acc and dir values
    def car_steering(self, t, car, acc_x, acc_y,  stop, tag):
        return
        car.steer(t, acc_x, acc_y, stop)

    # gives the cars their acc and dir values
    def car_steering_ackermann(self, t, car, acc, tag):
        #print('steer', t)
        car.steer(t, acc, 0)

    # appends the current state to the library, its entries are needed to
    # display the car in the animation
    def get_data(self, t):
        #print('data', t)
        for car in lib.carList:
            lib.last_get_data = t
            lib.data.append(car.get_data(t))
            
    def get_vis_data(self, t):
        #print('vis_data', t)
        for car in lib.carList:
            lib.vis_data.append(car.get_data(t))

    def check_for_collision(self, t):
        lib.collision.predict_collision(t)

    def control(self, t):
        #print('control', t)
        self.get_data(t)
        for car in lib.carList:
            if not car.ghost:
                # control car
                a, angle = car.controller.control(t)
                # controlled steering angle gets applied immediately
                car.steering_control = angle
                if car.channel.transmit():
                    ev = Event(t, car, (t, car, lambda: lib.eventqueue.car_control, (t, car, a, angle)),
                               lambda: lib.eventqueue.store_command)
                    if t <= car.stop_time:
                        lib.eventqueue.add_event(ev)
                    self.successes.append([t, car.id])
                    self.log.write(str(t)+' Car '+str(car.id)+' Number of Channels: '+str(car.channel.get_last_config())+' \n')
                else:
                    if car.channel.check_for_emergency():
                        print('BRAKING')
                        car.brake(t)
                    self.errors.append([t, car.id])
                    # self.log.write(str(t)+' Car '+str(car.id)+' ERROR\n')

    def car_control(self, t, car, a, angle):
        car.control(a)
        #car.steer(t, a, angle)

    def update_path(self, t, car):
        r = random.randint(0, 3)
        if r == 13:
            car.brake(t)
        car.update_path()

    # the stored command gets applied
    def apply_control(self, func, parameters):
        self.exe(func(), parameters)

    # a command is stored in the eventqueue,
    def store_command(self, t, obj, func, parameters):
        if obj.ghost:
            latency = 0
        else:
            latency = random.uniform(obj.min_latency, obj.max_latency)

        # Adding the latency
        par = list(parameters)
        par[0] += latency
        parameters = tuple(par)
        t += latency

        # storing command in the eventqueue, the command is later run by apply_control
        self.add_event(Event(t, obj, (func, parameters), lambda: lib.eventqueue.apply_control))

    def has_elements(self):
        return not len(self.events) == 0
