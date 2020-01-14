from newCarFree2D import CarFree2D
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
        # if the event is not inserted yet, it has to be inserted as the first
        if not_inserted:
            self.events.insert(0, event)

        # Add get_data
        difference = event.time - self.last_get_data
        to_add = int(difference / lib.dt)
        for i in range(to_add):
            self.last_get_data = round(self.last_get_data + lib.dt, 7)
            data_event = Event(self.last_get_data, None, (self.last_get_data,), lambda: lib.eventqueue.get_data)
            lib.eventqueue.add_event(data_event)

        # Add get_vis_data
        difference = event.time - self.last_get_vis_data
        vdt = 1/lib.fps
        to_add = int(difference / vdt)
        for i in range(to_add):
            self.last_get_vis_data = round(self.last_get_data + vdt, 7)
            vis_data_event = Event(self.last_get_vis_data, None, (self.last_get_vis_data,), lambda: lib.eventqueue.get_vis_data)
            lib.eventqueue.add_event(vis_data_event)

        # Add check_for_collision - deactivated at the moment for runtime reasons
        difference = event.time - self.last_coll_control
        to_add = int(difference / lib.coll_det_freq)
        for i in range(to_add):
            self.last_coll_control = round(self.last_coll_control + lib.coll_det_freq, 7)
            coll_event = Event(self.last_coll_control, None, (self.last_coll_control,), lambda: lib.eventqueue.check_for_collision)
            #lib.eventqueue.add_event(coll_event)

        # Add control
        difference = event.time - self.last_control
        to_add = int(difference / lib.ct)
        for i in range(to_add):
            self.last_control = round(self.last_control + lib.ct, 7)
            control_event = Event(self.last_control, None, (self.last_control,), lambda: lib.eventqueue.control)
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

        if not lib.latency == 0:
            car_copy = copy.deepcopy(car)
            car_copy.ghost = True
            car_copy.id = str(car_copy.id) + ' Ghost'

            # command to generate event queue entries, not needed anymore?
            # car_copy.make_controls()

            self.god.cars.append(car_copy)
            lib.carList.append(car_copy)

    # gives the cars their acc and dir values
    def car_steering(self, t, car: CarFree2D, acc_x, acc_y,  stop, tag):
        car.steer(t, acc_x, acc_y, stop)

    def car_steering_ackermann(self, t, car: CarFree2D, acc, drc, stop, tag):
        car.steer_ackermann(t, acc, drc)

    # appends the current state to the library, its entries are needed to
    # display the car in the animation
    def get_data(self, t):
        for car in lib.carList:
            lib.data.append(car.get_data(t))
            
    def get_vis_data(self, t):
        for car in lib.carList:
            lib.vis_data.append(car.get_data(t))

    def check_for_collision(self, t):
        lib.collision.predict_collision(t)

    def control(self, t):
        for car in lib.carList:
            if not car.ghost:
                r = random.randint(1, 100) / 100
                a, angle = car.controller.control(t)
                if r > car.errorrate:
                    ev = Event(t, car, (t, car, lambda: lib.eventqueue.car_control, (t, car, a, angle)),
                               lambda: lib.eventqueue.store_command)
                    if t <= car.stop_time:
                        lib.eventqueue.add_event(ev)
                    self.successes.append([t, car.id])
                    self.log.write(str(t)+' Car '+str(car.id)+' OK\n')
                else:
                    self.errors.append([t, car.id])
                    self.log.write(str(t)+' Car '+str(car.id)+' ERROR\n')

    def car_control(self, t, car, a, angle):
        return
        car.control_ackermann(a, angle)

    def correct_controls(self, t, car, ax, ay):
        try:
            for ev in self.events:
                if ev.object == car:
                    t, car, acc_x, acc_y, stop, tag = ev.parameters
                    if tag == 'steering':
                        acc_x += ax
                        acc_y += ay

                        # direction = tan(acc_y / acc_x)

                        ev.parameters = (t, car, acc_x, acc_y, stop, 'control')
                        break
        except ValueError:
            pass

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
