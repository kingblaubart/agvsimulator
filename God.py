from CarFree2D import CarFree2D
from CarAckermann import CarAckermann
from Obstacles2D import Obstacles2D
from CollisionControl import CollisionControl
from EventQueue import EventQueue
from Event import Event
import Lib as lib
import control
from RoutePlanner import RoutePlanner


class God:

    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # path_file = pygame.image.load(image_path)

    def __init__(self, parameters):
        self.parameters = parameters
        self.cars = []  # list of each car in simulation
        self.real_cars = [] # list of cars in simulation, does not contain the ghosts
        self.last_timestamp = 0  # stores the last timestamp - to stop the simulation after it
        self.size = parameters["God"]["size"]  # size of the canvas in m
        # self.size = [0, 0]         # stores highest x and y values for matching the simulation area
        self.calculation = []  # list of polynomial for a specific period of time --> WHAT DOES THIS MEAN?
        self.simulation = []
        self.controller_data = []
        dt = parameters["God"]["dt"]  # time between each data point in ms
        lib.set_dt(dt)
        self.ct = parameters["God"]["ct"]  # time between each controller input
        lib.set_ct(self.ct)
        lib.set_fps(parameters["SpaceFree2D"]["fps"])
        self.obstacles = []
        self.colldet = self.parameters["CollisionControl"]["activated"]
        self.collisions = [10000, 10000, 10000]
        # INSERT IN LIBRARY
        lib.set_coll_det_freq(parameters["CollisionControl"]["collision_detection_frequency"])
        eq = EventQueue(self)
        lib.set_eventqueue(eq)
        lib.set_k_d(parameters["God"]["k_d"])
        lib.set_k_p(parameters["God"]["k_p"])
        lib.set_pt(parameters["God"]["pt"])
        lib.set_holonom(parameters["God"]["holonom"])
        self.eventlist_debug = []
        self.make_statespace()

    def file_read(self):
        #############################
        ## INITIALIZE OBSTACLES #####
        #############################
        route_planner = RoutePlanner(self)
        if self.parameters["God"]["Layout"]:
            obs_x = self.parameters["Layout"]["Obstacles_in_x_direction"]
            obs_y = self.parameters["Layout"]["Obstacles_in_y_direction"]

            route_planner.make_layout(self.size[0], self.size[1], obs_x, obs_y)

        else:
            obstacles_origin = self.parameters["Obstacles"]

            for obst in obstacles_origin:
                spawn_x = float(obst["corners"][0])
                spawn_y = float(obst["corners"][1])
                if spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]:
                    raise Exception('An obstacle cannot be defined outside the canvas boundary.')
                edges = obst["corners"]
                color = obst["color"]

                obstacle = Obstacles2D(spawn_x, spawn_y, edges, color)
                self.obstacles.append(obstacle)

        ############################
        ## INITIALIZE EACH CAR #####
        ############################
        cars_origin = self.parameters["Cars"]

        for car in cars_origin:
            car_id = int(car["index"])
            if self.parameters["God"]["Path"]:
                spawn_x = float(car["spawn_x"])
                spawn_y = float(car["spawn_y"])
                start_dir = None
                end_dir = None
            else:
                pos = car["start"]
                spawn_x = (2 * pos[0] + 1.5) * route_planner.ob_width
                spawn_y = (2 * pos[1] + 1.5) * route_planner.ob_height
                start_dir = car["start_direction"]
                end_dir = car["end_direction"]

            if spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]:
                raise Exception('A car cannot spawn outside of canvas.')

            angle = float(car["angle"])
            length = float(car["length"])
            width = float(car["width"])
            max_vel = float(car["max_vel"])
            max_acc = float(car["max_acc"])
            color = str(car["color"])
            dest = car["finish"]
            min_latency = car["min_latency"]
            max_latency = car["max_latency"]
            start_time = car["start_time"]
            errorrate = car["errorrate"]
            segment_length = car['segment_length']
            sara_scheme = car['sara_scheme']

            if lib.holonom:
                car = CarFree2D(car_id, spawn_x, spawn_y, start_time, start_dir, end_dir, length, width, angle, max_vel,
                                max_acc, color, min_latency, max_latency, errorrate)
            else:
                max_steering_angle = float(car["max_steering_angle"])
                car = CarAckermann(car_id, spawn_x, spawn_y, start_time, start_dir, end_dir, length, width, angle,
                                   max_vel, max_acc, max_steering_angle, color, min_latency, max_latency, errorrate, segment_length, sara_scheme)
            car.destination = dest
            self.cars.append(car)
            lib.carList.append(car)

        ################################
        ## ASSIGN PATH TO EACH CAR #####
        ################################
        if self.parameters["God"]["Path"]:
            path_origin = self.parameters["Path"]
            #
            #
            # [car_id,timestamp,pos_x,pos_y,destination]
            #
            # car_id:       identifies the car
            # timestamp:    at what time does the car need to be at a certain location? Unit is s
            # pos_x,pos_y:  where does the car need to be? Unit is m
            # destination:  True  = the speed of the car at the specified location is supposed to be zero
            #               False = this is a midway point
            for path_data in path_origin:
                car_id = int(path_data["car_id"])
                pos_x = float(path_data["pos_x"])
                pos_y = float(path_data["pos_y"])
                if pos_x < 0 or pos_x > self.size[0] or pos_y < 0 or pos_y > self.size[1]:
                    raise Exception('The path of a car cannot reach outside the canvas.', car_id, pos_x, pos_y, self.size[0], self.size[1])

                try:
                    self.cars[car_id].set_waypoint(pos_x, pos_y)
                except IndexError:
                    print("No car with matching ID found")
                # CHECK IF EVERY CAR HAS AT LEAST ONE DESTINATION POINT
            for car in self.cars:
                if len(car.path.points) == 1:
                    raise Exception(
                        'Every car needs at least one point (the destination) in the path file. The car with id=' + str(
                            car.id) + ' is missing.')
        else:
            for car in self.cars:
                pos = car.destination
                end_x = (2 * pos[0] + 1.5) * route_planner.ob_width
                end_y = (2 * pos[1] + 1.5) * route_planner.ob_height

                car.set_waypoint(end_x, end_y)
                car.end = [end_x, end_y]
                points = route_planner.make_route(car)
                car.waypoints = points
                car.path.points = points

        # Adding outer boundary as obstacles
        # Bottom
        self.obstacles.append(Obstacles2D(0.1, 0.1, [0.1, 0.1, self.size[0], 0.1, self.size[0], -1, 0.1, -1], 'white'))
        # Left
        self.obstacles.append(Obstacles2D(-1, self.size[1], [-1, self.size[1], 0.1, self.size[1], 0.1, 0.1, -1, 0.1], 'white'))
        # Top
        self.obstacles.append(Obstacles2D(0.1, self.size[1]+1, [0.1, self.size[1]+1, self.size[0]-0.1, self.size[1]+1,
                                                              self.size[0]-0.1, self.size[1]-0.1, 0.1, self.size[1]-0.1], 'white'))
        # Right
        self.obstacles.append(Obstacles2D(self.size[0]-0.1, self.size[1]-0.1, [self.size[0]-0.1, self.size[1], self.size[0]+1,
                                                                       self.size[1], self.size[0]+1, 0, self.size[0]-0.1,
                                                                       0], 'white'))

        lib.set_collision(CollisionControl(self))
        lib.set_carcount(len(self.cars))
        self.real_cars = self.cars[:]

        # Make DC-motor
    def make_statespace(self):
        jm = self.parameters["DC-Motor"]["Jm"]
        bm = self.parameters["DC-Motor"]["Bm"]
        kme = self.parameters["DC-Motor"]["Kme"]
        kmt = self.parameters["DC-Motor"]["Kmt"]
        rm = self.parameters["DC-Motor"]["Rm"]
        lm = self.parameters["DC-Motor"]["Lm"]

        kdm = self.parameters["DC-Motor"]["Kdm"]
        kpm = self.parameters["DC-Motor"]["Kpm"]
        kim = self.parameters["DC-Motor"]["Kim"]
        nm = self.parameters["DC-Motor"]["Nm"]

        dc = control.TransferFunction([0, kmt], [jm * lm, bm * lm + jm * rm, bm * rm + kme * kmt])
        pidm = control.TransferFunction([kpm + kdm * nm, kpm * nm + kim, kim * nm], [1, nm, 0])

        ii = control.TransferFunction([1], [1, 0, 0])

        agv = ii * control.feedback(dc*pidm, sign=-1)

        # Laplace --> Z
        agvz = control.sample_system(agv, lib.pt, method='zoh')

        # Transferfunction --> StateSpace
        ss = control.tf2ss(agvz)

        lib.set_statespace(ss)

    def simulate(self):

        # Spline-Creation
        for car in self.cars:
            event = Event(-1, car, (car, ), lambda: lib.eventqueue.create_spline)
            lib.eventqueue.add_event(event)

        # executing the entries of the eventqueue
        while lib.eventqueue.has_elements():
            event = lib.eventqueue.events.pop(0)

            #######
            # only for debugging
            try:
                self.eventlist_debug.append(event.time)
            except AttributeError:
                self.eventlist_debug.append([event.function, event.object, event.parameters])
            ########
            pass
            lib.eventqueue.exe(event.function(), event.parameters)
        lib.eventqueue.log.write('\n\n\nSummary of connection:\n\n')
        transmissions = len(lib.eventqueue.successes) + len(lib.eventqueue.errors)
        if transmissions > 0:
            lib.eventqueue.log.write('Successful transmissions: '+str(len(lib.eventqueue.successes))+' (' +
                                     str(round(100*len(lib.eventqueue.successes)/transmissions, 2))+'%)\n')
            lib.eventqueue.log.write('Failed transmissions: '+str(len(lib.eventqueue.errors))+' (' +
                                     str(round(100*len(lib.eventqueue.errors)/transmissions, 2))+'%)')
        lib.eventqueue.log.close()

        self.last_timestamp = lib.data[-1][0]

        # Collision Detection
        if self.colldet:
            coll = CollisionControl(self)
            coll.check_for_collision()
