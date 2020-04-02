

class Event:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, t, obj, parameters, function, prio=0):
        self.time = t
        self.object = obj
        self.function = function
        self.parameters = parameters
        self.priority = prio

