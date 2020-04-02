import random


class Channel:
    def __init__(self, e, sara_scheme):
        self.errorrate = e
        self.failed_transmissions = 0
        self.max_consec_tolerance = len(sara_scheme)
        self.sara_scheme = sara_scheme
        self.last_config = 0

    def transmit(self):
        r = random.random()
        # print(self.channel_properties[self.failed_transmissions])
        success = r < 1 - self.errorrate ** (self.sara_scheme[self.failed_transmissions])
        self.last_config = self.sara_scheme[self.failed_transmissions]
        if success:
            self.failed_transmissions = 0
        else:
            self.failed_transmissions += 1
        return success

    def check_for_emergency(self):
        return self.failed_transmissions >= self.max_consec_tolerance

    def get_last_config(self):
        return self.last_config
