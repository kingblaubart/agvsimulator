import random


class Channel:
    def __init__(self, e, max_trans=3):
        self.errorrate = e
        self.failed_transmissions = 0
        self.max_transmissions = max_trans

    def transmit(self):
        r = random.random()

        success = r < 1 - self.errorrate ** (self.failed_transmissions + 1)

        if success:
            self.failed_transmissions = 0
        else:
            self.failed_transmissions += 1
        return success

    def check_for_emergency(self):
        return self.failed_transmissions >= self.max_transmissions

