import math
from random import randint
import random
secondsPerMin = 60


class Vessel:
    # x and y in meters!
    # speed m/s
    # rot_change in degree/s/s
    def __init__(self, x, y, rot, heading, speed, rot_change, rot_max, rot_min):
        self.x = x
        self.y = y
        self.rot = rot
        self.heading = heading
        self.speed = speed
        self.rot_change = rot_change
        self.rot_max = rot_max
        self.rot_min = rot_min

    # adds noise to the simulated values
    def simulate_noisy(self, rrot, delta_t = 1):
        rrot = self.__clamp(rrot)
        self.x = self.x + math.sin(self.heading / 180 * math.pi) * self.speed * delta_t + randint(-1,1)
        self.y = self.y + math.cos(self.heading / 180 * math.pi) * self.speed * delta_t + randint(-1,1)
        self.heading = (self.heading + self.rot / secondsPerMin * delta_t) % 360 + random.uniform(-1,1)
        if self.rot < rrot:
            self.rot = self.rot + self.rot_change * delta_t + random.uniform(-1,1)
            if self.rot > rrot:
                self.rot = rrot
        elif self.rot > rrot:
            self.rot = self.rot - self.rot_change * delta_t + random.uniform(-1,1)
            if self.rot < rrot:
                self.rot = rrot
        else:
            self.rot = self.rot + random.uniform(0,1)

    def simulate(self, rrot, delta_t = 1):
        rrot = self.__clamp(rrot)
        self.x = self.x + math.sin(self.heading / 180 * math.pi) * self.speed * delta_t
        self.y = self.y + math.cos(self.heading / 180 * math.pi) * self.speed * delta_t
        self.heading = (self.heading + self.rot / secondsPerMin * delta_t) % 360
        if self.rot < rrot:
            self.rot = self.rot + self.rot_change * delta_t
            if self.rot > rrot:
                self.rot = rrot
        elif self.rot > rrot:
            self.rot = self.rot - self.rot_change * delta_t
            if self.rot < rrot:
                self.rot = rrot
        else:
            self.rot = self.rot

    def __clamp(self, rot):
        if rot > self.rot_max:
            return self.rot_max
        elif rot < self.rot_min:
            return self.rot_min
        else:
            return rot



