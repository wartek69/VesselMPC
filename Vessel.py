import math


class Vessel:
    # x and y in meters!
    # speed m/s
    # rot_change in degree/s/s
    def __init__(self, x, y, rot, heading, speed, rot_change):
        self.x = x
        self.y = y
        self.rot = rot
        self.heading = heading
        self.speed = speed
        self.rot_change = rot_change

    def simulate(self, rrot, delta_t = 1):
        self.x = self.x + math.sin(self.heading / (180 * math.pi)) * self.speed * delta_t
        self.y = self.y + math.cos(self.heading / (180 * math.pi)) * self.speed * delta_t
        self.heading = (self.heading + self.rot * delta_t) % 360
        if self.rot < rrot:
            self.rot = self.rot + self.rot_change * delta_t
            if self.rot > rrot:
                self.rot = rrot
        elif self.rot > rrot:
            self.rot = self.rot - self.rot_change * delta_t
            if self.rot < rrot:
                self.rot = rrot
        else:
            self.rot = rrot
