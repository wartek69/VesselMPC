from Vessel import Vessel
#import matplotlib.pyplot as plt

heading = 0
x = 0
y = 0
rot = 0
speed = 1
rot_change = 0.01

if __name__ == '__main__':
    vessel = Vessel(x, y, rot, heading, speed, rot_change)
    while True:
        vessel.simulate(20)
