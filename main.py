import logging
import sys
import copy
from Coords import Coords
from MPC import MPC
from Vessel import Vessel
import matplotlib.pyplot as plt

heading = 20
x = 0 # m
y = 0 # m
rot = 0 # degree/min
speed = 1 #m/s
rot_change = 0.01 # degree/s/s
rrot = 20 # degree/min
rot_max = 20
rot_min = -20

path = []
px = []
py = []
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


def create_path():
    for k in range(3000):
        px.append(k)
        py.append(k)

if __name__ == '__main__':
    vessel = Vessel(x, y, rot, heading, speed, rot_change, rot_max, rot_min)
    mpc = MPC()
    i = 0
    x = []
    y = []

    create_path()
    while i < 1000:
        rrot = mpc.optimize_simple(px, py, vessel)
        vessel.simulate(rrot)
        x.append(vessel.x)
        y.append(vessel.y)
        i += 1
        logger.debug(i)

    plt.plot(px, py, 'bo', markersize = 1)
    plt.plot(x, y, 'ro', markersize = 1)
    plt.show()
    logger.debug(x)
    logger.debug(y)
    logger.debug(vessel.rot)

