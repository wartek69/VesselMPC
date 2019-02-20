import logging
import sys
import copy
from Coords import Coords
from MPC import MPC
from Vessel import Vessel
import matplotlib.pyplot as plt
import math

heading = 90.0
x = 0 # m
y = 0 # m
rot = 0  # degree/min
speed = 1 #m/s
rot_change = 0.01 # degree/s/s
rrot = 0 # degree/min
rot_max = 20
rot_min = -20

path = []
px = []
py = []
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


def create_path_straight_line(length):
    for k in range(length):
        px.append(-k)
        py.append(-k)

def create_path_straight_line_up(length):
    for k in range(length):
        px.append(0)
        py.append(k)

def create_path_straight_line_down(length):
    for k in range(length):
        px.append(0)
        py.append(-k)

def create_path_horizontal_line_right(length):
    for k in range(length):
        px.append(k)
        py.append(0)

def create_path_horizontal_line_left(length):
    for k in range(length):
        px.append(-k)
        py.append(0)

def create_path_curve_disc(length):
    for k in range(math.floor(length/2)):
        px.append(0)
        py.append(k)
        l = k
    for k in range(math.floor(length/2 + 1)):
        px.append(k)
        py.append(l)

def create_path_curve_cont(length):
    for k in range(length):
        px.append(k)
        py.append(-k**2)

def create_path(length):
    for k in range(length):
        if k < length / 2:
            px.append(k)
            py.append(k^2)
            l = k^2
            p = k
        else:
            px.append(p)
            py.append(k)


if __name__ == '__main__':
    vessel = Vessel(x, y, rot, heading, speed, rot_change, rot_max, rot_min)
    mpc = MPC()
    i = 0
    x = []
    y = []

    create_path(2001)
    while i < 4000:
        #rrot = mpc.optimize_simple(px, py, vessel)
        vessel.simulate(rrot)
        x.append(vessel.x)
        y.append(vessel.y)
        i += 1
        if i == 2000:
            pass
        logger.debug(i)

    plt.plot(px, py, 'bo', markersize = 1)
    #plt.plot(x, y, 'ro', markersize = 1)
    #plt.gca().set_ylim([4000, -4000])
    #plt.gca().set_xlim([-200, 200])
    plt.show()
    logger.debug(x)
    logger.debug(y)
    logger.debug(vessel.rot)

