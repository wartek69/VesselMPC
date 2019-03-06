import logging
import sys
import copy
from Coords import Coords
import numpy as np
from MPC import MPC
from Vessel import Vessel
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline
import time


heading = 30
x = -500 # m
y = -2500 # m
rot = 0  # degree/min
speed = 1 #m/s
rot_change = 0.03 # degree/s/s
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
    x = [0, 500, 2000, 3000]
    y = [-1500, -800, -1800, 4800]
    cs = CubicSpline(x, y)
    for k in range(length):
        px.append(k)
        py.append(cs(k))


def generate_data():
    requested_rot = 20
    xcoord = []
    ycoord = []
    f = open("generated_data/training.data", "w+")
    f.write("#x,y,rot,heading,speed,rrot\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(1000):
        if i > 500:
            requested_rot = -20
        f.write('{:.3f},{:.3f},{:.3f},{:.3f},{},{}\n'.format(
            simulated_vessel.x,
            simulated_vessel.y,
            simulated_vessel.rot,
            simulated_vessel.heading,
            simulated_vessel.speed,
            requested_rot))
        xcoord.append(simulated_vessel.x);
        ycoord.append(simulated_vessel.y);
        simulated_vessel.simulate(requested_rot)
    f.close();
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


if __name__ == '__main__':
    # generate_data()
    vessel = Vessel(x, y, rot, heading, speed, rot_change, rot_max, rot_min)
    mpc = MPC()
    i = 0
    x = []
    y = []
    create_path(2001)
    while i < 4000:
        start = time.time()
        rrot = mpc.optimize_simple(px, py, vessel)
        stop = time.time();
        print("elapsed time: {}".format(stop-start))
        vessel.simulate(rrot)
        x.append(vessel.x)
        y.append(vessel.y)
        i += 1
        logger.debug(i)

    plt.plot(x, y, 'ro', markersize = 1)
    plt.plot(px, py, 'bo', markersize = 1)
    #plt.gca().set_ylim([8000, -8000])
    #plt.gca().set_xlim([-200, 200])
    plt.show()
    logger.debug(x)
    logger.debug(y)
    logger.debug(vessel.rot)

