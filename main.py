import logging
import sys
import copy
import numpy as np
from MPC import MPC
from Vessel import Vessel
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline
from random import randrange
from random import randint
import time


heading = 30
#x = -800 # m
#y = 80# m


x = -10 # m
y = -1600# m
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
    x = [0, 500, 2000, 4000, 6000]
    y = [-1500, -800, -1800, 2000, -500]
    cs = CubicSpline(x, y)
    for k in range(length):
        px.append(k)
        py.append(cs(k))

def create_path_v2(length):
    x = [-900, 500, 2000, 4000, 6000]
    y = [1500, 2000, 1800, -2000, 500]
    cs = CubicSpline(x, y)
    for k in range(length):
        px.append(k)
        py.append(cs(k))

def create_path_v3(length):
    x = [-900, 500 , 1500, 1900, 2500]
    y = [0, -100, 200, -100, 0]
    cs = CubicSpline(x, y)
    for k in range(length):
        px.append(k)
        py.append(cs(k))


def generate_data(rrot1, rrot2, name, amount_of_samples, steering_switch):
    requested_rot = rrot1
    xcoord = []
    ycoord = []
    f = open("generated_data/" + name, "w+")
    f.write("#x;y;rot;heading;speed;rrot;x';y';heading'\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(amount_of_samples):
        if i > steering_switch:
            requested_rot = rrot2
        f.write('{:.3f};{:.3f};{:.3f};{:.3f};{};{};'.format(
            simulated_vessel.x,
            simulated_vessel.y,
            simulated_vessel.rot,
            simulated_vessel.heading,
            simulated_vessel.speed,
            requested_rot))
        xcoord.append(simulated_vessel.x)
        ycoord.append(simulated_vessel.y)
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x,
            simulated_vessel.y,
            simulated_vessel.heading
        ))
    f.close()
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


def generate_data_relative_with_rot(rrot1, rrot2, name, amount_of_samples, steering_switch):
    requested_rot = rrot1
    xcoord = []
    ycoord = []
    f = open("generated_data/" + name, "w+")
    f.write("#rot;heading;speed;rrot;x'-x;y'-y;heading';rot'\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(amount_of_samples):
        if i > steering_switch:
            requested_rot = rrot2
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        f.write('{:.3f};{:.3f};{};{};'.format(
            simulated_vessel.rot,
            simulated_vessel.heading,
            simulated_vessel.speed,
            requested_rot))
        xcoord.append(simulated_vessel.x)
        ycoord.append(simulated_vessel.y)
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x-start_x,
            simulated_vessel.y-start_y,
            simulated_vessel.heading,
            simulated_vessel.rot
        ))
    f.close()
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


def generate_data_relative_with_rot_no_speed(rrot1, rrot2, name, amount_of_samples, steering_switch):
    requested_rot = rrot1
    xcoord = []
    ycoord = []
    f = open("generated_data/" + name, "w+")
    f.write("#rot;heading;rrot;x'-x;y'-y;heading';rot'\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(amount_of_samples):
        if i > steering_switch:
            requested_rot = rrot2
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        f.write('{:.3f};{:.3f};{};'.format(
            simulated_vessel.rot,
            simulated_vessel.heading,
            requested_rot))
        xcoord.append(simulated_vessel.x)
        ycoord.append(simulated_vessel.y)
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x-start_x,
            simulated_vessel.y-start_y,
            simulated_vessel.heading,
            simulated_vessel.rot
        ))
    f.close()
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


def generate_data_relative_with_relative_everything(rrot1, rrot2, name, amount_of_samples, steering_switch):
    requested_rot = rrot1
    xcoord = []
    ycoord = []
    f = open("generated_data/" + name, "w+")
    f.write("#speed;rrot;x'-x;y'-y;heading'-heading;rot'-rot\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(amount_of_samples):
        if i > steering_switch:
            requested_rot = rrot2
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        start_rot = simulated_vessel.rot
        start_heading = simulated_vessel.heading
        f.write('{};{};'.format(
            simulated_vessel.speed,
            requested_rot))
        xcoord.append(simulated_vessel.x)
        ycoord.append(simulated_vessel.y)
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x-start_x,
            simulated_vessel.y-start_y,
            simulated_vessel.heading-start_heading,
            simulated_vessel.rot-start_rot
        ))
    f.close()
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


def generate_data_with_rot(rrot1, rrot2, name, amount_of_samples, steering_switch):
    requested_rot = rrot1
    xcoord = []
    ycoord = []
    f = open("generated_data/" + name, "w+")
    f.write("#x;y;rot;heading;speed;rrot;x';y';heading';rot'\n")
    simulated_vessel = Vessel(0, 0, 0, 0, speed, rot_change, rot_max, rot_min)
    for i in range(amount_of_samples):
        if i > steering_switch:
            requested_rot = rrot2
        f.write('{:.3f};{:.3f};{:.3f};{:.3f};{};{};'.format(
            simulated_vessel.x,
            simulated_vessel.y,
            simulated_vessel.rot,
            simulated_vessel.heading,
            simulated_vessel.speed,
            requested_rot))
        xcoord.append(simulated_vessel.x)
        ycoord.append(simulated_vessel.y)
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x,
            simulated_vessel.y,
            simulated_vessel.heading,
            simulated_vessel.rot
        ))
    f.close()
    plt.plot(xcoord, ycoord, 'ro', markersize = 1)
    plt.show()


def generate_less_random_data(samples, name):
    f = open("generated_data/random/" + name, "w+")
    f.write("#rot;heading;rrot;x'-x;y'-y;heading';rot'\n")
    for i in range(samples):
        requested_rot = randint(-10, 10)
        simulated_vessel = Vessel(randint(-100, 100),
                                  randint(-100, 100),
                                  randrange(-180, 180, 5),
                                  randrange(0, 360, 20), speed, rot_change, rot_max, rot_min)
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        f.write('{:.3f};{:.3f};{};'.format(
            simulated_vessel.rot,
            simulated_vessel.heading,
            requested_rot))
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x - start_x,
            simulated_vessel.y - start_y,
            simulated_vessel.heading,
            simulated_vessel.rot
        ))
    f.close()


def generate_random_data(samples, name):
    f = open("generated_data/random/" + name, "w+")
    f.write("#rot;heading;rrot;x'-x;y'-y;heading';rot'\n")
    for i in range(samples):
        requested_rot = randint(-10, 10)
        simulated_vessel = Vessel(randint(-100, 100),
                                  randint(-100, 100),
                                  randrange(-180, 180, 1),
                                  randrange(0, 360, 5), speed, rot_change, rot_max, rot_min)
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        f.write('{:.3f};{:.3f};{};'.format(
            simulated_vessel.rot,
            simulated_vessel.heading,
            requested_rot))
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f};{:.3f};{:.3f};{:.3f}\n'.format(
            simulated_vessel.x - start_x,
            simulated_vessel.y - start_y,
            simulated_vessel.heading,
            simulated_vessel.rot
        ))
    f.close();


def generate_random_data_rotdot(samples, name):
    f = open("generated_data/random_rot_dot/" + name, "w+")
    f.write("#rot;rrot;rot_dot'\n")
    for i in range(samples):
        requested_rot = randint(-10, 10)
        simulated_vessel = Vessel(randint(-100, 100),
                                  randint(-100, 100),
                                  randrange(-180, 180, 5),
                                  randrange(0, 360, 20), speed, rot_change, rot_max, rot_min)
        start_x = simulated_vessel.x
        start_y = simulated_vessel.y
        f.write('{:.3f};{};'.format(
            simulated_vessel.rot,
            requested_rot))
        start_rot = simulated_vessel.rot
        simulated_vessel.simulate(requested_rot)
        f.write('{:.3f}\n'.format(
            simulated_vessel.rot
        ))
    f.close()

if __name__ == '__main__':
    # generate_data_relative_with_rot_no_speed(20, -20, 'training_relative_rot_1k_no_speed.data', 1000, 500)
    # generate_data_relative_with_rot_no_speed(-5, 8, 'validation_relative_rot_1k_no_speed.data', 1000, 500)

    #generate_random_data(200000, 'training_random_200k.data')
    #generate_random_data(200000, 'validation_random_200k.data')

    vessel = Vessel(x, y, rot, heading, speed, rot_change, rot_max, rot_min)
    mpc = MPC()
    i = 0
    x = []
    y = []
    create_path(4001)
    elapsed_time = []
    path_xte = []
    mpc_xte = []
    path_heading_error = []
    while i < 6000:
        start = time.time()
        if i == 423:
            print("test")
        rrot = mpc.optimize_simple_MLP_rotdot_batch(px, py, vessel)

        stop = time.time()
        print("elapsed time: {}".format(stop-start))
        elapsed_time.append(stop-start)

        vessel.simulate(rrot)

        xte, index= mpc.calc_xte(px, py, vessel)
        xtempc, indexmpc = mpc.calc_xte_improved(px, py, vessel.x, vessel.y)
        mpc_xte.append(math.sqrt(xtempc))
        #path xte is more accurate
        path_xte.append(math.sqrt(xte))
        temp_heading = mpc.get_heading_curve(px, py, index)
        heading_error = mpc.angular_diff(vessel.heading, temp_heading)
        path_heading_error.append(heading_error)
        x.append(vessel.x)
        y.append(vessel.y)
        i += 1
        logger.debug(i)

    #plots
    fig = plt.figure(1)
    plt.plot(x, y, 'ro', markersize = 1, label='Vessel')
    plt.plot(px, py, 'bo', markersize = 1, label='Track')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend()
    fig.suptitle('Simulated vessel')


    fig = plt.figure(2)
    plt.plot(elapsed_time)
    plt.xlabel('iteration (-)')
    plt.ylabel('elapsed time (s)')
    fig.suptitle('Elapsed time per iteration')


    fig = plt.figure(3)
    plt.plot(path_xte);
    plt.xlabel('iteration (-)')
    plt.ylabel('xte (m)')
    fig.suptitle('Path xte')

    fig = plt.figure(4)
    plt.plot(path_heading_error);
    plt.xlabel('iteration (-)')
    plt.ylabel('heading error (degrees)')
    fig.suptitle('Heading error')

    fig = plt.figure(5)
    plt.plot(mpc_xte);
    #plt.gca().set_ylim([8000, -8000])
    #plt.gca().set_xlim([-200, 200])
    plt.show()
    logger.debug(x)
    logger.debug(y)
    logger.debug(vessel.rot)

