from Vessel import Vessel
import matplotlib.pyplot as plt

heading = 0
x = 0
y = 0
rot = 0
speed = 1
rot_change = 0.01
rrot = -10

if __name__ == '__main__':
    vessel = Vessel(x, y, rot, heading, speed, rot_change)
    i = 0
    x = []
    y = []
    while i < 5000:
        i += 1
        print(vessel.heading)
        vessel.simulate(rrot)
        x.append(vessel.x)
        y.append(vessel.y)
    plt.plot(x, y, 'ro', markersize = 1)
    plt.show()
    print(x)
    print(y)
    print(vessel.rot)

