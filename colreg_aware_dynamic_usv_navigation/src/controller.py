import math
import numpy as np


def wrap(a):
    return (a + math.pi)%(2*math.pi)-math.pi


def los_yaw_rate(own, waypoint, kp=1.2, max_rate=0.35):
    desired=math.atan2(waypoint[1]-own.y, waypoint[0]-own.x)
    e=wrap(desired-own.heading)
    return float(np.clip(kp*e,-max_rate,max_rate))
