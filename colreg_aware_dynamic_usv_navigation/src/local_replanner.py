import math
import numpy as np


def avoidance_waypoint(own, target, encounter, offset=10.0):
    """
    Generates a generic local waypoint biased to own-ship starboard side
    for give-way style encounters.
    """
    fwd=np.array([math.cos(own.heading), math.sin(own.heading)])
    starboard=np.array([math.sin(own.heading), -math.cos(own.heading)])

    if encounter in ("HEAD_ON","CROSSING_GIVE_WAY","OVERTAKING"):
        p=np.array([own.x,own.y]) + 2.0*offset*fwd + offset*starboard
    else:
        p=np.array([own.x,own.y]) + 2.0*offset*fwd
    return p
