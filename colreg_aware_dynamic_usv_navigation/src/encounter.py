import math


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def relative_bearing(own, target):
    bearing = math.atan2(target.y-own.y, target.x-own.x)
    return wrap(bearing-own.heading)


def encounter_type(own, target, angle_tol_deg=15.0):
    """
    Simplified geometric COLREG-inspired classifier.
    Not a legal or certified rules engine.
    """
    rb = relative_bearing(own, target)
    rel_heading = wrap(target.heading-own.heading)
    tol = math.radians(angle_tol_deg)

    # Nearly reciprocal headings and target near bow.
    if abs(abs(rel_heading)-math.pi) < math.radians(25) and abs(rb) < tol:
        return "HEAD_ON"

    # Target roughly astern and similar heading -> overtaking geometry.
    if abs(abs(rb)-math.pi) < math.radians(25) and abs(rel_heading) < math.radians(35):
        return "OVERTAKING"

    # Starboard-side crossing geometry.
    if -math.radians(112.5) < rb < 0:
        return "CROSSING_GIVE_WAY"

    if 0 < rb < math.radians(112.5):
        return "CROSSING_STAND_ON"

    return "SAFE"
