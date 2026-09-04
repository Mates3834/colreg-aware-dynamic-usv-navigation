import numpy as np


def cpa_tcpa(own, target):
    p_rel = np.array([target.x-own.x, target.y-own.y], dtype=float)
    v_rel = target.velocity() - own.velocity()
    vv = float(v_rel @ v_rel)

    if vv < 1e-9:
        tcpa = 0.0
        dcpa = float(np.linalg.norm(p_rel))
    else:
        tcpa = -float(p_rel @ v_rel) / vv
        closest = p_rel + tcpa * v_rel
        dcpa = float(np.linalg.norm(closest))

    return dcpa, tcpa
