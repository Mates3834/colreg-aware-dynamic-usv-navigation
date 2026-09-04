from dataclasses import dataclass
import math
import numpy as np


@dataclass
class VesselState:
    x: float
    y: float
    speed: float
    heading: float

    def velocity(self):
        return np.array([
            self.speed * math.cos(self.heading),
            self.speed * math.sin(self.heading)
        ], dtype=float)


def propagate(state, yaw_rate, dt, max_yaw_rate=0.35):
    yaw_rate = float(np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate))
    heading = state.heading + yaw_rate * dt
    x = state.x + state.speed * math.cos(heading) * dt
    y = state.y + state.speed * math.sin(heading) * dt
    return VesselState(x, y, state.speed, heading)
