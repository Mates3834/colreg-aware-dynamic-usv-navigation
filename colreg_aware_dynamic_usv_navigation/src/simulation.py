import math
import numpy as np
from .models import VesselState, propagate
from .cpa_tcpa import cpa_tcpa
from .encounter import encounter_type
from .global_planner import GridMap, astar
from .local_replanner import avoidance_waypoint
from .controller import los_yaw_rate


def run(duration=180.0, dt=0.2):
    grid=GridMap()
    grid.add_rect(45,20,55,50)
    grid.add_rect(75,45,84,70)

    own=VesselState(8,8,1.8,0.0)
    targets=[
        VesselState(80,12,1.6,math.pi),
        VesselState(65,65,1.4,-math.pi/2),
        VesselState(100,40,1.2,math.pi)
    ]

    goal=np.array([110.0,72.0])
    global_path=astar(grid,(own.x,own.y),goal)
    path_idx=0

    history={"own":[],"targets":[],"risk":[],"encounters":[]}
    replans=0
    min_sep=float("inf")
    high_risk=0

    for k in range(int(duration/dt)):
        own_xy=np.array([own.x,own.y])
        if np.linalg.norm(goal-own_xy)<2.0:
            break

        # Advance global reference index.
        while path_idx < len(global_path)-1 and np.linalg.norm(global_path[path_idx]-own_xy)<3.0:
            path_idx += 1
        wp=global_path[path_idx].copy()

        current_enc=[]
        local_override=None

        for t in targets:
            sep=np.hypot(t.x-own.x,t.y-own.y)
            min_sep=min(min_sep,sep)
            dcpa,tcpa=cpa_tcpa(own,t)
            enc=encounter_type(own,t)
            current_enc.append((enc,dcpa,tcpa))

            if 0.0 < tcpa < 45.0 and dcpa < 8.0 and enc != "SAFE":
                high_risk += 1
                candidate=avoidance_waypoint(own,t,enc)
                if grid.free(*candidate):
                    local_override=candidate

        if local_override is not None:
            wp=local_override
            replans += 1

        yaw_rate=los_yaw_rate(own,wp)
        own=propagate(own,yaw_rate,dt)

        new_targets=[]
        for t in targets:
            new_targets.append(propagate(t,0.0,dt))
        targets=new_targets

        history["own"].append([own.x,own.y])
        history["targets"].append([[t.x,t.y] for t in targets])
        history["encounters"].append(current_enc)

    history["own"]=np.asarray(history["own"])
    history["targets"]=np.asarray(history["targets"])

    # Approximate compliance score: fraction of risky steps that resulted in starboard-biased override.
    compliance = 1.0 if high_risk == 0 else min(1.0, replans/max(high_risk,1))

    metrics={
        "goal_reached": bool(np.linalg.norm(goal-np.array([own.x,own.y]))<2.0),
        "minimum_separation": float(min_sep),
        "travel_time_s": len(history["own"])*dt,
        "replans": int(replans),
        "high_risk_encounters": int(high_risk),
        "approx_rule_compliance": float(compliance)
    }
    return grid, global_path, goal, history, metrics
