# COLREG-Aware Dynamic USV Navigation

A research-oriented simulation framework for **autonomous surface vessel navigation in dynamic multi-vessel environments**.

The project combines:

- Global path planning
- CPA / TCPA based collision-risk assessment
- Simplified COLREG-inspired encounter classification
- Dynamic obstacle prediction
- Local collision-avoidance waypoint generation
- LOS-style path tracking
- Safety and navigation performance evaluation

The implementation is designed as a generic autonomous-navigation research framework and does not represent a certified maritime collision-avoidance or navigation system.

---

# 1. Motivation

Autonomous surface vessels must navigate safely in environments containing both static and moving obstacles.

A conventional path planner may generate a collision-free route with respect to static obstacles, but maritime autonomy also requires the vessel to react to moving traffic.

The navigation problem therefore becomes:

```text
Static Map
    +
Moving Vessels
    +
Collision Prediction
    +
Encounter Classification
    +
Local Replanning
    +
Path Tracking
```

The objective is to generate safe and dynamically updated navigation behavior while maintaining progress toward the global destination.

---

# 2. System Architecture

The overall architecture is:

```text
Static Environment
        ↓
Global Goal
        ↓
A* Global Planner
        ↓
Nominal Path
        ↓
Dynamic Vessel States
        ↓
CPA / TCPA Assessment
        ↓
Encounter Classification
        ↓
Local Avoidance Decision
        ↓
Temporary Waypoint
        ↓
LOS-Style Controller
        ↓
USV Kinematic Model
        ↓
Updated Vessel State
```

The system operates continuously as the own vessel and surrounding vessels move through the environment.

---

# 3. Global Path Planning

The project uses A* as the global planner.

The search function is:

```text
f(n) = g(n) + h(n)
```

where:

```text
g(n)
```

represents the accumulated cost from the starting node, and:

```text
h(n)
```

represents the estimated remaining distance to the goal.

Euclidean distance is used as the heuristic.

The global planner produces a nominal path through the static environment.

---

# 4. Dynamic Vessel Model

Each vessel is represented using a planar kinematic state:

```text
x
y
V
ψ
```

where:

```text
x = East-like Cartesian position
y = North-like Cartesian position
V = vessel speed
ψ = heading
```

The velocity vector is:

```text
v =
[
V cos(ψ)
V sin(ψ)
]
```

The own vessel may change heading through a bounded yaw-rate command.

---

# 5. CPA and TCPA

Dynamic collision risk is evaluated using:

- CPA — Closest Point of Approach
- TCPA — Time to Closest Point of Approach

For the relative position:

```text
p_rel =
p_target - p_own
```

and relative velocity:

```text
v_rel =
v_target - v_own
```

TCPA is calculated as:

```text
TCPA =
-(p_rel · v_rel)
/
||v_rel||²
```

The relative position at the predicted closest approach is:

```text
p_CPA =
p_rel
+
TCPA v_rel
```

and the corresponding distance is:

```text
DCPA =
||p_CPA||
```

---

# 6. Collision-Risk Assessment

A vessel encounter is considered potentially relevant when:

```text
TCPA > 0
```

and the predicted closest distance is below a configurable safety threshold.

The current generic implementation uses a condition of the form:

```text
0 < TCPA < T_limit
```

and:

```text
DCPA < D_safe
```

This allows the system to distinguish between distant traffic and vessels that may require avoidance behavior.

---

# 7. Relative Bearing

Encounter geometry is evaluated using relative bearing.

The line-of-sight bearing from own vessel to target vessel is:

```text
β =
atan2(
y_target - y_own,
x_target - x_own
)
```

Relative bearing is then:

```text
β_rel =
wrap(
β - ψ_own
)
```

This allows surrounding traffic to be interpreted relative to the own vessel heading.

---

# 8. Encounter Classification

The project implements simplified geometric encounter categories:

```text
SAFE
HEAD_ON
CROSSING_GIVE_WAY
CROSSING_STAND_ON
OVERTAKING
```

The classifier considers:

- Relative bearing
- Relative heading
- Approximate reciprocal-heading geometry

This is intentionally a simplified research representation rather than a formal legal COLREG engine.

---

# 9. Head-On Encounter

A head-on encounter is approximately identified when:

```text
Target Relative Bearing ≈ 0°
```

and the headings are nearly reciprocal:

```text
|Δψ| ≈ 180°
```

Conceptually:

```text
Own Vessel  →      ←  Target Vessel
```

The local avoidance layer generates a temporary waypoint biased toward the own vessel's starboard side.

---

# 10. Crossing Encounter

Crossing geometry is classified using relative bearing.

A simplified distinction is made between:

```text
CROSSING_GIVE_WAY
```

and:

```text
CROSSING_STAND_ON
```

depending on the relative location of the target vessel.

This provides a rule-inspired structure for testing autonomous navigation behavior.

---

# 11. Overtaking Encounter

The framework also detects approximate overtaking geometry.

The condition is based on:

```text
Target located approximately astern
+
Similar vessel headings
```

The current implementation uses this primarily as a simulation encounter label.

---

# 12. Local Avoidance Waypoint

If the collision-risk conditions are met:

```text
Low DCPA
+
Positive TCPA
+
Relevant Encounter
```

the system generates a temporary local waypoint.

For give-way-style situations, the waypoint is biased in the own-vessel starboard direction.

Conceptually:

```text
Nominal Global Path
        ↓
Risk Detected
        ↓
Temporary Avoidance Waypoint
        ↓
Avoidance Maneuver
        ↓
Return Toward Global Route
```

---

# 13. Starboard-Biased Avoidance

The vessel heading defines two local directions:

```text
Forward
```

and:

```text
Starboard
```

A generic local avoidance waypoint can be generated as:

```text
p_local =
p_own
+
k1 p_forward
+
k2 p_starboard
```

where:

```text
k1 > 0
k2 > 0
```

This creates a temporary reference ahead and to starboard of the current vessel.

---

# 14. LOS-Style Tracking

The vessel tracks either:

```text
Global Path Waypoint
```

or:

```text
Temporary Avoidance Waypoint
```

using a LOS-style heading controller.

Desired heading:

```text
ψ_d =
atan2(
y_ref - y,
x_ref - x
)
```

Heading error:

```text
e_ψ =
wrap(
ψ_d - ψ
)
```

Yaw-rate command:

```text
r_cmd =
sat(
Kψ e_ψ
)
```

---

# 15. Closed-Loop Navigation

The complete control loop becomes:

```text
Reference Waypoint
        ↓
Desired Heading
        ↓
Heading Error
        ↓
Yaw-Rate Command
        ↓
USV Kinematics
        ↓
Updated Position / Heading
        ↓
New Encounter Assessment
```

This process repeats throughout the simulation.

---

# 16. Static Obstacles

The simulation includes a generic occupancy-grid environment containing static rectangular obstacles.

The A* planner generates a global path around these obstacles.

Dynamic vessels are then handled separately by the local encounter-management layer.

This creates a two-level structure:

```text
Static Obstacles
      ↓
Global Planning

Dynamic Vessels
      ↓
Local Avoidance
```

---

# 17. Multi-Vessel Scenario

The demonstration includes several independently moving target vessels.

Each target has its own:

```text
Position
Speed
Heading
```

The own vessel evaluates every target separately.

For each target:

```text
Relative Position
      ↓
Relative Velocity
      ↓
CPA / TCPA
      ↓
Encounter Type
      ↓
Risk Assessment
```

---

# 18. Performance Metrics

The simulation reports several metrics.

## Goal Reached

```text
Goal Reached =
True / False
```

---

## Minimum Separation

```text
D_min =
min distance(
own vessel,
other vessels
)
```

This measures the closest observed vessel-to-vessel separation.

---

## Travel Time

```text
T =
N_steps × Δt
```

---

## Replanning Count

The number of local avoidance interventions is recorded.

```text
N_replan
```

This indicates how frequently dynamic traffic modifies the nominal path-following process.

---

## High-Risk Encounter Count

A high-risk encounter is counted when:

```text
0 < TCPA < threshold
```

and:

```text
DCPA < safety threshold
```

---

# 19. Approximate Rule-Compliance Metric

The framework contains a lightweight approximate compliance score.

It is used only as a simulation diagnostic.

It should not be interpreted as formal COLREG compliance certification.

A future implementation could replace this with a much more rigorous rule-evaluation framework.

---

# 20. Repository Structure

```text
colreg_aware_dynamic_usv_navigation/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── docs/
│   └── scope.md
│
├── src/
│   ├── __init__.py
│   ├── models.py
│   ├── cpa_tcpa.py
│   ├── encounter.py
│   ├── global_planner.py
│   ├── local_replanner.py
│   ├── controller.py
│   └── simulation.py
│
├── examples/
│   └── run_demo.py
│
└── results/
```

---

# 21. Module Description

| Module | Purpose |
|---|---|
| `models.py` | Generic vessel-state and kinematic propagation |
| `cpa_tcpa.py` | CPA / TCPA calculations |
| `encounter.py` | Simplified encounter classification |
| `global_planner.py` | A* static global planning |
| `local_replanner.py` | Dynamic avoidance waypoint generation |
| `controller.py` | LOS-style heading tracking |
| `simulation.py` | Integrated navigation simulation |
| `run_demo.py` | Multi-vessel demonstration |

---

# 22. Installation

Clone the repository:

```bash
git clone <repository-url>
cd colreg-aware-dynamic-usv-navigation
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
NumPy
Matplotlib
```

---

# 23. Running the Simulation

Run:

```bash
python examples/run_demo.py
```

The demonstration simulates:

```text
Own USV
+
Static Obstacles
+
Multiple Moving Vessels
+
CPA/TCPA Assessment
+
Encounter Classification
+
Dynamic Avoidance
+
Path Tracking
```

---

# 24. Recommended Result Figures

After running the simulation, useful figures include:

```text
results/
├── global_path.png
├── dynamic_encounters.png
├── cpa_tcpa_history.png
├── minimum_separation.png
├── avoidance_maneuver.png
└── encounter_classification.png
```

Only actual simulation outputs should be published.

---

# 25. Recommended Result Table

A final evaluation can use:

| Scenario | Goal Reached | Min. Separation | Travel Time | Replans | High-Risk Encounters |
|---|---:|---:|---:|---:|---:|
| Head-On | measured | measured | measured | measured | measured |
| Crossing | measured | measured | measured | measured | measured |
| Overtaking | measured | measured | measured | measured | measured |
| Multi-Vessel | measured | measured | measured | measured | measured |

No assumed numerical results are required.

---

# 26. Technologies

- Python
- NumPy
- Matplotlib
- A* Path Planning
- CPA / TCPA Analysis
- Collision-Risk Assessment
- Autonomous Navigation
- Marine Robotics
- LOS Guidance
- Dynamic Replanning
- Kinematic Simulation

---

# 27. Research Areas

The project is related to:

- Autonomous Surface Vessels
- Marine Robotics
- Autonomous Navigation
- Path Planning
- Collision Avoidance
- Maritime Traffic Interaction
- Guidance and Control
- Motion Planning
- Dynamic Obstacle Avoidance
- Intelligent Transportation Systems

---

# 28. Current Scope

The current implementation includes:

- Static occupancy-grid obstacles
- Global A* planning
- Multiple moving vessels
- Relative velocity calculation
- CPA calculation
- TCPA calculation
- Relative-bearing analysis
- Simplified encounter classification
- Local starboard-biased avoidance
- LOS-style path tracking
- Kinematic USV simulation
- Safety metrics

---

# 29. Current Limitations

The current implementation does not include:

- Formal COLREG legal interpretation
- Certified collision-avoidance logic
- AIS message decoding
- Radar target tracking
- Electronic navigational charts
- Vessel-domain modelling
- Full 3-DoF vessel hydrodynamics
- Wind/current/wave disturbances
- MPC or NMPC collision avoidance
- Multi-object tracking
- Sensor uncertainty
- Real vessel integration
- Hardware-in-the-loop testing
- Sea-trial validation

Therefore, the project should be interpreted as a **COLREG-inspired autonomous navigation simulation framework**, not as a certified maritime navigation or collision-avoidance system.

---

# 30. Future Extensions

## Dynamic Prediction

The current constant-velocity prediction could be extended using:

```text
Kalman Filter
      ↓
Target State Estimate
      ↓
Trajectory Prediction
      ↓
CPA / TCPA
```

---

## Sensor Fusion

Future work could integrate:

```text
AIS
+
Radar
+
Camera
+
Kalman Filter
```

into a unified vessel-tracking architecture.

---

## 3-DoF USV Dynamics

The current kinematic vehicle model could be replaced with:

```text
M ν_dot
+
C(ν)ν
+
D(ν)ν
=
τ
+
τ_env
```

with:

```text
ν =
[u,v,r]
```

representing surge, sway, and yaw-rate dynamics.

---

## MPC Collision Avoidance

Local waypoint generation could be replaced with constrained MPC:

```text
Predicted Vessel Trajectories
            ↓
Collision Constraints
            ↓
MPC Optimization
            ↓
Safe Control Sequence
```

---

## Environmental Disturbances

Future simulations could include:

```text
Current
+
Wind
+
Wave Disturbances
```

to evaluate robustness of the navigation system.

---

## Multi-Agent Navigation

The framework could also be extended toward:

```text
Multiple Autonomous USVs
        ↓
Decentralized Encounter Assessment
        ↓
Cooperative Collision Avoidance
        ↓
Distributed Planning
```

---

# 31. Public Implementation Notice

This repository contains a **generic and sanitized implementation for research and educational purposes**.

The public implementation intentionally excludes:

- Real operational vessel parameters
- Restricted maritime information
- Proprietary navigation algorithms
- Confidential sensor data
- Real navigation charts
- Platform-specific control parameters
- Operational mission logic

All maps, vessels, speeds, thresholds, and scenarios are generic simulation examples.

---

# 32. Status

**Research-oriented simulation framework / active development**

The current project demonstrates:

```text
Global Planning
      ↓
Dynamic Traffic Assessment
      ↓
CPA / TCPA
      ↓
Encounter Classification
      ↓
Local Avoidance
      ↓
Path Tracking
      ↓
Safety Evaluation
```

The primary focus is on **dynamic maritime navigation, collision-risk assessment, autonomous path management, and USV guidance research**.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Marine Robotics
- Autonomous Surface Vessels
- Path Planning
- Guidance, Navigation and Control
- Collision Avoidance
- State Estimation
- Model Predictive Control
- Reinforcement Learning
- Multi-Agent Systems
