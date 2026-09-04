# COLREG-Aware Dynamic USV Navigation

A research-oriented simulation framework for autonomous surface-vessel navigation
in dynamic multi-vessel environments.

The project combines:

- Static global path planning
- CPA / TCPA based encounter assessment
- Simplified COLREG-inspired encounter classification
- Dynamic obstacle prediction
- Local waypoint replanning
- LOS-style path tracking
- Quantitative safety and navigation metrics

The implementation is generic and educational. It is not a certified maritime
navigation system and does not replace formal COLREG interpretation, bridge
procedures, or vessel-specific navigation software.

## Architecture

```text
Static Map / Global Goal
        ↓
Global A* Path
        ↓
Dynamic Vessel States
        ↓
CPA / TCPA Assessment
        ↓
Encounter Classification
        ↓
Local Replanning
        ↓
LOS-Style Tracking
        ↓
USV Kinematic Model
        ↓
Safety / Performance Metrics
```

## Implemented encounter labels

- SAFE
- HEAD_ON
- CROSSING_GIVE_WAY
- CROSSING_STAND_ON
- OVERTAKING

These are simplified geometric classifications intended for simulation studies.

## Metrics

- Goal reached
- Minimum separation
- Path length
- Travel time
- Number of replans
- Number of high-risk encounters
- Approximate rule-compliance score

## Run

```bash
pip install -r requirements.txt
python examples/run_demo.py
```
