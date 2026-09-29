#!/usr/bin/env python3

# Legacy generic distance-control entry point.
from px4_wall_ceiling_control.legacy.behavior_fsm import run_behavior_fsm


if __name__ == "__main__":
    run_behavior_fsm("wall")
