#!/usr/bin/env python3

# Legacy generic distance-control entry point.
from px4_wall_ceiling_control.legacy.surface_controller import run_surface_controller


if __name__ == "__main__":
    run_surface_controller("wall")
