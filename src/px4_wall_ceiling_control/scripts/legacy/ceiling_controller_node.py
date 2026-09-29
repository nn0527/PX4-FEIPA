#!/usr/bin/env python3

# Legacy generic distance-controller entry point. The unified attachment
# system uses the dedicated decision nodes and attachment_output_mux_node.

from px4_wall_ceiling_control.legacy.surface_controller import run_surface_controller


if __name__ == "__main__":
    run_surface_controller("ceiling")
