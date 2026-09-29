#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "px4_wall_ceiling_control",
        "px4_wall_ceiling_control.adapters",
        "px4_wall_ceiling_control.domain",
        "px4_wall_ceiling_control.legacy",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
