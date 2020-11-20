#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'control', 
        'utility', 
        'connections',
        'connections.bosch_data',
        'connections.command',
        'connections.diagnostics',
        'connections.machine_info',
        'connections.relay_data',
        'connections.server_info',
        'connections.task'
        ],
    package_dir={'': 'src'},
)

setup(**setup_args)
