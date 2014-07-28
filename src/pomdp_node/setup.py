#!/usr/bin/python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['pomdp_node'],
    scripts=[],
    package_dir={'': 'src'}
)

setup(**d)
