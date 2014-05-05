# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Provides some basic trigonometry functions.
"""

import math


def sind(x):
  return math.sin(math.radians(x))


def cosd(x):
  return math.cos(math.radians(x))


def atan2d(y, x):
  return math.degrees(math.atan2(math.radians(y), math.radians(x)))
