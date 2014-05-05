# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Provides some basic trigonometry functions.
"""

import math

EARTH_RADIUS = 6372797.560856


def sind(x):
  return math.sin(math.radians(x))


def cosd(x):
  return math.cos(math.radians(x))


def atan2d(y, x):
  return math.atan2(math.radians(y), math.radians(x))


def distance(lat1, lon1, lat2, lon2):
  """Return distance in meters between two lat/lon points.
  """
  lat_arc = math.radians(lat1 - lat2)
  lon_arc = math.radians(lon1 - lon2)
  latH = math.sin(lat_arc * 0.5)**2
  lonH = math.sin(lon_arc * 0.5)**2
  temp = cosd(lat1) * cosd(lat2);
  return 2.0 * EARTH_RADIUS * math.asin(math.sqrt(latH + temp * lonH))


def course(lat1, lon1, lat2, lon2):
  """Return course in degrees between two lat/lon points relative to grid north.
  """
  delta_y = sind(lon2 - lon1) * cosd(lat2)
  delta_x = (cosd(lat1) * sind(lat2) -
             sind(lat1) * cosd(lat2) * cosd(lon2 - lon1))
  return atan2d(delta_y, delta_x) % 360.0


def lat_degree_len(lat):
  """Return the length of a degree of latitude at a given latitude.

  From http://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude
  """
  return 111132.954 - 559.822 * cosd(2.0 * lat) + 1.175 * cosd(4.0 * lat)


def lon_degree_len(lat):
  """Return the length of a degree of longitude at a given latitude.

  From http://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
  """
  a = 6378137.0
  b = 6356752.3142
  e = math.sqrt((a**2 - b**2) / a**2)
  return ((math.pi * a * cosd(lat)) /
          180.0 * (1.0 - e**2 * sind(lat)**2)**(1.0/2.0))
