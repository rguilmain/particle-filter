# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Provides some basic geography utilities.
"""

import math

import trig

EARTH_RADIUS = 6372797.560856

SONAR_X_RE_GPS = 0.19
SONAR_Y_RE_GPS = 7.92

def distance(lat1, lon1, lat2, lon2):
  """Return distance in meters between two lat/lon points.
  """
  lat_arc = math.radians(lat1 - lat2)
  lon_arc = math.radians(lon1 - lon2)
  latH = math.sin(lat_arc * 0.5)**2
  lonH = math.sin(lon_arc * 0.5)**2
  temp = trig.cosd(lat1) * trig.cosd(lat2);
  return 2.0 * EARTH_RADIUS * math.asin(math.sqrt(latH + temp * lonH))


def course(lat1, lon1, lat2, lon2):
  """Return course in degrees between two lat/lon points relative to grid north.
  """
  delta_y = trig.sind(lon2 - lon1) * trig.cosd(lat2)
  delta_x = (trig.cosd(lat1) * trig.sind(lat2) -
             trig.sind(lat1) * trig.cosd(lat2) * trig.cosd(lon2 - lon1))
  return trig.atan2d(delta_y, delta_x) % 360.0


def lat_degree_len(lat):
  """Return the length of a degree of latitude at a given latitude.

  From http://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude
  """
  return (111132.954 - 559.822 * trig.cosd(2.0 * lat) +
          1.175 * trig.cosd(4.0 * lat))


def lon_degree_len(lat):
  """Return the length of a degree of longitude at a given latitude.

  From http://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
  """
  a = 6378137.0
  b = 6356752.3142
  e = math.sqrt((a**2 - b**2) / a**2)
  return ((math.pi * a * trig.cosd(lat)) /
          180.0 * (1.0 - e**2 * trig.sind(lat)**2)**(1.0/2.0))


def add_offsets_to_latlons(geo_position, x_particle, y_particle):
  """Return latitude and longitude after adding x (positive along east)
  and y (positive along north) offsets in meters to input latitude and
  longitude.

  From http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
  """
  lat, lon, hdg = geo_position.lat, geo_position.lon, geo_position.heading
  x_sonar = SONAR_X_RE_GPS * trig.cosd(hdg) + SONAR_Y_RE_GPS * trig.sind(hdg)
  y_sonar = -SONAR_X_RE_GPS * trig.sind(hdg) + SONAR_Y_RE_GPS * trig.cosd(hdg)
  x_meters = x_sonar + x_particle
  y_meters = y_sonar + y_particle
  lat_out = lat + math.degrees(y_meters / EARTH_RADIUS)
  lon_out = lon + math.degrees(x_meters / (EARTH_RADIUS * trig.cosd(lat)))
  return lat_out, lon_out
