# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Localizes divers in the node field of view by use of a particle filter.
"""

import argparse
import json
import math
import os
import random
import sys

import trig
import utils

# Notice (Nabin): This assumes you have protobuf when in Windows. If that is
# not the case modify it and make sure it does not break in Mac (darwin).
if sys.platform != 'darwin':
  import proto.node_detection_pb2
  import proto.util


class SensorPosition(object):

  def __init__(self, lat, lon, heading):
    self.lat = lat
    self.lon = lon
    self.heading = heading

  def __repr__(self):
    return "[lat={}, lon={}, heading={}]".format(
      self.lat, self.lon, self.heading)


class Particle(object):

  def __init__(self, fov_range, fov_angle, gps_noise, compass_noise):
    self.surface_range = random.random() * fov_range
    self.hor_angle = random.random() * fov_angle - fov_angle / 2
    self.gps_noise = gps_noise
    self.compass_noise = compass_noise

  def move(self, last_position, curr_position):
    """Given sensor motion, move the relative location of the particle.
    """
    # Add some Gaussian noise to the motion measurements.
    last_lat = last_position.lat + random.gauss(0.0, self.gps_noise)
    last_lon = last_position.lon + random.gauss(0.0, self.gps_noise)
    last_heading = last_position.heading + random.gauss(0.0, self.compass_noise)
    curr_lat = curr_position.lat + random.gauss(0.0, self.gps_noise)
    curr_lon = curr_position.lon + random.gauss(0.0, self.gps_noise)
    curr_heading = curr_position.heading + random.gauss(0.0, self.compass_noise)

    # Calculate how the sensor moved.
    course = trig.course(last_lat, last_lon, curr_lat, curr_lon)
    sensor_displacement = trig.distance(last_lat, last_lon, curr_lat, curr_lon)

    # Calculate how to move the particle relative to the sensor.
    target_bearing = last_heading - self.hor_angle
    # In Cartesian coordinates, let (0, 0) represent the sensor's previous
    # position, (e1, n1) represent the sensor's current position, and (e2, n2)
    # represent the target's position.
    e1 = sensor_displacement * trig.sind(course)
    n1 = sensor_displacement * trig.cosd(course)
    e2 = self.surface_range * trig.sind(target_bearing)
    n2 = self.surface_range * trig.cosd(target_bearing)
    predicted_target_bearing = trig.atan2d(e2 - e1, n2 - n1)
    # The following check converts atan2d()'s output in the range of
    # [-180.0, +180.0] to the expected range for a target bearing, i.e.
    # [0.0, +360.0).
    if predicted_target_bearing < 0.0:
      predicted_target_bearing += 360.0
    self.hor_angle = curr_heading - predicted_target_bearing
    self.surface_range = math.sqrt((e2 - e1)**2 + (n2 - n1)**2)

  def __repr__(self):
    return "[range={}, hor_angle={}]".format(
      self.surface_range, self.hor_angle)


def get_feature_datas(directory, data_format='proto'):
  for f in os.listdir(directory):
    if data_format == 'proto':
      yield proto.util.read(filename=(directory + "/" + f))
    else:
      yield utils.Struct(json.loads(open(os.path.join(directory, f)).read()))


def main(argv=None):
  if argv is not None:
    sys.argv = argv

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("-d", "--directory", default="featuredatas-proto",
                      help="feature data location (default featuredatas-proto)")
  parser.add_argument("-n", "--num-particles", type=int, default=1000,
                      help="number of particles to simulate (default 1000)")
  parser.add_argument("--gps-noise", type=float, default=0.05,
                      help="sensor lat and lon motion noise (default 0.05m)")
  parser.add_argument("--compass-noise", type=float, default=1.5,
                      help="sensor heading noise (default 1.5 degrees)")
  parser.add_argument("-r", "--range", type=float, default=500.0,
                      help="range of the field of view (default 500.0m)")
  parser.add_argument("-a", "--hor-angle", type=float, default=90.0,
                      help="number of degrees in field of view (default 90.0)")
  args = parser.parse_args()

  if not os.path.isdir(args.directory):
    sys.stdout.write("Could not find the {} directory!".format(args.directory))
    return

  # This assumes the input directory is of the form *-<data_format> where
  # <data_format> is either 'proto' or 'json'.
  data_format = args.directory.split('-')[-1]

  particles = []
  for i in range(args.num_particles):
    particles.append(Particle(args.range, args.hor_angle, args.gps_noise,
                              args.compass_noise))

  last_position = None
  for feature_data in get_feature_datas(args.directory, data_format):
    # Cache the current sensor heading and position variables.
    current_position = SensorPosition(feature_data.position.lat,
                                      feature_data.position.lon,
                                      feature_data.heading.heading)

    # If the sensor has moved, update the relative locations of the particles.
    if last_position is not None:
      for particle in particles:
        particle.move(last_position, current_position)

    # TODO(Rich): Measurement step.
    for cluster in feature_data.filtered_mobile_clusters:
      pass

    # Save the heading and position information so we can calculate our course
    # and sensor displacement at the next ping.
    last_position = current_position

if __name__ == "__main__":
  sys.exit(main())
