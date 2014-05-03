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

if sys.platform != 'darwin':
  import proto.node_detection_pb2
  import proto.util


class Particle(object):

  def __init__(self, fov_range, fov_angle, course_noise, displacement_noise):
    self._surface_range = random.random() * fov_range
    self._hor_angle = random.random() * fov_angle - fov_angle / 2
    self._course_noise = course_noise
    self._displacement_noise = displacement_noise

  def move(self, heading, last_heading, course, sensor_displacement):
    """Given sensor motion, move the relative location of the particle.
    """
    # Add some Gaussian noise to the motion measurements.
    course += random.gauss(0.0, self._course_noise)
    sensor_displacement += random.gauss(0.0, self._displacement_noise)
    target_bearing = last_heading - self._hor_angle
    # In Cartesian coordinates, let (0, 0) represent the sensor's previous
    # position, (e1, n1) represent the sensor's current position, and (e2, n2)
    # represent the target's position.
    e1 = sensor_displacement * trig.sind(course)
    n1 = sensor_displacement * trig.cosd(course)
    e2 = self._surface_range * trig.sind(self._hor_angle)
    n2 = self._surface_range * trig.cosd(self._hor_angle)
    predicted_target_bearing = trig.atan2d(e2 - e1, n2 - n1)
    # The following check converts atan2d()'s output in the range of
    # [-180.0, +180.0] to the expected range for a target bearing, i.e.
    # [0.0, +360.0).
    if predicted_target_bearing < 0.0:
      predicted_target_bearing += 360.0
    self._hor_angle = heading - predicted_target_bearing
    self._surface_range = math.sqrt((e2 - e1)**2 + (n2 - n1)**2)

  def __repr__(self):
    return "[range={}, hor_angle={}]".format(
      self._surface_range, self._hor_angle)


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
  parser.add_argument("--course-noise", type=float, default=0.05,
                      help="sensor course movement noise (default 0.05m)")
  parser.add_argument("--displacement-noise", type=float, default=0.05,
                      help="sensor displacement movement noise (default 0.05m)")
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
    particles.append(Particle(args.range, args.hor_angle, args.course_noise,
                              args.displacement_noise))

  last_heading = None
  last_lat = None
  last_lon = None
  for feature_data in get_feature_datas(args.directory, data_format):
    # Cache the current sensor heading and position variables.
    curr_heading = feature_data.heading.heading
    curr_lat = feature_data.position.lat
    curr_lon = feature_data.position.lon
    print curr_heading, curr_lat, curr_lon

    # If the sensor has moved, update the relative locations of the particles.
    # TODO(Rich): Pretty sure we want lat, lon, and heading noise instead of
    #             course and displacement noise.
    if last_heading is not None:
      course = trig.get_course(last_lat, last_lon, curr_lat, curr_lon)
      displacement = trig.get_distance(last_lat, last_lon, curr_lat, curr_lon)
      for particle in particles:
        particle.move(curr_heading, last_heading, course, displacement)

    # TODO(Rich): Measurement step.
    for cluster in feature_data.filtered_mobile_clusters:
      pass

    # Save the heading and position information so we can calculate our course
    # and sensor displacement at the next ping.
    last_heading = curr_heading
    last_lat = curr_lat
    last_lon = curr_lon


if __name__ == "__main__":
  sys.exit(main())
