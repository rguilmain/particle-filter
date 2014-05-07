# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Localizes divers in the node field of view by use of a particle filter.
"""

import argparse
import copy
import json
import math
import os
import random
import sys
import time

import matplotlib.pyplot as plt

import geo
import trig
import utils


class Particle(object):

  def __init__(self, fov_range, fov_angle, range_noise, angle_noise,
               range_resolution, angular_resolution):
    # Initialize at a random location in the field of view.
    self.surface_range = random.random() * fov_range
    self.hor_angle = random.random() * fov_angle - fov_angle / 2

    # Cache our movement and measurement noise variables.
    self.range_noise = range_noise
    self.angle_noise = angle_noise
    self.range_resolution = range_resolution
    self.angular_resolution = angular_resolution

  def move(self, last_position, curr_position, e1, n1):
    """Given sensor motion, move the relative location of the particle.
    """
    # Calculate how to move the particle relative to the sensor.
    target_bearing = last_position.heading - self.hor_angle

    # In Cartesian coordinates, let (0, 0) represent the sensor's previous
    # position, (e1, n1) represent the sensor's current position, and (e2, n2)
    # represent the target's position.
    e2 = self.surface_range * trig.sind(target_bearing)
    n2 = self.surface_range * trig.cosd(target_bearing)
    predicted_target_bearing = trig.atan2d(e2 - e1, n2 - n1) % 360.0

    self.hor_angle = (curr_position.heading - predicted_target_bearing +
                      random.gauss(0, self.angle_noise))
    self.surface_range = (math.sqrt((e2 - e1)**2 + (n2 - n1)**2) +
                          random.gauss(0, self.range_noise))

  def measurement_prob(self, measurement):
    """Return how likely it is that this particle came from a measured target.
    """
    return (self._gaussian(measurement.surface_range, self.range_resolution,
                           self.surface_range) *
            self._gaussian(measurement.hor_angle, self.angular_resolution,
                           self.hor_angle))

  def _gaussian(self, mu, sigma, x):
    """Return the probability of x for a 1D Gaussian.
    """
    return math.exp((-((mu - x)**2) / (sigma**2) / 2.0) /
                    math.sqrt(2.0 * math.pi * (sigma**2)))


class SensorPosition(object):

  def __init__(self, lat, lon, heading):
    self.lat = lat
    self.lon = lon
    self.heading = heading


class Measurement(object):

  def __init__(self, surface_range, hor_angle):
    self.surface_range = surface_range
    self.hor_angle = hor_angle


def get_first_feature_data(directory, data_format):
  """Read first feature data file to initialize position
  related functions.

  Valid data_format values are 'proto' and 'json'.
  """
  if data_format == 'proto':
    import proto.node_detection_pb2
    import proto.util
    return proto.util.read(os.path.join(directory, os.listdir(directory)[0]))
  elif data_format == 'json':
    return utils.Struct(json.loads(open(os.path.join(
      directory, os.listdir(directory)[0])).read()))
  else:
    raise NotImplementedError("Unknown data format {}.".format(data_format))


def get_feature_datas(directory, data_format):
  """Yield parsed feature data from the binary files in the given directory.

  Valid data_format values are 'proto' and 'json'.
  """
  if data_format == 'proto':
    import proto.node_detection_pb2
    import proto.util
    for f in os.listdir(directory):
      yield proto.util.read(os.path.join(directory, f))
  elif data_format == 'json':
    for f in os.listdir(directory):
      yield utils.Struct(json.loads(open(os.path.join(directory, f)).read()))
  else:
    raise NotImplementedError("Unknown data format {}.".format(data_format))


def get_measurements(feature_data):
  """Return the locations of the detected targets in the feature data.
  """
  measurements = []
  if hasattr(feature_data, 'filtered_mobile_clusters'):
    for cluster in feature_data.filtered_mobile_clusters:
      measurements.append(
        Measurement(cluster.centroid.range, cluster.centroid.hor_ang))
  return measurements


def get_weights(particles, measurements):
  """Return a list of likelihood weights parallel to the particles list.
  """
  weights = []
  if not measurements:
    return weights
  for particle in particles:
    weight = 0.0
    for measurement in measurements:
      weight = max(weight, particle.measurement_prob(measurement))
    weights.append(weight)
  return weights


def resample_particles(old_particles, weights):
  """Do a weighted resampling with replacement of our particles.
  """
  new_particles = []
  num_particles = len(old_particles)
  beta = 0.0
  max_weight = max(weights)
  index = int(random.random() * num_particles)
  for i in range(num_particles):
    beta += random.random() * 2.0 * max_weight
    while beta > weights[index]:
      beta -= weights[index]
      index = (index + 1) % num_particles
    new_particles.append(copy.deepcopy(old_particles[index]))
  return new_particles


def main(argv=None):
  if argv is not None:
    sys.argv = argv

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("-d", "--directory", default="featuredatas-json",
                      help="feature data location (default featuredatas-json)")
  parser.add_argument("-n", "--num-particles", type=int, default=1000,
                      help="number of particles to simulate (default 1000)")
  parser.add_argument("--range-noise", type=float, default=3.0,
                      help="range prediction noise (default 3.0m)")
  parser.add_argument("--angle-noise", type=float, default=3.0,
                      help="angle prediction noise (default 3.0 degrees)")
  parser.add_argument("--range-resolution", type=float, default=3.0,
                      help="range resolution of the sensor (default 3.0m)")
  parser.add_argument("--angular-resolution", type=float, default=1.5,
                      help="angular sensor resolution (default 1.5 degrees)")
  parser.add_argument("-t", "--timeout", type=float, default=0.01,
                      help="seconds to wait between pings (default 0.01s)")
  parser.add_argument("-r", "--fov-range", type=float, default=500.0,
                      help="range of the field of view (default 500.0m)")
  parser.add_argument("-a", "--fov-hor-angle", type=float, default=90.0,
                      help="number of degrees in field of view (default 90.0)")
  parser.add_argument("-m", "--hide-measurements", action="store_true",
                      help="hide measurements in the particle plot")
  parser.add_argument("--save-figure", action="store_true",
                      help="save figure frames as images")
  parser.add_argument("--write-latlon", action="store_true",
                      help=("Save diver position estimates as"
                            "lat/lon's to a file"))
  args = parser.parse_args()

  if not os.path.isdir(args.directory):
    sys.stdout.write("Could not find the {} directory!".format(args.directory))
    return

  if args.save_figure and not os.path.isdir("images"):
    os.makedirs("images")
  if args.write_latlon:
    latlon_file = open("diver_position_estimates.txt", 'w')

  # This assumes the input directory is of the form *-<data_format> where
  # <data_format> is either 'proto' or 'json'.
  data_format = args.directory.split('-')[-1]

  # Read first data file to get position for drawing initialization.
  feature_data = get_first_feature_data(args.directory, data_format)
  init_position = SensorPosition(feature_data.position.lat,
                                 feature_data.position.lon,
                                 feature_data.heading.heading)

  # Initialize plot.
  plt.ion()
  fig = plt.figure()
  particle_plot = fig.add_subplot(111)

  # Accumulated x- and y-dispalcement of sensor (ship) in meters re starting
  # position.
  acc_dx, acc_dy = 0.0, 0.0

  # Initialize our particles.
  particles = []
  for i in range(args.num_particles):
    particles.append(Particle(args.fov_range, args.fov_hor_angle,
                              args.range_noise, args.angle_noise,
                              args.range_resolution, args.angular_resolution))

  # Collection of filtered position computed from posterior particles.
  filtered_xs, filtered_ys = [], []

  particle_xs, particle_ys = utils.get_particle_positions(
    particles, acc_dx, acc_dy, init_position.heading)
  utils.plot_data(particle_xs, particle_ys, filtered_xs, filtered_ys, [], -1,
                  True, acc_dx, acc_dy, init_position.heading, particle_plot)
  plt.draw()

  # Pump.
  last_position = None
  for i, feature_data in enumerate(
      get_feature_datas(args.directory, data_format)):
    current_position = SensorPosition(feature_data.position.lat,
                                      feature_data.position.lon,
                                      feature_data.heading.heading)
    dx, dy = 0.0, 0.0
    if last_position is not None:
      dx, dy = utils.compute_sensor_movement(last_position, current_position)
      for particle in particles:
        particle.move(last_position, current_position, dx, dy)
    acc_dx += dx
    acc_dy += dy
    measurements = get_measurements(feature_data)
    new_weights = get_weights(particles, measurements)
    if new_weights:
      weights = new_weights
    particles = resample_particles(particles, weights)
    particle_xs, particle_ys = utils.get_particle_positions(
      particles, acc_dx, acc_dy, current_position.heading)
    filtered_x, filtered_y = utils.extract_position_from_particles(
      particle_xs, particle_ys)
    filtered_lat, filtered_lon = geo.add_offsets_to_latlons(
      current_position, filtered_x - acc_dx, filtered_y - acc_dy)
    filtered_xs.append(filtered_x)
    filtered_ys.append(filtered_y)
    last_position = current_position

    utils.plot_data(particle_xs, particle_ys, filtered_xs, filtered_ys,
                    measurements, i, args.hide_measurements, acc_dx, acc_dy,
                    current_position.heading, particle_plot)
    plt.draw()

    if args.save_figure:
      plt.savefig("images//%03d.png" % i, format='png')
    if args.write_latlon:
      latlon_file.write(
        "%d,%0.9f,%0.9f\n" % (utils.date_to_sec(
          feature_data.time), filtered_lat, filtered_lon))
      latlon_file.flush()
    time.sleep(args.timeout)
  if args.write_latlon:
    latlon_file.close()


if __name__ == "__main__":
  sys.exit(main())
