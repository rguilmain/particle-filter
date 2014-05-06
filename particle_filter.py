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

  def __init__(self, fov_range, fov_angle, gps_noise, compass_noise,
               range_resolution, angular_resolution):
    # Initialize at a random location in the field of view.
    self.surface_range = random.random() * fov_range
    self.hor_angle = random.random() * fov_angle - fov_angle / 2

    # Cache our movement and measurement noise variables.
    self.gps_noise = gps_noise
    self.compass_noise = compass_noise
    self.range_resolution = range_resolution
    self.angular_resolution = angular_resolution

  def move(self, last_position, curr_position):
    """Given sensor motion, move the relative location of the particle.
    """
    # Add some Gaussian noise to the motion measurements.
    lat_deg_len = geo.lat_degree_len(curr_position.lat)
    lon_deg_len = geo.lon_degree_len(curr_position.lat)
    lat1 = last_position.lat + random.gauss(0.0, self.gps_noise) / lat_deg_len
    lon1 = last_position.lon + random.gauss(0.0, self.gps_noise) / lon_deg_len
    last_heading = last_position.heading + random.gauss(0.0, self.compass_noise)
    lat2 = curr_position.lat + random.gauss(0.0, self.gps_noise) / lat_deg_len
    lon2 = curr_position.lon + random.gauss(0.0, self.gps_noise) / lon_deg_len
    curr_heading = curr_position.heading + random.gauss(0.0, self.compass_noise)

    # Calculate how the sensor moved.
    course = geo.course(lat1, lon1, lat2, lon2)
    sensor_displacement = geo.distance(lat1, lon1, lat2, lon2)

    # Calculate how to move the particle relative to the sensor.
    target_bearing = last_heading - self.hor_angle
    # In Cartesian coordinates, let (0, 0) represent the sensor's previous
    # position, (e1, n1) represent the sensor's current position, and (e2, n2)
    # represent the target's position.
    e1 = sensor_displacement * trig.sind(course)
    n1 = sensor_displacement * trig.cosd(course)
    e2 = self.surface_range * trig.sind(target_bearing)
    n2 = self.surface_range * trig.cosd(target_bearing)
    predicted_target_bearing = trig.atan2d(e2 - e1, n2 - n1) % 360.0
    self.hor_angle = curr_heading - predicted_target_bearing
    self.surface_range = math.sqrt((e2 - e1)**2 + (n2 - n1)**2)

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


def get_particle_positions(particles):
  """Return the x and y coordinates of the particles as two parallel lists.
  """
  xs = [p.surface_range * trig.sind(p.hor_angle) for p in particles]
  ys = [p.surface_range * trig.cosd(p.hor_angle) for p in particles]
  return xs, ys


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
  parser.add_argument("--gps-noise", type=float, default=10.0,
                      help="sensor lat and lon motion noise (default 10.0m)")
  parser.add_argument("--compass-noise", type=float, default=1.0,
                      help="sensor heading noise (default 1.0 degrees)")
  parser.add_argument("--range-resolution", type=float, default=3.0,
                      help="range resolution of the sensor (default 3.0m)")
  parser.add_argument("--angular-resolution", type=float, default=1.5,
                      help="angular sensor resolution (default 1.5 degrees)")
  parser.add_argument("-t", "--timeout", type=float, default=0.0,
                      help="seconds to wait between pings (default 0.0s)")
  parser.add_argument("-r", "--fov-range", type=float, default=500.0,
                      help="range of the field of view (default 500.0m)")
  parser.add_argument("-a", "--fov-hor-angle", type=float, default=90.0,
                      help="number of degrees in field of view (default 90.0)")
  parser.add_argument("-m", "--show-measurements", action="store_true",
                      help="show measurements in the particle plot")
  parser.add_argument("--save-figure", action="store_true",
                      help="save figure frames as images")
  args = parser.parse_args()

  if not os.path.isdir(args.directory):
    sys.stdout.write("Could not find the {} directory!".format(args.directory))
    return

  # This assumes the input directory is of the form *-<data_format> where
  # <data_format> is either 'proto' or 'json'.
  data_format = args.directory.split('-')[-1]

  # Initialize our particles.
  particles = []
  for i in range(args.num_particles):
    particles.append(Particle(args.fov_range, args.fov_hor_angle,
                              args.gps_noise, args.compass_noise,
                              args.range_resolution, args.angular_resolution))

  # Initialize plot.
  # Notice (Nabin): We are plotting in interactive mode. So don't move mouse
  # once plotting starts. You can do 'Ctrl-C' to break and exit.
  plt.ion()
  fig = plt.figure()
  particle_plot = fig.add_subplot(111)
  if args.save_figure and not os.path.isdir("images"):
    os.makedirs("images")
    

  # Collection of filtered position computed from posterior particles.
  # Todo (Nabin): Save filtered lat/lons to file and compare with
  # diver gps data (probably in separate script).
  filtered_xs, filtered_ys = [], []
  filtered_lats, filtered_lons = [], []

  # Pump.
  last_position = None
  for i, feature_data in enumerate(
      get_feature_datas(args.directory, data_format)):
    current_position = SensorPosition(feature_data.position.lat,
                                      feature_data.position.lon,
                                      feature_data.heading.heading)
    if last_position is not None:
      for particle in particles:
        particle.move(last_position, current_position)
    measurements = get_measurements(feature_data)
    new_weights = get_weights(particles, measurements)
    if new_weights:
      weights = new_weights
    particles = resample_particles(particles, weights)
    particle_xs, particle_ys = get_particle_positions(particles)
    filtered_x, filtered_y = utils.extract_position_from_particles(
      particle_xs, particle_ys)
    filtered_lat, filtered_lon = geo.add_offsets_to_latlons(
      current_position.lat, current_position.lon, filtered_x, filtered_y)
    filtered_xs.append(filtered_x)
    filtered_ys.append(filtered_y)
    filtered_lats.append(filtered_lat)
    filtered_lons.append(filtered_lon)
    last_position = current_position

    utils.plot_data(particle_xs, particle_ys, filtered_xs, filtered_ys,
                    measurements, i, args.show_measurements, particle_plot)
    plt.draw()
    if args.save_figure:
      plt.savefig("images//%03d.png" % i, format='png')
    time.sleep(args.timeout)


if __name__ == "__main__":
  sys.exit(main())
