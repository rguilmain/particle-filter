# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Plot diver ground truth (gps) and particle filter estimaes for compariosn.
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


class GeoData(object):
  def __init__(self):
    self.times = []
    self.lats = []
    self.lons = []


def extract_geo_data(filename):
  with open(filename, 'r') as f:
    data = GeoData()
    for line in f.readlines():
      t, la, lo = line.split(',')
      data.times.append(t)
      data.lats.append(la)
      data.lons.append(lo)
  return data


def constrain_particle_data(particle_data, gps_start_time, gps_stop_time):
  data = GeoData()
  for i in range(len(particle_data.times)):
    if gps_start_time <= particle_data.times[i] <= gps_stop_time:
      data.times.append(particle_data.times[i])
      data.lats.append(particle_data.lats[i])
      data.lons.append(particle_data.lons[i])
  return data


def interpolate_gps_data(gps_data, times):
  lats = np.interp(times, gps_data.times, gps_data.lats)
  lons = np.interp(times, gps_data.times, gps_data.lons)
  data = GeoData()
  data.times = times
  data.lats = lats
  data.lons = lons
  return data


def plot_data(gps_data, particle_data):
  fig = plt.figure()
  particle_plot = fig.add_subplot(111)
  plt.plot(gps_data.lons, gps_data.lats, 'go', label="GPS")
  plt.hold(True)
  plt.plot(particle_data.lons, particle_data.lats, 'mo', label="Filter")
  plt.xlabel("lon (deg.)")
  plt.ylabel("lat (deg.)")
  plt.legend()
  plt.show()


def main(argv=None):
  if argv is not None:
    sys.argv = argv

  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--gps-file", default="diver_gps_locations.txt",
                      help="text file containing diver gps data")
  parser.add_argument("--particle-file", default="diver_position_estimates.txt",
                      help=("text file containing diver position estimates "
                            "(latitudes and longitudes)"))
  args = parser.parse_args()

  if not (os.path.isfile(args.gps_file) and
          os.path.isfile(args.particle_file)):
    print("Invalid input file(s)!")

  gps_data = extract_geo_data(args.gps_file)
  particle_data = extract_geo_data(args.particle_file)
  particle_data = constrain_particle_data(
      particle_data, gps_data.times[0], gps_data.times[-1])
  gps_data = interpolate_gps_data(gps_data, particle_data.times)
  plot_data(gps_data, particle_data)


if __name__ == "__main__":
  sys.exit(main())
