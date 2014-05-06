# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Utilities
"""
import numpy

import matplotlib

import trig


# Convert a dictionary to a class
# Reference: http://kiennt.com/blog/2012/06/14/python-object-and-dictionary-convertion.html
class Struct(object):

  def __init__(self, adict):
    """Convert a dictionary to a class. Can handle dict of dicts that
    might have list of dicts.
    """
    self.__dict__.update(adict)
    for k, v in adict.items():
      if isinstance(v, dict):
        self.__dict__[k] = Struct(v)
      if isinstance(v, list):
        self.__dict__[k] = []
        for e in v:
          self.__dict__[k].append(Struct(e))


def extract_position_from_particles(particle_xs, particle_ys):
  """Compute the average of particles as filtered diver state (position).
  """
  assert(len(particle_xs) == len(particle_ys))
  x = 0.0
  y = 0.0
  for px, py in zip(particle_xs, particle_ys):
    x += px
    y += py
  return x / len(particle_xs), y / len(particle_xs)


def get_measurement_positions(measurements):
  """Return the x and y coordinates of the measurements as two parallel lists.
  """
  xs = [-m.surface_range * trig.sind(m.hor_angle) for m in measurements]
  ys = [m.surface_range * trig.cosd(m.hor_angle) for m in measurements]
  return xs, ys


def draw_fov(ax):
  """Draw sonar FoV. Don't mind the magic numbers for now.
  """
  p_left = 500.0 * numpy.array(
    [trig.sind(-45.0), trig.cosd(45.0)])
  p_right = 500.0 * numpy.array(
    [trig.cosd(45.0), trig.sind(45.0)])
  line_left = matplotlib.lines.Line2D(
    [0, p_left[0]], [0, p_left[1]], color='b')
  line_right = matplotlib.lines.Line2D(
    [0, p_right[0]], [0, p_right[1]], color='b')
  arc = matplotlib.patches.Arc((0, 0), 2*500, 2*500,
                               theta1=45, theta2=135, color='b')
  ax.add_line(line_left)
  ax.add_line(line_right)
  ax.add_patch(arc)


def plot_data(particle_xs, particle_ys, filtered_xs, filtered_ys,
              measurements, i, hide_measurements, particle_plot):
  """Plot particles, estimated diver locations and measurements.
  """
  particle_plot.hold(False)
  particle_plot.grid(True)
  particle_plot.plot(particle_xs, particle_ys, '.')
  particle_plot.hold(True)
  particle_plot.plot(filtered_xs, filtered_ys, 'co:', markeredgecolor='none')
  if not hide_measurements:
    measurement_xs, measurement_ys = get_measurement_positions(
      measurements)
    particle_plot.plot(measurement_xs, measurement_ys, 'ro')
  particle_plot.axis([-400, 400, -50, 550])
  draw_fov(particle_plot)
  particle_plot.set_xlabel("meters")
  particle_plot.set_ylabel("meters")
  particle_plot.set_title("Step: %03d" % i)


def date_to_sec(time_struct):
  """Returns number of seconds equivalent to the HHMMSS.uuu
  of a date, discards YYMMDD.
  """
  return (time_struct.hour * 3600.0 +
          time_struct.minute * 60.0 +
          time_struct.second * 1.0 +
          time_struct.millisecond * 1e-3)
