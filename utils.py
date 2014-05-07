# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Utilities
"""
import numpy

import matplotlib

import trig
import geo


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


def compute_sensor_movement(last_position, current_position):
  # Calculate how the sensor moved.
  course = geo.course(
    last_position.lat, last_position.lon,
    current_position.lat, current_position.lon)
  displacement = geo.distance(
    last_position.lat, last_position.lon,
    current_position.lat, current_position.lon)
  dx = displacement * trig.sind(course)
  dy = displacement * trig.cosd(course)
  return dx, dy


def get_particle_positions(particles, acc_dx, acc_dy, heading):
  """Return the x and y coordinates of the particles as two parallel lists.
  """
  xs = [-p.surface_range * trig.sind(p.hor_angle) for p in particles]
  ys = [p.surface_range * trig.cosd(p.hor_angle) for p in particles]

  xs_global, ys_global = [], []
  for x, y in zip(xs, ys):
    xg = x * trig.cosd(heading) + y * trig.sind(heading) + acc_dx
    yg = -x * trig.sind(heading) + y * trig.cosd(heading) + acc_dy
    xs_global.append(xg)
    ys_global.append(yg)
  return xs_global, ys_global


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


def get_measurement_positions(measurements, acc_dx, acc_dy, heading):
  """Return the x and y coordinates of the measurements as two parallel lists.
  """
  xs = [-m.surface_range * trig.sind(m.hor_angle) for m in measurements]
  ys = [m.surface_range * trig.cosd(m.hor_angle) for m in measurements]
  xs_global, ys_global = [], []
  for x, y in zip(xs, ys):
    xg = x * trig.cosd(heading) + y * trig.sind(heading) + acc_dx
    yg = -x * trig.sind(heading) + y * trig.cosd(heading) + acc_dy
    xs_global.append(xg)
    ys_global.append(yg)
  return xs_global, ys_global


def draw_fov(ax, acc_dx, acc_dy, heading, c='b'):
  """Draw sonar FoV. Don't mind the magic numbers for now.
  """
  look_dir = heading
  ang_left = 90.0 - look_dir + 45
  ang_right = 90.0 - look_dir - 45
  rad_ang_left = ang_left * numpy.pi / 180
  rad_ang_right = ang_right * numpy.pi / 180
  p_start = numpy.array([acc_dx, acc_dy])
  max_range = 500.0

  p_left = (p_start + max_range * numpy.array(
    [numpy.cos(rad_ang_left), numpy.sin(rad_ang_left)]))
  p_right = (p_start + max_range * numpy.array(
    [numpy.cos(rad_ang_right), numpy.sin(rad_ang_right)]))
  line_left = matplotlib.lines.Line2D([p_start[0], p_left[0]],
                                      [p_start[1], p_left[1]], color=c)
  line_right = matplotlib.lines.Line2D([p_start[0], p_right[0]],
                                       [p_start[1], p_right[1]], color=c)
  arc = matplotlib.patches.Arc(tuple(p_start), 2*max_range, 2*max_range,
                               theta1=ang_right, theta2=ang_left, color=c)

  ax.add_line(line_left)
  ax.add_line(line_right)
  ax.add_patch(arc)


def plot_data(particle_xs, particle_ys, filtered_xs, filtered_ys,
              measurements, i, hide_measurements, acc_dx, acc_dy,
              heading, particle_plot):
  """Plot particles, estimated diver locations and measurements.
  """
  particle_plot.hold(False)
  particle_plot.grid(True)
  particle_plot.plot(particle_xs, particle_ys, '.')
  particle_plot.hold(True)
  particle_plot.plot(filtered_xs, filtered_ys, 'co:', markeredgecolor='none')
  if not hide_measurements:
    measurement_xs, measurement_ys = get_measurement_positions(
      measurements, acc_dx, acc_dy, heading)
    particle_plot.plot(measurement_xs, measurement_ys, 'ro')
  particle_plot.axis([-200, 600, -400, 400])
  draw_fov(particle_plot, acc_dx, acc_dy, heading)
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
