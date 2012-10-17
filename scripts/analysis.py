#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_diagnostic_controller')
from pr2_diagnostic_controller.msg import *
import os
from yaml import load
import matplotlib.pyplot as plt 
import numpy
from scipy.stats import scoreatpercentile
import sys

if len(sys.argv) == 2:
  fn = sys.argv[1]
else:
  fn = 'results.yaml'
  print "using default file", fn

stream = file(fn, 'r')
samples = load(stream)
velocity = []
encoder_position = []
ppar = 1200.0
delay = 1 
filt1 = numpy.array([-1,1])

def cut(d, begin, end):
  return d[begin:end]

def get_acceleration(velocity):
  acceleration = numpy.convolve(filt1, velocity)
  acceleration = acceleration[delay:len(velocity)+delay]
  acceleration = abs(acceleration)
  acceleration = acceleration / acceleration.max()
  return acceleration

def check_for_spikes(spikes):
  sorted_data = numpy.sort(spikes,kind='mergesort')
  iq_range = scoreatpercentile(sorted_data,75) - scoreatpercentile(sorted_data,25)
  outlier_limit = iq_range * 3 + scoreatpercentile(sorted_data,75)

  if (outlier_limit == 0):
    print "using mean multiplier as outlier limit"
    outlier_limit = 5 * spikes.mean()

  print "mean",spikes.mean()
  print "outlier limit",outlier_limit

  outliers = [val for val in spikes if val > outlier_limit]

  print "outliers in filtered data", outliers
  if len(outliers) >= 2:
    print "Encoder could be spoilt"

def check_for_unplugged(velocity):
  zero_velocities = [zeros for zeros in velocity if zeros == 0]
  zero_encoder_value = 0
  zero_encoder_positions = []

  for sample in encoder_position:
    if(sample == zero_encoder_value):
      zero_encoder_positions.append(sample)
    zero_encoder_value = sample

  if (len(zero_velocities) > 250 and (len(zero_encoder_positions) == len(zero_velocities))):
    print "Encoder could unplugged"

for s in samples.sample_buffer:
  velocity.append(s.velocity)
  encoder_position.append(s.encoder_position)

velocity = numpy.array(velocity)
encoder_position = numpy.array(encoder_position)

acceleration = get_acceleration(velocity)

spikes = acceleration * velocity

check_for_spikes(spikes)
check_for_unplugged(velocity)


plt.figure()
plt.subplot(311)
plt.plot(velocity,label='velocity')
plt.legend()

plt.subplot(312)
plt.plot(spikes,'*-', label='acceleration * velocity')
plt.legend()

plt.subplot(313)
plt.plot(acceleration,'g*-', label='acceleration')
plt.legend()

if False:
  plt.figure()
  plt.subplot(111)
  plt.plot(encoder_velocity, '.', label='encoder velocity')

plt.show()
