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
ppr = 1200.0
delay = 1 
filt1 = numpy.array([-1,1])

def get_acceleration(velocity):
  acceleration = numpy.convolve(filt1, velocity)
  acceleration = acceleration[delay:len(velocity)+delay]
  acceleration = abs(acceleration)
  print velocity 
  acceleration = acceleration / acceleration.max()
  return acceleration

def check_for_spikes(spikes):
#sorted_data = numpy.sort(spikes,kind='mergesort')
# iq_range = scoreatpercentile(sorted_data,75) - scoreatpercentile(sorted_data,25)
# outlier_limit = iq_range * 3 + scoreatpercentile(sorted_data,75)
  neg = [val for val in spikes if val < 0]
  pos = [val for val in spikes if val > 0]

  neg = numpy.sort(neg,kind='mergesort')
  pos = numpy.sort(pos,kind='mergesort')
  
  if (len(neg) > 1):
    iq_range_neg = scoreatpercentile(neg,-75) - scoreatpercentile(neg,-25)
#print "iqr75 is ", scoreatpercentile(neg,-75), " iqr25 is ", scoreatpercentile(neg,-25)
    outlier_limit_neg = iq_range_neg * 3 + scoreatpercentile(neg,-75)

  if (len(pos) > 1):
    iq_range_pos = scoreatpercentile(pos,75) - scoreatpercentile(pos,25)
#  print "iqr75 is ", scoreatpercentile(pos,75), " iqr25 is ", scoreatpercentile(pos,25)
    outlier_limit_pos = iq_range_pos * 3 + scoreatpercentile(pos,75)

#sorted_data = numpy.sort(spikes,kind='mergesort')
# iq_range = scoreatpercentile(sorted_data,75) - scoreatpercentile(sorted_data,25)
#  outlier_limit = iq_range * 3 + scoreatpercentile(sorted_data,75)

#print "mean",spikes.mean()
# print "outlier limit",outlier_limit
  print "length of neg is ",len(neg)," len of pos is ",len(pos)
 
  if (len(neg) > 1):
    print "neg mean",neg.mean()
    print "neg outlier limit",outlier_limit_neg
  
  if (len(pos) > 1):
    print "pos mean",pos.mean()
    print "pos outlier limit",outlier_limit_pos

  outliers_neg = [val for val in neg if val < outlier_limit_neg]
  outliers_pos = [val for val in pos if val > outlier_limit_pos]

  print "outliers in filtered data", outliers_neg, outliers_pos
#if len(outliers) >= 2:
  if (len(outliers_neg) + len(outliers_pos)) >= 2:
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

#a = numpy.array(encoder_position)
#encoder_velocity = a[1:] - a[:-1]
#encoder_velocity = (encoder_velocity / (2*numpy.pi)) * ppr
velocity = numpy.array(velocity)
encoder_position = numpy.array(encoder_position)
#encoder_position_mod = encoder_position % (2*numpy.pi)
acceleration = get_acceleration(velocity)
print encoder_position
#spikes = acceleration * (1.0/(abs(velocity) + 1))
spikes = acceleration * (velocity)
#velocity_ticks = (velocity / (2*numpy.pi)) * ppr / 1000.0
check_for_spikes(spikes)
check_for_unplugged(velocity)

plt.figure()
plt.subplot(311)
plt.plot(velocity,label='velocity')
plt.legend()

plt.subplot(312)
plt.plot(spikes,label='acceleration * abs(velocity)')
#plt.plot(encoder_position_mod[:-2],spikes[:-2],'*',label='acceleration * abs(velocity)')
plt.legend()

plt.subplot(313)
#plt.plot(encoder_position_mod[:-2],acceleration[:-2],'g*', label='acceleration')
plt.plot(acceleration,'g', label='acceleration')
plt.legend()

#plt.subplot(414)
#plt.plot(encoder_position_mod,'g*-', label='encoder_position')
#plt.legend()

if False:
  plt.figure()
  plt.subplot(111)
  plt.plot(encoder_velocity, '.', label='encoder velocity')

plt.show()
