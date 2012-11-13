#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_diagnostic_controller')
from pr2_diagnostic_controller.msg import *
import os
from yaml import load
import matplotlib.pyplot as plt 
import numpy
from scipy.stats import scoreatpercentile
import sys

r_arm_actuators = {'r_wrist_r_motor':2.52,'r_wrist_l_motor':2.52,'r_forearm_roll_motor':1.16,'r_upper_arm_roll_motor':1.16, 'r_elbow_flex_motor':1.16,'r_shoulder_lift_motor':1.16,'r_shoulder_pan_motor':1.16}
l_arm_actuators = {'l_wrist_r_motor':2.52,'l_wrist_l_motor':2.52,'l_forearm_roll_motor':1.16,'l_upper_arm_roll_motor':1.16, 'l_elbow_flex_motor':1.16,'l_shoulder_lift_motor':1.16,'l_shoulder_pan_motor':1.16}

def get_acceleration(velocity):
  acceleration = numpy.convolve(filt1, velocity)
  acceleration = acceleration[delay:len(velocity) + delay]
  acceleration[-1] = 0
  acceleration = abs(acceleration)
  acceleration = acceleration / acceleration.max()
  return acceleration

def check_for_spikes(spikes):
  global outlier_limit_neg
  global outlier_limit_pos

  neg = [val for val in spikes if val < 0]
  pos = [val for val in spikes if val > 0]

  neg = numpy.sort(neg,kind='mergesort')
  pos = numpy.sort(pos,kind='mergesort')
  
  if (len(neg) > 1):
    iq_range_neg = scoreatpercentile(neg,-75) - scoreatpercentile(neg,-25)
    outlier_limit_neg = iq_range_neg * 3 + scoreatpercentile(neg,-75)

  if (len(pos) > 1):
    iq_range_pos = scoreatpercentile(pos,75) - scoreatpercentile(pos,25)
    outlier_limit_pos = iq_range_pos * 3 + scoreatpercentile(pos,75)

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

  if (len(outliers_neg) + len(outliers_pos)) >= 2:
    print "Encoder could be spoilt"

def check_for_unplugged(velocity):
  print "test"
#calculated_velocity = [

if __name__ == '__main__':
  if len(sys.argv) == 2:
    fn = sys.argv[1]
  else:
    fn = 'results.yaml'
    print 'using', fn
 

  outlier_limit_neg = 0
  outlier_limit_pos = 0
  stream = file(fn, 'r')
  samples = load(stream)
  velocity = []
  encoder_position = []
  supply_voltage = []
  measured_motor_voltage = []
  executed_current = []
  measured_current = []

  ppr = 1200.0
  delay = 1 
  filt1 = numpy.array([-1,1])

  for s in samples.sample_buffer:
    velocity.append(s.velocity)
    encoder_position.append(s.encoder_position)
    supply_voltage.append(s.supply_voltage)
    measured_motor_voltage.append(s.measured_motor_voltage)
    executed_current.append(s.executed_current)
    measured_current.append(s.measured_current)

  velocity = numpy.array(velocity)
  encoder_position = numpy.array(encoder_position)
   
  supply_voltage = numpy.array(supply_voltage)
  measured_motor_voltage = numpy.array(measured_motor_voltage)
  exceuted_current = numpy.array(executed_current)
  measured_current = numpy.array(measured_current)

  acceleration = get_acceleration(velocity)

  spikes = acceleration * (velocity)

  check_for_spikes(spikes)
  check_for_unplugged(velocity)

  plt.figure()
  plt.subplot(311)
  plt.plot(velocity,label='velocity')
  plt.legend()

  temp_neg = outlier_limit_neg
  temp_pos = outlier_limit_pos
  outlier_limit_neg = [temp_neg for val in range(0,len(velocity))]
  outlier_limit_pos = [temp_pos for val in range(0,len(velocity))]

  plt.subplot(312)
  plt.plot(spikes,label='acceleration * velocity')
  plt.plot(outlier_limit_neg,'r')
  plt.plot(outlier_limit_pos,'r')

  plt.legend()

  plt.subplot(313)
  plt.plot(acceleration,'g', label='acceleration')
  plt.legend()

  plt.figure()
  plt.plot(supply_voltage,'b',label='supply_voltage')
  plt.plot(measured_motor_voltage,'g',label='measured_motor_voltage')
  plt.plot(executed_current,'*y',label='executed_current')
  plt.plot(measured_current,'m',label='measured_current')
  plt.legend() 

  plt.show()
