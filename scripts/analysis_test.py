#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_diagnostic_controller')
from pr2_diagnostic_controller.msg import *
import os
from yaml import load
import argparse
import matplotlib.pyplot as plt 
import numpy
from scipy.stats import scoreatpercentile
import sys

#dictionary of actuators and their torque constants
r_arm_actuators = {'r_wrist_r_motor':0.0538,'r_wrist_l_motor':0.0538,'r_forearm_roll_motor':0.0603,'r_upper_arm_roll_motor':0.0603, 'r_elbow_flex_motor':0.0603,'r_shoulder_lift_motor':0.0603,'r_shoulder_pan_motor':0.0603}
l_arm_actuators = {'l_wrist_r_motor':0.0538,'l_wrist_l_motor':0.0538,'l_forearm_roll_motor':0.0603,'l_upper_arm_roll_motor':0.0603, 'l_elbow_flex_motor':0.0603,'l_shoulder_lift_motor':0.0603,'l_shoulder_pan_motor':0.0603}

def get_acceleration(velocity, timestamp):
#  acceleration = numpy.convolve(filt1, velocity)
#  acceleration = acceleration[delay:len(velocity) + delay]
  acceleration = (numpy.array(velocity[1:]) - numpy.array(velocity[:-1])) / (numpy.array(timestamp[1:]) - numpy.array(timestamp[:-1]))
  
# acceleration[-1] = 0
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

  if (len(outliers_neg) + len(outliers_pos)) >= 4:
    print "Encoder could be spoilt"
    return True

  return False

def check_for_unplugged(velocity, measured_motor_voltage):
  zero_velocity = [val for val in velocity if val == 0]
  zero_voltage = [val for val in measured_motor_voltage if val == 0]
  zero_velocity = len(zero_velocity) / (len(velocity) + 0.0)
  zero_voltage = len(zero_voltage) / (len(measured_motor_voltage) + 0.0)
  print "percentage of zero velocity is", zero_velocity
  print "percentage of zero voltage is", zero_voltage

  if zero_velocity > 0.1 and zero_voltage < 0.05:
    print "encoder could be unplugged"
    return True
  return False

def check_for_open(velocity, measured_motor_voltage):
  measured_motor_voltage = abs(measured_motor_voltage)
  velocity = abs(velocity)
  mean_voltage = measured_motor_voltage.mean()
  mean_velocity = velocity.mean()
  print "mean_voltage is ",mean_voltage
  print "mean_velocity is ",mean_velocity
  if mean_voltage < 0.05:
    if mean_velocity > 0.3:
      print "Motor wires could be cut causing open circuit"
      return True
  return False

def plot(param):
  (filename,velocity,spikes,acceleration,outlier_limit_neg,outlier_limit_pos) = param
  global plot_enabled
  plot_enabled = True

  plt.figure(filename + '_1')
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

  plt.figure(filename + '_2')
  plt.plot(supply_voltage,'b',label='supply_voltage')
  plt.plot(measured_motor_voltage,'g',label='measured_motor_voltage')
  plt.plot(executed_current,'*y',label='executed_current')
  plt.plot(measured_current,'m',label='measured_current')
  plt.legend() 

    
if __name__ == '__main__':
  parser = argparse.ArgumentParser("script to analyse diagnostic data for PR2")
  parser.add_argument("files", help="Specify a file name or a folder with files to analyse")
  parser.add_argument("-s","--show", help="show all results or show only bad results, default is all results", action="store_true")
  args = parser.parse_args()
  
  positional_args = args.files.split(".")

  filelist = [] 
  plot_enabled = False

  if (positional_args[-1] == 'yaml'):
    filelist.append(".".join(positional_args)) 
  else:
    dirpath = os.getcwd() + '/' + positional_args[0]  
    filelist = os.listdir(dirpath)
    os.chdir(dirpath)

  print filelist
  ppr = 1200.0
  delay = 1 
  filt1 = numpy.array([-1,1])

  for filename in filelist:
    outlier_limit_neg = 0
    outlier_limit_pos = 0
    stream = file(filename, 'r')
    samples = load(stream)
    velocity = []
    encoder_position = []
    supply_voltage = []
    measured_motor_voltage = []
    executed_current = []
    measured_current = []
    timestamp = []

    for s in samples.sample_buffer:
      velocity.append(s.velocity)
      encoder_position.append(s.encoder_position)
      supply_voltage.append(s.supply_voltage)
      measured_motor_voltage.append(s.measured_motor_voltage)
      executed_current.append(s.executed_current)
      measured_current.append(s.measured_current)
      timestamp.append(s.timestamp)

    velocity = numpy.array(velocity)
    encoder_position = numpy.array(encoder_position)
   
    supply_voltage = numpy.array(supply_voltage)
    measured_motor_voltage = numpy.array(measured_motor_voltage)
    executed_current = numpy.array(executed_current)
    measured_current = numpy.array(measured_current)

    acceleration = get_acceleration(velocity, timestamp)

    spikes = acceleration * (velocity[:-1])

    result1 = check_for_spikes(spikes) 
    result2 = check_for_unplugged(velocity, measured_motor_voltage)
    result3 = check_for_open(velocity, measured_motor_voltage)
    result = result1 and result2 and result3

    if args.show:
      if result:
        param = (filename,velocity,spikes,acceleration,outlier_limit_neg,outlier_limit_pos)
        plot(param)
    else:
      param = (filename,velocity,spikes,acceleration,outlier_limit_neg,outlier_limit_pos)
      plot(param)
  
  if plot_enabled:  
    plt.show()
