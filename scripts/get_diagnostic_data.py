#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_diagnostic_controller')
import rospy
from pr2_diagnostic_controller.srv import *
from std_msgs.msg import Float64
import time
from yaml import dump

def test_controller():
#rospy.init_node('pr2_diagnostic_controller')
  get_data = rospy.ServiceProxy('diagnostic_controller/get_diagnostic_data',DiagnosticData)
  foo = DiagnosticDataRequest();
  rv = get_data(foo)
  stream = file('results.yaml', 'w')
  dump(rv,stream)
  #print len(rv.sample_buffer) 
  #print rv

if __name__ == '__main__':
  try:
    test_controller()
  except rospy.ROSInterruptException: pass

