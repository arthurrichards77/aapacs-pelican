#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_pelican')
import rospy
from asctec_msgs.msg import CtrlInput
import numpy

# init node and publisher to CTRL_INPUT
rospy.init_node('pelitest', anonymous=False)
pub_ctrl = rospy.Publisher('asctec/CTRL_INPUT', CtrlInput)

# basic message to send
my_ctrl = CtrlInput()
# set for thrust control
my_ctrl.ctrl = 8

# define checksum function
def ctrl_chksum(ctrl):
  chksum = ctrl.pitch + ctrl.roll + ctrl.yaw + ctrl.thrust + ctrl.ctrl + 0xAAAA
  chksum2 = numpy.int16(chksum)
  print ctrl
  print chksum2
  return(chksum2)

# run at 0.5 Hz for now
r = rospy.Rate(0.5)

# the loop!
while not rospy.is_shutdown():
  my_ctrl.thrust = 600
  my_ctrl.chksum = ctrl_chksum(my_ctrl)
  pub_ctrl.publish(my_ctrl)
  r.sleep()
  my_ctrl.thrust = 1400
  my_ctrl.chksum = ctrl_chksum(my_ctrl)
  pub_ctrl.publish(my_ctrl)
  r.sleep()
