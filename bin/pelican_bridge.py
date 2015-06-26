#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_pelican')
import rospy
from asctec_msgs.msg import CtrlInput
from geometry_msgs.msg import Twist
import numpy

# Node naming
rospy.init_node('pelican_bridge', anonymous=False)  # anonymous=True)

# Node publisher to CTRL_INPUT
pub_ctrl = rospy.Publisher('asctec/CTRL_INPUT', CtrlInput)

# Message structure to send
my_ctrl = CtrlInput()
my_ctrl.ctrl = 1 + 2 + 4 + 8 # Order of bitmask is pitch, roll, yaw, thrust

# define checksum function
def ctrl_chksum(ctrl):
  chksum = ctrl.pitch + ctrl.roll + ctrl.yaw + ctrl.thrust + ctrl.ctrl + 0xAAAA
  chksum2 = numpy.int16(chksum)
  print ctrl
  print chksum2
  return(chksum2)

# Saturate function, values betwee -1 and 1
def saturate(data):
  if data > 1:
    data = 1
  elif data < -1:
    data = -1

  return data

def send_control(data):
  # Check control data is valid
  if 1:
    # Convert to values into -2047 to 2047 range (or 0 to 4095 on thr)
    if type(data) == Twist:
      # Saturate controls
      data.linear.x = saturate(data.linear.x)
      data.linear.y = saturate(data.linear.y)
      data.linear.z = saturate(data.linear.z)
      data.angular.z = saturate(data.angular.z)

      # Populate channels
      my_ctrl.roll = int(data.linear.y*2047)
      my_ctrl.pitch = int(data.linear.x*2047)
      my_ctrl.thrust = int(data.linear.z*2047 + 2047)
      my_ctrl.yaw = int(data.angular.z*2047)

    else:
      return

    # Compute checksum
    my_ctrl.chksum = ctrl_chksum(my_ctrl)

    # Send control
    pub_ctrl.publish(my_ctrl)

def callback_ctrl(data):
  # Log incoming control message
  rospy.loginfo(rospy.get_caller_id() + "Inputs = [%5.2f %5.2f %5.2f %5.2f]", data.linear.x, data.linear.y, data.linear.z, data.angular.z)
  send_control(data)

# Callbacks for incoming control messages
rospy.Subscriber("pelican_ctrl", Twist, callback_ctrl)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
