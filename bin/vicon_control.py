#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_pelican')
import sys
import rospy
import numpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from asctec_msgs.msg import CtrlInput

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

# define checksum function
def ctrl_chksum(ctrl):
  chksum = ctrl.pitch + ctrl.roll + ctrl.yaw + ctrl.thrust + ctrl.ctrl + 0xAAAA
  chksum2 = numpy.int16(chksum)
  return(chksum2)

def viconCallback(data):
  # need to explicitly ref message store global as I'm writing to it
  global last_msg
  global ref_transform
  # downsample - only act on every N callbacks
  down_sample = 1
  if data.header.seq % down_sample == 0:
    # default control is nothing
    cmd_twist = Twist()
    # run yaw feedback - keep aligned with world frame
    cmd_twist.angular.z = -1*data.transform.rotation.z
    # limit in [-0.5,0.5]
    cmd_twist.angular.z = saturate(cmd_twist.angular.z,0.5)
    # proportional gains on position
    cmd_twist.linear.x = kpx*(ref_transform.translation.x - data.transform.translation.x)
    cmd_twist.linear.y = kpy*(ref_transform.translation.y - data.transform.translation.y)
    cmd_twist.linear.z = kpz*(ref_transform.translation.z - data.transform.translation.z)
    # derivative gains depend on having earlier message stored
    try:
      # find time since last message
      delta_t = data.header.stamp.secs - last_msg.header.stamp.secs
      delta_t = delta_t + (data.header.stamp.nsecs - last_msg.header.stamp.nsecs)/1000000000.0
      # estimate velocities coarsely using finite difference
      vx = (data.transform.translation.x - last_msg.transform.translation.x)/delta_t
      vy = (data.transform.translation.y - last_msg.transform.translation.y)/delta_t
      # add derivative gain
      cmd_twist.linear.x = cmd_twist.linear.x - kdx*vx
      cmd_twist.linear.y = cmd_twist.linear.y - kdy*vy
      # propagate the reference
      ref_vel_scale = 2.0
      ref_transform.translation.x = ref_transform.translation.x + delta_t*ref_velocity.linear.x*ref_vel_scale
      ref_transform.translation.y = ref_transform.translation.y + delta_t*ref_velocity.linear.y*ref_vel_scale
      ref_transform.translation.z = ref_transform.translation.z + delta_t*ref_velocity.linear.z*ref_vel_scale
    except ValueError as e:
      print 'Nothing yet'
    # store it for next time
    last_msg = data
    # limit
    cmd_twist.linear.x = saturate(cmd_twist.linear.x,0.3)  
    cmd_twist.linear.y = saturate(cmd_twist.linear.y,0.3)  
    cmd_twist.linear.z = saturate(cmd_twist.linear.z,0.3)  
    # convert from Twist message to Pelican CTRL_INPUT
    my_ctrl = CtrlInput()
    # set bits for which axes to control: 1 = pitch, 2 = roll, 4 = yaw, 8 = thrust
    my_ctrl.ctrl = 1
    # transcribe
    my_ctrl.pitch = cmd_twist.linear.x * -2000.0
    # add checksum
    my_ctrl.chksum = ctrl_chksum(my_ctrl)
    # send message
    pub_ctrl.publish(my_ctrl)

def refCallback(data):
  global ref_velocity
  ref_velocity = data

def reftfCallback(data):
  global ref_velocity
  # set reference movement to zero
  ref_velocity = Twist()
  # and set position to whatever was received
  ref_transform.translation.x = data.transform.translation.x
  ref_transform.translation.y = data.transform.translation.y
  ref_transform.translation.z = data.transform.translation.z

def nudgeCallback(data):
  global ref_velocity
  # set reference movement to zero
  ref_velocity = Twist()
  # and add to position whatever was received
  ref_transform.translation.x = ref_transform.translation.x + data.translation.x
  ref_transform.translation.y = ref_transform.translation.y + data.translation.y
  ref_transform.translation.z = ref_transform.translation.z + data.translation.z

rospy.init_node('vicon_control', anonymous=True)
sub_vicondata = rospy.Subscriber('drone', TransformStamped, viconCallback)
sub_ref_vel = rospy.Subscriber('ref_vel', Twist, refCallback)
sub_ref_pos = rospy.Subscriber('ref_tf', TransformStamped, reftfCallback)
sub_nudge = rospy.Subscriber('ref_nudge', Transform, nudgeCallback)

pub_ctrl = rospy.Publisher('asctec/CTRL_INPUT', CtrlInput)

# global for velocity memory
last_msg = TransformStamped()
# and for reference position
ref_transform = Transform()
# default altitude
ref_transform.translation.z = 1.2
# and the reference velocity
ref_velocity = Twist()

# grab gain parameters
kpx = rospy.get_param('~kpx',0.1)
kdx = rospy.get_param('~kdx',0.1)
kpy = rospy.get_param('~kpy',0.6)
kdy = rospy.get_param('~kdy',0.3)
kpz = rospy.get_param('~kpz',0.6)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
