#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32, Vector3Stamped, Vector3
import numpy as np
import matplotlib.pyplot as plotter
from tf import TransformListener
import message_filters
from tf2_msgs.msg import TFMessage

def feedback_callback(data):
  global trajectory

  if not data.trajectories: # empty
    trajectory = []
    return
  trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
  
#def tf_callback(frame):
#  global gV
#  gV = Vector3Stamped()
#  gV.vector = 0
#  gV.header.stamp = 0
#  gV.header.frame_id = ""
#  global gV2
#  gV2 = Vector3Stamped()
#  gV2.vector = 0
#  gV2.header.stamp = 0
#  gV2.header.frame_id = ""
#  return
  
def plot_velocity_profile(fig, ax_v, ax_roll, t, v, x1):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_roll.cla()
  ax_roll.grid()
  ax_roll.set_ylabel('Roll [rad]')
  ax_roll.set_xlabel('Time [s]')
  ax_roll.plot(t, x1, '-bx')
  fig.canvas.draw()

  
  
def velocity_plotter():
  global trajectory
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  global gV
  global gV2
  topic_name1 = "/move_base/TebLocalPlannerROS/teb_feedback"
  #topic_name2 = "/tf"
  rospy.Subscriber(topic_name1, FeedbackMsg, feedback_callback, queue_size = 1) # define feedback topic here!
  #rospy.Subscriber(topic_name2, TFMessage, tf_callback, queue_size = 1)

  #sub = message_filters.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg)
  #sub.registerCallback(feedback_callback)
  
  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name1) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # two subplots sharing the same t axis
  fig, (ax_v, ax_roll) = plotter.subplots(2, sharex=True)
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    t = []
    v = []
    omega = []
    roll = []
    roll_rate = []
    a = 0
    b = 2.3 #-12.05
    c = 2.64#1.26

    x1_0 = 0
    x2_0 = 0
    x1 = 0
    x2 = 0
#if point.time_from_start.to_sec() == 0.0:
#	roll.append(x1_0)
#        roll_rate.append(x2_0) 
#else:
    t_old = 0.0
    vy_old = 0.0
    for point in trajectory:
      t.append(point.time_from_start.to_sec())
      
      #gV = Vector3Stamped()
      #gV.vector = point.velocity.linear
      #gV.header.stamp = rospy.Time()
      #gV.header.frame_id = "/odom"
      #tV = Vector3Stamped()

      #gV2 = Vector3Stamped()
      #gV2.vector = point.velocity.angular
      #gV2.header.stamp = rospy.Time()
      #gV2.header.frame_id = "/odom"
      #tV2 = Vector3Stamped()

      #tV = TransformListener.transformVector3("/base_link", vector3StampedMsgToTF (Vector3Stamped &gV)) #
      #v.append(tV.vector.x)#
      #tV2 = TransformListener.transformVector3("/base_link", vector3StampedMsgToTF (Vector3Stamped &gV2)) #
      #omega.append(tV2.vector.z)#
      if point.velocity.linear.x == 0.0:
	theta = np.pi/2
      else:	
        theta = np.arctan(point.velocity.linear.y / point.velocity.linear.x)
      converted_x = point.velocity.linear.x * np.cos(theta) - point.velocity.linear.y * np.sin(theta)
      converted_y = point.velocity.linear.x * np.sin(theta) + point.velocity.linear.y * np.cos(theta)

      v.append(converted_x)      
      omega.append(point.velocity.angular.z)
      y_acc = point.acceleration.linear.y
      yaw_rate = point.velocity.angular.z
      #yaw_rate = tV2.vector.z
      dt = (point.time_from_start.to_sec() - t_old)/100
      t_old = point.time_from_start.to_sec()
      vy = converted_y
#    double x1_0,x2_0,x1,x2,dt, t_0, t_f          #//for initial values, width, etc.

#    dt = 0.01	#should actually be the gap between two timestamps of trajectory divided by 100

      for i in range(0, 100):   #100 is randomly chosen#
		if dt == 0.0:
			y_acc = 0.0
		else:
			y_acc = (vy - vy_old)/dt;  
        	x2 = x2_0 + (dt * (-a*x2 -b*x1 -c*(y_acc + converted_x * yaw_rate)))
        	x1 = x1_0 + dt*x2_0            #//calculate new y, which is y0+h*dy/dx   
        
        	x2_0=x2                    #//pass this new y as y0 in the next iteration.
        	x1_0=x1                #//calculate new x.
      vy_old = vy
      roll.append(x1)
      roll_rate.append(x2)      
#    plot_velocity_profile(fig, ax_v, ax_roll, np.asarray(t), np.asarray(v), np.asarray(omega))
    plot_velocity_profile(fig, ax_v, ax_roll, np.asarray(t), np.asarray(v), np.asarray(roll))        
    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass



#-------------------------
# REQUIRED IF TRANSFORM FUNCTION DOES NOT WORK
#def q_mult(q1, q2):
#    w1, x1, y1, z1 = q1
#    w2, x2, y2, z2 = q2
#    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
#    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
#    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
#    return w, x, y, z#

#def q_conjugate(q):
#    w, x, y, z = q
#    return (w, -x, -y, -z)

#def qv_mult(q1, v1):
#    q2 = (0.0,) + v1
#    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

#def normalize(v, tolerance=0.00001):
#    mag2 = sum(n * n for n in v)
#    if abs(mag2 - 1.0) > tolerance:
#        mag = sqrt(mag2)
#        v = tuple(n / mag for n in v)
#    return v

#subscribe to /tf2
#buffer = tf2_ros.Buffer()
#tfl = tf2_ros.TransformListener(buffer)

#apply a message filter to it so that every time a data comes we can match it with its correct frame transform
#from this /tf message, we will extract the quaternion (IF THE transformVector3 FUNCTION DOES NOT WORK) and then
#	v = qv_mult(transforms.transform.rotation, point.velocity.linear) //Note that v2 is not a quaternion since the array splice operator is used
#	transformed_q = normalize(v) #PROBABLY NOT NEEDED BECAUSE OF ABOVE COMMENT
#	transformed_v = transformed_q #SAME AS ABOVE COMMENT

#SEARCH AGAIN FOR VECTOR3STAMPEDMSGTOTF IN PYTHON OR ELSE CONVERT EVERYTHING TO C++
