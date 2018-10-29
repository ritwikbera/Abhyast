#!/usr/bin/env python

import roslib
import rospy
import tf
from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

prev_time,curr_time=0,0
lvel,rvel=0,0
x,y,th=0,0,0
WheelSeparation=1.06
vth,v=0,0
OdomTurnMultiplier=1
odom_pub=0

def tfbc():
	global curr_time
	br=tf.TransformBroadcaster()
	br.sendTransform((x,y,0),tf.transformations.quaternion_from_euler(0, 0,th),curr_time,'base_link','odom')

def odom_ekf():
	# prepare odom message for robot_pose_ekf
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
	odom = Odometry()
    	odom.header.stamp = curr_time
    	odom.header.frame_id = "odom"
    	# set the position
    	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    	# set the velocity
    	odom.child_frame_id = "base_link"
    	odom.twist.twist = Twist(Vector3(v,0, 0), Vector3(0, 0, vth))
    	# publish the message
	odom_pub.publish(odom)

def generate_odom_msg():
	global prev_time,curr_time,x,y,th,lvel,rvel,WheelSeparation,OdomTurnMultiplier,v,vth
	curr_time=rospy.Time.now()
	elapsed_time=curr_time-prev_time
	prev_time=curr_time
	dl=(elapsed_time.nsecs*lvel)/1000000000
	dr=(elapsed_time.nsecs*rvel)/1000000000
	dxy=(dl+dr)/2
	dth=(((dr-dl)/WheelSeparation))*OdomTurnMultiplier
	x+=dxy*cos(th)
	y+=dxy*sin(th)
	th+=dth
	rospy.loginfo(str(x)+' '+str(y)+' '+str(th*180/pi))
	# send transform
	tfbc()
	if elapsed_time.nsecs>0:
		v=dxy/(elapsed_time.nsecs/1000000000.0)
		vth=dth/(elapsed_time.nsecs/1000000000.0)
	#publish odom message
	odom_ekf()
	
def callback(twist_msg):
	global lvel,rvel
	lvel=twist_msg.linear.x
	rvel=twist_msg.linear.y
	generate_odom_msg()

def master():
	global prev_time, curr_time, odom_pub
	rospy.init_node('abhyast_node')
	prev_time,curr_time=rospy.Time.now(),rospy.Time.now()
	rospy.Subscriber('ard_odom',Twist,callback)
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	rospy.spin()

if __name__=='__main__':
	try:
		master()
	except rospy.ROSInterruptException:
		pass
