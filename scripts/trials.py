#!/usr/bin/env python

import roslib
import rospy
import tf
from std_msgs.msg import String

def func():
	pub=rospy.Publisher("britwik",String,queue_size=10)
	rospy.init_node('mynode')
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish('fuck you')
		rospy.loginfo(str(rospy.Time.now()))
		tfbc()
		rate.sleep()
def callback(in_data):
	rospy.loginfo(in_data.data)
def func2():
	rospy.init_node('mynode2')
	rospy.Subscriber('ritwik',String, callback)
	rospy.spin()
def tfbc():
	br=tf.TransformBroadcaster()
	br.sendTransform((1,1,1),tf.transformations.quaternion_from_euler(0, 0,0),rospy.Time.now(),'map','odom')

if __name__=='__main__':
	try:
		func2()
	except rospy.ROSInterruptException:
		pass
