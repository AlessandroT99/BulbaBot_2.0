#!/usr/bin/env python3

import rospy

from bulbabot.msg import positionArray
from std_msgs.msg import UInt8

def callback(data):
	rospy.loginfo("I just heard %s", data.data)

def publishAngles(data):
    rospy.loginfo("Listened something from publishAngles")

def connection0(data):
	    rospy.loginfo("Listened something from connection")

def executeLoop(data):
	    rospy.loginfo("Listened something from executeLoop")

def listener():
	#In ROS, nodes are uniquely named. If two nodes with the same
	#name are launched, the previous one is kicked off. The
	#anonymous=True flag means that rospy will choose a unique
	#name for our 'listener' node so that multiple listeners can
	#run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Final_Angle", positionArray, publishAngles)
    rospy.Subscriber("Estabilish_Connection", UInt8, connection0)
    rospy.Subscriber("Desidered_Position", positionArray, executeLoop)

if __name__ == '__main__':
	try:
		listener()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Error - Exception occurs")
