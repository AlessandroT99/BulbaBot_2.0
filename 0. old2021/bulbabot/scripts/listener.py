#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import String
from bulbabot.msg import Leg

def callback(data):
	rospy.loginfo("I just heard %s", data.data)

def calibration(leg_data):
	toprint = "Leg number %s" % leg_data.leg_number
	if leg_data.shoulder_position != 0: 
		toprint = toprint +  " it is changing its shoulder position at %d" % leg_data.shoulder_position
	elif leg_data.femur_position != 0:
		toprint = toprint + " it is changing its femur position at %d" % leg_data.femur_position
	elif leg_data.tibia_position != 0:
		toprint = toprint + " it is changing its tibia position at %d" % leg_data.tibia_position
	rospy.loginfo("%s", toprint)

def listener():
	#In ROS, nodes are uniquely named. If two nodes with the same
	#name are launched, the previous one is kicked off. The
	#anonymous=True flag means that rospy will choose a unique
	#name for our 'listener' node so that multiple listeners can
	#run simultaneously.
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("chatter", String, callback)
	rospy.Subscriber("leg_calibration", Leg, calibration)

if __name__ == '__main__':
	try:
		listener()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Error - Exception occurs")
