#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from time import sleep

rospy.init_node('talker', anonymous=True) 

def talker():
	toprint = "ciao"
	rospy.loginfo(toprint)
	npub = rospy.Publisher('chatter', String, queue_size=10)
	npub.publish(toprint)

rate = rospy.Rate(10)

while 1:
	if __name__ == '__main__':
		try:
			talker()
			rate.sleep()
		except rospy.ROSInterruptException:
			pass
