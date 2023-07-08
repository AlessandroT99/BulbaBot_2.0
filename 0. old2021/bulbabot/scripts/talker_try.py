#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import String
from bulbabot.msg import Leg
from time import sleep

leg_ref = Leg()

rospy.init_node('telegram_input', anonymous=True) 

def talker():
	leg_ref.leg_number = 3
	leg_ref.shoulder_position = 0
	leg_ref.femur_position = 0
	leg_ref.tibia_position = 30
	rospy.loginfo(leg_ref)
	npub = rospy.Publisher('leg_calibration', Leg, queue_size=10)
	npub.publish(leg_ref)

while 1:
	if __name__ == '__main__':
		talker()
		sleep(3)
