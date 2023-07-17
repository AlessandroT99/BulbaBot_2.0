#!/usr/bin/env python3

from time import sleep
from os import system
from std_msgs.msg import UInt16MultiArray
from inspect import currentframe, getframeinfo

import rospy

import ServosConfiguration


#Class and functions definition ---------------------------------------- 
## DESCRIPTION: before starts the firmware, check the correct initialization of the working mode
def properWorkingMode():
	sum = 0
	for i in WORKING_MODES:
		sum = sum + i
	if sum > 1 or sum < 0:
		raise ValueError("Working Mode has not been setted properly. Check InitialConfiguration.py - " + str(LINEINFO_workingMode.lineno))
	
## DESCRIPTION: set the angle the gets back from the Control Loop execution for a single leg
def publishAngles(Leg, theta):
	for i in len(theta):
		ServosConfiguration.echoAngle(Leg[i], ServosConfiguration.angleConversion(theta[i]))
		print("Set " + Leg[i].name + " at " + str(theta[i]) + " degrees")

## DESCRIPTION: initialize ROS node and subscribing nodes and callback functions
def MPnode_init():
    #In ROS, nodes are uniquely named. If two nodes with the same
    #name are launched, the previous one is kicked off. The
    #anonymous=True flag means that rospy will choose a unique
    #name for our 'listener' node so that multiple listeners can
    #run simultaneously.
    rospy.init_node('Main_Program', anonymous=True)
    rospy.Subscriber("Final_Angle", UInt16MultiArray, publishAngles)

#Global variable definition --------------------------------------------

#Global defines --------------------------------------------------------
LINEINFO_workingMode = getframeinfo(currentframe())
TIBIAS_DEBUG 	= 0 	#check working state of the motors
KEYBOARD 		= 1 	#set manually an angle of a motor
WORKING 		= 0		#working mode

WORKING_MODES = {TIBIAS_DEBUG, KEYBOARD, WORKING}

#Init ------------------------------------------------------------------
MPnode_init()
positionPublisher = rospy.Publisher("Desidered_Position", UInt16MultiArray, queue_size = 18)
rate = rospy.Rate(10000)

#Main ------------------------------------------------------------------
if __name__ == '__main__':
	properWorkingMode()
	ServosConfiguration.servoInit()

	if TIBIAS_DEBUG:
		motion = 1 #used to change the movement direction of the tibias
		while 1:	
			if motion == 1:
				ServosConfiguration.sxTF.angle = ServosConfiguration.sxTF.angle + ServosConfiguration.angleConversion(20)
				ServosConfiguration.sxTM.angle = ServosConfiguration.sxTM.angle + ServosConfiguration.angleConversion(20)
				ServosConfiguration.sxTR.angle = ServosConfiguration.sxTR.angle + ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTF.angle = ServosConfiguration.dxTF.angle - ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTM.angle = ServosConfiguration.dxTM.angle - ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTR.angle = ServosConfiguration.dxTR.angle - ServosConfiguration.angleConversion(20)
				motion = 0
			else:
				ServosConfiguration.sxTF.angle = ServosConfiguration.sxTF.angle - ServosConfiguration.angleConversion(20)
				ServosConfiguration.sxTM.angle = ServosConfiguration.sxTM.angle - ServosConfiguration.angleConversion(20)
				ServosConfiguration.sxTR.angle = ServosConfiguration.sxTR.angle - ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTF.angle = ServosConfiguration.dxTF.angle + ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTM.angle = ServosConfiguration.dxTM.angle + ServosConfiguration.angleConversion(20)
				ServosConfiguration.dxTR.angle = ServosConfiguration.dxTR.angle + ServosConfiguration.angleConversion(20)
				motion = 1
			
			ServosConfiguration.echoAngle(ServosConfiguration.sxTF,ServosConfiguration.sxTF.angle)
			ServosConfiguration.echoAngle(ServosConfiguration.sxTM,ServosConfiguration.sxTM.angle)
			ServosConfiguration.echoAngle(ServosConfiguration.sxTR,ServosConfiguration.sxTR.angle)
			ServosConfiguration.echoAngle(ServosConfiguration.dxTF,ServosConfiguration.dxTF.angle)
			ServosConfiguration.echoAngle(ServosConfiguration.dxTM,ServosConfiguration.dxTM.angle)
			ServosConfiguration.echoAngle(ServosConfiguration.dxTR,ServosConfiguration.dxTR.angle)
			sleep(1)

	elif KEYBOARD:
		ServosConfiguration.infoTable()
		while 1:
			newPin = input("\nWhich motor you wanna move? ")
			if 0<=int(newPin)<18:
				newAngle = input("Which angle do you wanna try? [def as default]: ")
				if newAngle == "def":
					newAngle = ServosConfiguration.defaultValue(int(newPin))
				if ServosConfiguration.VELOCITY:
					velocity = int(input("Set the velocity [1 as default]: "))
				else: 
					velocity = 1
				for k in ServosConfiguration.servoList:
					if k.snum == int(newPin):
						if ServosConfiguration.echoAngle(k,ServosConfiguration.angleConversion(newAngle),velocity): #proceed to update angle value only if the movement is completed
							k.angle = int(newAngle)
							print("Changing " + k.name + " angle into " + str(k.angle) + " degrees")
						break
			else: 
				print("\nServo not found. The only avaible servos are in the following.")
				ServosConfiguration.infoTable()
		
			sleep(1)
		

	elif WORKING:
		print("Welcome user, BulbaBot 2.0 is awake, and ready to follow your instructions.\n")
		rd = [189,139,-23]
		rospy.loginfo(rd)
		positionPublisher.publish(rd)

