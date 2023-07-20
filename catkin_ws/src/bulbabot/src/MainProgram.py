#!/usr/bin/env python3

from time import sleep, time
from bulbabot.msg import positionArray
from std_msgs.msg import UInt8
from inspect import currentframe, getframeinfo

import rospy

import ServosConfiguration
import CommonFeatures

#Class and functions definition ---------------------------------------- 
## DESCRIPTION: before starts the firmware, check the correct initialization of the working mode
def properWorkingMode():
	sum = 0
	for i in WORKING_MODES:
		sum = sum + i
	if sum > 1 or sum < 0:
		raise ValueError("Working Mode has not been setted properly. Check InitialConfiguration.py - " + str(LINEINFO_workingMode.lineno))
	
## DESCRIPTION: set the angle the gets back from the Control Loop execution for a single leg
def publishAngles(data):
	theta = [data.x,data.y,data.z]
	for i in len(theta):
		legServo = ServosConfiguration.servoFinding(i)
		ServosConfiguration.echoAngle(legServo, ServosConfiguration.angleConversion(theta[i]))
		print("Set " + legServo.name + " at " + str(theta[i]) + " degrees")

def passFunction(data):
	global connect
	connect = data.data

def checkNodeConnection(nodeName,node,sender_id,receiver_id):
	global connect, rate
	rospy.Subscriber("Receive_Connection", UInt8, passFunction)
	CommonFeatures.waitingPoints("\nContacting " + str(nodeName) + " node")
	connect = 0
	for i in range(1,CommonFeatures.NUMBER_OF_CONNECTION_TRIES):
		if connect != 0:
			break
		CommonFeatures.waitingPoints("Connection attempt " + str(i))
		node.publish(sender_id)
		start_time = time()
		while connect == 0: #if still no connection
			if connect == 0:
				elapsed_time = time()-start_time
				if elapsed_time > CommonFeatures.CONNECTION_TIMEOUT:
					if i == CommonFeatures.NUMBER_OF_CONNECTION_TRIES-1: #the last try
						raise TimeoutError("Connection Error - Unable to reach " + str(nodeName) + " node")
					else:
						print("Connection attempt " + str(i) + " failed\n")
						break
	if connect == receiver_id:
		print("Connected with " + str(nodeName) +  " succesfully\n")

## DESCRIPTION: initialize ROS node and subscribing nodes and callback functions
def MPnode_init():
    #In ROS, nodes are uniquely named. If two nodes with the same
    #name are launched, the previous one is kicked off. The
    #anonymous=True flag means that rospy will choose a unique
    #name for our 'listener' node so that multiple listeners can
    #run simultaneously.
    rospy.init_node('Main_Program', anonymous=False)
    rospy.Subscriber("Final_Angle", positionArray, publishAngles)

#Init ------------------------------------------------------------------
MPnode_init()
positionPublisher = rospy.Publisher("Desidered_Position", positionArray, queue_size = 1)
trasmitterConnection = rospy.Publisher("Transmit_Connection", UInt8, queue_size = 1)
rate = rospy.Rate(CommonFeatures.ROS_COMMUNICATION_FREQUENCY)

#Global variable definition --------------------------------------------
rd = positionArray()

#Global defines --------------------------------------------------------
LINEINFO_workingMode = getframeinfo(currentframe())
TIBIAS_DEBUG 	= 0 	#check working state of the motors
KEYBOARD 		= 0 	#set manually an angle of a motor
WORKING 		= 1		#working mode

WORKING_MODES = {TIBIAS_DEBUG, KEYBOARD, WORKING}

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
		
		CommonFeatures.waitingPoints("Waiting for nodes set up")
		sleep(20)
		CommonFeatures.waitingPoints("Checking nodes state")
		try: 
			checkNodeConnection("Control_Loop", trasmitterConnection, CommonFeatures.MAIN_PROGRAM_ID, CommonFeatures.CONTROL_LOOP_ID)
		except TimeoutError as err: 
			print(err)
			exit(0)

		rd.x = 189
		rd.y = 139
		rd.z = -23
		rd.shoulderSnum = ServosConfiguration.dxSM.snum
		rd.femurSnum = ServosConfiguration.dxFM.snum
		rd.tibiaSnum = ServosConfiguration.dxTM.snum
		print("Starting movement in 10s")
		sleep(10)
		#while 1:
		rospy.loginfo("Requested position: [%d,%d,%d] for left middle leg",rd.x,rd.y,rd.z)
		positionPublisher.publish(rd)
		#	sleep(10)
		rospy.spin()