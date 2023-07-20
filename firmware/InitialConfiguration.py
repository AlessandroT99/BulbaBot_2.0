#!/usr/bin/env python3.6

from time import sleep
from os import system
from inspect import currentframe, getframeinfo

from LegsParameters import *

#Class and functions definition ---------------------------------------- 
class stru:
	def __init__(self):
		self.pin = 0
		self.snum = 0
		self.angle = 0
		self.name = ""

## DESCRIPTION: before starts the firmware, check the correct initialization of the working mode
def properWorkingMode():
	sum = 0
	for i in WORKING_MODES:
		sum = sum + i
	if sum > 1 or sum < 0:
		raise ValueError("Working Mode has not been setted properly. Check InitialConfiguration.py - " + str(LINEINFO_workingMode.lineno))

## DESCRIPTION: maps from 0-180 grades to 50-250 time period
def angleConversion(alpha):
	if int(alpha)>180 or int(alpha)<0:
		raise TypeError("Only degrees values between 0 and 180 are allowed")
	return round(int(alpha)*10/9+50)

## DESCRIPTION: check correctness of the angle and the publish it to the servoBlaster manager
def echoAngle(localServo,newAngle,velocity):
	try:
		localServoName = localServo.name.split()
		angleCheck(localServoName[0],localServoName[2],newAngle)
		if velocity > VELOCITY_DEFAULT: #if velocity is setted
			frameSteps = abs(round((newAngle-localServo.angle)/velocity)) #divide path space into "velocity" steps
			if localServo.angle <= newAngle: #if the new angle is higher it has to be done a summation 
				while localServo.angle != newAngle: #repeat until the steps are over
					localServo.angle += frameSteps
					if localServo.angle > newAngle: 
						localServo.angle = newAngle
					cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle
					system(cmd)
					sleep(VELOCITY_WAIT)
			else: #if the new angle is lower it has to be done a substraction
				while localServo.angle != newAngle: #repeat until the steps are over
					localServo.angle -= frameSteps
					if localServo.angle < newAngle: 
						localServo.angle = newAngle
					cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle
					system(cmd)
					sleep(VELOCITY_WAIT)
		elif velocity < VELOCITY_DEFAULT: #velocity input error
			raise ValueError("Velocity cannot be negative.")
		else: #default velocity as fast as possibile
			localServo.angle = newAngle
			cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle 
			system(cmd)
		return 1 #operation goes in the right way
	except Exception as err:
		print(err)
		return 0 #an error as been found
		
## DESCRIPTION: print a table which contains all the servos and affiliated infos
def infoTable():
	print("\n")
	print(' List of all the servos '.center(44,'-'))
	print("{:<14} {:<22} {:<8}".format('Servo number','Servo Name','HW Pin'))
	for k in servoList:
		print("{:<14} {:<22} {:<8}".format(k.snum,k.name,k.pin))
	print(''.center(44,'-'))

## DESCRIPTION: initialization of the servos pins and setting the default positions
def servoInit():
	servoString = ""
	for i in servoList:
		servoString = servoString + str(i.pin) + ","
	servoStart = '/home/ubuntu/servoblaster/servod --p1pins="%s" --pcm &' %servoString[:-1]
	system(servoStart)
	sleep(2)
	for i in servoList:
		echoAngle(i,i.angle,VELOCITY_DEFAULT) #setting default position

#Struct define and init ------------------------------------------------
#SX Legs
sxTF = stru(); sxTF.name = "Sx Front Tibia"; sxTF.pin = SxF_tibia; sxTF.snum = 0; sxTF.angle = angleConversion(SX_DEFAULT_TIBIA)
sxTM = stru(); sxTM.name = "Sx Middle Tibia"; sxTM.pin = SxM_tibia; sxTM.snum = 1; sxTM.angle = angleConversion(SX_DEFAULT_TIBIA)
sxTR = stru(); sxTR.name = "Sx Rear Tibia"; sxTR.pin = SxR_tibia; sxTR.snum = 2; sxTR.angle = angleConversion(SX_DEFAULT_TIBIA)
sxFF = stru(); sxFF.name = "Sx Front Femur"; sxFF.pin = SxF_femur; sxFF.snum = 3; sxFF.angle = angleConversion(SX_DEFAULT_FEMUR)
sxFM = stru(); sxFM.name = "Sx Middle Femur"; sxFM.pin = SxM_femur; sxFM.snum = 4; sxFM.angle = angleConversion(SX_DEFAULT_FEMUR)
sxFR = stru(); sxFR.name = "Sx Rear Femur"; sxFR.pin = SxR_femur; sxFR.snum = 5; sxFR.angle = angleConversion(SX_DEFAULT_FEMUR)
sxSF = stru(); sxSF.name = "Sx Front Shoulder"; sxSF.pin = SxF_shoulder; sxSF.snum = 6; sxSF.angle = angleConversion(SX_DEFAULT_SHOULDER)
sxSM = stru(); sxSM.name = "Sx Middle Shoulder"; sxSM.pin = SxM_shoulder; sxSM.snum = 7; sxSM.angle = angleConversion(SX_DEFAULT_SHOULDER)
sxSR = stru(); sxSR.name = "Sx Rear Shoulder"; sxSR.pin = SxR_shoulder; sxSR.snum = 8; sxSR.angle = angleConversion(SX_DEFAULT_SHOULDER)

#DX Legs
dxTF = stru(); dxTF.name = "Dx Front Tibia"; dxTF.pin = DxF_tibia; dxTF.snum = 9; dxTF.angle = angleConversion(DX_DEFAULT_TIBIA)
dxTM = stru(); dxTM.name = "Dx Middle Tibia"; dxTM.pin = DxM_tibia; dxTM.snum = 10; dxTM.angle = angleConversion(DX_DEFAULT_TIBIA)
dxTR = stru(); dxTR.name = "Dx Rear Tibia"; dxTR.pin = DxR_tibia; dxTR.snum = 11; dxTR.angle = angleConversion(DX_DEFAULT_TIBIA)
dxFF = stru(); dxFF.name = "Dx Front Femur"; dxFF.pin = DxF_femur; dxFF.snum = 12; dxFF.angle = angleConversion(DX_DEFAULT_FEMUR)
dxFM = stru(); dxFM.name = "Dx Middle Femur"; dxFM.pin = DxM_femur; dxFM.snum = 13; dxFM.angle = angleConversion(DX_DEFAULT_FEMUR)
dxFR = stru(); dxFR.name = "Dx Rear Femur"; dxFR.pin = DxR_femur; dxFR.snum = 14; dxFR.angle = angleConversion(DX_DEFAULT_FEMUR)
dxSF = stru(); dxSF.name = "Dx Front Shoulder"; dxSF.pin = DxF_shoulder; dxSF.snum = 15; dxSF.angle = angleConversion(DX_DEFAULT_SHOULDER)
dxSM = stru(); dxSM.name = "Dx Middle Shoulder"; dxSM.pin = DxM_shoulder; dxSM.snum = 16; dxSM.angle = angleConversion(DX_DEFAULT_SHOULDER)
dxSR = stru(); dxSR.name = "Dx Rear Shoulder"; dxSR.pin = DxR_shoulder; dxSR.snum = 17; dxSR.angle = angleConversion(DX_DEFAULT_SHOULDER)

servoList = [sxTF,sxTM,sxTR,sxFF,sxFM,sxFR,sxSF,sxSM,sxSR,
	     	 dxTF,dxTM,dxTR,dxFF,dxFM,dxFR,dxSF,dxSM,dxSR]

#Global variable definition --------------------------------------------

#Global defines --------------------------------------------------------
LINEINFO_workingMode = getframeinfo(currentframe())
TIBIAS_DEBUG 	= 0 	#check working state of the motors
KEYBOARD 		= 1 	#set manually an angle of a motor
WORKING 		= 0		#working mode

WORKING_MODES = {TIBIAS_DEBUG, KEYBOARD, WORKING}

VELOCITY			= 1		#set a non default velocity for the motors movements
VELOCITY_WAIT 		= 0.1	#set a default value for sleep time in velocity alghoritm
VELOCITY_DEFAULT 	= 1		#value of default velocity of the servos

#Main ------------------------------------------------------------------
if __name__ == '__main__':
	properWorkingMode()
	servoInit()

	if TIBIAS_DEBUG:
		motion = 1 #used to change the movement direction of the tibias
		while 1:	
			if motion == 1:
				sxTF.angle = sxTF.angle + angleConversion(20)
				sxTM.angle = sxTM.angle + angleConversion(20)
				sxTR.angle = sxTR.angle + angleConversion(20)
				dxTF.angle = dxTF.angle - angleConversion(20)
				dxTM.angle = dxTM.angle - angleConversion(20)
				dxTR.angle = dxTR.angle - angleConversion(20)
				motion = 0
			else:
				sxTF.angle = sxTF.angle - angleConversion(20)
				sxTM.angle = sxTM.angle - angleConversion(20)
				sxTR.angle = sxTR.angle - angleConversion(20)
				dxTF.angle = dxTF.angle + angleConversion(20)
				dxTM.angle = dxTM.angle + angleConversion(20)
				dxTR.angle = dxTR.angle + angleConversion(20)
				motion = 1
			
			echoAngle(sxTF,sxTF.angle,VELOCITY_DEFAULT)
			echoAngle(sxTM,sxTM.angle,VELOCITY_DEFAULT)
			echoAngle(sxTR,sxTR.angle,VELOCITY_DEFAULT)
			echoAngle(dxTF,dxTF.angle,VELOCITY_DEFAULT)
			echoAngle(dxTM,dxTM.angle,VELOCITY_DEFAULT)
			echoAngle(dxTR,dxTR.angle,VELOCITY_DEFAULT)
			sleep(1)

	elif KEYBOARD:
		infoTable()
		while 1:
			newPin = input("\nWhich motor you wanna move? ")
			if 0<=int(newPin)<18:
				newAngle = input("Which angle do you wanna try? [def as default]: ")
				if newAngle == "def":
					newAngle = defaultValue(int(newPin))
				if VELOCITY:
					velocity = int(input("Set the velocity [1 as default]: "))
				else: 
					velocity = 1
				for k in servoList:
					if k.snum == int(newPin):
						if echoAngle(k,angleConversion(newAngle),velocity): #proceed to update angle value only if the movement is completed
							k.angle = int(newAngle)
							print("Changing " + k.name + " angle into " + str(k.angle) + " degrees")
						break
			else: 
				print("\nServo not found. The only avaible servos are in the following.")
				infoTable()
		
			sleep(1)
		

	elif WORKING:
		print("Welcome user, BulbaBot 2.0 is awake, and ready to follow your instructions.\n")
