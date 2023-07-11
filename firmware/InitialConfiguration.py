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
def echoAngle(localServo):
	try:
		angleCheck(localServo.name[1],localServo.name[3],localServo.angle)
		localServo.angle = angleConversion(localServo.angle)
		#print(str(localServo.angle))
		cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle 
		system(cmd)
	except Exception as err:
		print(err)
		
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
		echoAngle(i) #setting default position

#Struct define and init ------------------------------------------------
#SX Legs
sxTF = stru(); sxTF.name = "Sx Front Tibia"; sxTF.pin = SxF_tibia; sxTF.snum = 0; sxTF.angle = SX_DEFAULT_TIBIA
sxTM = stru(); sxTM.name = "Sx Middle Tibia"; sxTM.pin = SxM_tibia; sxTM.snum = 1; sxTM.angle = SX_DEFAULT_TIBIA
sxTR = stru(); sxTR.name = "Sx Rear Tibia"; sxTR.pin = SxR_tibia; sxTR.snum = 2; sxTR.angle = SX_DEFAULT_TIBIA
sxFF = stru(); sxFF.name = "Sx Front Femur"; sxFF.pin = SxF_femur; sxFF.snum = 3; sxFF.angle = SX_DEFAULT_FEMUR
sxFM = stru(); sxFM.name = "Sx Middle Femur"; sxFM.pin = SxM_femur; sxFM.snum = 4; sxFM.angle = SX_DEFAULT_FEMUR
sxFR = stru(); sxFR.name = "Sx Rear Femur"; sxFR.pin = SxR_femur; sxFR.snum = 5; sxFR.angle = SX_DEFAULT_FEMUR
sxSF = stru(); sxSF.name = "Sx Front Shoulder"; sxSF.pin = SxF_shoulder; sxSF.snum = 6; sxSF.angle = SX_DEFAULT_SHOULDER
sxSM = stru(); sxSM.name = "Sx Middle Shoulder"; sxSM.pin = SxM_shoulder; sxSM.snum = 7; sxSM.angle = SX_DEFAULT_SHOULDER
sxSR = stru(); sxSR.name = "Sx Rear Shoulder"; sxSR.pin = SxR_shoulder; sxSR.snum = 8; sxSR.angle = SX_DEFAULT_SHOULDER

#DX Legs
dxTF = stru(); dxTF.name = "Dx Front Tibia"; dxTF.pin = DxF_tibia; dxTF.snum = 9; dxTF.angle = DX_DEFAULT_TIBIA
dxTM = stru(); dxTM.name = "Dx Middle Tibia"; dxTM.pin = DxM_tibia; dxTM.snum = 10; dxTM.angle = DX_DEFAULT_TIBIA
dxTR = stru(); dxTR.name = "Dx Rear Tibia"; dxTR.pin = DxR_tibia; dxTR.snum = 11; dxTR.angle = DX_DEFAULT_TIBIA
dxFF = stru(); dxFF.name = "Dx Front Femur"; dxFF.pin = DxF_femur; dxFF.snum = 12; dxFF.angle = DX_DEFAULT_FEMUR
dxFM = stru(); dxFM.name = "Dx Middle Femur"; dxFM.pin = DxM_femur; dxFM.snum = 13; dxFM.angle = DX_DEFAULT_FEMUR
dxFR = stru(); dxFR.name = "Dx Rear Femur"; dxFR.pin = DxR_femur; dxFR.snum = 14; dxFR.angle = DX_DEFAULT_FEMUR
dxSF = stru(); dxSF.name = "Dx Front Shoulder"; dxSF.pin = DxF_shoulder; dxSF.snum = 15; dxSF.angle = DX_DEFAULT_SHOULDER
dxSM = stru(); dxSM.name = "Dx Middle Shoulder"; dxSM.pin = DxM_shoulder; dxSM.snum = 16; dxSM.angle = DX_DEFAULT_SHOULDER
dxSR = stru(); dxSR.name = "Dx Rear Shoulder"; dxSR.pin = DxR_shoulder; dxSR.snum = 17; dxSR.angle = DX_DEFAULT_SHOULDER

servoList = [sxTF,sxTM,sxTR,sxFF,sxFM,sxFR,sxSF,sxSM,sxSR,
	     	 dxTF,dxTM,dxTR,dxFF,dxFM,dxFR,dxSF,dxSM,dxSR]

#Global variable definition --------------------------------------------

#Global defines --------------------------------------------------------
LINEINFO_workingMode = getframeinfo(currentframe())
TIBIAS_DEBUG 	= 0 	#check working state of the motors
KEYBOARD 		= 1 	#set manually an angle of a motor
WORKING 		= 0		#working mode

WORKIN_MODES = {TIBIAS_DEBUG, KEYBOARD, WORKING}

#Main ------------------------------------------------------------------
if __name__ == '__main__':
	properWorkingMode()
	servoInit()

	if TIBIAS_DEBUG:
		motion = 1 #used to change the movement direction of the tibias
		while 1:	
			if motion == 1:
				sxTF.angle = sxTF.angle + 20
				sxTM.angle = sxTM.angle + 20
				sxTR.angle = sxTR.angle + 20
				dxTF.angle = dxTF.angle - 20
				dxTM.angle = dxTM.angle - 20
				dxTR.angle = dxTR.angle - 20
				motion = 0
			else:
				sxTF.angle = sxTF.angle - 20
				sxTM.angle = sxTM.angle - 20
				sxTR.angle = sxTR.angle - 20
				dxTF.angle = dxTF.angle + 20
				dxTM.angle = dxTM.angle + 20
				dxTR.angle = dxTR.angle + 20
				motion = 1
			
			echoAngle(sxTF)
			echoAngle(sxTM)
			echoAngle(sxTR)
			echoAngle(dxTF)
			echoAngle(dxTM)
			echoAngle(dxTR)
			sleep(1)

	elif KEYBOARD:
		infoTable()
		while 1:
			newPin = int(input("\nWhich motor you wanna move? "))
			if newPin>=0 and newPin<18:
				newAngle = input("Which angle do you wanna try? ")	
				for k in servoList:
					if k.snum == int(newPin):
						k.angle = newAngle
						print("Changing " + k.name + " angle into " + str(k.angle) + " degrees")
						echoAngle(k)
						break
			else: 
				print("\nServo not found. The only avaible servos are in the following.")
				infoTable()
		
			sleep(1)

	elif WORKING:
		print("Welcome user, BulbaBot 2.0 is awake, and ready to follow your instructions.\n")
