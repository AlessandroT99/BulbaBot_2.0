#!/usr/bin/env python3

from time import sleep
from os import system

import LegsParameters

# Global defines --------------------------------------------------------
VELOCITY			= 1		# Set a non default velocity for the motors movements
VELOCITY_WAIT 		= 0.1	# Set a default value for sleep time in velocity alghoritm
VELOCITY_DEFAULT 	= 1		# Value of default velocity of the servos

# Class and functions definition ---------------------------------------- 
class stru:
	def __init__(self):
		self.pin = 0
		self.snum = 0
		self.angle = 0
		self.name = ""

def angleConversion(alpha):
	"""
    angleConversion()
    -------------------
    Maps from 0-180 grades to 50-250 time period.
    
    ### INPUTS
    * `alpha`: angle in degrees.
    ### OUTPUTS
    * `processedValue`: angle value properly setted for servoBlaster commands.
    """

	if int(alpha)>180 or int(alpha)<0:
		raise TypeError("Only degrees values between 0 and 180 are allowed")
	processedValue = round(float(alpha)*10/9+50)
	return processedValue 

def echoAngle(localServo,newAngle,velocity=VELOCITY_DEFAULT):
	"""
    echoAngle()
    -------------------
    Check correctness of the angle and the publish it to the servoBlaster manager.
    
    ### INPUTS
    * `localServo`: the data of the selected servo as struct `stru()`. 
    * `newAngle`: the angle to reach from the servo.
    * `velocity`: the velocity of the movements.
    ### OUTPUTS
    * [not explicit] 1 or 0: results of the function.
    1 if succesfull, 0 if failed. 
    """

	#FIXME: the velocity control does not work as expected
	#  		the servos lags too much

	try:
		localServoName = localServo.name.split()
		LegsParameters.angleCheck(localServoName[0],localServoName[2],newAngle)
		if velocity > VELOCITY_DEFAULT: # If velocity is setted
			velocityExecution(localServo,newAngle,velocity)
		elif velocity < VELOCITY_DEFAULT: # Velocity input error
			raise ValueError("Velocity cannot be negative.")
		else: # Default velocity as fast as possibile
			localServo.angle = newAngle
			cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle 
			system(cmd)
		return 1 # Operation goes in the right way
	except Exception as err:
		print(err)
		return 0 # An error as been found
	
def velocityExecution(localServo,newAngle,velocity):
	"""
    velocityExecution()
    -------------------
    Varies the velocity of the movement if a different value 
	from VELOCITY_DEFAULT is chosen.
    
    ### INPUTS
    * `localServo`: the data of the selected servo as struct `stru()`. 
    * `newAngle`: the angle to reach from the servo.
    * `velocity`: the velocity of the movements.
    ### OUTPUTS
    * none.
    """

	# Divide path space into "velocity" steps
	frameSteps = abs(round((newAngle-localServo.angle)/velocity))

	# If the new angle is higher it has to be done a summation 
	if localServo.angle <= newAngle:  
		while localServo.angle != newAngle: # Repeat until the steps are over
			localServo.angle += frameSteps
			if localServo.angle > newAngle: 
				localServo.angle = newAngle
			cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle
			system(cmd)
			sleep(VELOCITY_WAIT)
	else: # If the new angle is lower it has to be done a substraction
		while localServo.angle != newAngle: # Repeat until the steps are over
			localServo.angle -= frameSteps
			if localServo.angle < newAngle: 
				localServo.angle = newAngle
			cmd = "echo %s" %localServo.snum + "=%s > /dev/servoblaster" %localServo.angle
			system(cmd)
			sleep(VELOCITY_WAIT)
		
def infoTable():
	"""
    ()
    -------------------
    Print a table which contains all the servos and affiliated infos.
    
    ### INPUTS
    * none.
    ### OUTPUTS
    * none.
    """

	print("\n")
	print(' List of all the servos '.center(44,'-'))
	print("{:<14} {:<22} {:<8}".format('Servo number','Servo Name','HW Pin'))
	for k in servoList:
		print("{:<14} {:<22} {:<8}".format(k.snum,k.name,k.pin))
	print(''.center(44,'-'))

def servoInit():
	"""
    ()
    -------------------
	Initialization of the servos pins and setting the default positions.
    
    ### INPUTS
    * none.
    ### OUTPUTS
    * none.
    """

	servoString = ""
	for i in servoList:
		servoString = servoString + str(i.pin) + ","
	servoStart = 'echo "ubuntu" | sudo -S /home/ubuntu/servoblaster/servod --p1pins="%s" --pcm &' %servoString[:-1]
	system(servoStart)
	cmd = 'echo "ubuntu" | sudo -S chmod a=rwx /dev/servoblaster'
	system(cmd)
	sleep(2)
	for i in servoList:
		echoAngle(i,i.angle) # Setting default position

def servoFinding(servoSnum):
	"""
    servoFinding()
    -------------------
    Finds the number of the servos in the servoblaster list.
    
    ### INPUTS
    * `servoSnum`: number of the servo to check.
    ### OUTPUTS
    * [not explicit] `i`:returning the servos servoBlaster identification number 
	if exist.
    """

	for i in servoList:
		if i.snum == servoSnum:
			return i
	raise ValueError("Error in servoFinding function [ServosConfiguration.py] - The requested servo does not exist")

# Struct define and init ------------------------------------------------
# SX Legs
sxTF = stru(); sxTF.name = "Sx Front Tibia"; sxTF.pin = LegsParameters.SxF_tibia
sxTF.snum = 0; sxTF.angle = angleConversion(LegsParameters.SX_DEFAULT_TIBIA)
sxTM = stru(); sxTM.name = "Sx Middle Tibia"; sxTM.pin = LegsParameters.SxM_tibia
sxTM.snum = 1; sxTM.angle = angleConversion(LegsParameters.SX_DEFAULT_TIBIA)
sxTR = stru(); sxTR.name = "Sx Rear Tibia"; sxTR.pin = LegsParameters.SxR_tibia
sxTR.snum = 2; sxTR.angle = angleConversion(LegsParameters.SX_DEFAULT_TIBIA)
sxFF = stru(); sxFF.name = "Sx Front Femur"; sxFF.pin = LegsParameters.SxF_femur
sxFF.snum = 3; sxFF.angle = angleConversion(LegsParameters.SX_DEFAULT_FEMUR)
sxFM = stru(); sxFM.name = "Sx Middle Femur"; sxFM.pin = LegsParameters.SxM_femur
sxFM.snum = 4; sxFM.angle = angleConversion(LegsParameters.SX_DEFAULT_FEMUR)
sxFR = stru(); sxFR.name = "Sx Rear Femur"; sxFR.pin = LegsParameters.SxR_femur
sxFR.snum = 5; sxFR.angle = angleConversion(LegsParameters.SX_DEFAULT_FEMUR)
sxSF = stru(); sxSF.name = "Sx Front Shoulder"; sxSF.pin = LegsParameters.SxF_shoulder
sxSF.snum = 6; sxSF.angle = angleConversion(LegsParameters.SX_DEFAULT_SHOULDER)
sxSM = stru(); sxSM.name = "Sx Middle Shoulder"; sxSM.pin = LegsParameters.SxM_shoulder
sxSM.snum = 7; sxSM.angle = angleConversion(LegsParameters.SX_DEFAULT_SHOULDER)
sxSR = stru(); sxSR.name = "Sx Rear Shoulder"; sxSR.pin = LegsParameters.SxR_shoulder
sxSR.snum = 8; sxSR.angle = angleConversion(LegsParameters.SX_DEFAULT_SHOULDER)

# DX Legs
dxTF = stru(); dxTF.name = "Dx Front Tibia"; dxTF.pin = LegsParameters.DxF_tibia 
dxTF.snum = 9; dxTF.angle = angleConversion(LegsParameters.DX_DEFAULT_TIBIA)
dxTM = stru(); dxTM.name = "Dx Middle Tibia"; dxTM.pin = LegsParameters.DxM_tibia
dxTM.snum = 10; dxTM.angle = angleConversion(LegsParameters.DX_DEFAULT_TIBIA)
dxTR = stru(); dxTR.name = "Dx Rear Tibia"; dxTR.pin = LegsParameters.DxR_tibia
dxTR.snum = 11; dxTR.angle = angleConversion(LegsParameters.DX_DEFAULT_TIBIA)
dxFF = stru(); dxFF.name = "Dx Front Femur"; dxFF.pin = LegsParameters.DxF_femur
dxFF.snum = 12; dxFF.angle = angleConversion(LegsParameters.DX_DEFAULT_FEMUR)
dxFM = stru(); dxFM.name = "Dx Middle Femur"; dxFM.pin = LegsParameters.DxM_femur
dxFM.snum = 13; dxFM.angle = angleConversion(LegsParameters.DX_DEFAULT_FEMUR)
dxFR = stru(); dxFR.name = "Dx Rear Femur"; dxFR.pin = LegsParameters.DxR_femur
dxFR.snum = 14; dxFR.angle = angleConversion(LegsParameters.DX_DEFAULT_FEMUR)
dxSF = stru(); dxSF.name = "Dx Front Shoulder"; dxSF.pin = LegsParameters.DxF_shoulder
dxSF.snum = 15; dxSF.angle = angleConversion(LegsParameters.DX_DEFAULT_SHOULDER)
dxSM = stru(); dxSM.name = "Dx Middle Shoulder"; dxSM.pin = LegsParameters.DxM_shoulder 
dxSM.snum = 16; dxSM.angle = angleConversion(LegsParameters.DX_DEFAULT_SHOULDER)
dxSR = stru(); dxSR.name = "Dx Rear Shoulder"; dxSR.pin = LegsParameters.DxR_shoulder 
dxSR.snum = 17; dxSR.angle = angleConversion(LegsParameters.DX_DEFAULT_SHOULDER)

servoList = [sxTF,sxTM,sxTR,sxFF,sxFM,sxFR,sxSF,sxSM,sxSR,
	     	 dxTF,dxTM,dxTR,dxFF,dxFM,dxFR,dxSF,dxSM,dxSR]

Leg_sxF = [sxSF,sxFF,sxTF]
Leg_sxM = [sxSM,sxFM,sxTM]
Leg_sxR = [sxSR,sxFR,sxTR]
Side_sx = [Leg_sxF,Leg_sxM,Leg_sxR]

Leg_dxF = [dxSF,dxFF,dxTF]
Leg_dxM = [dxSM,dxFM,dxTM]
Leg_dxR = [dxSR,dxFR,dxTR]
Side_dx = [Leg_dxF,Leg_dxM,Leg_dxR]
