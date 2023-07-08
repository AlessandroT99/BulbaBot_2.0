#!/usr/bin/env python3.6

from time import sleep
from os import system

from LegsPinConfiguration import *

#Class and functions definition ---------------------------------------- 
class stru:
	def __init__(self):
		self.pin = 0
		self.snum = 0
		self.angle = 0

def echoAngle(servoNum,angle):
	cmd = "echo %s" %servoNum + "=%s > /dev/servoblaster" %angle 
	system(cmd)

#Struct define and init ------------------------------------------------
#SX Legs
sxTF = stru(); sxTF.pin = SxF_tibia; sxTF.snum = 0; sxTF.angle = 90
sxTM = stru(); sxTM.pin = SxM_tibia; sxTM.snum = 1; sxTM.angle = 90
sxTR = stru(); sxTR.pin = SxR_tibia; sxTR.snum = 2; sxTR.angle = 90
sxFF = stru(); sxFF.pin = SxF_femur; sxFF.snum = 3; sxFF.angle = 160
sxFM = stru(); sxFM.pin = SxM_femur; sxFM.snum = 4; sxFM.angle = 160
sxFR = stru(); sxFR.pin = SxR_femur; sxFR.snum = 5; sxFR.angle = 160
sxSF = stru(); sxSF.pin = SxF_shoulder; sxSF.snum = 6; sxSF.angle = 90
sxSM = stru(); sxSM.pin = SxM_shoulder; sxSM.snum = 7; sxSM.angle = 90
sxSR = stru(); sxSR.pin = SxR_shoulder; sxSR.snum = 8; sxSR.angle = 90

#DX Legs
dxTF = stru(); dxTF.pin = DxF_tibia; dxTF.snum = 9; dxTF.angle = 90
dxTM = stru(); dxTM.pin = DxM_tibia; dxTM.snum = 10; dxTM.angle = 90
dxTR = stru(); dxTR.pin = DxR_tibia; dxTR.snum = 11; dxTR.angle = 90
dxFF = stru(); dxFF.pin = DxF_femur; dxFF.snum = 12; dxFF.angle = 160
dxFM = stru(); dxFM.pin = DxM_femur; dxFM.snum = 13; dxFM.angle = 160
dxFR = stru(); dxFR.pin = DxR_femur; dxFR.snum = 14; dxFR.angle = 160
dxSF = stru(); dxSF.pin = DxF_shoulder; dxSF.snum = 15; dxSF.angle = 90
dxSM = stru(); dxSM.pin = DxM_shoulder; dxSM.snum = 16; dxSM.angle = 90
dxSR = stru(); dxSR.pin = DxR_shoulder; dxSR.snum = 17; dxSR.angle = 90

servoList = [sxTF,sxTM,sxTR,sxFF,sxFM,sxFR,sxSF,sxSM,sxSR,
	     	 dxTF,dxTM,dxTR,dxFF,dxFM,dxFR,dxSF,dxSM,dxSR]
servoString = ""
for i in servoList:
	servoString = servoString + str(i.pin) + ","
	echoAngle(i.snum,i.angle) #setting default position
servoInit = '/home/ubuntu/servoblaster/servod --p1pins="%s" --pcm &' %servoString[:-1]
system(servoInit)

sleep(1)

#Global variable definition --------------------------------------------
motion = 1

#Global defines --------------------------------------------------------
DEBUG = 1

#Main ------------------------------------------------------------------
while 1:
	if __name__ == '__main__':
		if DEBUG:
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
			
			echoAngle(sxTF.snum,sxTF.angle)
			echoAngle(sxTM.snum,sxTM.angle)
			echoAngle(sxTR.snum,sxTR.angle)
			echoAngle(dxTF.snum,dxTF.angle)
			echoAngle(dxTM.snum,dxTM.angle)
			echoAngle(dxTR.snum,dxTR.angle)
			sleep(1)
	
