import time
from os import system

from LegsPinConfiguration import *

def angleConversion(alpha):
	#map from 0-180 grades to 50-250 time period
	return round(alpha*10/9+50)

#NameConfiguration: Servo _ Sx/Dx F(Front)/M(Middle)/R(Rear) S(Shoulder)/F(Femur)/T(Tibia)
#Set default angles
Servo_SxFS = angleConversion(90)
Servo_SxFF = angleConversion(90)
Servo_SxFT = angleConversion(90)
Servo_SxMS = angleConversion(90)
Servo_SxMF = angleConversion(90)
Servo_SxMT = angleConversion(90)
Servo_SxRS = angleConversion(90)
Servo_SxRF = angleConversion(90)
Servo_SxRT = angleConversion(90)
Servo_DxFS = angleConversion(90)
Servo_DxFF = angleConversion(90)
Servo_DxFT = angleConversion(90)
Servo_DxMS = angleConversion(90)
Servo_DxMF = angleConversion(90)
Servo_DxMT = angleConversion(90)
Servo_DxRS = angleConversion(90)
Servo_DxRF = angleConversion(90)
Servo_DxRT = angleConversion(90)

#Control group defining
DxShoulders = {Servo_DxFS, Servo_DxMS, Servo_DxRS}
DxFemurs = {Servo_DxFF, Servo_DxMF, Servo_DxRF}
DxTibias = {Servo_DxFT, Servo_DxMT, Servo_DxRT}
SxShoulders = {Servo_SxFS, Servo_SxMS, Servo_SxRS}
SxFemurs = {Servo_SxFF, Servo_SxMF, Servo_SxRF}
SxTibias = {Servo_SxFT, Servo_SxMT, Servo_SxRT}

time.sleep(1)

angle = 90
for DxS in DxShoulders:
	DxS.ChangeDutyCycle(round(17*angle/180))
for SxS in SxShoulders:
	SxS.ChangeDutyCycle(round(17*angle/180))

angle = 100
for DxF in DxFemurs:
	DxF.ChangeDutyCycle(round(17*angle/180))

angle = 80
for SxF in SxFemurs:
	SxF.ChangeDutyCycle(round(17*angle/180))

duty = 1
while duty <= 17:
	for DxT in DxTibias:
		DxT.ChangeDutyCycle(duty)
	for SxT in SxTibias:
		SxT.ChangeDutyCycle(17-duty)
	
	time.sleep(0.5)
	duty = duty + 1
	if duty == 10: #stop before ending of the full travel
		duty = 1 

