import RPi.GPIO as GPIO
import time

from LegsPinConfiguration import *

GPIO.setmode(GPIO.BCM)

GPIO.setup(SxF_shoulder,GPIO.OUT)
GPIO.setup(SxF_femur,GPIO.OUT)
GPIO.setup(SxF_tibia,GPIO.OUT)

GPIO.setup(SxM_shoulder,GPIO.OUT)
GPIO.setup(SxM_femur,GPIO.OUT)
GPIO.setup(SxM_tibia,GPIO.OUT)

GPIO.setup(SxR_shoulder,GPIO.OUT)
GPIO.setup(SxR_femur,GPIO.OUT)
GPIO.setup(SxR_tibia,GPIO.OUT)

GPIO.setup(DxF_shoulder,GPIO.OUT)
GPIO.setup(DxF_femur,GPIO.OUT)
GPIO.setup(DxF_tibia,GPIO.OUT)

GPIO.setup(DxM_shoulder,GPIO.OUT)
GPIO.setup(DxM_femur,GPIO.OUT)
GPIO.setup(DxM_tibia,GPIO.OUT)

GPIO.setup(DxR_shoulder,GPIO.OUT)
GPIO.setup(DxR_femur,GPIO.OUT)
GPIO.setup(DxR_tibia,GPIO.OUT)

#NameConfiguration: Servo _ Sx/Dx F(Front)/M(Middle)/R(Rear) S(Shoulder)/F(Femur)/T(Tibia)
frequency = 49
Servo_SxFS = GPIO.PWM(SxF_shoulder,frequency)
Servo_SxFF = GPIO.PWM(SxF_femur,frequency)
Servo_SxFT = GPIO.PWM(SxF_tibia,frequency)
Servo_SxMS = GPIO.PWM(SxM_shoulder,frequency)
Servo_SxMF = GPIO.PWM(SxM_femur,frequency)
Servo_SxMT = GPIO.PWM(SxM_tibia,frequency)
Servo_SxRS = GPIO.PWM(SxR_shoulder,frequency)
Servo_SxRF = GPIO.PWM(SxR_femur,frequency)
Servo_SxRT = GPIO.PWM(SxR_tibia,frequency)
Servo_DxFS = GPIO.PWM(DxF_shoulder,frequency)
Servo_DxFF = GPIO.PWM(DxF_femur,frequency)
Servo_DxFT = GPIO.PWM(DxF_tibia,frequency)
Servo_DxMS = GPIO.PWM(DxM_shoulder,frequency)
Servo_DxMF = GPIO.PWM(DxM_femur,frequency)
Servo_DxMT = GPIO.PWM(DxM_tibia,frequency)
Servo_DxRS = GPIO.PWM(DxR_shoulder,frequency)
Servo_DxRF = GPIO.PWM(DxR_femur,frequency)
Servo_DxRT = GPIO.PWM(DxR_tibia,frequency)

#Start all servos PWM
Servo_SxFS.start(0)
Servo_SxFF.start(0)
Servo_SxFT.start(0)
Servo_SxMS.start(0)
Servo_SxMF.start(0)
Servo_SxMT.start(0)
Servo_SxRS.start(0)
Servo_SxRF.start(0)
Servo_SxRT.start(0)
Servo_DxFS.start(0)
Servo_DxFF.start(0)
Servo_DxFT.start(0)
Servo_DxMS.start(0)
Servo_DxMF.start(0)
Servo_DxMT.start(0)
Servo_DxRS.start(0)
Servo_DxRF.start(0)
Servo_DxRT.start(0)

#Control's group defining
DxShoulders 	= {Servo_DxFS, Servo_DxMS, Servo_DxRS}
DxFemurs 	= {Servo_DxFF, Servo_DxMF, Servo_DxRF}
DxTibias 	= {Servo_DxFT, Servo_DxMT, Servo_DxRT}
SxShoulders 	= {Servo_SxFS, Servo_SxMS, Servo_SxRS}
SxFemurs	= {Servo_SxFF, Servo_SxMF, Servo_SxRF}
SxTibias 	= {Servo_SxFT, Servo_SxMT, Servo_SxRT}


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

