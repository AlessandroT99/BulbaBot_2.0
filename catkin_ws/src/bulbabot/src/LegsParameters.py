#!/usr/bin/env python3

# Pins set as Pin Value #GPIO Value -------------------------------------
# Pins number defined through the following code
# Sx/Dx F(Front)/M(Middle)/R(Rear) _ shoulder/femur/tibia
SxF_shoulder 	= 3     # 2
SxF_femur 	    = 8     # 14
SxF_tibia 	    = 10    # 15
 
SxM_shoulder	= 16    # 18
SxM_femur	    = 18    # 24
SxM_tibia 	    = 24    # 8

SxR_shoulder 	= 35    # 19
SxR_femur	    = 36    # 16
SxR_tibia 	    = 37    # 26

DxF_shoulder 	= 5     # 3
DxF_femur	    = 7     # 4
DxF_tibia   	= 11    # 17

DxM_shoulder  	= 13    # 27
DxM_femur	    = 15    # 22
DxM_tibia 	    = 19    # 10

DxR_shoulder 	= 21    # 9
DxR_femur	    = 23    # 11
DxR_tibia 	    = 29    # 5

# Legs admitted angles --------------------------------------------------
SX_MAX_TIBIA        = 0     # Raise
SX_MIN_TIBIA        = 180   # Lowest position
SX_DEFAULT_TIBIA    = 150   # Default angle

SX_MAX_FEMUR        = 50    # Raise
SX_MIN_FEMUR        = 180   # Lowest position
SX_DEFAULT_FEMUR    = 70    # Default angle

SX_MAX_SHOULDER     = 180   # Towards the head
SX_MIN_SHOULDER     = 50    # Behind the head
SX_DEFAULT_SHOULDER = 110   # Default angle

DX_MAX_TIBIA        = 180   # Raise
DX_MIN_TIBIA        = 0     # Lowest position
DX_DEFAULT_TIBIA    = 30    # Default angle

DX_MAX_FEMUR        = 130   # Raise
DX_MIN_FEMUR        = 0     # Lowest position
DX_DEFAULT_FEMUR    = 110   # Default angle

DX_MAX_SHOULDER     = 40    # Towards the head
DX_MIN_SHOULDER     = 170   # Behind the head
DX_DEFAULT_SHOULDER = 120   # Default angle

def angleCheck(side, part, angle):
    """
    angleCheck()
    -------------------
    Checks the constraints fullfillness for the angle requested.
    
    ### INPUTS
    * `side`: leg's side of the robot.
    * `part`: part of the leg of the robot.
    * `angle`: angle to reach for the servo.
    ### OUTPUTS
    * none.
    """

    if side=="Sx":
        if part=="Shoulder":
            if SX_MIN_SHOULDER>angle or SX_MAX_SHOULDER<angle: 
                TypeError("The angle has to be between " + str(SX_MIN_SHOULDER) + " and " + str(SX_MAX_SHOULDER))
        elif part=="Femur":
            if SX_MIN_FEMUR<angle or SX_MAX_FEMUR>angle: 
                TypeError("The angle has to be between " + str(SX_MIN_FEMUR) + " and " + str(SX_MAX_FEMUR))
        elif part=="Tibia":
            if SX_MIN_TIBIA<angle or SX_MAX_TIBIA>angle: 
                TypeError("The angle has to be between " + str(SX_MIN_TIBIA) + " and " + str(SX_MAX_TIBIA))
        else:
            raise ValueError("Part not recognised - Error in InitialConfiguration.py - def echoAngle()")
    elif side=="Dx":
        if part=="Shoulder":
            if DX_MIN_SHOULDER<angle or DX_MAX_SHOULDER>angle: 
                TypeError("The angle has to be between " + str(DX_MIN_SHOULDER) + " and " + str(DX_MAX_SHOULDER))
        elif part=="Femur":
            if DX_MIN_FEMUR>angle or DX_MAX_FEMUR<angle: 
                TypeError("The angle has to be between " + str(DX_MIN_FEMUR) + " and " + str(DX_MAX_FEMUR))
        elif part=="Tibia":
            if DX_MIN_TIBIA>angle or DX_MAX_TIBIA<angle: 
                TypeError("The angle has to be between " + str(DX_MIN_TIBIA) + " and " + str(DX_MAX_TIBIA))
        else:
            raise ValueError("Part not recognised - Error in InitialConfiguration.py - def echoAngle()")
    else:
        raise ValueError("Side not recognised - Error in InitialConfiguration.py - def echoAngle()")

def defaultValue(pin):
    """
    defaultValue()
    -------------------
    Returns the default value of the leg part chosen.     
    
    ### INPUTS
    * `pin`: servoBlaster number of the chosen servo.
    ### OUTPUTS
    * [not explicit] defaultAngle: depending on the servo, it returns a different \
    default angle. 
    """

    if 0<= pin <=2:
        return SX_DEFAULT_TIBIA
    elif 3<= pin <=5:
        return SX_DEFAULT_FEMUR
    elif 6<= pin <=8:
        return SX_DEFAULT_SHOULDER
    elif 9<= pin <=11:
        return DX_DEFAULT_TIBIA
    elif 12<= pin <=14:
        return DX_DEFAULT_FEMUR
    elif 15<= pin <=17:
        return DX_DEFAULT_SHOULDER
    else:
        raise ValueError("Error in function defaultValue() in LegsParameters.py - pin value not valid")
    