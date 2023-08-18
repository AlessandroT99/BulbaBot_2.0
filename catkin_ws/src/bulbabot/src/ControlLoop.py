#!/usr/bin/env python3

import rospy
from bulbabot.msg import positionArray
from std_msgs.msg import UInt8
from math import cos, sin, pi, pow
from time import sleep

import CommonFeatures

#Class and functions definition ----------------------------------------
def q_dotGenerator(q,ke):
    """
    q_dotGenerator()
    -------------------
    From the position gives the angular velocity in output through 
    jacobian calculation. 

    ### INPUTS
    * `q`: the difference angle to make in order to reach the desidered
    position.
    * `ke`: the gain of the controller for the error.
    ### OUTPUTS
    * `q_dot`: the resulting angular velocities.
    """

    q1=q[0]
    q2=q[1]
    q3=q[2]
    
    # Jacobian computation
    J1 = [- (101*sin(q1))/2 - 160*sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - (710389336803076073*cos(q1)*sin(q2))/162259276829213363391578010288128 - (143*cos(q2)*sin(q1))/2 - 160*cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)), - 160*sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - (143*cos(q1)*sin(q2))/2 - (710389336803076073*cos(q2)*sin(q1))/162259276829213363391578010288128 - 160*cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064), - 160*sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - 160*cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064)]
    J2 = [  (101*cos(q1))/2 - 160*sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) + (143*cos(q1)*cos(q2))/2 - (710389336803076073*sin(q1)*sin(q2))/162259276829213363391578010288128 + 160*cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064),   (710389336803076073*cos(q1)*cos(q2))/162259276829213363391578010288128 - 160*sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - (143*sin(q1)*sin(q2))/2 + 160*cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)),   160*cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - 160*sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))]
    J3 = [                                                                                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                    (143*cos(q2))/2 + 160*cos(q2)*cos(q3) - 160*sin(q2)*sin(q3),                                                                                                                                                                   160*cos(q2)*cos(q3) - 160*sin(q2)*sin(q3)]
 
    # Evaluation of the angular velocity
    # q_dot = J'*ke*e
    q1_dot = J1[0]*ke[0]+J2[0]*ke[1]+J3[0]*ke[2]
    q2_dot = J1[1]*ke[0]+J2[1]*ke[1]+J3[1]*ke[2]
    q3_dot = J1[2]*ke[0]+J2[2]*ke[1]+J3[2]*ke[2]
    q_dot = [q1_dot,q2_dot,q3_dot]
    
    return q_dot

def forwardKinematics(q):
    """
    forwardKinematics()
    -------------------
    Applying forward kinematics theory the real angle is converted 
    into the real position coordinates.
    
    ### INPUTS
    * `q`: the real actual angle of the servos.
    ### OUTPUTS
    * [non explicit] `xe`: the resulting expected x position.
    * [non explicit] `ye`: the resulting expected y position.
    * [non explicit] `ze`: the resulting expected z position.
    """

    global xe, ye, ze
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    # Transformation matrix computation (last row has been omitted)
    T1 = [cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064), - sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064),                                           sin(q1), (101*cos(q1))/2 - 160*sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) + (143*cos(q1)*cos(q2))/2 - (710389336803076073*sin(q1)*sin(q2))/162259276829213363391578010288128 + 160*cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064)]
    T2 = [sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)),   cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)),                                          -cos(q1), (101*sin(q1))/2 + 160*sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + (710389336803076073*cos(q1)*sin(q2))/162259276829213363391578010288128 + (143*cos(q2)*sin(q1))/2 + 160*cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))]
    T3 = [                                                                                                                                                                cos(q2)*sin(q3) + cos(q3)*sin(q2),                                                                                                                                                                   cos(q2)*cos(q3) - sin(q2)*sin(q3), 4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                                                                                                    (143*sin(q2))/2 + 160*cos(q2)*sin(q3) + 160*cos(q3)*sin(q2)]
    
    # Evaluation of the expected coordinates
    xe = T1[3]
    ye = T2[3]
    ze = T3[3]

def errorInvestigation(q_ideal_deg):
    """
    errorInvestigation()
    -------------------
    This function compute the error from the ideal angle obtained 
    from Jacobian calculation.
    Procedure explained in details simulink simulation.
    
    ### INPUTS
    * `q_ideal_deg`: the ideal angle in degrees.
    ### OUTPUTS
    * `q_real_deg`: the resulting real angle in degrees.
    """

    q_real_deg = [0,0,0]
    for i in range(len(q_ideal_deg)):
        floatValue = q_ideal_deg[i]*10/9+50
        intValueNormalized = round(floatValue)/2000
        pwmPercentage = positiveValueFilter(intValueNormalized)
        meanVoltage = 3.3*pwmPercentage
        potenziometerValue = 0.0825+round(q_ideal_deg[i])*0.00183
        voltage_err = meanVoltage-potenziometerValue
        angle_err_deg = angleFinder(voltage_err)
        q_real_deg[i] = q_ideal_deg[i]-angle_err_deg

    return q_real_deg

def angleFinder(voltage_err):
    """
    angleFinder()
    -------------------
    The error in volts is converted into degrees.
    
    ### INPUTS
    * `voltage_err`: voltage error incoming from the difference
    of the mean PWM signal and the potenziometer read.
    ### OUTPUTS
    * `angle_err_degree`: the resulting expected error in degrees of the servos.
    """

    # map voltage error from 0/3.3 V to 0/20 ms 
    pwm_dc_err = voltage_err*0.02/3.3

    # map error from 0.5/2.5 ms to 0/180
    if abs(pwm_dc_err) < 0.0005: # mean that the error is less than 0°
        angle_err_deg = 0
    else:
        if pwm_dc_err >= 0:
            angle_err_deg = 90000*pwm_dc_err-45
        else:
            angle_err_deg = pwm_dc_err*90000+45

    return angle_err_deg

def positiveValueFilter(var):
    """
    positiveValueFilter()
    -------------------
    A function that let pass positive value and put equal to 0 negative ones.
    
    ### INPUTS
    * `var`: general numerical variable.
    ### OUTPUTS
    * `outVar`: the variable `var` filtered.
    """

    if var < 0:
        outVar = 0
    else:
        outVar = var
    
    return outVar
    
def executeLoop(rd):
    """
    executeLoop()
    -------------------
    Main program of the Control Loop executed as callback when an angle is published 
    from another ROS node.

    ### INPUTS
    * `rd`: the desidered coordinates of the selected end effector.
    ### OUTPUTS
    * none
    """

    global xG, yG, zG, xe, ye, ze, q, re, foundAngles
    xd = rd.x
    yd = rd.y
    zd = rd.z
    rospy.loginfo("Received position: [" + str(xd) + "," + str(yd) + "," + str(zd) + "]")
    
    # Store the servo numbers
    re.shoulderSnum = rd.shoulderSnum
    re.femurSnum = rd.femurSnum
    re.tibiaSnum = rd.tibiaSnum
    
    # Evaluate the coordinate error and multiply it to its gain
    coordinateErr = [xd-xe,yd-ye,zd-ze]
    while (abs(coordinateErr[0]) > ACCEPTED_ERROR and abs(coordinateErr[1]) > ACCEPTED_ERROR and abs(coordinateErr[2]) > ACCEPTED_ERROR):
        ke = [xG*coordinateErr[0],yG*coordinateErr[1],zG*coordinateErr[2]]
    
        # Evaluate q_dot
        q_dot = q_dotGenerator(q,ke)
        #rospy.loginfo("q_dot evaluated: [" + str(q_dot[0]) + "," + str(q_dot[1]) + "," + str(q_dot[2]) + "]")

        # Integrate q_dot to have q s.t. -pi/2 <= q <= pi/2
        q_rad_double = [0,0,0]
        for i in range(len(q_dot)):
            q_rad_double[i] = q[i] + q_dot[i]*integrationStep # integration
            if q_rad_double[i] < -pi*0.5: 
                q_rad_double[i] = -pi*0.5
            elif q_rad_double[i] > pi*0.5: 
                q_rad_double[i] = pi*0.5
        
        q = q_rad_double
        
        # Find the angle to be reached and publish it 
        re.x = int((q[0]+pi*0.5)*180/pi)
        re.y = int((q[1]+pi*0.5)*180/pi)
        re.z = int((q[2]+pi*0.5)*180/pi)
        rospy.loginfo("q evaluated: [" + str(re.x) + "," + str(re.y) + "," + str(re.z) + "]")

        # Find the value of the real reached angle in degrees
        q_real_deg = errorInvestigation([re.x,re.y,re.z])

        # Evaluate the reached angle in radiants
        q_real_rad = [0,0,0]
        for i in range(len(q_real_deg)):
            q_real_rad[i] = q_real_deg[i]*pi/180

        # Evalaute the real reached position
        forwardKinematics(q_real_rad)
        coordinateErr = [xd-xe,yd-ye,zd-ze]
        rospy.loginfo("Error residual: [" + str(coordinateErr[0]) + "," + str(coordinateErr[1]) + "," + str(coordinateErr[2]) + "]\n")
        sleep(1)

    # The final angle has been obtained with an accectable error
    anglePublisher.publish(re)

def connection1(data):
    """
    connection1()
    -------------------
    A function that answers to MainProgram.py publisher for successfull 
    initial connection.
    Is it runned only during the starting process.
    
    ### INPUTS
    * `data`: corresponding to the data imported from the ROS node.
    ### OUTPUTS
    * none
    """

    if data.data == CommonFeatures.MAIN_PROGRAM_ID:
        rospy.loginfo("Connected with Main Program succesfully")
        receiverConnection.publish(CommonFeatures.CONTROL_LOOP_ID)

def CLnode_init():
    """
    angleFinder()
    -------------------
    Initialize ROS node and subscribing nodes and callback functions.    
    
    In ROS, nodes are uniquely named. If two nodes with the same
    name are launched, the previous one is kicked off. The
    anonymous=True flag means that rospy will choose a unique
    name for our 'listener' node so that multiple listeners can
    run simultaneously.

    ### INPUTS
    * none
    ### OUTPUTS
    * none
    """
    
    rospy.init_node('Control_Loop', anonymous=False)
    rospy.Subscriber("Desidered_Position", positionArray, executeLoop)
    rospy.Subscriber("Transmit_Connection", UInt8, connection1)

# Struct define and init ------------------------------------------------

# Global variable definition --------------------------------------------
xG = 0.01 # Gain of x coordinate
yG = 0.01 # Gain of y coordinate
zG = 0.01 # Gain of z coordinate

xe = 0 # Value of real x coordinate
ye = 0 # Value of real x coordinate
ze = 0 # Value of real x coordinate

q = [0,0,0]

# Global defines --------------------------------------------------------
ACCEPTED_ERROR = pow(10,-5) # The error below which the control loop stops
re = positionArray()
integrationStep = 0.01

# Init ------------------------------------------------------------------
CLnode_init()
anglePublisher = rospy.Publisher("Final_Angle", positionArray, queue_size = 1)
receiverConnection = rospy.Publisher("Receive_Connection", UInt8, queue_size = 1)

# Main ------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS ERROR - Interrupt Exception occured in ControlLoop.py")
