#!/usr/bin/env python3

import scipy
import rospy
from std_msgs.msg import UInt16MultiArray
from math import cos, sin, pi, pow

#Class and functions definition ----------------------------------------
## DESCRIPTION: from the position gives the angular velocity in output through jacobian calculation 
def q_dotGenerator(q,ke):
    q1=q[1]
    q2=q[2]
    q3=q[3]
    
    #Jacobian computation
    J1 = [- (101*sin(q1))/2 - 160*sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - (710389336803076073*cos(q1)*sin(q2))/162259276829213363391578010288128 - (143*cos(q2)*sin(q1))/2 - 160*cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)), - 160*sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - (143*cos(q1)*sin(q2))/2 - (710389336803076073*cos(q2)*sin(q1))/162259276829213363391578010288128 - 160*cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064), - 160*sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - 160*cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064)]
    J2 = [  (101*cos(q1))/2 - 160*sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) + (143*cos(q1)*cos(q2))/2 - (710389336803076073*sin(q1)*sin(q2))/162259276829213363391578010288128 + 160*cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064),   (710389336803076073*cos(q1)*cos(q2))/162259276829213363391578010288128 - 160*sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - (143*sin(q1)*sin(q2))/2 + 160*cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)),   160*cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - 160*sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))]
    J3 = [                                                                                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                    (143*cos(q2))/2 + 160*cos(q2)*cos(q3) - 160*sin(q2)*sin(q3),                                                                                                                                                                   160*cos(q2)*cos(q3) - 160*sin(q2)*sin(q3)]
 
    #Evaluation of the angular velocity
    #q_dot = J'*ke*e
    q1_dot = J1[1]*ke[1]+J2[1]*ke[2]+J3[1]*ke[3]
    q2_dot = J1[2]*ke[1]+J2[2]*ke[2]+J3[2]*ke[3]
    q3_dot = J1[3]*ke[1]+J2[3]*ke[2]+J3[3]*ke[3]
    q_dot = [q1_dot,q2_dot,q3_dot]

    return q_dot

## DESCRIPTION: Applying forward kinematics theory the real angle is converted into the real position coordinates
def forwardKinematics(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]

    #Transformation matrix computation (last row has been omitted)
    T1 = [cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064), - sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064),                                           sin(q1), (101*cos(q1))/2 - 160*sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) + (143*cos(q1)*cos(q2))/2 - (710389336803076073*sin(q1)*sin(q2))/162259276829213363391578010288128 + 160*cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064)]
    T2 = [sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)),   cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) - sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)),                                          -cos(q1), (101*sin(q1))/2 + 160*sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + (710389336803076073*cos(q1)*sin(q2))/162259276829213363391578010288128 + (143*cos(q2)*sin(q1))/2 + 160*cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))]
    T3 = [                                                                                                                                                                cos(q2)*sin(q3) + cos(q3)*sin(q2),                                                                                                                                                                   cos(q2)*cos(q3) - sin(q2)*sin(q3), 4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                                                                                                    (143*sin(q2))/2 + 160*cos(q2)*sin(q3) + 160*cos(q3)*sin(q2)]
    
    #Evaluation of the expected coordinates
    xe = T1[3]
    ye = T2[3]
    ze = T3[3]

    re = [xe,ye,ze]

    return re

## DESCRIPTION: This function compute the error from the ideal angle obtained from Jacobian calculation
def errorInvestigation(q_ideal_deg):
    #Procedure explained in simulink simulation
    q_real_deg = [0,0,0]
    for i in len(q_ideal_deg):
        floatValue = q_ideal_deg[i]*10/9+50
        intValueNormalized = round(floatValue)/2000
        pwmPercentage = positiveValueFilter(intValueNormalized)
        meanVoltage = 3.3*pwmPercentage
        potenziometerValue = 0.0825+round(q_ideal_deg)*0.00183
        voltage_err = meanVoltage-potenziometerValue
        angle_err_deg = angleFinder(voltage_err)
        q_real_deg[i] = q_ideal_deg-angle_err_deg

    return q_real_deg

## DESCRIPTION: The error in volts is converted into degrees
def angleFinder(voltage_err,angle_err_deg):
    #map voltage error from 0/3.3 V to 0/20 ms 
    pwm_dc_err = voltage_err*0.02/3.3

    #map error from 0.5/2.5 ms to 0/180
    if abs(pwm_dc_err) < 0.0005: #mean that the error is less than 0°
        angle_err_deg = 0
    else:
        if pwm_dc_err >= 0:
            angle_err_deg = 90000*pwm_dc_err-45
        else:
            angle_err_deg = pwm_dc_err*90000+45

    return angle_err_deg

## DESCRIPTION: A function that let pass positive value and put equal to 0 negative ones
def positiveValueFilter(var):
    if var < 0:
        outVar = 0
    else:
        outVar = var
    
    return outVar
    
## DESCRIPTION: Main program of the Control Loop executed as callback when an angle is published from another ROS node
def executeLoop(rd):
    xd = rd[0]
    yd = rd[1]
    zd = rd[2]

    #Evaluate the coordinate error and multiply it to its gain
    coordinateErr = [xd-xe,yd-ye,zd-ze]
    while coordinateErr > ACCEPTED_ERROR:
        ke = [xG*coordinateErr[1],yG*coordinateErr[2],zG*coordinateErr[3]]
    
        #Evaluate q_dot
        q_dot = q_dotGenerator(q,ke)
        
        #Integrate q_dot to have q <=> -pi <= q <= pi
        q_rad_double = scipy.integrate.simps(q_dot)
        if q_rad_double < -pi: 
            q_rad_double = -pi
        elif q_rad_double > pi: 
            q_rad_double = pi

        #Find the value of the real reached angle in degrees
        q_deg_double = q_rad_double*180/pi
        q_real_deg = errorInvestigation(q_deg_double)

        #Evaluate the reached angle in radiants
        q_real_rad = q_real_deg*pi/180

        #Evalaute the real reached position
        re = forwardKinematics(q_real_rad)

        anglePublisher.publish(re)
        rospy.loginfo(re)
        rate.sleep()

## DESCRIPTION: initialize ROS node and subscribing nodes and callback functions
def CLnode_init():
    #In ROS, nodes are uniquely named. If two nodes with the same
    #name are launched, the previous one is kicked off. The
    #anonymous=True flag means that rospy will choose a unique
    #name for our 'listener' node so that multiple listeners can
    #run simultaneously.
    rospy.init_node('Control_Loop', anonymous=True)
    rospy.Subscriber("Desidered_Position", UInt16MultiArray, executeLoop)

#Struct define and init ------------------------------------------------

#Global variable definition --------------------------------------------
xG = 0.01 #Gain of x coordinate
yG = 0.01 #Gain of y coordinate
zG = 0.01 #Gain of z coordinate

xe = 0 #Value of real x coordinate
ye = 0 #Value of real x coordinate
ze = 0 #Value of real x coordinate

q = [0,0,0]

#Global defines --------------------------------------------------------
ACCEPTED_ERROR = pow(10,-5) #The error below which the control loop stops

#Init ------------------------------------------------------------------
CLnode_init()
anglePublisher = rospy.Publisher("Final_Angle", UInt16MultiArray, queue_size = 18)
rate = rospy.Rate(10000)

#Main ------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS ERROR - Interrupt Exception occured in ControlLoop.py")