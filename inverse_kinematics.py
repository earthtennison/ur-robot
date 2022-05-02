from cmath import acos, sin
import math

def inverse_kinematics(x ,y ,l1, l2): #input x,y coordinates of end effector and link length
    thetha_2 =math.acos((x^2+y^2-(l1^2+l2^2))/2*l1*l2) 
    thetha_1 = math.acos(((x*(l1+l2*math.cos(thetha_2)))+(y*(l2*math.sin(thetha_2))))/(l1^2+l2^2+(2*l1*l2*math.cos(thetha_2))))
    result = [math.degrees(thetha_1),math.degrees(thetha_2)] #return angle in degrees
    return result
