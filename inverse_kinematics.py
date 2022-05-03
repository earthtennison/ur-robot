from cmath import acos, sin
import math
l1=420
l2=390
def ik_2dof(x, y):
    """
    input x,y coordinates of end effector
    return list of angle, in radians, of joint1 and joint2
    """
    x, y = float(x), float(y)
    # print((x ** 2 + y ** 2 - (l1 ** 2 + l2 ** 2)) / (2 * l1 * l2))
    try:
        thetha_2 = math.acos((x ** 2 + y ** 2 - (l1 ** 2 + l2 ** 2)) / (2 * l1 * l2))
        thetha_1 = math.acos(((x * (l1 + l2 * math.cos(thetha_2))) + (y * (l2 * math.sin(thetha_2)))) / (
                l1 ** 2 + l2 ** 2 + (2 * l1 * l2 * math.cos(thetha_2))))
    except:
        print("end effector position ({},{}) not possible!!!".format(x, y))

        return None, None
    theta_list = [math.degrees(thetha_1), math.degrees(thetha_2)]
    return theta_list

print(ik_2dof(400,350))
