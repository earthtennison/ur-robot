import numpy as np
import matplotlib.pyplot as plt
import math

####### config ########
l1 = 400 # mm
l2 = 400 # mm

######################


def first_blend(theta_i, alpha, t):
    return [theta + a * t * t / 2 for theta, a in zip(theta_i, alpha)]


def linear_segment(theta_i, theta_f, v, tf, t):
    linear_thetas = []
    for i, f, v in zip(theta_i, theta_f, v):
        result_angle = (i + f - v * tf) / 2 + v * t
        linear_thetas.append(result_angle)
    return linear_thetas


def last_blend(theta_f, alpha, t, tf):
    return [theta - a * tf * tf / 2 + a * tf * t - a * t * t / 2 for theta, a in zip(theta_f, alpha)]


def lspb(theta_i, theta_f, tf, tb):
    v = []
    alpha = []
    for i, f in zip(theta_i, theta_f):
        result_v = (i - f) / (tb - tf)
        result_alpha = result_v / tb
        v.append(result_v)
        alpha.append(result_alpha)

    theta_list = []
    for t in np.arange(0, tf, 0.1):
        if t <= tb:
            theta_list.append(first_blend(theta_i, alpha, t))
        elif t <= (tf - tb):
            theta_list.append(linear_segment(theta_i, theta_f, v, tf, t))
        else:
            theta_list.append(last_blend(theta_f, alpha, t, tf))

    return theta_list

def fk(theta_list):
    x_list=[]
    y_list=[]
    
    for theta1, theta2 in theta_list:
        x_list.append((l1*math.cos(math.radians(theta1))) + (l2*math.cos(math.radians(theta1+theta2))))
        y_list.append((l1*math.sin(math.radians(theta1))) + (l2*math.sin(math.radians(theta1+theta2))))
    
    return x_list, y_list

def ik(theta_list):
    pass

def plot(y_list, x_list):

    ## TODO siulate arm


    x, y = np.array(x_list), np.array(y_list)
    print(x_list, y_list)
    plt.plot(x,y, 'bo', linewidth = 2)
    plt.show()

if __name__ == "__main__":
    
    ######## parameter #########
    theta_i = [30, 20]
    theta_f = [40, 50]
    tf = 5
    tb = 2
    ############################


    theta_list = lspb(theta_i, theta_f, tf, tb)
    # print(theta_list)
    x_list, y_list = fk(theta_list)
    plot(x_list, y_list)
