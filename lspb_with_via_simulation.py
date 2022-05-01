import numpy as np
import matplotlib.pyplot as plt
import math

####### config ########
l1 = 420 # mm
l2 = 390 # mm

######################


def blend(theta_i, v, alpha, t):
    return theta_i + v*t + alpha * t * t / 2


def linear_segment(theta_i, v, t):
    return theta_i + v*t

def get_time(theta, alpha, td):
    """
    theta = [theta1, theta2, theta3]
    alpha = [a1, a2, a3]
    td = [td12, td23]
    
    """
    tn = []
    tn_n_1 = []
    v = []
    new_alpha = []

    for i in range(len(theta)-1):
        
        theta_curr = theta[i]
        alp = alpha[i]
        theta_next = theta[i+1]
        t_d = td[i]
        
        
        

        if i == 0:
            alp = np.sign(theta_next - theta_curr)*alp
            result_t = t_d - math.sqrt(t_d**2 - 2*(theta_next-theta_curr)/alp)
            result_v = (theta_next-theta_curr)/(t_d-result_t/2)
            tn.append(result_t)
            v.append(result_v)

        elif i == len(theta)-2:

            # last a, last t
            last_alp = np.sign(theta_curr-theta_next)*(alpha[i+1])
            result_t_last = t_d - math.sqrt(t_d**2 + 2*(theta_next-theta_curr)/last_alp )

            result_v = (theta_next-theta_curr)/(t_d-result_t_last/2)
            alp = np.sign(result_v - v[i-1])*alp
            result_t_before_last = (result_v - v[i-1])/alp

            if i == 1:
                result_t_btw_before_last = td[i-1] - tn[i-1] - result_t_before_last/2
            else:
                result_t_btw_before_last = td[i-1] - tn[i-1]/2 - result_t_before_last/2
            
            
            result_t_btw_last = t_d - result_t_before_last/2-result_t_last

            tn.append(result_t_before_last)
            tn.append(result_t_last)
            tn_n_1.append(result_t_btw_before_last)
            tn_n_1.append(result_t_btw_last)
            v.append(result_v)
            
        else:

            
            result_v = (theta_next-theta_curr)/t_d
            alp = np.sign(result_v - v[i-1])*alp
            result_t = (result_v - v[i-1])/alp

            if i == 1:
                result_t_btw = td[i-1] - tn[i-1] - result_t/2
            else:
                result_t_btw = td[i-1] - tn[i-1]/2 - result_t/2

            tn.append(result_t)
            v.append(result_v)
            tn_n_1.append(result_t_btw)


        
        
        new_alpha.append(alp)

    new_alpha.append(last_alp)

    return tn, tn_n_1, v, new_alpha
            
# def lspb(tn, tn_n_1, v_list, alpha_list, tf):

#     v = []
#     alpha = []
#     theta_cumulate = 0
#     theta_list = []
#     for t, v, alpha in zip(tn, v_list, alpha_list):

#         theta_curr = blend(theta_cumulate, v, alpha, t)
#         theta_list.append(theta_curr)
#         theta_cumulate += 

#     for t, v in zip(tn_n_1, v_list):
#         theta_list.append(linear_segment(theta_cumulate, v, t))

#     for i in range(len(tn)):
#         for t in np.arange(0, tf, 0.1):
#             if t <
#                 theta_list.append(blend(theta_cumulate, v, alpha, t))
        
        
#         theta_cumulate += 
    

#     for i, f in zip(theta_i, theta_f):
#         result_v = (i - f) / (tb - tf)
#         result_alpha = result_v / tb
#         v.append(result_v)
#         alpha.append(result_alpha)

#     theta_list = []
#     for t in np.arange(0, tf, 0.1):
#         if t <= tb:
#             theta_list.append(first_blend(theta_i, alpha, t))
#         elif t <= (tf - tb):
#             theta_list.append(linear_segment(theta_i, theta_f, v, tf, t))
#         else:
#             theta_list.append(last_blend(theta_f, alpha, t, tf))

#     return theta_list

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

def generate_via_point(p1, p2, step=10):
    """
    p1, p2 is end point
    step is the step interpolating x axis (mm)
    """
    m = (p2[1] - p1[1])/(p2[0] - p1[0])
    b = p1[1] - m*p1[0]

    x_via_list = []
    y_via_list = []
    for x in np.arange(p1[0], p2[0], step):
        x_via_list.append(x)
        y_via_list.append(m*x + b)

    return x_via_list, y_via_list

if __name__ == "__main__":
    
    ######## parameter #########
    theta_i = [30, 20]
    theta_f = [40, 50]
    tf = 5
    tb = 1
    p1 = [0,0]
    p2 = [100,100]
    ############################

    # x_via_list, y_via_list = generate_via_point(p1, p2, step=10)
    # plot(x_via_list, y_via_list)

    theta_via_points = [10,35,25,10]
    alpha = [50,50,50,50]
    td = [2,1,3]
    tn, tn_n_1, v, new_alpha = get_time(theta_via_points, alpha, td)
    print(tn, tn_n_1, v, new_alpha)


