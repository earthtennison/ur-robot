import numpy as np
import matplotlib.pyplot as plt
import math

####### config ########
l1 = 420  # mm
l2 = 390  # mm

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
            result_t_last = t_d - \
                math.sqrt(t_d**2 + 2*(theta_next-theta_curr)/last_alp)

            result_v = (theta_next-theta_curr)/(t_d-result_t_last/2)
            alp = np.sign(result_v - v[i-1])*alp
            result_t_before_last = (result_v - v[i-1])/alp

            if i == 1:
                result_t_btw_before_last = td[i-1] - \
                    tn[i-1] - result_t_before_last/2
            else:
                result_t_btw_before_last = td[i-1] - \
                    tn[i-1]/2 - result_t_before_last/2

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


def lspb(tn, tn_n_1, v_list, alpha_list, theta_i, step=0.01):

    theta_cumulate = theta_i
    v_cumulate = 0
    time_cumulate = 0
    theta_list = []

    # tn have more element than tn_n_1 by 1
    for i in range(len(tn)):
        for t in np.arange(0, tn[i], step):
            theta_list.append(
                blend(theta_cumulate, v_cumulate, alpha_list[i], t))
        # add last calculated theta
        theta_cumulate = theta_list[-1]
        time_cumulate += tn[i]

        # tn have more element than tn_n_1 by 1
        if i < len(tn_n_1):
            for t in np.arange(0, tn_n_1[i], step):
                theta_list.append(linear_segment(theta_cumulate, v_list[i], t))
            theta_cumulate = theta_list[-1]
            v_cumulate = v_list[i]
            time_cumulate += tn_n_1[i]

    return theta_list


def fk(theta_list):
    x_list = []
    y_list = []

    for theta1, theta2 in theta_list:
        x_list.append((l1*math.cos(math.radians(theta1))) +
                      (l2*math.cos(math.radians(theta1+theta2))))
        y_list.append((l1*math.sin(math.radians(theta1))) +
                      (l2*math.sin(math.radians(theta1+theta2))))

    return x_list, y_list


def ik(theta_list):
    pass


def plot(y_list, x_list):

    # TODO simulate arm

    x, y = np.array(x_list), np.array(y_list)
    print(x_list, y_list)
    plt.plot(x, y, 'bo', linewidth=2)
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
    p1 = [0, 0]
    p2 = [100, 100]
    ############################

    # cross check with solution from text book
    theta_via_points = [10, 35, 25, 10]
    alpha = [50, 50, 50, 50]
    td = [2, 1, 3]
    tn, tn_n_1, v, new_alpha = get_time(theta_via_points, alpha, td)
    print(tn, tn_n_1, v, new_alpha)

    theta_list = lspb(tn, tn_n_1, v, new_alpha, theta_via_points[0])
    print(theta_list)
    plt.plot(theta_list, 'r')

    plt.plot([0, 200, 300, 600], theta_via_points, 'bo')
    plt.show()

    # simulate ur robot
    # x_via_list, y_via_list = generate_via_point(p1, p2, step=10)
    # plot(x_via_list, y_via_list)

    # theta_list1 = lspb(tn, tn_n_1, v, new_alpha)
    # theta_list2 = lspb(tn, tn_n_1, v, new_alpha)
    # print(theta_list)
    # x_list, y_list = fk(theta_list)
    # plot(x_list, y_list)
