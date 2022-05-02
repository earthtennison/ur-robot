import numpy as np
import matplotlib.pyplot as plt
import math

####### config ########
l1 = 420  # mm
l2 = 390  # mm


######################


def blend(theta_i, v, alpha, t):
    return theta_i + v * t + alpha * t * t / 2


def linear_segment(theta_i, v, t):
    return theta_i + v * t


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

    for i in range(len(theta) - 1):

        theta_curr = theta[i]
        alp = alpha[i]
        theta_next = theta[i + 1]
        t_d = td[i]

        if i == 0:
            alp = np.sign(theta_next - theta_curr) * alp
            result_t = t_d - math.sqrt(t_d ** 2 - 2 * (theta_next - theta_curr) / alp)
            result_v = (theta_next - theta_curr) / (t_d - result_t / 2)
            tn.append(result_t)
            v.append(result_v)

        elif i == len(theta) - 2:

            # last a, last t
            last_alp = np.sign(theta_curr - theta_next) * (alpha[i + 1])
            result_t_last = t_d - \
                            math.sqrt(t_d ** 2 + 2 * (theta_next - theta_curr) / last_alp)

            result_v = (theta_next - theta_curr) / (t_d - result_t_last / 2)
            alp = np.sign(result_v - v[i - 1]) * alp
            result_t_before_last = (result_v - v[i - 1]) / alp

            if i == 1:
                result_t_btw_before_last = td[i - 1] - \
                                           tn[i - 1] - result_t_before_last / 2
            else:
                result_t_btw_before_last = td[i - 1] - \
                                           tn[i - 1] / 2 - result_t_before_last / 2

            result_t_btw_last = t_d - result_t_before_last / 2 - result_t_last

            tn.append(result_t_before_last)
            tn.append(result_t_last)
            tn_n_1.append(result_t_btw_before_last)
            tn_n_1.append(result_t_btw_last)
            v.append(result_v)

        else:

            result_v = (theta_next - theta_curr) / t_d
            alp = np.sign(result_v - v[i - 1]) * alp
            result_t = (result_v - v[i - 1]) / alp

            if i == 1:
                result_t_btw = td[i - 1] - tn[i - 1] - result_t / 2
            else:
                result_t_btw = td[i - 1] - tn[i - 1] / 2 - result_t / 2

            tn.append(result_t)
            v.append(result_v)
            tn_n_1.append(result_t_btw)

        new_alpha.append(alp)

    new_alpha.append(last_alp)

    return tn, tn_n_1, v, new_alpha


def lspb(tn, tn_n_1, v_list, alpha_list, theta_i, step=0.01):
    theta_cumulate = theta_i
    v_cumulate = 0
    theta_list = []

    # tn have more element than tn_n_1 by 1
    for i in range(len(tn)):
        for t in np.arange(0, tn[i], step):
            theta_list.append(
                blend(theta_cumulate, v_cumulate, alpha_list[i], t))
        # add last calculated theta
        theta_cumulate = theta_list[-1]

        # tn have more element than tn_n_1 by 1
        if i < len(tn_n_1):
            for t in np.arange(0, tn_n_1[i], step):
                theta_list.append(linear_segment(theta_cumulate, v_list[i], t))
            theta_cumulate = theta_list[-1]
            v_cumulate = v_list[i]

    return theta_list


def fk(theta_list, is_deg=True):
    x_list = []
    y_list = []
    if is_deg:
        for theta1, theta2 in theta_list:
            x_list.append((l1 * math.cos(math.radians(theta1))) +
                          (l2 * math.cos(math.radians(theta1 + theta2))))
            y_list.append((l1 * math.sin(math.radians(theta1))) +
                          (l2 * math.sin(math.radians(theta1 + theta2))))
    else:
        for theta1, theta2 in theta_list:
            x_list.append((l1 * math.cos(theta1)) +
                          (l2 * math.cos(theta1 + theta2)))
            y_list.append((l1 * math.sin(theta1)) +
                          (l2 * math.sin(theta1 + theta2)))

    return x_list, y_list


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
    theta_list = [thetha_1, thetha_2]
    return theta_list


def plot(y_list, x_list):
    # TODO simulate arm

    x, y = np.array(x_list), np.array(y_list)
    plt.plot(x, y, 'bo', linewidth=2)
    plt.show()


def generate_via_point(p1, p2, step=10):
    """
    p1, p2 is end point
    step is the step interpolating x axis (mm)
    """
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    b = p1[1] - m * p1[0]

    x_via_list = []
    y_via_list = []
    for x in np.arange(p1[0], p2[0], step):
        x_via_list.append(x)
        y_via_list.append(m * x + b)

    return x_via_list, y_via_list


if __name__ == "__main__":

    # --> cross check with solution from text book
    ######## parameter #########
    # theta_via_points = [10, 35, 25, 10]
    # alpha = [50, 50, 50, 50]
    # td = [2, 1, 3]
    ############################
    # tn, tn_n_1, v, new_alpha = get_time(theta_via_points, alpha, td)
    # print(tn, tn_n_1, v, new_alpha)
    #
    # theta_list = lspb(tn, tn_n_1, v, new_alpha, theta_via_points[0])
    # print(theta_list)
    # plt.plot(theta_list, 'r')
    #
    # plt.plot([0, 200, 300, 600], theta_via_points, 'bo')
    # plt.show()

    # --> simulate ur robot
    ######## parameter #########
    p1 = [400, 400]
    p2 = [500, 500]
    step = 10 # step of x [mm]
    small_time_step = 0.001 # [second]
    alpha = int((p2[0] - p1[0]) / step) * [20]  # interval counts * [desired angular acceleration]
    td = int((p2[0] - p1[0]) / step) * [0.1]  # interval counts * [desired time duration [s]]
    ############################

    x_via_list, y_via_list = generate_via_point(p1, p2, step=step)
    # plot(x_via_list, y_via_list)
    plt.plot(x_via_list, y_via_list, "ro")

    theta_via_points1 = []
    theta_via_points2 = []
    for x, y in zip(x_via_list, y_via_list):
        theta1, theta2 = ik_2dof(x, y)
        theta_via_points1.append(theta1)
        theta_via_points2.append(theta2)

    tn_1, tn_via_1, v_1, new_alpha_1 = get_time(theta_via_points1, alpha, td)
    theta_list1 = lspb(tn_1, tn_via_1, v_1, new_alpha_1, theta_via_points1[0], step=small_time_step)

    tn_2, tn_via_2, v_2, new_alpha_2 = get_time(theta_via_points2, alpha, td)
    theta_list2 = lspb(tn_2, tn_via_2, v_2, new_alpha_2, theta_via_points2[0], step=small_time_step)

    print(len(theta_list1), len(theta_list2))
    if len(theta_list1) > len(theta_list2):
        theta_list1 = theta_list1[:len(theta_list2)]
    elif len(theta_list1) < len(theta_list2):
        theta_list2 = theta_list2[:len(theta_list1)]

    theta_list1, theta_list2 = np.array(theta_list1).reshape(-1, 1), np.array(theta_list2).reshape(-1, 1)
    # theta_list1, theta_list2 = theta_list1.reshape(theta_list1.shape[0],1)
    print(theta_list1)
    theta_list = np.concatenate((theta_list1, theta_list2), axis=1)
    print(theta_list)
    x_list, y_list = fk(theta_list, is_deg=False)
    plt.plot(x_list, y_list, "bo")

    plt.show()
