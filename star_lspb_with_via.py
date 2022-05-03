#!/usr/bin/env python
# encoding: utf=8 

""" 
#UR Controller Primary Client Interface Reader
# For software version 3.0
#
# Datastream info found here: http://support.universal-robots.com/Technical/PrimaryAndSecondaryClientInterface
# Struct library used to extract data, info found here: https://docs.python.org/2/library/struct.html
"""

import socket
import time
import matplotlib.pyplot as plt
import numpy

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
            if result_v - v[i - 1] != 0:
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
            if result_v - v[i - 1] != 0:
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


def generate_via_point_theta(theta_i, theta_f, via_points_count):
    theta_via_list = []
    step = (theta_f - theta_i) / via_points_count
    if step == 0:
        return [theta_i] * (via_points_count)
    for theta in np.linspace(theta_i, theta_f, num=via_points_count, endpoint=True):
        theta_via_list.append(theta)
    return theta_via_list


def filter_list_length(ll):
    min_length = min([len(theta) for theta in theta_list_6_joints])

    filtered_ll = []
    for l in ll:
        l = l[:min_length]
        l = np.array(l).reshape(-1, 1)
        filtered_ll.append(l)
    return filtered_ll


def create_traj(p1, p2):
    theta_6_joints = []
    for theta_i, theta_f in zip(p1, p2):
        # print("theta_i: {}, theta_f: {}".format(theta_i, theta_f))
        # print("theta_via_points: {}".format(generate_via_point_theta(theta_i, theta_f, via_points_count)))
        theta_6_joints.append(generate_via_point_theta(theta_i, theta_f, via_points_count))

    theta_list_6_joints = []
    for theta_via_points in theta_6_joints:
        if theta_via_points[-1] == theta_via_points[0]:
            theta_list_6_joints.append([theta_via_points[0]] * int(sum(td) / small_time_step))
            continue
        tn, tn_via, v, new_alpha = get_time(theta_via_points, alpha, td)
        theta_list1 = lspb(tn, tn_via, v, new_alpha, theta_via_points[0], step=small_time_step)
        theta_list_6_joints.append(theta_list1)
        # print("theta_list_6_joints: {}".format(theta_list_6_joints))
    # print([len(theta) for theta in theta_list_6_joints])

    theta_list_6_joints = filter_list_length(theta_list_6_joints)

    theta_list = np.concatenate((theta_list_6_joints[0], theta_list_6_joints[1], theta_list_6_joints[2],
                                 theta_list_6_joints[3], theta_list_6_joints[4], theta_list_6_joints[5]), axis=1)
    # print(theta_list)
    return theta_list


if __name__ == '__main__':
    print("program running...")

    # Establish connection to controller
    HOST = '10.10.0.26'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    time.sleep(5)
    # move in star

    ######## parameter #########
    p1 = [-0.40645, 0.2157, 0.59268, 2.749, 2.318, 2.340]
    p2 = [-0.307, 0.2157, 0.555, 2.749, 2.318, 2.340]
    p3 = [-0.2598, 0.2157, 0.68118, 2.749, 2.318, 2.340]
    p3 = [-0.2598, 0.2157, 0.68118, 2.749, 2.318, 2.340]
    p3 = [-0.2598, 0.2157, 0.68118, 2.749, 2.318, 2.340]

    small_time_step = 0.008  # [second]
    step_joint = 1  # step of joint [deg]
    via_points_count = 10
    alpha = (via_points_count) * [50]  # interval counts * [desired angular acceleration]
    td = (via_points_count) * [5]  # interval counts * [desired time duration [s]]
    ############################

    theta_list = []
    theta_list.append(create_traj(p1, p2))
    theta_list.append(create_traj(p2, p3))
    theta_list.append(create_traj(p3, p4))
    theta_list.append(create_traj(p4, p5))
    theta_list.append(create_traj(p5, p6))

    for theta1, theta2, theta3, theta4, theta5, theta6 in theta_list:
        print("servoj(get_inverse_kin(p[{},{},{},{},{},{}]), 0, 0, {}, 0.1, 1000)".format(theta1, theta2, theta3,
                                                                                          theta4, theta5, theta6,
                                                                                          small_time_step))
        s.send(bytes(
            "servoj(get_inverse_kin(p[{},{},{},{},{},{}]), 0, 0, {}, 0.1, 1000)".format(theta1, theta2, theta3,
                                                                                        theta4, theta5, theta6,
                                                                                        small_time_step) + "\n",
            "utf-8"))

    time.sleep(10)
    print("Finished")
