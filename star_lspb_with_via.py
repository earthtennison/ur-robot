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
import numpy as np
import math


def blend(pose_i, v, alpha, t):
    return pose_i + v * t + alpha * t * t / 2


def linear_segment(pose_i, v, t):
    return pose_i + v * t


def get_time(theta, alpha, td):
    """
    calculate the list of time duration for blending and linear segment, also velocity at linear segment and accel at blend
    Note: this function can be calculated in both joint space (theta) and cartesian space (x, y)
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


def lspb(tn, tn_n_1, v_list, alpha_list, pose_i, step=0.01):
    theta_cumulate = pose_i
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


def generate_via_point_theta(pose_i, pose_f, via_points_count):
    theta_via_list = []
    step = (pose_f - pose_i) / via_points_count
    if step == 0:
        return [pose_i] * (via_points_count)
    for theta in np.linspace(pose_i, pose_f, num=via_points_count, endpoint=True):
        theta_via_list.append(theta)
    return theta_via_list


def filter_list_length(ll):
    min_length = min([len(theta) for theta in ll])

    filtered_ll = []
    for l in ll:
        l = l[:min_length]
        l = np.array(l).reshape(-1, 1)
        filtered_ll.append(l)
    return filtered_ll


def create_traj(p1, p2):
    poses = []
    for pose_i, pose_f in zip(p1, p2):
        poses.append(generate_via_point_theta(pose_i, pose_f, via_points_count))
    print(poses)
    poses_list = []
    for pose_via_points in poses:
        if pose_via_points[-1] == pose_via_points[0]:
            poses_list.append([pose_via_points[0]] * int(sum(td) / small_time_step))
            continue
        tn, tn_via, v, new_alpha = get_time(pose_via_points, alpha, td)
        theta_list1 = lspb(tn, tn_via, v, new_alpha, pose_via_points[0], step=small_time_step)
        poses_list.append(theta_list1)

    poses_list = filter_list_length(poses_list)

    poses_list = np.concatenate((poses_list[0], poses_list[1], poses_list[2],
                                 poses_list[3], poses_list[4], poses_list[5]), axis=1)
    return np.array(poses_list)


if __name__ == '__main__':
    ####### config ########
    l1 = 420  # mm
    l2 = 390  # mm
    ######################

    print("program running...")

    # Establish connection to controller
    HOST = '10.10.0.26'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    time.sleep(5)

    ######## parameter #########
    p0 = np.array([-0.40645, 0.2157, 0.69268, 2.749, 2.318, 2.340])
    p1 = p0 + np.array([0.058, 0, -0.081, 0, 0, 0])
    p2 = p0 + np.array([-0.095, 0, 0.031, 0, 0, 0])
    p3 = p0 + np.array([0.095, 0, 0.031, 0, 0, 0])
    p4 = p0 + np.array([-0.058, 0, -0.081, 0, 0, 0])
    p5 = p0 + np.array([0, 0, 0.1, 0, 0, 0])
    p6 = p1

    small_time_step = 0.008  # [second]
    via_points_count = 10
    alpha = (via_points_count) * [50]  # interval counts * [desired angular acceleration]
    td = (via_points_count) * [1]  # interval counts * [desired time duration [s]]
    ############################

    poses_list = np.concatenate((create_traj(p1, p2), create_traj(p2, p3), create_traj(p3, p4), create_traj(p4, p5), create_traj(p5, p6)), axis=0)

    for x, y, z, row, pitch, yaw in poses_list:
        print("servoj(get_inverse_kin(p[{},{},{},{},{},{}]), 0, 0, {}, 0.1, 300)".format(x, y, z,
                                                                                         row, pitch, yaw,
                                                                                         small_time_step))
        s.send(bytes(
            "servoj(get_inverse_kin(p[{},{},{},{},{},{}]), 0, 0, {}, 0.1, 300)".format(x, y, z,
                                                                                       row, pitch, yaw,
                                                                                       small_time_step) + "\n",
            "utf-8"))

    time.sleep(100)
    print("Finished")
