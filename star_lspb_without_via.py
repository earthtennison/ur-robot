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

    for t in range(0, tf, 0.1):
        theta_list = []
        if t <= tb:
            theta_list.append(first_blend(theta_i, alpha, t))
        elif t <= (tf - tb):
            theta_list.append(linear_segment(theta_i, theta_f, v, tf, t))
        else:
            theta_list.append(last_blend(theta_f, alpha, t, tf))

    return theta_list

def plot(theta_list):
    pass

def main():
    tf = 5
    tb = 1
    theta_i = [0, 0]
    theta_f = [0, 0]

    # Establish connection to controller
    HOST = '10.10.0.26'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    time.sleep(5)
    # move in square


    print("Finished")


if __name__ == '__main__':
    print("program running...")
    main()
