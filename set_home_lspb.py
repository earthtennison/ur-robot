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


def main():
    # Establish connection to controller
    HOST = '10.10.0.26'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    time.sleep(5)
    # move in square

    # current_joint = "[d2r(0.17),d2r(-88.15),d2r(51.81),d2r(-17.19),d2r(-180.2),d2r(46.23)]"
    # current_joint = "[d2r(0),d2r(-100.82),d2r(56.36),d2r(-90),d2r(-180),d2r(0)]"
    # current_joint = "[d2r(0), d2r(-110.55), d2r(75.48), d2r(-34.35), d2r(-180), d2r(30.33)]"
    current_joint = "[d2r(0),d2r(-104.6),d2r(83.58),d2r(-49.25),d2r(-180.0),d2r(29.5)]"

    p0 = np.array([-0.40645, 0.2157, 0.69268, 2.749, 2.318, 2.340])
    p1 = p0 + np.array([0.058, 0, -0.081, 0, 0, 0])

    # start
    s.send(bytes("movej({},1,0.25,10,0)".format(current_joint) + "\n", "utf-8"))
    # s.send(bytes("movel(p[{},{},{},{},{},{}], 1, 0.1, 0, 0)".format(p1[0],p1[1],p1[2],p1[3],p1[4],p1[5]) + "\n", "utf-8"))
    # print("movel(p[{},{},{},{},{},{}], 1, 0.1, 0, 0)".format(p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]) + "\n")
    time.sleep(3)

    print("Finished.. home set")


if __name__ == '__main__':
    print("program running...")
    main()
