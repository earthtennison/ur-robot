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

def main():
    # Establish connection to controller
    HOST = '10.10.0.26'
    PORT = 30002

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.connect((HOST, PORT))
    time.sleep(5)
    # move in square

    # x1 = 58.7785
    # mm, y1 = -80.9017
    # mm
    # x2 = -95.1057
    # mm, y2 = 30.9017
    # mm
    # x3 = 95.1057
    # mm, y3 = 30.9017
    # mm
    # x4 = -58.7785
    # mm, y4 = -80.9017
    # mm
    # x5 = 0.0000
    # mm, y5 = 100.0000
    # mm

    current_pose = "p[-0.297,-0.016,0.100,0,3.14,0]"
    current_joint = "[d2r(-18.4),d2r(-119.3),d2r(124.15),d2r(-94.77),d2r(-89.46),d2r(-108.3)]"
    # start
    s.send(bytes("movej({},1,0.25,0,0)".format(current_joint) + "\n", "utf-8"))
    time.sleep(3)

    print("Finished")

if __name__ == '__main__':
    print("program running...")
    main()
