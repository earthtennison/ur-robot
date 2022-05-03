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

    current_joint = "[d2r(0),d2r(-100),d2r(90.24),d2r(-90),d2r(-180),d2r(0)]"
    # current_joint = "[d2r(0),d2r(-100.82),d2r(56.36),d2r(-90),d2r(-180),d2r(0)]"
    # start
    for i in np.arange(-100, -80, step=0.01):
        s.send(bytes(
            "servoj([d2r(0),d2r({}),d2r(90.24),d2r(-90),d2r(-180),d2r(0)], 0, 0, {}, 0.1, 300)".format(i,
                                                                                                    0.008) + "\n",
            "utf-8"))
            # "movej([d2r(0),d2r({}),d2r(90.24),d2r(-90),d2r(-180),d2r(0)], 1, 1, {}, 0)".format(i, 0.25) + "\n",
            # "utf-8"))
        # time.sleep(0.008)
        print("sending... movej([d2r(0),d2r({}),d2r(90.24),d2r(-90),d2r(-180),d2r(0)], 1, 1, {}, 0)".format(-100, 0.25))

    # s.send(bytes("movej([d2r(0),d2r({}),d2r(90.24),d2r(-90),d2r(-180),d2r(0)], 1, 1, {}, 0)".format(-100,10) + "\n","utf-8"))

    time.sleep(3)

    print("Finished")

if __name__ == '__main__':
    print("program running...")
    main()
