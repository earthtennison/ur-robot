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
    while True:
        s.send(bytes("movel(pose_trans(get_actual_tcp_pose(),p[0.1,0,0,0,0,0]),1,0.25,0,0)" + "\n", "utf-8"))
        time.sleep(2)
        s.send(bytes("movel(pose_trans(get_actual_tcp_pose(),p[0,0.1,0,0,0,0]),1,0.25,0,0)" + "\n", "utf-8"))
        time.sleep(2)
        s.send(bytes("movel(pose_trans(get_actual_tcp_pose(),p[-0.1,0,0,0,0,0]),1,0.25,0,0)" + "\n", "utf-8"))
        time.sleep(2)
        s.send(bytes("movel(pose_trans(get_actual_tcp_pose(),p[0,-0.1,0,0,0,0]),1,0.25,0,0)" + "\n", "utf-8"))
        time.sleep(2)
        print("Finished")

if __name__ == '__main__':
    print("program running...")
    main()
