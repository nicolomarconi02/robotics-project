#! /usr/bin/env python
import rospy as ros
from rospy import Rate
import os

class RobotReadyManager():

    def __init__(self):
        ros.init_node('robot_ready_manager')
        
        ur5_check_rate = Rate(0.5)
        ur5_ready = False

        while not ur5_ready:
            ur5_check_rate.sleep()
            ur5_ready = "/ur5/ready" in [topic[0] for topic in ros.get_published_topics()]

        os.system("make position-blocks")

def main():
    RobotReadyManager()

if __name__ == '__main__':
    main()