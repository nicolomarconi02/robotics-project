#! /usr/bin/env python

"""!
This routine is called with the robot, it checks whether
the lattest completed its homing procedure or not.
"""

import rospy as ros
from rospy import Rate
import os

class RobotReadyManager():
    """!
    Class that instantiates the ros communication and waits for the /ur5/topic to be created. 
    It checks with a frequency of 0.5Hz, once every two seconds.
    When the robot is detected to be ready, this routine spawns the blocks. 
    """

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