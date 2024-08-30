#! /usr/bin/env python
import rospy as ros

import numpy as np
from ctypes import * # convert float to uint32

import math

from robotics_project.srv import GetBlocks, GetBlocksResponse

from geometry_msgs.msg import Pose

import pickle

import sys
sys.path.append("src/world")
from world import Models, TABLE_HEIGHT

class VisionManagerClass():

    def __init__(self, robot_name="ur5"):
        f = open('spawned.pickle', 'rb')
        self.blocks = pickle.load(f)
        f.close()

    def start_service(self):
        # Service's definition and its handler's setting
        ros.init_node('vision')
        s = ros.Service('vision', GetBlocks, self.handle_get_blocks)
        print('VISION Process: Service STARTED')

        # The node runs indefinitely
        ros.spin()

    def handle_get_blocks(self, req):
        """
        This function gets called in order to reply to the service's calls coming from the clients.
        Arguments:
            Nothing
        Returns:
            Nothing
        """

        print('VISION PROCESS: handle_obtain_blocks CALLED')

        poses = []
        blocks_id = []
        n_blocks = len(self.blocks)
        finished = True

        print(f'The blocks to be SENT are {n_blocks}, the following lines describe them')

        for block in self.blocks:
            print(f'{block.model} with centre in ({block.center.x}, {block.center.y}) and an angle of {block.angle}')

            pose = Pose()

            # position
            pose.position.x = block.center.x
            pose.position.y = block.center.y
            pose.position.z = block.center.z
            
            # quaternion
            pose.orientation.w = math.cos((block.angle + math.pi/2)/2)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin((block.angle + math.pi/2)/2)

            poses.append(pose)
            blocks_id.append(block.model)

        print('VISION PROCESS: handle_obtain_blocks ENDED')

        return GetBlocksResponse(poses, blocks_id, np.int8(n_blocks), finished)


def main():
    # Starting the ROS Node and keep it running
    manager = VisionManagerClass()
    manager.start_service()


if __name__ == '__main__':
    main()