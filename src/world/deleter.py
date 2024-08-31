#! /usr/bin/env python

"""!
Removes all the blocks spawned in the table.
"""

import rospy as ros
from gazebo_msgs.srv import DeleteModel
import os
import random


class Deleter():
    """!
    This ROS Node calls the gazebo/delete_model service on all the blocks
    """

    def __init__(self, robot_name="ur5", models_path="models"):
        self.robot_name = robot_name
        self.models_path=models_path

        ros.init_node('deleter')

    def delete_model(self, model):
        """!
        This function deletes the specified model

        @param model: model to be deleted
        """

        ros.wait_for_service('gazebo/delete_model')
        
        delete_model = ros.ServiceProxy('gazebo/delete_model', DeleteModel)
        
        delete_model(model)

    def delete_blocks(self):
        """!
        This function deletes all the blocks by calling delete_model on them
        """

        models = os.listdir(self.models_path)

        for model in models:
            self.delete_model(model)

def main():
    """!
    Define the deleter and remove all the blocks.
    """
    print('DELETER PROCESS: STARTED')

    spawner = Deleter()
    spawner.delete_blocks()

    print('DELETER: PROCESS ENDED')


if __name__ == '__main__':
    main()