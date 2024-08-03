import rospy as ros
from gazebo_msgs.srv import DeleteModel
import os
import random


class Deleter():

    def __init__(self, robot_name="ur5", models_path="models"):
        self.robot_name = robot_name
        self.models_path=models_path

        ros.init_node('deleter')

    def delete_model(self, model):

        ros.wait_for_service('gazebo/delete_model')
        
        delete_model = ros.ServiceProxy('gazebo/delete_model', DeleteModel)
        
        delete_model(model)

    def delete_blocks(self):
        models = os.listdir(self.models_path)

        for model in models:
            self.delete_model(model)

def main():
    print('DELETER PROCESS: STARTED')

    spawner = Deleter()
    spawner.delete_blocks()

    print('DELETER: PROCESS ENDED')


if __name__ == '__main__':
    main()