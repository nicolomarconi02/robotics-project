import rospy as ros
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import math
import os
import uuid
import random


class Spawner():
    # static attribute
    colors=['Red', 'Green', 'Blue', 'Yellow', 'Purple', 'Orange', 'Indigo', 'Turquoise']

    def __init__(self, robot_name="ur5", models_path="models"):
        self.robot_name = robot_name
        self.models_path=models_path

        # in each instance of spawner the static attribute gets shuffled
        random.shuffle(self.colors)
        ros.init_node('spawner')

    def spawn_content(self, pose : Pose, model, color="Grey"):
        sdf = self.get_model_sdf(model)
        sdf = sdf.replace('Gazebo/Grey', f'Gazebo/{color}')

        ros.wait_for_service('gazebo/spawn_sdf_model')
        
        spawn_model = ros.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        
        spawn_model(model, sdf, self.robot_name, pose, "world")

    def get_model_sdf(self, model):
        sdf_file = open(f'{self.models_path}/{model}/model.sdf','r')
        return sdf_file.read()

    def spawn_block(self, model, x, y, z, angle, color="Grey"):
        pose = Pose()
        
        # position
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # quaternion
        pose.orientation.w = math.cos(angle/2)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(angle/2)

        self.spawn_content(pose, model, color)

    def spawn_blocks(self):
        models = os.listdir(self.models_path)

        for idx, model in enumerate(models):
            self.spawn_block(
                model=model,
                x=random.uniform(0.15, 0.85), 
                y=random.uniform(0.3, 0.7), 
                z=random.uniform(0.9, 1),
                angle=random.uniform(0, 1),
                color=self.colors[idx%len(self.colors)]
            )


def main():
    print('SPAWNER PROCESS: STARTED')

    spawner = Spawner()
    spawner.spawn_blocks()

    print('SPAWNER PROCESS: ENDED')


if __name__ == '__main__':
    main()