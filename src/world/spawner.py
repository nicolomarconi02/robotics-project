import rospy as ros
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import math
import os
import random
from block import Block

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Spawner():
    # static attribute
    colors=['Red', 'Green', 'Blue', 'Yellow', 'Purple', 'Orange', 'Indigo', 'Turquoise']
    MAX_SPAWNS = 100000
    MARGIN=0.05

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

    def spawn_block(self, block : Block):
        self.spawn_content(block.pose, block.model, block.color)

    def spawn_blocks(self, plot=False):
        models = os.listdir(self.models_path)
        final_blocks = []
        margins = []

        for idx, model in enumerate(models):
            collision = True
            generated = 0
            
            while collision and generated < Spawner.MAX_SPAWNS:
                block = Block(
                    model=model,
                    x=random.uniform(0.15, 0.85), 
                    y=random.uniform(0.3, 0.7), 
                    z=0.9,
                    angle=math.radians(random.uniform(0, 360)),
                    color=self.colors[idx%len(self.colors)]
                )

                # same block with 0.02 margin, in order to distanciate blocks
                blockMargin = Block(
                    model=model,
                    x=block.x,
                    y=block.y,
                    z=0.9,
                    angle=block.angle,
                    color=block.color,
                    margin=Spawner.MARGIN
                )
                i=0
                for fblock in final_blocks:
                    if fblock.collides(blockMargin):
                        generated = generated+1
                        break
                    i = i+1
                if i == len(final_blocks):
                    collision = False
                    final_blocks.append(block)
                    margins.append(blockMargin)
                    self.spawn_block(block)



        if plot:
            #define Matplotlib figure and axis
            fig, ax = plt.subplots()

            #add rectangle to plot
            for a in final_blocks:
                ax.add_patch(Rectangle((a.x, a.y), a.width, a.height, 
                                angle=a.angle,
                                edgecolor='red',
                                facecolor='none'))
            for b in margins:
                ax.add_patch(Rectangle((b.x, b.y), b.width, b.height, 
                                angle=b.angle,
                                edgecolor='blue',
                                facecolor='none'))

            #display plot
            plt.show()


def main():
    print('SPAWNER PROCESS: STARTED')

    spawner = Spawner()
    spawner.spawn_blocks()

    print('SPAWNER PROCESS: ENDED')


if __name__ == '__main__':
    main()