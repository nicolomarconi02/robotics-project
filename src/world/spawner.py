import rospy as ros
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import math
import os
import sys
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

from block import Block
from world import Range, GenerationLimits

class Spawner():
    # static attribute
    colors=['Red', 'Green', 'Blue', 'Yellow', 'Purple', 'Orange', 'Indigo', 'Turquoise']
    MAX_SPAWNS = 100000
    MARGIN=0.05
    LIMITS=GenerationLimits(
        Range(0.15, 0.85),
        Range(0.3, 0.7),
        Range(0, 360)
        )

    def __init__(self, robot_name="ur5", models_path="models"):
        self.robot_name = robot_name
        self.models_path=models_path

        # link ros service
        ros.init_node('spawner')
        ros.wait_for_service('gazebo/spawn_sdf_model')
        self.spawn_model = ros.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

        # in each instance of spawner the static attribute gets shuffled
        random.shuffle(self.colors)

    def spawn_content(self, pose : Pose, model, color="Grey"):
        sdf = self.get_model_sdf(model)
        sdf = sdf.replace('Gazebo/Grey', f'Gazebo/{color}')

        self.spawn_model(model, sdf, self.robot_name, pose, "world")

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
            selected_color=self.colors[idx%len(self.colors)]
            
            while collision and generated < Spawner.MAX_SPAWNS:
                block = Block(
                    model=model,
                    x=random.uniform(Spawner.LIMITS.x.min, Spawner.LIMITS.x.max), 
                    y=random.uniform(Spawner.LIMITS.y.min, Spawner.LIMITS.y.max), 
                    z=0.9,
                    angle=math.radians(random.uniform(Spawner.LIMITS.angle.min, Spawner.LIMITS.angle.max)),
                    color=selected_color
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
                ax.add_patch(Rectangle((a.x, a.y), a.size.width, a.size.length, 
                                angle=math.degrees(a.angle),
                                color='red'))
            for b in margins:
                ax.add_patch(Rectangle((b.x, b.y), b.size.width, b.size.length, 
                                angle=math.degrees(b.angle),
                                edgecolor='blue',
                                facecolor='none'))

            #display plot
            plt.show()


def main():
    print('SPAWNER PROCESS: STARTED')

    # check plot in program's arguments
    plot="plot" in sys.argv[1:]

    spawner = Spawner()
    spawner.spawn_blocks(plot)

    print('SPAWNER PROCESS: ENDED')


if __name__ == '__main__':
    main()