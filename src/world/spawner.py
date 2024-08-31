#! /usr/bin/env python

"""!
Generates the blocks to be placed on the table

If this program is run using plot as an argument, a plot will be shown in the screen.
If test is added to the arguments, the class doesn't actually spawn the content. 
"""

import rospy as ros
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import math
import os
import sys
import random
import pickle

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arrow

from block import Block
from world import Range, GenerationLimits, Cfr

## Based on this flag, the routine spawns either 8 or 11 blocks
SPAWN_ALL=False

class Spawner():
    """!
    This class positions and places the blocks on the table.

    It sends 8 random blocks to the gazebo/spawn_sdf_model service, 
    the data's format is the same as the one required by the movement module.
    """
    ## Admissed block colors
    colors=['Red', 'Green', 'Yellow', 'Purple', 'Orange', 'Turquoise']

    ## Max number of spawn attempts for a block
    MAX_SPAWNS = 100000

    ## Margin between the blocks
    MARGIN=0.05

    ## Table's internal margin from the blocks
    TABLE_PADDING=0.02

    ## Admissible spawning limits on the table
    LIMITS=GenerationLimits(
        Range(0, 1),
        Range(0.25, 0.8),
        Range(0, 360)
        )
    
    ## Circular area that the arm can't reach, the blocks' centers can't spawn here
    ARM_AREA=Cfr(0.5, 0.5, 0.25)

    def __init__(self, robot_name="ur5", models_path="models", plot=False, test=False):
        self.robot_name = robot_name
        self.models_path=models_path

        self.plot = plot # if True see plots
        self.test = test # it True block ros communications

        # link ros service
        if not self.test:
            ros.init_node('spawner')
            ros.wait_for_service('gazebo/spawn_sdf_model')
            self.spawn_model = ros.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

        # in each instance of spawner the static attribute gets shuffled
        random.shuffle(self.colors)

    def spawn_content(self, pose : Pose, model, color="Grey"):
        """!
        Spawn a SDF model with a defined pose

        @param pose: pose of the spawned model
        @param model: id of the model to be spawned (ex. X1-Y2-Z2)
        @param color: color of the model

        @return Nothing, it calls Gazebo's spawning service
        """

        sdf = self.get_model_sdf(model)
        sdf = sdf.replace('Gazebo/Grey', f'Gazebo/{color}')

        if not self.test:
            self.spawn_model(model, sdf, self.robot_name, pose, "world")

    def get_model_sdf(self, model):
        """!
        Given a model, it returns its' sdf code

        @param model: model's id

        @return the model's sdf string, to be given to Gazebo's service
        """

        sdf_file = open(f'{self.models_path}/{model}/model.sdf','r')
        return sdf_file.read()

    def spawn_block(self, block : Block):
        """!
        Given a block, it spawns its content by calling spawn_content

        @param block: the block to be placed

        @return Nothing
        """

        self.spawn_content(block.pose, block.model, block.color)

    def spawn_blocks(self):
        """!
        This function reads the block models' IDs from the directory models, 
        and spawns all of them by calling the function spawn_block.

        If the plot arguments is specified, it plot the table with the
        unrestricted areas, the blocks (red), their angle (green arrow), their
        margins between each other (blue) and the margins from the table (orange)

        @return Nothing
        """

        models = os.listdir(self.models_path)

        if not SPAWN_ALL:
            random.shuffle(models)
            models = models[:8]

        final_blocks = []
        margins = []
        table_margins = []

        for idx, model in enumerate(models):
            collision = True
            generated = 0
            selected_color=self.colors[idx%len(self.colors)]
            
            while collision and generated < Spawner.MAX_SPAWNS:

                outsideArea = False
                count = 0
                while not outsideArea and count < Spawner.MAX_SPAWNS: 
                    block = Block(
                        model=model,
                        x=random.uniform(Spawner.LIMITS.x.min, Spawner.LIMITS.x.max), 
                        y=random.uniform(Spawner.LIMITS.y.min, Spawner.LIMITS.y.max), 
                        z=0.9,
                        angle=math.radians(random.uniform(Spawner.LIMITS.angle.min, Spawner.LIMITS.angle.max)),
                        color=selected_color
                    )

                    blockMargin = Block(
                        model=model,
                        x=block.x,
                        y=block.y,
                        z=0.9,
                        angle=block.angle,
                        color=block.color,
                        margin=Spawner.MARGIN
                    )
                    count = count + 1

                    # check if center inside circle
                    outsideArea = math.pow(Spawner.ARM_AREA.x - block.center.x, 2) + math.pow(Spawner.ARM_AREA.y - block.center.y, 2) >= math.pow(Spawner.ARM_AREA.r, 2)
                    
                    tableMargin = Block(
                        model=model,
                        x=block.x,
                        y=block.y,
                        z=0.9,
                        angle=block.angle,
                        color=block.color,
                        margin=Spawner.TABLE_PADDING
                    )
                    # check if vertexes are inside the table
                    if outsideArea:
                        for vertex in tableMargin.vertexes:
                            if vertex.x < Spawner.LIMITS.x.min or vertex.x > Spawner.LIMITS.x.max or \
                                vertex.y < Spawner.LIMITS.y.min or vertex.y > Spawner.LIMITS.y.max:
                                outsideArea = False
                                break

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
                    table_margins.append(tableMargin)
                    self.spawn_block(block)



        if self.plot:
            #define Matplotlib figure and axis
            fig, ax = plt.subplots()

            ax.invert_xaxis()
            ax.invert_yaxis()

            # add table to plot
            ax.add_patch(Rectangle((Spawner.LIMITS.x.min, Spawner.LIMITS.y.min), Spawner.LIMITS.x.max, (Spawner.LIMITS.y.max - Spawner.LIMITS.y.min), 
                                edgecolor='black',
                                facecolor='none',
                                linewidth=2))
            
            ax.add_patch(Rectangle((Spawner.LIMITS.x.min, 0.0), Spawner.LIMITS.x.max, Spawner.LIMITS.y.min, 
                                hatch="///"))
            
            ax.add_patch(Rectangle((Spawner.LIMITS.x.min, Spawner.LIMITS.y.max), Spawner.LIMITS.x.max, 1, 
                                hatch="///"))
            
            # add arm area
            ax.add_patch(Circle((Spawner.ARM_AREA.x, Spawner.ARM_AREA.y), Spawner.ARM_AREA.r,
                                hatch="///"))

            #add blocks and their margins to the plot
            for a in final_blocks:
                ax.add_patch(Rectangle((a.x, a.y), a.size.width, a.size.length, 
                                angle=math.degrees(a.angle),
                                color='red'))
                ax.add_patch(Circle((a.x, a.y), 0.005,
                                color='blue'))
                ax.add_patch(Circle((a.center.x, a.center.y), 0.005,
                                color='green'))
                ax.add_patch(Arrow(x=a.center.x, y=a.center.y, dx=(0.06 * math.cos(a.angle)), dy=(0.06 * math.sin(a.angle)),
                                color='green', width=0.03))
            for b in margins:
                ax.add_patch(Rectangle((b.x, b.y), b.size.width, b.size.length, 
                                angle=math.degrees(b.angle),
                                edgecolor='blue',
                                facecolor='none'))
            for c in table_margins:
                ax.add_patch(Rectangle((c.x, c.y), c.size.width, c.size.length, 
                                angle=math.degrees(c.angle),
                                edgecolor='orange',
                                facecolor='none'))

            #display plot
            plt.show()
        
        # serialization
        f = open('spawned.pickle', 'wb')
        pickle.dump(final_blocks, f)
        f.close()


def main():
    """!
    Executes the spawner on all the blocks, if plot is specified,
    all the data is shown.
    """

    print('SPAWNER PROCESS: STARTED')

    # check plot in program's arguments
    plot="plot" in sys.argv[1:]
    test="test" in sys.argv[1:]

    spawner = Spawner(plot=plot, test=test)
    spawner.spawn_blocks()

    print('SPAWNER PROCESS: ENDED')


if __name__ == '__main__':
    main()