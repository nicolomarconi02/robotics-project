#! /usr/bin/env python

import os
import random

class RealPoseIn2D: 

    def __init__(self, label, color, x, y, theta):
        self.label = label
        self.color = color
        self.x = x
        self.y = y
        self.theta = theta

models = os.listdir("models")
random.shuffle(models)

block_skeleton = """
    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{x} {y} {z} 0. 0. {angle}</pose>
    </include>
    """

blocks = list(map(lambda model: block_skeleton.format(
        name=model, 
        model=model, 
        x=random.uniform(0.15, 0.85), 
        y=random.uniform(0.3, 0.7), 
        z=random.uniform(0.9, 1),
        angle=random.uniform(0, 1)
    ), models))

with open("src/world/robotics_project.world", "r+") as f:
    world_skeleton = f.read()

ws_start = world_skeleton.find("<gui>")

with open("src/world/tmp.world", "w") as f:
    f.write(world_skeleton[:ws_start] + ''.join(blocks) + world_skeleton[ws_start:])