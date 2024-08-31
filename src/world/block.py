#! /usr/bin/env python

"""!
Class used during the spawning phase
"""

from geometry_msgs.msg import Pose
import math
from world import Models, Size, TABLE_HEIGHT
from dataclasses import dataclass

@dataclass
class Coord:
    """!
    Coordinates' class, used to express either the start of the block or it's center
    """
    x: float
    y: float
    z: float

class Block:
    """!
    This class computes the information needed to spawn the block, the pose,
    and the methods used during the spawning phase.
    """

    def __init__(self, model, x=0, y=0, z=0, angle=0, color="Grey", margin=0.0):
        self.color = color
        self.model = model

        # Data used to compute distances
        self.angle = angle

        # in order to add a margin I have to enforce this distance to the sides.
        # To be able to do that, the two origins have to have a distance
        # equal to the diagonal of the square having a margin's size
        self.x = x + margin * math.sqrt(2) * math.cos(angle -3/4 * math.pi)
        self.y = y + margin * math.sqrt(2) * math.sin(angle -3/4 * math.pi)
        self.z = z

        self.size = Size(
            width  = Models[model].size.width  + margin * 2,
            length = Models[model].size.length + margin * 2,
            height = Models[model].size.height + margin * 2
        )

        # ---- Set Center -----
        self.center = Coord(
            x = self.x + self.size.width / 2 * math.cos(self.angle) - self.size.length / 2 * math.sin(self.angle), 
            y = self.y + self.size.length / 2 * math.cos(self.angle) + self.size.width / 2 * math.sin(self.angle), 
            z = TABLE_HEIGHT + Models[model].center.height
        )
        
        # Vertexes
        self.vertexes = self.get_vertexes()

        # ---- Set Pose ----
        self.pose = Pose()

        # position
        self.pose.position.x = self.center.x
        self.pose.position.y = self.center.y
        self.pose.position.z = self.z

        # quaternion
        self.pose.orientation.w = math.cos(angle/2)
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = math.sin(angle/2)

    
    def get_vertexes(self):
        """!
        @return The vertexes of the block, they're used to check if a block exceeds the table's limits or not.
        """
        
        cx, cy, z = [self.center.x, self.center.y, self.center.z]
        w, l = [self.size.width, self.size.length]
        ca, sa = [math.cos(self.angle), math.sin(self.angle)]

        vertexes = []

        # top right vertex
        vertexes.append(Coord(
            cx + w/2 * ca - l/2 * sa,
            cy + w/2 * sa + l/2 * ca,
            z
        ))
        # top left vertex
        vertexes.append(Coord(
            cx - w/2 * ca - l/2 * sa,
            cy - w/2 * sa + l/2 * ca,
            z
        ))
        # bottom left vertex
        vertexes.append(Coord(
            cx - w/2 * ca + l/2 * sa,
            cy - w/2 * sa - l/2 * ca,
            z
        ))
        # bottom left vertex
        vertexes.append(Coord(
            cx + w/2 * ca + l/2 * sa,
            cy + w/2 * sa - l/2 * ca,
            z
        ))
        
        return vertexes
  

    def collides(self, other):
        """!
        This function checks the collision by calculating the
        circular distance (point-diagonal, point-diagonal)

        @param other: another block

        @return collision truth value
        """

        def getDiag(self):
            return math.sqrt(math.pow(self.size.width,2) + math.pow(self.size.length,2))
        
        xc1, yc1 = [self.center.x, self.center.y]
        xc2, yc2 = [other.center.x, other.center.y]

        # the minimum distance is the sum of the two diagonals (circle distance)
        min_distance = (getDiag(self) + getDiag(other)) / 2
        # the actual distance (euclidean) is the one between the rectangles' centers
        actual_distance = math.sqrt(math.pow((xc1 - xc2),2) + math.pow((yc1 - yc2),2))

        return actual_distance < min_distance
    
    def __str__(self):
        return f'{self.model} at ({self.x}, {self.y}), angle = {math.degrees(self.angle)}° with size (width={self.size.width}, length={self.size.length}, height={self.size.height})'