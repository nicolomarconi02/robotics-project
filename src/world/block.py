from geometry_msgs.msg import Pose
import math
from world import Models, Size, TABLE_HEIGHT
from dataclasses import dataclass

@dataclass
class Coord:
    x: float
    y: float
    z: float

class Block:
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
            z = TABLE_HEIGHT + self.size.height / 2
        )

        # ---- Set Pose ----
        self.pose = Pose()

        # position
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z

        # quaternion
        self.pose.orientation.w = math.cos(angle/2)
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = math.sin(angle/2)

    def collides(self, other):

        def getDiag(self):
            return math.sqrt(math.pow(self.size.width,2) + math.pow(self.size.length,2))
        
        xc1, yc1 = [self.center.x, self.center.y]
        xc2, yc2 = [other.center.x, other.center.y]

        # the minimum distance is the sum of the two diagonals (circle distance)
        min_distance = (getDiag(self) + getDiag(other)) / 2
        # the actual distance (euclidean) is the one between the rectangles' centers
        actual_distance = math.sqrt(math.pow((xc1 - xc2),2) + math.pow((yc1 - yc2),2))

        return actual_distance < min_distance

        


    def collidesUnused(self, other):
        x1 = self.x
        y1 = self.y
        w1 = self.size.width
        h1 = self.size.length
        A1 = self.angle

        x2 = other.x
        y2 = other.y
        w2 = other.size.width
        h2 = other.size.length
        A2 = other.angle

        w1H = w1 / 2
        h1H = h1 / 2
        w2H = w2 / 2
        h2H = h2 / 2

        x1c = x1 + w1H
        y1c = y1 + h1H
        x2c = x2 + w2H
        y2c = y2 + h2H

        cosA1 = math.cos(A1)
        sinA1 = math.sin(A1)
        cosA2 = math.cos(A2)
        sinA2 = math.sin(A2)

        x1r =  cosA2 * (x1c - x2c) + sinA2 * (y1c - y2c) + x2c - w1H
        y1r = -sinA2 * (x1c - x2c) + cosA2 * (y1c - y2c) + y2c - h1H
        x2r =  cosA1 * (x2c - x1c) + sinA1 * (y2c - y1c) + x1c - w2H
        y2r = -sinA1 * (x2c - x1c) + cosA1 * (y2c - y1c) + y1c - h2H

        cosA1A2 = abs(cosA1 * cosA2 + sinA1 * sinA2)
        sinA1A2 = abs(sinA1 * cosA2 - cosA1 * sinA2)
        cosA2A1 = abs(cosA2 * cosA1 + sinA2 * sinA1)
        sinA2A1 = abs(sinA2 * cosA1 - cosA2 * sinA1)

        bbh1 = w1 * sinA1A2 + h1 * cosA1A2
        bbw1 = w1 * cosA1A2 + h1 * sinA1A2
        bbx1 = x1r + w1H - bbw1 / 2
        bby1 = y1r + h1H - bbh1 / 2

        bbh2 = w2 * sinA2A1 + h2 * cosA2A1
        bbw2 = w2 * cosA2A1 + h2 * sinA2A1
        bbx2 = x2r + w2H - bbw2 / 2
        bby2 = y2r + h2H - bbh2 / 2

        return x1 < bbx2 + bbw2 and x1 + w1 > bbx2 and y1 < bby2 + bbh2 and y1 + h1 > bby2 and \
                x2 < bbx1 + bbw1 and x2 + w2 > bbx1 and y2 < bby1 + bbh1 and y2 + h2 > bby1
    
    def __str__(self):
        return f'{self.model} at ({self.x}, {self.y}), angle = {math.degrees(self.angle)}° with size (width={self.size.width}, length={self.size.length}, height={self.size.height})'