import os
import re
from dataclasses import dataclass

UNIT_LENGTH = 0.031
UNIT_HEIGHT = 0.019
TABLE_HEIGHT = 0.87

@dataclass
class Range:
    min: float
    max: float

@dataclass
class GenerationLimits:
    x: Range
    y: Range
    angle: Range

@dataclass
class Cfr:
    x: float
    y: float
    r: float

@dataclass
class Size:
    width: float
    length: float
    height: float

class Model:
    def __init__(self, id : str, factor: Size):
        self.id = id
        self.factor = factor

        # compute the actual sizes of each side of the model
        self.size = Size(
            width  = float(factor.width) * UNIT_LENGTH,
            length = float(factor.length) * UNIT_LENGTH,
            height = float(factor.height) * UNIT_HEIGHT 
        )

        # compute the center of the model
        self.center = Size(
            width  = self.size.width/2,
            length = self.size.length/2,
            height = self.size.height/2 
        )

        # based on the type of the block
        if factor.height == 1:
            self.center.height -= 0.005
        elif factor.width == 2 and factor.length == 2 and factor.height == 2:
            self.center.height += 0.01

    def __str__(self):
        return f'{self.id} with size (width={self.size.width}, length={self.size.length}, height={self.size.height})'


def compute_model(model : str):
    # compute a regex that splits the model name, in order to get the coordinates' factors
    factors = re.findall(r'[X,Y,Z][0-9]', model)

    # each factor gets muliplied by the unit length
    return Model(model, Size(
        width  = float(factors[0][1:]), # X#
        length = float(factors[1][1:]), # Y#
        height = float(factors[2][1:])  # Z#
    ))


Models = dict(zip(
    os.listdir("models"),                           # keys
    list(map(compute_model, os.listdir("models")))  # values
    ))