import os
import re
from dataclasses import dataclass

UNIT_LENGTH = 0.031
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
    def __init__(self, id : str, size : Size, factor: Size):
        self.id = id
        self.size = size
        self.factor = factor

    def __str__(self):
        return f'{self.id} with size (width={self.size.width}, length={self.size.length}, height={self.size.height})'


def compute_model(model : str):
    # compute a regex that splits the model name, in order to get the coordinates' factors
    factors = re.findall(r'[X,Y,Z][0-9]', model)

    # each factor gets muliplied by the unit length
    return Model(model, Size(
        width  = float(factors[0][1:]) * UNIT_LENGTH, # X#
        length = float(factors[1][1:]) * UNIT_LENGTH, # Y#
        height = float(factors[2][1:]) * UNIT_LENGTH  # Z#
    ), Size(
        width  = float(factors[0][1:]), # X#
        length = float(factors[1][1:]), # Y#
        height = float(factors[2][1:])  # Z#
    ))


Models = dict(zip(
    os.listdir("models"),                           # keys
    list(map(compute_model, os.listdir("models")))  # values
    ))