#! /usr/bin/env python

"""!
World utility information and classes
"""

import os
import re
from dataclasses import dataclass

## Size of a singular side of a block
UNIT_LENGTH = 0.031

## Unit height of a block, it doesn't count the pillar's size
UNIT_HEIGHT = 0.019

## Height of the working table
TABLE_HEIGHT = 0.87

@dataclass
class Range:
    """!
    Scalar range
    """
    ## minimum value
    min: float
    ## maximum value
    max: float

@dataclass
class GenerationLimits:
    """!
    Spawning Limits' class
    """
    ## spawning limits on the x-axis
    x: Range
    ## spawning limits on the y-axis
    y: Range
    ## spawning cryteria for the angle
    angle: Range

@dataclass
class Cfr:
    """!
    Circonference class
    """
    x: float
    y: float
    r: float

@dataclass
class Size:
    """!
    Size used to express values on the three different axis
    """
    ## x-axis
    width: float
    ## x-axis
    length: float
    ## z-axis
    height: float

class Model:
    """!
    It defines a model's general data
    """

    def __init__(self, id : str, factor: Size):
        ## Model id
        self.id = id
        ## scalar factor on the axis, ex. (1,2,2) for a x1-Y2-Z2
        self.factor = factor

        ## actual size of a block
        self.size = Size(
            width  = float(factor.width) * UNIT_LENGTH,
            length = float(factor.length) * UNIT_LENGTH,
            height = float(factor.height) * UNIT_HEIGHT 
        )

        ## coordinates of the center of the model
        self.center = Size(
            width  = self.size.width/2,
            length = self.size.length/2,
            height = self.size.height/2 
        )

    def __str__(self):
        return f'{self.id} with size (width={self.size.width}, length={self.size.length}, height={self.size.height})'


def compute_model(model : str):
    """!
    Given a model id, it finds the factor and defines the model

    @param model: model id
    @return Model Object
    """
    # compute a regex that splits the model name, in order to get the coordinates' factors
    factors = re.findall(r'[X,Y,Z][0-9]', model)

    # each factor gets muliplied by the unit length
    return Model(model, Size(
        width  = float(factors[0][1:]), # X#
        length = float(factors[1][1:]), # Y#
        height = float(factors[2][1:])  # Z#
    ))


## Dictionary containing all of the blocks' models, used in most of the code
Models = dict(zip(
    os.listdir("models"),                           # keys
    list(map(compute_model, os.listdir("models")))  # values
    ))