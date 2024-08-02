# Copyright 2024 The Johns Hopkins University Applied Physics Laboratory LLC

from scenic.core.regions import BoxRegion
from scenic.core.object_types import Vector
import math

class Zone:
    def __init__(self, region:BoxRegion, no_earlier_than:float=-math.inf, no_later_than:float=math.inf)->None:
        self.region:BoxRegion = region
        self.no_earlier_than = no_earlier_than
        self.no_later_than = no_later_than

COLORS = {
    "white" : (248/255., 248/255., 248/255.),
    "black" : (50/255., 50/255., 50/255.),
    "silver" : (188/255., 185/255., 183/255.),
    "gray" : (130/255., 130/255., 130/255.),
    "red" : (194/255., 92/255., 85/255.),
    "blue" : (75/255., 119/255., 157/255.),
    "brown" : (197/255., 166/255., 134/255.),
    "yellow" : (219/255., 191/255., 105/255.),
    "green" : (68/255., 160/255., 135/255.),
    "orange" : (242/255., 125/255., 32/255.),
    "violet" : (107/255., 31/255., 123/255.)
}
