"""Scenic world model for traffic scenarios in the Replay simulator.

This model implements the basic :obj:`~scenic.domains.driving.model.Car` class from the
:obj:`scenic.domains.driving` domain.
Vehicles support the basic actions and behaviors from the driving domain.

A path to a map file for the scenario should be provided as the ``map`` global parameter;
see the driving domain's documentation for details.
"""

from scenic.simulators.replay.model import *

from scenic.domains.driving.replay import *  # includes basic actions and behaviors

from scenic.simulators.utils.colors import Color

simulator ReplaySimulator(render=render)

class ReplayActor(DrivingObject):
    throttle: 0
    steer: 0
    brake: 0
    hand_brake: 0
    reverse: 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    def setPosition(self, pos, elevation):
        self.position = pos

    def setVelocity(self, vel):
        self.velocity = vel

class Vehicle(Vehicle, ReplayActor):
    pass

class Car(Vehicle):
    @property
    def isCar(self):
        return True

class Pedestrian(Pedestrian, ReplayActor):
    pass

class Debris:
    """Abstract class for debris scattered randomly in the workspace."""
    position: new Point in workspace
    yaw: Range(0, 360) deg
