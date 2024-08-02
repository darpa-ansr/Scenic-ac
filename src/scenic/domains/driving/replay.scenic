"""Scenic world model for scenarios using the driving domain.

Imports actions and behaviors for dynamic agents from
:doc:`scenic.domains.driving.actions` and :doc:`scenic.domains.driving.behaviors`.

The map file to use for the scenario must be specified before importing this model by
defining the global parameter ``map``. This path is passed to the `Network.fromFile`
function to create a `Network` object representing the road network. Extra options may be
passed to the function by defining the global parameter ``map_options``, which should be
a dictionary of keyword arguments. For example, we could write::

    param map = localPath('mymap.xodr')
    param map_options = { 'tolerance': 0.1 }
    model scenic.domains.driving.model

If you are writing a generic scenario that supports multiple maps, you may leave the
``map`` parameter undefined; then running the scenario will produce an error unless the
user uses the :option:`--param` command-line option to specify the map.

The ``use2DMap`` global parameter determines whether or not maps are generated in 2D. Currently
3D maps are not supported, but are under development. By default, this parameter is `False`
(so that future versions of Scenic will automatically use 3D maps), unless
:ref:`2D compatibility mode` is enabled, in which case the default is `True`. The parameter
can be manually set to `True` to ensure 2D maps are used even if the scenario is not compiled
in 2D compatibility mode.


.. note::

    If you are using a simulator, you may have to also define simulator-specific global
    parameters to tell the simulator which world to load. For example, our LGSVL
    interface uses a parameter ``lgsvl_map`` to specify the name of the Unity scene.
    See the :doc:`documentation <scenic.simulators>` of the simulator interfaces for
    details.
"""

from typing import Optional

from scenic.domains.driving.workspace import Workspace
from scenic.core.regions import PolygonalRegion

from scenic.core.distributions import RejectionException
from scenic.simulators.utils.colors import Color

## 2D mode flag & checks

def is2DMode():
    from scenic.syntax.veneer import mode2D
    return mode2D

param use2DMap = True if is2DMode() else False

if is2DMode() and (not globalParameters.use2DMap):
    raise RuntimeError('in 2D mode, global parameter "use2DMap" must be True')

# Note: The following should be removed when 3D maps are supported
if (not globalParameters.use2DMap) and ("replay" in globalParameters) and (not globalParameters.replay):
    raise RuntimeError('3D maps not supported at this time.'
        '(to use 2D maps set global parameter "use2DMap" to True)')

workspace = Workspace()

class DrivingObject:
    """Abstract class for objects in a road network.

    Provides convenience properties for the lane, road, intersection, etc. at the
    object's current position (if any).

    Also defines the ``elevation`` property as a standard way to access the Z
    component of an object's position, since the Scenic built-in property
    ``position`` is only 2D. If ``elevation`` is set to :obj:`None`, the simulator is
    responsible for choosing an appropriate Z coordinate so that the object is
    on the ground, then updating the property. 2D simulators should set the
    property to zero.

    Properties:
        elevation (float or None; dynamic): default ``None`` (see above).
        requireVisible (bool): Default value ``False`` (overriding the default
            from `Object`).
    """

    elevation[dynamic]: None if is2DMode() else float(self.position.z)

    requireVisible: False

    # Semantic category properties

    @property
    def isVehicle(self):
        return False

    @property
    def isCar(self):
        return False

    # Simulator interface implemented by subclasses

    def setPosition(self, pos, elevation):
        raise NotImplementedError

    def setVelocity(self, vel):
        raise NotImplementedError

class Vehicle(DrivingObject):
    """Vehicles which drive, such as cars.

    Properties:
        position: The default position is uniformly random over the `road`.
        parentOrientation: The default parentOrientation is aligned with `roadDirection`, plus an offset
            given by **roadDeviation**.
        roadDeviation (float): Relative heading with respect to the road direction at
            the `Vehicle`'s position. Used by the default value for **parentOrientation**.
        regionContainedIn: The default container is :obj:`roadOrShoulder`.
        viewAngle: The default view angle is 90 degrees.
        width: The default width is 2 meters.
        length: The default length is 4.5 meters.
        color (:obj:`Color` or RGB tuple): Color of the vehicle. The default value is a
            distribution derived from car color popularity statistics; see
            :obj:`Color.defaultCarColor`.
    """
    parentOrientation: Range(0, 360) deg
    roadDeviation: 0
    viewAngle: 90 deg
    width: 2
    length: 4.5
    color: Color.defaultCarColor()

    @property
    def isVehicle(self):
        return True

class Car(Vehicle):
    """A car."""
    @property
    def isCar(self):
        return True

class NPCCar(Car):
    """Car for which accurate physics is not required."""
    pass

class Pedestrian(DrivingObject):
    """A pedestrian.

    Properties:
        position: The default position is uniformly random over sidewalks and crosswalks.
        parentOrientation: The default parentOrientation has uniformly random yaw.
        viewAngle: The default view angle is 90 degrees.
        width: The default width is 0.75 m.
        length: The default length is 0.75 m.
        color: The default color is turquoise. Pedestrian colors are not necessarily
            used by simulators, but do appear in the debugging diagram.
    """
    parentOrientation: Range(0, 360) deg
    viewAngle: 90 deg
    width: 0.75
    length: 0.75
    color: [0, 0.5, 1]

## Utility functions

def withinDistanceToAnyCars(car, thresholdDistance):
    """ returns boolean """
    objects = simulation().objects
    for obj in objects:
        if obj is car or not isinstance(obj, Vehicle):
            continue
        if (distance from car to obj) < thresholdDistance:
            return True
    return False

def withinDistanceToAnyObjs(vehicle, thresholdDistance):
    """ checks whether there exists any obj
    (1) in front of the vehicle, (2) within thresholdDistance """
    objects = simulation().objects
    for obj in objects:
        if not (vehicle can see obj):
            continue
        if (distance from vehicle.position to obj.position) < 0.1:
            # this means obj==vehicle
            pass
        elif (distance from vehicle.position to obj.position) < thresholdDistance:
            return True
    return False