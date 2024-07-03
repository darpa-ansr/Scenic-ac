"""Replay simulator implementation."""

from cmath import atan, pi, tan
import math
from math import copysign, degrees, radians, sin
import os
import pathlib
import pandas as pd
import scipy
import time

from PIL import Image
import numpy as np

import scenic.core.errors as errors  # isort: skip

if errors.verbosityLevel == 0:  # suppress pygame advertisement at zero verbosity
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame
import shapely

from scenic.core.geometry import allChains, findMinMax
from scenic.core.regions import toPolygon
from scenic.core.simulators import SimulationCreationError
from scenic.core.vectors import Orientation, Vector
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)
from scenic.domains.driving.roads import Network
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.syntax.veneer import verbosePrint, _globalParameters

current_dir = pathlib.Path(__file__).parent.absolute()

WIDTH = 1000 #1280
HEIGHT = 1000 #800
MAX_ACCELERATION = 5.6  # in m/s2, seems to be a pretty reasonable value
MAX_BRAKING = 4.6

ROAD_COLOR = (0, 0, 0)
ROAD_WIDTH = 2
LANE_COLOR = (96, 96, 96)
CENTERLINE_COLOR = (224, 224, 224)
SIDEWALK_COLOR = (0, 128, 255)
SHOULDER_COLOR = (96, 96, 96)


class ReplaySimulator(DrivingSimulator):
    """Implementation of `Simulator` for the Replay simulator.

    Args:
        network (Network): road network to display in the background, if any.
        render (bool): whether to render the simulation in a window.

    .. versionchanged:: 3.0

        The **timestep** argument is removed: it can be specified when calling
        `simulate` instead. The default timestep for the Replay simulator
        when not otherwise specified is still 0.1 seconds.
    """

    def __init__(self, network=None, render=False, export_gif=False):
        super().__init__()
        self.export_gif = export_gif
        self.render = render
        self.network = network

    def createSimulation(self, scene, **kwargs):
        simulation = ReplaySimulation(
            scene, self.network, self.render, self.export_gif, **kwargs
        )
        if self.export_gif and self.render:
            simulation.generate_gif("simulation.gif")
        return simulation


class ReplaySimulation(DrivingSimulation):
    """Implementation of `Simulation` for the Replay simulator."""

    def __init__(self, scene, network, render, export_gif, timestep, **kwargs):
        self.export_gif = export_gif
        self.render = render
        self.network = network
        self.frames = []

        if timestep is None:
            timestep = 0.1

        self.sim_data: pd.DataFrame = _globalParameters["sim_data"]
        self.sim_data_iterrows = self.sim_data.iterrows()
        self.now_time: float = float(self.sim_data.head(1).Timestamp.values[0])
        self.obj_from_id = {}
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()
        for obj in self.objects:
            self.obj_from_id[obj.id] = obj

        if self.render:
            # determine window size
            min_x, max_x = -500, 500 #findMinMax(obj.x for obj in self.objects)
            min_y, max_y = -500, 500 #findMinMax(obj.y for obj in self.objects)

            pygame.init()
            pygame.font.init()
            self.screen = pygame.display.set_mode(
                (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            img_path = _globalParameters["map_data"]
            self.map = pygame.image.load(img_path)
            self.screen.blit(self.map, (0, 0)) #.fill((255, 255, 255))
            x, y, _ = self.objects[0].position
            self.min_x, self.max_x = min_x - 50, max_x + 50
            self.min_y, self.max_y = min_y - 50, max_y + 50
            self.size_x = self.max_x - self.min_x
            self.size_y = self.max_y - self.min_y
            self.screen_poly = shapely.geometry.Polygon(
                (
                    (self.min_x, self.min_y),
                    (self.max_x, self.min_y),
                    (self.max_x, self.max_y),
                    (self.min_x, self.max_y),
                )
            )

            img_path = os.path.join(current_dir, "car.png")
            self.car = pygame.image.load(img_path)
            self.car_width = 10*int(3.5 * WIDTH / self.size_x)
            self.car_height = self.car_width
            self.car = pygame.transform.scale(self.car, (self.car_width, self.car_height))
            img_path = os.path.join(current_dir, "drone.png")
            self.blue_drone = pygame.image.load(img_path)
            self.blue_drone = pygame.transform.scale(self.blue_drone, (self.car_width, self.car_height))
            # Reverse objects so that ego is always drawn last
            self.objects.reverse()
            # Save reference to the ego.targets_reported
            self.targets_reported = self.obj_from_id["ego"].targets_reported

    def parse_network(self):
        self.network_polygons = []
        if not self.network:
            return

        def addRegion(region, color, width=1):
            poly = toPolygon(region)
            if not poly or not self.screen_poly.intersects(poly):
                return
            for chain in allChains(poly):
                coords = tuple(self.scenicToScreenVal(pt) for pt in chain.coords)
                self.network_polygons.append((coords, color, width))

        addRegion(self.network.walkableRegion, SIDEWALK_COLOR)
        addRegion(self.network.shoulderRegion, SHOULDER_COLOR)
        for road in self.network.roads:  # loop over main roads
            for lane in road.lanes:
                addRegion(lane.leftEdge, LANE_COLOR)
                addRegion(lane.rightEdge, LANE_COLOR)
            addRegion(road, ROAD_COLOR, ROAD_WIDTH)
        for lane in self.network.lanes:  # loop over all lanes, even in intersections
            addRegion(lane.centerline, CENTERLINE_COLOR)
        addRegion(self.network.intersectionRegion, ROAD_COLOR)

    def scenicToScreenVal(self, pos):
        x, y = pos[:2]
        x_prop = (x - self.min_x) / self.size_x
        y_prop = (y - self.min_y) / self.size_y
        return int(x_prop * WIDTH), HEIGHT - 1 - int(y_prop * HEIGHT)

    def createObjectInSimulator(self, obj):
        # Set actor's initial speed
        obj.speed = math.hypot(*obj.velocity)

        if hasattr(obj, "elevation"):
            obj.elevation = 0.0

    def isOnScreen(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def clear_targets_reported(self):
        """
        Clears targets_reported array.
        """
        # NOTE: The original list object must be maintained to allow require property monitors
        # which are implemented as closures, depending on targets_reported to work. Replacing
        # this code with the more natural `self.targets_reported = []` creates a new list,
        # disconnecting the class variable from the scenario variable, and the scenario variable
        # used in require statements will no longer be updated.
        while self.targets_reported != []:
            self.targets_reported.pop()

    def step(self):
        self.clear_targets_reported()
        for i, msg in self.sim_data_iterrows:
            if msg.Event == "GT_POSITION":
                obj = self.obj_from_id[msg.EntityID]
                obj.position = Vector(msg.X, msg.Y, msg.Z)
                obj.roll = msg.Roll
                obj.pitch = msg.Pitch
                obj.yaw = msg.Yaw
            elif msg.Event == "ODOM":
                obj = self.obj_from_id["ego"]
                obj.position = Vector(msg.X, msg.Y, msg.Z)
                obj.velocity = Vector(msg.Xdot, msg.Ydot, msg.Zdot)
                obj.speed = scipy.linalg.norm([msg.Xdot, msg.Ydot, msg.Zdot])
                obj.angularVelocity = Vector(msg.Surge, msg.Heave, msg.Sway)
                obj.angularSpeed = msg.Sway
                obj.roll = msg.Roll
                obj.pitch = msg.Pitch
                obj.yaw = msg.Yaw
            elif msg.Event == "MSG":
                overrides={"position": Vector(msg.X, msg.Y, msg.Z),
                           "roll": msg.Roll,
                           "pitch": msg.Pitch,
                           "yaw": msg.Yaw,
                           "color": msg.Color,
                           "vehicle_type": msg.Type
                           }
                target = self.obj_from_id[msg.EntityID]._copyWith(overrides=overrides)
                self.targets_reported.append(target)
            # TODO: Need to stop one row earlier
            if msg.Timestamp > self.now_time + self.timestep:
                self.now_time = msg.Timestamp
                self.obj_from_id["ego"].T = self.now_time
                break
        if self.render:
            self.draw_objects()
            pygame.event.pump()

    def draw_objects(self):
        self.screen.blit(self.map, (0, 0))
        # for screenPoints, color, width in self.network_polygons:
        #     pygame.draw.lines(self.screen, color, False, screenPoints, width=width)

        for i, obj in enumerate(self.objects):
            color = (255, 0, 0) if i == 0 else (0, 0, 255)
            h, w = obj.length, obj.width
            pos_vec = Vector(-1.75, 1.75)
            neg_vec = Vector(w / 2, h / 2)
            heading_vec = Vector(0, 10).rotatedBy(obj.heading)
            dx, dy = int(heading_vec.x), -int(heading_vec.y)
            x, y = self.scenicToScreenVal(obj.position)
            rect_x, rect_y = self.scenicToScreenVal(obj.position + pos_vec)
            if obj.id == "ego":
                self.rotated_car = pygame.transform.rotate(
                    self.blue_drone, math.degrees(obj.heading)
                )
                self.screen.blit(self.rotated_car, (rect_x, rect_y))
            elif hasattr(obj, "isCar") and obj.isCar:
                self.rotated_car = pygame.transform.rotate(
                    self.car, math.degrees(obj.heading)
                )
                self.screen.blit(self.rotated_car, (rect_x, rect_y))
        pygame.display.update()

        if self.export_gif:
            frame = pygame.surfarray.array3d(self.screen)
            frame = np.transpose(frame, (1, 0, 2))
            self.frames.append(frame)

        time.sleep(self.timestep/10)

    def generate_gif(self, filename="simulation.gif"):
        imgs = [Image.fromarray(frame) for frame in self.frames]
        imgs[0].save(filename, save_all=True, append_images=imgs[1:], duration=50, loop=0)

    def getProperties(self, obj, properties):
        yaw, _, _ = obj.parentOrientation.globalToLocalAngles(obj.heading, 0, 0)

        values = dict(
            position=obj.position,
            yaw=yaw,
            pitch=0,
            roll=0,
            velocity=obj.velocity,
            speed=obj.speed,
            angularSpeed=obj.angularSpeed,
            angularVelocity=obj.angularVelocity,
        )
        if "elevation" in properties:
            values["elevation"] = obj.elevation
        return values

    def destroy(self):
        if self.render:
            pygame.quit()

    def getLaneFollowingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.1, K_I=0.02, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getTurningControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.2, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.4, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getLaneChangingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.02, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.3, K_I=0.0, dt=dt)
        return lon_controller, lat_controller
