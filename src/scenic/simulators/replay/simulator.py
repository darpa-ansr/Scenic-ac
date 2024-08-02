"""
Copyright 2024 The Johns Hopkins University Applied Physics Laboratory LLC

Replay simulator implementation.
"""

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

    def __init__(self, render=False, export_gif=False):
        super().__init__()
        self.export_gif = export_gif
        self.render = render

    def createSimulation(self, scene, **kwargs):
        simulation = ReplaySimulation(
            scene, self.render, self.export_gif, **kwargs
        )
        if self.export_gif and self.render:
            simulation.generate_gif("simulation.gif")
        return simulation


class ReplaySimulation(DrivingSimulation):
    """Implementation of `Simulation` for the Replay simulator."""

    def __init__(self, scene, render, export_gif, timestep, **kwargs):
        self.export_gif = export_gif
        self.render = render
        self.frames = []

        if timestep is None:
            timestep = 0.1

        self.sim_data: pd.DataFrame = _globalParameters["sim_data"]
        self.sim_data_iterrows = self.sim_data.iterrows()
        self.now_time: float = float(self.sim_data.head(1).Timestamp.values[0])
        self.sim_start_time = self.now_time
        self.obj_from_id = {}
        self.ego = None
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()
        for obj in self.objects:
            if obj.id == "ego":
                self.ego = obj
            self.obj_from_id[obj.id] = obj
        assert self.ego is not None

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
            # Update ego time variable
            self.ego.T = self.now_time

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
        while self.ego.targets_reported != []:
            self.ego.targets_reported.pop()

    def step(self):
        targets_groundtruth = {}
        targets_reported = {}
        ego = {}
        # Process simulation data for the next timestep
        for i, msg in self.sim_data_iterrows:
            if msg.Event == "GT_POSITION":
                if msg.EntityID not in targets_groundtruth:
                    targets_groundtruth[msg.EntityID] = {}
                    t = targets_groundtruth[msg.EntityID]
                    t["position"] = np.array([0, 0, 0], dtype=float)
                    t["roll"] = 0.0
                    t["pitch"] = 0.0
                    t["yaw"] = 0.0
                    t["update_count"] = 0
                t = targets_groundtruth[msg.EntityID]
                t["position"] += np.array([msg.X, msg.Y, msg.Z], dtype=float)
                t["roll"] += msg.Roll
                t["pitch"] += msg.Pitch
                t["yaw"] += msg.Yaw
                t["update_count"] += 1
            elif msg.Event == "ODOM":
                if ego == {}:
                    ego["position"] = np.array([0, 0, 0], dtype=float)
                    ego["velocity"] = np.array([0, 0, 0], dtype=float)
                    ego["angularVelocity"] = np.array([0, 0, 0], dtype=float)
                    ego["roll"] = 0.0
                    ego["pitch"] = 0.0
                    ego["yaw"] = 0.0
                    ego["update_count"] = 0
                ego["position"] += np.array([msg.X, msg.Y, msg.Z], dtype=float)
                ego["velocity"] += np.array([msg.Xdot, msg.Ydot, msg.Zdot], dtype=float)
                ego["angularVelocity"] += np.array([msg.Surge, msg.Heave, msg.Sway], dtype=float)
                ego["roll"] += msg.Roll
                ego["pitch"] += msg.Pitch
                ego["yaw"] += msg.Yaw
                ego["update_count"] += 1
            elif msg.Event == "MSG":
                if msg.EntityID not in targets_reported:
                    targets_reported[msg.EntityID] = {}
                    t = targets_reported[msg.EntityID]
                    t["position"] = np.array([0, 0, 0], dtype=float)
                    t["roll"] = 0.0
                    t["pitch"] = 0.0
                    t["yaw"] = 0.0
                    t["update_count"] = 0
                t = targets_reported[msg.EntityID]
                t["position"] += np.array([msg.X, msg.Y, msg.Z], dtype=float)
                t["roll"] += msg.Roll
                t["pitch"] += msg.Pitch
                t["yaw"] += msg.Yaw
                # TODO: Should the color be averaged? What about vehicle_type?
                t["color"] = msg.Color
                t["vehicle_type"] = msg.Type
                t["update_count"] += 1
            elif msg.Event == "CLSN":
                ego["collision"] = True

            # TODO: Need to stop one row earlier but there is no way to peek
            #       at message timestamp without stepping sim_data_iterrows.
            if msg.Timestamp > self.now_time + self.timestep:
                self.now_time = msg.Timestamp
                break

        # Update simulation objects
        self.clear_targets_reported()
        self.ego.collision = False
        for id, t in targets_groundtruth.items():
            obj = self.obj_from_id[id]
            obj.position = Vector(*(t["position"]/t["update_count"]))
            obj.roll = t["roll"]/t["update_count"]
            obj.pitch = t["pitch"]/t["update_count"]
            obj.yaw = t["yaw"]/t["update_count"]
            obj.heading = t["pitch"]/t["update_count"]
        for id, t in targets_reported.items():
            obj = self.obj_from_id[id]._copyWith()
            obj.position = Vector(*(t["position"]/t["update_count"]))
            obj.roll = t["roll"]/t["update_count"]
            obj.pitch = t["pitch"]/t["update_count"]
            obj.yaw = t["yaw"]/t["update_count"]
            obj.heading = t["pitch"]/t["update_count"]
            obj.color = t["color"]
            obj.vehicle_type = t["vehicle_type"]
            self.ego.targets_reported.append(obj)
        if "position" in ego:
            self.ego.position = Vector(*(ego["position"]/ego["update_count"]))
            self.ego.velocity = Vector(*(ego["velocity"]/ego["update_count"]))
            self.ego.speed = scipy.linalg.norm(self.ego.velocity)
            self.ego.angularVelocity = Vector(*(ego["angularVelocity"]/ego["update_count"]))
            self.ego.angularSpeed = self.ego.angularVelocity.z
            self.ego.roll = ego["roll"]/ego["update_count"]
            self.ego.pitch = ego["pitch"]/ego["update_count"]
            self.ego.yaw = ego["yaw"]/ego["update_count"]
            self.ego.heading = self.ego.yaw
        if "collision" in ego:
            self.ego.collision = ego["collision"]
        self.ego.T = self.now_time - self.sim_start_time
        if self.render:
            self.draw_objects()
            pygame.event.pump()

    def draw_objects(self):
        self.screen.blit(self.map, (0, 0))

        for i, obj in enumerate(self.objects):
            h, w = obj.length, obj.width
            pos_vec = Vector(-1.75, 1.75)
            neg_vec = Vector(w / 2, h / 2)
            heading_vec = Vector(0, 10).rotatedBy(obj.heading)
            dx, dy = int(heading_vec.x), -int(heading_vec.y)
            x, y = self.scenicToScreenVal(obj.position)
            rect_x, rect_y = self.scenicToScreenVal(obj.position + pos_vec)
            # pygame.draw.circle(self.screen, (255,0,0), (0,0), 100)
            if obj.id == "ego":
                self.rotated_drone = pygame.transform.rotate(
                    self.blue_drone, math.degrees(obj.heading)
                )
                self.screen.blit(self.rotated_drone, (rect_x, rect_y))
            elif hasattr(obj, "isCar") and obj.isCar:
                self.rotated_car = pygame.transform.rotate(
                    self.car, math.degrees(obj.heading)
                )
                color = tuple([int(x*255) for x in obj.color])
                # TODO: Check that this gives top left corner
                left_top = self.scenicToScreenVal(obj.position + pos_vec)
                width_height = (self.car_width, self.car_height)
                if obj.vehicle_type == "SUV":
                    pygame.draw.rect(self.screen, color, pygame.Rect(left_top, width_height))
                elif obj.vehicle_type == "SEDAN":
                    pygame.draw.circle(self.screen, color, (x,y), w)
                else:
                    assert False
        pygame.display.update()

        if self.export_gif:
            frame = pygame.surfarray.array3d(self.screen)
            frame = np.transpose(frame, (1, 0, 2))
            self.frames.append(frame)

        time.sleep(self.timestep/100)

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