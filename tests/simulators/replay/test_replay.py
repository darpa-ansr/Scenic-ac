import os
from pathlib import Path

from PIL import Image as IPImage
import pytest

from scenic.simulators.replay import ReplaySimulator
from tests.utils import pickle_test, sampleScene, tryPickling


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("replay.scenic")
    scene, _ = scenario.generate(maxIterations=1)
    simulator = ReplaySimulator()
    simulator.simulate(scene, maxSteps=100)
