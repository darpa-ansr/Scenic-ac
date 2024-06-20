"""Scenic world model for the Replay simulator.

This is a completely generic model that does not assume the scenario takes
place in a road network (unlike `scenic.simulators.newtonian.driving_model`).
"""

from scenic.simulators.replay.simulator import ReplaySimulator    # for use in scenarios

if 'render' not in globalParameters:
    render = True
else:
    render = globalParameters.render
simulator ReplaySimulator(None, render=render)
