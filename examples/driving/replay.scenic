param time_step = 1.0/10
param replay = True

import math
import pandas
import scenic.formats.parse_bag as parse_bag
from scenic.core.dynamics.actions import _EndSimulationAction

base_dir = localPath("../replay_test")
bag_path = base_dir.joinpath("bags_0.mcap")
description_path = base_dir.joinpath("description.json")

param sim_data = parse_bag.bag_to_dataframe(bag_path, description_path)
param map_data = base_dir.joinpath("neighborhood_grid.png")

model scenic.domains.driving.model

class UAVObject(Object):
    targets_reported: []
    T: -math.inf

behavior Idle():
    while True:
        if self.targets_reported != []:
            print(self.targets_reported)
        wait

ego = new UAVObject with id "ego", with behavior Idle

car4006 = new Car with id "car4006", with color (0,1,0), with vehicle_type "SEDAN"

targets_groundtruth = [car4006]
targets_reported = ego.targets_reported

# This implements `eventually (\/ t in targets_reported: t.id == "car4006")
require eventually any([x.id=="car4006" for x in targets_reported])

# This implements
#`eventually (/\ x in targets_groundtruth :
#   (\/ y intargets_reported: x.id == y.id and (distance from x.position to y.position < 2*x.width)))`
require eventually all([
            any([
                ((y.id == x.id) and (distance from x.position to y.position < 2*x.width))
                for y in targets_reported])
            for x in targets_groundtruth])

# Attempting to implement example from https://github.com/darpa-ansr/assurance-claims/blob/main/eval01/doc/example_map.md
# true ->
# /\ p in targets_groundtruth:
#     (
#     eventually
#         \/ q in targets_reported:
#             (
#                 # reported to within two widths
#                 p.entity_id == q.entity_id /\
#                 in_region(q.pose.position, p.bounding_box)
#             )
#     )

# require all([
#     eventually any([
#         x.id == y.id and distance from x to y < 2*x.width for x in targets_reported])
#         for y in targets_groundtruth])

# Can't use eventually inside a list comprehension?
# require all([(eventually (y.id == "car4006")) for y in targets_groundtruth])

# The monitor never reaches the last `require` because termination happens earlier.
monitor AllTargetsEventuallyFound():
    targets = [x.id for x in targets_groundtruth]
    targets_found = set()
    print(f"Started: {targets}")
    while (sim_time_end - ego.T) > globalParameters["time_step"]:
        for y in targets_reported:
            for x in targets_groundtruth:
                if (x.id == y.id) and (distance from x.position to y.position < 5):
                    targets_found.add(y.id)
        wait
    print("Done")
    require len(targets_found) == len(targets)

require monitor AllTargetsEventuallyFound()

sim_time_end = float(globalParameters["sim_data"].tail(1).Timestamp.values[0])
terminate when (sim_time_end - ego.T) < globalParameters["time_step"]
