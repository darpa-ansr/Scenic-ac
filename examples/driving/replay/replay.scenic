param time_step = 1.0/10
param replay = True

import json
import math
import pandas
import scenic.formats.parse_bag as parse_bag

import scenic_ansr

base_dir = localPath("sim_data")
bag_path = base_dir.joinpath("bags_0.mcap")
description_path = base_dir.joinpath("description.json")
param map_data = base_dir.joinpath("neighborhood_grid.png")

description = json.load(open(description_path))

param sim_data = parse_bag.bag_to_dataframe(bag_path, description_path)

model scenic.domains.driving.replay

class UAVObject(Object):
    targets_reported: []
    T: -math.inf

behavior Idle():
    while True:
        if self.targets_reported != []:
            print(self.targets_reported)
        # print(all([not (ego.position in z.region) for z in keep_out_zones]))
        wait

ego = new UAVObject with id "ego", with behavior Idle

car000 = new Car with id "car000", with color (1,0.5,0), with vehicle_type "SUV"
car001 = new Car with id "car001", with color (1,0,1), with vehicle_type "SUV"
car002 = new Car with id "car002", with color (0,0,1), with vehicle_type "SEDAN"
car003 = new Car with id "car003", with color (0,0,1), with vehicle_type "SEDAN"

# Do not sample
ego._needsSampling = False
car000._needsSampling = False
car001._needsSampling = False
car002._needsSampling = False
car003._needsSampling = False

# Do not check for collisions
# NOTE: Because objects are not sampled at the start of the simulation
#       they seem to be spawned in overlapping positions
ego.allowCollisions = True
car000.allowCollisions = True
car001.allowCollisions = True
car002.allowCollisions = True
car003.allowCollisions = True

keep_out_zones = scenic_ansr.parse_koz(description)
targets_groundtruth = [car000, car001, car002, car003]
targets_reported = ego.targets_reported

# require always not ego.position in keep_out_zones[1].region
# always
#    /\ z in keep_out_zones: not in_region(z.region, ego.pose.position)

# This should work per Daniel Fremont's email but errors out
# for z in keep_out_zones:
#     require always not (ego.position in z.region)

# THIS WORKS
require always all([(ego.position not in z.region) for z in keep_out_zones])

# but this never fails like it should
# require always all([not (ego.position in z.region) for z in keep_out_zones])

# There is a problem combining `always`, `all`, and `not`
#
# This
#   require always all([not True])
# never fails but this
#   require always all([False])
# fails as expected.

# This implements
#`eventually (/\ x in targets_groundtruth :
#   (\/ y intargets_reported: x.id == y.id and (distance from x.position to y.position < 2*x.width)))`
require eventually all([
            any([((y.id == x.id) and (distance from x.position to y.position < 2*x.width))
                for y in ego.targets_reported])
            for x in targets_groundtruth])

require eventually all([
            any([distance from x.position to y.position < 2*x.width
                for y in targets_reported if y.id == x.id])
            for x in targets_groundtruth])

monitor AllTargetsFound():
    while True:
        print(all([
            any([
                ((y.id == x.id) and (distance from x.position to y.position < 2*x.width))
                for y in ego.targets_reported])
            for x in targets_groundtruth]))
        wait

# require monitor AllTargetsFound()

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

# NOTE: Cannot have eventually inside the list comprehension
# require all([
#     eventually any([
#         x.id == y.id and distance from x to y < 2*x.width for x in targets_reported])
#         for y in targets_groundtruth])

# Terminate the simulation when the end of the mission is reached
sim_time_end = float(globalParameters["sim_data"].tail(1).Timestamp.values[0])
terminate when (sim_time_end - ego.T) < globalParameters["time_step"]
