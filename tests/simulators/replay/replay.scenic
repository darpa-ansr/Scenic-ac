param time_step = 1.0/10
param replay = True

import json
import math
import pandas
import scenic.simulators.utils.parse_bag as parse_bag
import scenic.simulators.utils.scenic_ansr as scenic_ansr

base_dir = localPath("sim_data")
bag_path = base_dir.joinpath("bags_0.mcap")
description_path = base_dir.joinpath("description.json")
param map_data = base_dir.joinpath("neighborhood_grid.png")

description = json.load(open(description_path))

param sim_data = parse_bag.bag_to_dataframe(bag_path,
    [
        '/adk_node/input/perception',
        '/adk_node/SimpleFlight/odom_local_ned',
        '/adk_node/SimpleFlight/collision_state',
        '/airsim_node/car000/envcar_pose',
        '/airsim_node/car001/envcar_pose',
        '/airsim_node/car002/envcar_pose',
        '/airsim_node/car003/envcar_pose'
    ],
    {
        'car000': {'color': 'yellow', 'class': 'SEDAN'},
        'car001': {'color': 'violet', 'class': 'SUV'},
        'car002': {'color': 'blue', 'class': 'SEDAN'},
        'car003': {'color': 'blue', 'class': 'SEDAN'}
     })

keep_out_zones = [
    scenic_ansr.Zone(BoxRegion(position=Vector(75, 175, 0), dimensions=Vector(50, 50, 1e3))),
    scenic_ansr.Zone(BoxRegion(position=Vector(-175, -175, 0), dimensions=Vector(150, 150, 1e3)))
]

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

targets_groundtruth = [car000, car001, car002, car003]
targets_reported = ego.targets_reported

# require always not ego.position in keep_out_zones[1].region
# always
#    /\ z in keep_out_zones: not in_region(z.region, ego.pose.position)

require always all([(ego.position not in z.region) for z in keep_out_zones])

# Terminate the simulation when the end of the mission is reached
sim_time_end = float(globalParameters["sim_data"].tail(1).Timestamp.values[0])
terminate when (sim_time_end - ego.T) < globalParameters["time_step"]
