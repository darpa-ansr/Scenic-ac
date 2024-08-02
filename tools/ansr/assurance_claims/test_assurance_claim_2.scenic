import scenic.simulators.utils.parse_bag as parse_bag
from scenic.simulators.utils.scenic_ansr import Zone, COLORS
param time_step = 1.0/10
param render = False

param map_data = None
param sim_data = parse_bag.bag_to_dataframe(
    "sim_data/bags_0.mcap",
    ['/adk_node/input/perception', '/adk_node/SimpleFlight/odom_local_ned', '/adk_node/SimpleFlight/collision_state', '/airsim_node/car000/envcar_pose', '/airsim_node/car001/envcar_pose', '/airsim_node/car002/envcar_pose', '/airsim_node/car003/envcar_pose'],
    {'car000': {'color': 'orange', 'class': 'SUV'}, 'car001': {'color': 'violet', 'class': 'SUV'}, 'car002': {'color': 'blue', 'class': 'SEDAN'}, 'car003': {'color': 'violet', 'class': 'SEDAN'}}
)

model scenic.domains.driving.replay

class UAVObject(Object):
    targets_reported: []
    T: 0.0

keep_out_zones = [
	Zone(BoxRegion(position=Vector(75.0, 175.0), dimensions=(50.0, 50.0, 1000.0))),
	Zone(BoxRegion(position=Vector(175.0, -175.0), dimensions=(150.0, 150.0, 1000.0)))
]

car000 = new Car at (-43.0, 59.5),
    with yaw 1.571,
    with id 'car000',
    with name 'car000',
    with width 3,
    with length 4,
    with color COLORS['orange'],
    with vehicle_type 'SUV',
    with _needsSampling False

car001 = new Car at (-46.0, 56.5),
    with yaw 0.000,
    with id 'car001',
    with name 'car001',
    with width 3,
    with length 4,
    with color COLORS['violet'],
    with vehicle_type 'SUV',
    with _needsSampling False

car002 = new Car at (-154.75, 25.25),
    with yaw 0.000,
    with id 'car002',
    with name 'car002',
    with width 3,
    with length 4,
    with color COLORS['blue'],
    with vehicle_type 'SEDAN',
    with _needsSampling False

car003 = new Car at (-160.25, -41.25),
    with yaw -1.571,
    with id 'car003',
    with name 'car003',
    with width 3,
    with length 4,
    with color COLORS['violet'],
    with vehicle_type 'SEDAN',
    with _needsSampling False


ego = new UAVObject at (-120.0, 93.0),
    with id 'ego',
    with name 'UAV',
    with _needsSampling False

targets_groundtruth = {
    'car000': car000,
	'car001': car001,
	'car002': car002,
	'car003': car003
}
targets_reported = ego.targets_reported

# Terminate the simulation when the end of the mission is reached
sim_time_end = float(globalParameters["sim_data"].tail(1).Timestamp.values[0]) - float(globalParameters["sim_data"].head(1).Timestamp.values[0])
terminate when (sim_time_end - ego.T) < globalParameters["time_step"]

require (
eventually any([
    any([
        (p.id == q.id) and (distance from p.position to q.position <= 2*p.width)
        for q in targets_reported
    ])
    p in targets_groundtruth
])
)