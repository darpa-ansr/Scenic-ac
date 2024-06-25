param time_step = 1.0/10
param replay = True

import pandas
import scenic.formats.parse_bag as parse_bag

base_dir = localPath("../replay_test")
bag_path = base_dir.joinpath("bags_0.mcap")
description_path = base_dir.joinpath("description.json")

param sim_data = parse_bag.bag_to_dataframe(bag_path, description_path)
param map_data = base_dir.joinpath("neighborhood_grid.png")

model scenic.domains.driving.model

behavior Idle():
    while True:
        wait

targetCar = new Car with id "car4006", with color (0,1,0), with vehicle_type "SEDAN"

ego = new Car with id "ego", with behavior Idle

# NOTE: Putting targetCar in the list causes targets_reported to be typed as
# scenic.core.distributions.TupleDistribution, which causes problems in replay.simulator.step().
# Because, for example, it isn't allowed to be used for control flow, i.e., cannot appear in
# conditional statements.

# NOTE: `targets_reported` is not cleared between consecutive randomly generated scenarios. This is
# not really an issue for assurance claim verification because it does not rely on the random
# scenario generation capability (in fact it should be turned off if possible). However, the
# variables like `targets_reported` can get stuck in a state that will cause newly generated
# scenarios to fail immediately without executing, which may seem odd.

targets_reported = []

# Note: This effectively implements `eventually (\/ t in targets_reported: t.id == "car4006") 
require eventually any([x.id=="car4006" for x in targets_reported])