param map = localPath('../../assets/maps/CARLA/Town01.xodr')
# param carla_map = 'Town05'
param time_step = 1.0/10


import pandas

param sim_data = pandas.read_csv(localPath("dataframe.csv"))
param topo_map = "/home/genindi1/projects/ANSR/metrics/test/simple_scenario/maneuver_task_output/maneuver_task_output.png"

model scenic.domains.driving.model

behavior Idle():
    while True:
        wait

targetCar = new Car with id "car4006"

ego = new Car with id "ego", with behavior Idle
ego.color = (0,0,1)