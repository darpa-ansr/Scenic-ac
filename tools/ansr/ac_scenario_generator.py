# Copyright 2024 The Johns Hopkins University Applied Physics Laboratory LLC
import json
import marko
import os
import argparse

koz_def_str = """Zone(BoxRegion(position=Vector({center[0]}, {center[1]}), dimensions={dimensions}))"""

ego_def_str = """
ego = new UAVObject at ({location[0]}, {location[1]}),
    with id 'ego',
    with name 'UAV',
    with _needsSampling False
"""

eoi_def_str = """{id} = new Car at ({location[0]}, {location[1]}),
    with yaw {yaw:.3f},
    with id '{id}',
    with name '{id}',
    with width 3,
    with length 4,
    with color COLORS['{color}'],
    with vehicle_type '{vehicle_type}',
    with _needsSampling False
"""

preamble = """import scenic.simulators.utils.parse_bag as parse_bag
from scenic.simulators.utils.scenic_ansr import Zone, COLORS
param time_step = 1.0/10
param render = False

param map_data = {map_path}
param sim_data = parse_bag.bag_to_dataframe(
    "{bag_path}",
    {topics},
    {entity_attribute_map}
)

model scenic.domains.driving.replay

class UAVObject(Object):
    targets_reported: []
    T: 0.0

"""

coda = """
targets_groundtruth = {{
    {id_list}
}}
targets_reported = ego.targets_reported

# Terminate the simulation when the end of the mission is reached
sim_time_end = float(globalParameters["sim_data"].tail(1).Timestamp.values[0]) - float(globalParameters["sim_data"].head(1).Timestamp.values[0])
terminate when (sim_time_end - ego.T) < globalParameters["time_step"]

"""

def parse_assurance_claim(assurance_claim_file: str) -> str:
    """
    Extracts assurance claim formula from the assurance claim markdown document
    """
    claim_doc: marko.block.Document = marko.parse(open(assurance_claim_file).read())
    claim_heading_ind = -1
    claim_assertion = ""
    # Find the Claim section of the assurance claim document
    for i, c in enumerate(claim_doc.children):
        if (isinstance(c, marko.block.Heading)
            and len(c.children) > 0
            and isinstance(c.children[0], marko.inline.RawText)
            and c.children[0].children == "Claim"):
            claim_heading_ind = i
    if claim_heading_ind < 0:
        raise Exception("The provided file does not have a Claim section")

    # Extract the code of claim
    for i, c in enumerate(claim_doc.children[claim_heading_ind:]):
        # print(c)
        if (isinstance(c, marko.block.FencedCode)
            and len(c.children) > 0
            and isinstance(c.children[0], marko.inline.RawText)):
            claim_assertion = c.children[0].children
            break

    return claim_assertion

def parse_koz(description: dict) -> list:
    """
    Extracts keep-out zones coordinates and dimensions from the description dict
    """
    # Process Keep Out Zones
    keepout_zones: list = []
    keepout_zones_spec: dict  = {}
    if ((description["scenario_constraints"] != [])
        and ("spatial_constraints" in description["scenario_constraints"])
        and ("keep_out_zones" in description["scenario_constraints"]["spatial_constraints"])):
        keepout_zones_spec = description["scenario_constraints"]["spatial_constraints"]["keep_out_zones"]
    for z in keepout_zones_spec:
        p = z["keep_out_polygon_vertices"]
        if p != []:
            # NOTE: Assuming the provided keep out zone is a list of consecutive vertices of a
            # rectangle with the first and last vertices coinciding
            center = ((p[0][0] + p[2][0])/2, (p[0][1] + p[2][1])/2)
            # Sanity check the center computation
            # center1 = ((p[1][0] + p[3][0])/2, (p[1][1] + p[3][1])/2)
            # assert distance from center to center1 < 1e-9
            dimensions = (
                max([q[0] for q in p]) - min([q[0] for q in p]),
                max([q[1] for q in p]) - min([q[1] for q in p]),
                1e3
            )
            keepout_zones.append((center, dimensions))
    return keepout_zones

def parse_eois(config: dict) -> list:
    """
    Extract entities of interest section from the config dict
    """
    eois_spec: list = []
    if (config["entities_of_interest"] != []):
        eois_spec = config["entities_of_interest"]
    return eois_spec

def parse_mission_description(description_file) -> str:
    """
    Constructs scenario definitions needed for AC verification from the description file 
    """
    global koz_def_str
    description = json.load(open(description_file))
    keepout_zones = parse_koz(description)
    koz_defs = []
    for z in keepout_zones:
        koz_defs.append(koz_def_str.format(center=z[0], dimensions=z[1]))
    koz_definition = "keep_out_zones = [\n\t" + ",\n\t".join(koz_defs) + "\n]\n\n"
    return koz_definition

def parse_mission_config(config_file: str) -> tuple:
    """
    Constructs scenario definitions needed for AC verification from the config file
    """
    global eoi_def_str, ego_def_str
    config = json.load(open(config_file))
    eois: list = parse_eois(config)
    entity_defs: list = []
    entity_attrs: dict = {}
    for e in eois:
        entity_attrs[e["entity_id"]] = e["attributes"]
        entity_defs.append(
            eoi_def_str.format(
                id=e["entity_id"],
                color=e["attributes"]["color"],
                vehicle_type=e["attributes"]["class"],
                location=e["location_ground_truth"][0:2],
                yaw=e["location_ground_truth"][2]
            )
        )
    entity_defs.append(ego_def_str.format(location=config["controllable_vehicle_start_loc"][0:2]))
    return "\n".join(entity_defs), entity_attrs

def main(args: argparse.Namespace) -> None:
    """
    Main
    """
    koz_defs = ""
    entity_defs = ""
    enitities = []
    topics = ['/adk_node/input/perception',
              '/adk_node/SimpleFlight/odom_local_ned',
              '/adk_node/SimpleFlight/collision_state']

    for f in os.listdir(args.mission_dir):
        if os.path.basename(f) == "config.json":
            entity_defs, entity_attributes = parse_mission_config(os.path.join(args.mission_dir, f))
        elif os.path.basename(f) == "description.json":
            koz_defs = parse_mission_description(os.path.join(args.mission_dir, f))
    topics += [f"/airsim_node/{i}/envcar_pose" for i in entity_attributes]
    targets_groundtruth_def = ",\n\t".join(map(lambda x:"'{0}': {0}".format(x), entity_attributes.keys()))
    for f in os.listdir(args.assurance_claim_dir):
        if f.endswith(".md"):
            scenario = preamble.format(bag_path=os.path.join(args.bag_dir, "bags_0.mcap"),
                                       topics=topics,
                                       entity_attribute_map=entity_attributes,
                                       map_path=args.map_path)
            scenario += koz_defs
            scenario += entity_defs
            scenario += coda.format(id_list=targets_groundtruth_def)
            scenario += f"require (\n{parse_assurance_claim(os.path.join(args.assurance_claim_dir, f))})"
            open(os.path.join(args.assurance_claim_dir, os.path.splitext(f)[0]+".scenic"), "w").write(scenario)

if __name__ == "__main__":
    """
    Parses command line arguments and calls main()
    """
    parser = argparse.ArgumentParser(
        usage="python3 ac_scenario_generator.py <mission inputs directory> <bag directory> <assurance claim directory>",
        description="Scenic scenario file generator for assurance claim verification")
    parser.add_argument(
        "mission_dir", help="The mission briefing directory containing config.json and description.json"
    )
    parser.add_argument(
        "bag_dir", help="The directory containing the bag file to evaluate"
    )
    parser.add_argument(
        "assurance_claim_dir", help="The directory containing assurance claim markdown documents"
    )
    parser.add_argument(
        "--map_path", help="Path to the map image file", default=None
    )

    # Parse command line arguments
    args: argparse.Namespace = parser.parse_args()
    main(args)
