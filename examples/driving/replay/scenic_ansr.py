from scenic.core.regions import BoxRegion
from scenic.core.object_types import Vector
import math

class Zone:
    def __init__(self, region:BoxRegion, no_earlier_than:float=-math.inf, no_later_than:float=math.inf)->None:
        self.region:BoxRegion = region
        self.no_earlier_than = no_earlier_than
        self.no_later_than = no_later_than

def parse_koz(description: dict) -> None:
    # Process Keep Out Zones
    keep_out_zones: list = []
    if ((description["scenario_constraints"] != [])
        and ("spatial_constraints" in description["scenario_constraints"])
        and ("keep_out_zones" in description["scenario_constraints"]["spatial_constraints"])):
        keep_out_zones_spec = description["scenario_constraints"]["spatial_constraints"]["keep_out_zones"]
    for z in keep_out_zones_spec:
        p = z["keep_out_polygon_vertices"]
        if p != []:
            # NOTE: Assuming the provided keep out zone is a list of consecutive vertices of a
            # rectangle with the first and last vertices coinciding
            center = Vector((p[0][0] + p[2][0])/2, (p[0][1] + p[2][1])/2)
            center1 = Vector((p[1][0] + p[3][0])/2, (p[1][1] + p[3][1])/2)
            # Sanity check the center computation
            # assert distance from center to center1 < 1e-9
            dimensions = (
                max([q[0] for q in p]) - min([q[0] for q in p]),
                max([q[1] for q in p]) - min([q[1] for q in p]),
                1e3
            )
            keep_out_zones.append(Zone(BoxRegion(position=center, dimensions=dimensions)))
    return keep_out_zones
