import launch_utils as utils
import krpc
import pid
import math
import numpy as np
from numpy import linalg

def initialize_test():
    _conn, vessel = utils.initialize()
    test_part_with_tag(vessel)

def test_part_with_tag(vessel):
    # for part in vessel.parts.with_tag("pad-separator"):
    #     print(part.name)
        # part.docking_port.undock()

    part = vessel.parts.with_tag("control_point")[0]
    print(part.name)
    vessel.parts.controlling = part



if __name__ == "__main__":

    v1 = np.array([1,0,0])
    v2 = np.array([0,0,-1])

    dotted = v1.dot(v2)
    v1_norm = linalg.norm(v1)
    v2_norm = linalg.norm(v2)
    angle = np.arccos(dotted/(v1_norm*v2_norm))*180/np.pi
    print(angle)