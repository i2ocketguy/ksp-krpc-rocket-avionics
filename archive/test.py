import launch_utils as utils
import krpc
import controllers
from math import degrees, atan2, sqrt
import numpy as np
from numpy import linalg

def initialize_test():
    conn, vessel = utils.initialize()
    test_retrograde_heading(conn, vessel)

def test_part_with_tag(vessel):
    # for part in vessel.parts.with_tag("pad-separator"):
    #     print(part.name)
        # part.docking_port.undock()

    part = vessel.parts.with_tag("control_point")[0]
    print(part.name)
    vessel.parts.controlling = part

def test_retrograde_heading(conn, vessel):
    body = vessel.orbit.body
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=body.reference_frame,
        rotation=vessel.surface_reference_frame,
    )
    # ref_frame = vessel.surface_reference_frame

    while True:
        # Obtain the velocity vector of the vessel in the non-rotating reassaasasference frame
        velocity_vector = vessel.flight(ref_frame).velocity
        
        # Calculate the horizontal velocity vector using x and z components
        # horizontal_velocity_vector = np.array([velocity_vector[0]])  # x and z components in non-rotating reference frame
        speed = np.linalg.norm(velocity_vector)

        # Calculate the direction to null out horizontal velocity
        retrograde = -np.array(velocity_vector) / speed
        correction_direction = np.arctan2(retrograde[2], retrograde[1])
        correction_direction_deg = np.degrees(correction_direction)
        if correction_direction_deg < 0:
            correction_direction_deg += 360

        # Convert correction direction to vessel's heading
        # Assuming the vessel's current heading is aligned with the y-axis direction
        # current_heading = vessel.flight(ref_frame).heading
        desired_heading = (correction_direction_deg)
            
        print(desired_heading, np.linalg.norm([velocity_vector[0], velocity_vector[1]]), vessel.flight(ref_frame).horizontal_speed)

initialize_test()