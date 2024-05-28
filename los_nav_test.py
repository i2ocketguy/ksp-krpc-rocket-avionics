import launch_utils as utils
import numpy as np
import pid
import time

conn, vessel = utils.initialize()
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)

def corrected_compute_los_angle(current_position, target_position):
    delta_y = target_position[0] - current_position[0]  # Change in latitude
    delta_x = target_position[1] - current_position[1]   # Change in longitude
    los_angle = np.arctan2(delta_y, delta_x)
    return los_angle

def corrected_compute_los_rate(current_velocity, current_position, target_position):
    los_angle = corrected_compute_los_angle(current_position, target_position)
    vel_angle = np.arctan2(current_velocity[1], current_velocity[2]) # atan2(y, x) = atan2(lat, lon) lat = N/S, long = E/W

    # Compute shortest angle difference in degrees
    los_rate_heading = np.degrees(vel_angle - los_angle)

    # Adjusting los_rate_heading to be between -180 to 180 degrees
    if los_rate_heading > 180:
        los_rate_heading -= 360
    elif los_rate_heading < -180:
        los_rate_heading += 360

    print(vessel.flight().heading, calculate_new_heading(vessel.flight().heading, los_rate_heading))

    return np.degrees(los_rate_heading) # vessel.flight(vessel.surface_reference_frame).heading - 

def normalize_angle(angle):
    return angle % 360

def calculate_new_heading(current_heading, los_rate_heading):
    new_heading = normalize_angle(current_heading + los_rate_heading)
    return new_heading

def compass_heading(angle):
    offset = 90
    if angle > 180:
        heading = normalize_angle(angle-offset)
    else:
        heading = normalize_angle(offset-angle)
    
    return heading


target_lat = vessel.flight().latitude
target_lon = vessel.flight().longitude
landing_site = (target_lat, target_lon)

yaw_control = pid.PID(1,0,0,0,360)
vessel.auto_pilot.engage()

while True:

    # Compute line of sight angle to landing pad
    current_position = (vessel.flight().latitude, vessel.flight().longitude)
    heading = corrected_compute_los_angle(current_position, landing_site)
    heading = compass_heading(np.degrees(heading))

    # Compute the rate of change of line of sight angle
    current_velocity = vessel.flight(ref_frame).velocity #vessel.flight(vessel.orbit.body.reference_frame).velocity
    rate_heading = corrected_compute_los_rate(current_velocity, current_position, landing_site)

    # Update control input
    yaw_control.set_point = rate_heading
    heading = yaw_control.update(vessel.flight().heading)
    # print(heading, rate_heading)
    vessel.auto_pilot.target_heading = heading

    time.sleep(0.1)