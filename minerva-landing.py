import spacecraft as sc
import launch_utils as utils
import mission
import controllers
import time
import plotting_utils as pu
import matplotlib.pyplot as plt
import steering_logic as steering
from digitalfilter import low_pass_filter as LPF
import numpy as np
from math import radians, sin, cos, sqrt, atan2, degrees

def corrected_compute_los_angle(current_position, target_position):
    delta_y = target_position[0] - current_position[0]  # Change in latitude
    delta_x = target_position[1] - current_position[1]   # Change in longitude
    # print("target_position", target_position, "current_position:", current_position)
    # print("delta_x:", delta_x, "delta_y:", delta_y)
    los_angle = np.arctan2(delta_y, delta_x)
    # print("los_angle (radians):", los_angle)
    return los_angle

def compass_heading(angle):
    offset = 90
    if angle > 180:
        heading = normalize_angle(angle-offset)
    else:
        heading = normalize_angle(offset-angle)
    
    return heading

def normalize_angle(angle):
    return angle % 360

def haversine_distance(lat1, lon1, lat2, lon2):
    r = 600000
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2-lat1)
    delta_lambda = radians(lon2-lon1)
    a = sin(delta_phi / 2)**2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance = r * c
    return distance

def compute_heading_error(current_heading, target_heading):
    error = target_heading - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

def landing_site_coord(landing_site_name):
    if landing_site_name == "LZ-1":
        return(0.00248536258708449, -74.712536691836)
    elif landing_site_name == "LZ-2":
        return (-0.391534560271904, -74.5353650808787)
    else:
        return (-0.391534560271904, -74.5353650808787)

conn, vessel = utils.initialize()
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = vessel.name
mission_params = mission.MissionParameters(root_vessel,
                                        state="init",
                                        target_inc=0,
                                        target_roll=180,
                                        altimeter_bias=71,
                                        grav_turn_end=85000,
                                        max_q=16000,
                                        max_g=3.0)
landing_site_name = "LZ-2"
vessel= utils.check_active_vehicle(conn, root_vessel)
telem = mission.Telemetry(conn, vessel)
landing_site = landing_site_coord(landing_site_name)

# Create the hybrid reference frame
body = vessel.orbit.body
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=body.reference_frame,
    rotation=vessel.surface_reference_frame,
    velocity=body.reference_frame
)

vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time = (0.5, 1.8, 0.6)
new_throttle_limit = 0.0
vessel.control.throttle = new_throttle_limit
vert_vel_controller = controllers.PID(0.0, 0.25, 0.01, 0, new_throttle_limit, 1.0)
vert_vel_controller.set_point = -0.02  # vertical velocity target, m/s
alt_controller = controllers.PID(0.0, 0.5, 0.05, 0.05, min_output=-20, max_output=10, velocity_form=False)
alt_controller.set_point = telem.apoapsis()
slam_controller = controllers.PID(0.0, 1, 0.3, 0.0, 0.0, 1.0)
slam_controller.set_point = 1.0
roll_controller = controllers.PID(0.0, 0.25, 0.01, 0.0,-20, 20)
dist_controller = controllers.PID(0.0, 0.15, 0.0, 0.01, -250.0, 250.0, velocity_form=False)
hvel_controller = controllers.PID(0.0, 0.3, 0.09, 0.05, -60.0, 60.0)
pad_targeting_pid = controllers.CascadeController(dist_controller, hvel_controller, True)

for thruster in vessel.parts.rcs:
    thruster.enabled = True
vessel.control.rcs =True
for cs in vessel.parts.control_surfaces:
    cs.pitch_enabled = True
    cs.yaw_enabled = True
    # cs.roll_enabled = True
0
mode = 1
status = vessel.situation.landed
starting_time = time.time()
throttle_update = starting_time
burn_flag = False
burn_start_flag = False
single_engine_flag = False
throttle_datastream = pu.data_stream_plot()
altitude_datastream = pu.data_stream_plot()
vertvel_datastream = pu.data_stream_plot()
distance_to_pad = 10000
prev_dist = 0
vel_sign = -1
starting_LF = vessel.resources.amount("LiquidFuel")
g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
engine_offset = (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
# vessel.auto_pilot.target_roll = 0.0
while vessel.situation != status:
    elapsed_time = time.time() - starting_time
    
    # Mode 1 is descent and landing burn start calculation
    if mode == 1:
        tb = steering.calculate_landing_burn_time(vessel, g, engine_offset)
        if telem.surface_altitude() < 1500 or distance_to_pad < 50 or (tb < 0.5 and telem.surface_altitude() < 3000):
            mode = 2
            pad_targeting_pid.set_max_output(20)
            pad_targeting_pid.set_min_output(-20)
            pad_targeting_pid.reset()

        # Compute line of sight angle to landing pad
        current_position = (vessel.flight().latitude, vessel.flight().longitude)
        heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))
        heading += 180
        if heading > 360:
                heading -= 360
        # Compute pitch parameter inputs
        distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
        current_horizontal_velocity = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
        
        # Apply velocity sign based on whether moving toward/away from pad
        if distance_to_pad < prev_dist:
            current_horizontal_velocity *= -1  # Moving away from pad
        prev_dist = distance_to_pad

        pitch_input = pad_targeting_pid.update(distance_to_pad, current_horizontal_velocity)

        # Roll control implementation
        current_heading = vessel.flight().heading
        heading_error = steering.compute_heading_error(current_heading, heading)
        roll_input = -1*roll_controller.update(heading_error)

        # Update control input
        vessel.auto_pilot.target_heading = heading
        vessel.auto_pilot.target_roll = 0
        vessel.auto_pilot.target_pitch = 90+pitch_input

        print(f"Mode 1 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {pitch_input:.2f}, {tb:.2f}")

    if mode == 2:
        tb = steering.calculate_landing_burn_time(vessel, g, engine_offset)

        for thruster in vessel.parts.rcs:
            thruster.enabled = True

        # Compute line of sight angle to landing pad
        current_position = (vessel.flight().latitude, vessel.flight().longitude)
        heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

        # Compute pitch parameter inputs
        distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
        current_horizontal_velocity = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
        
        # Apply velocity sign based on whether moving toward/away from pad
        if distance_to_pad < prev_dist:
            current_horizontal_velocity *= -1  # Moving away from pad
        prev_dist = distance_to_pad

        # Update cascade controller
        pitch_input = pad_targeting_pid.update(distance_to_pad, current_horizontal_velocity)

        # Update heading based on engine thrust status
        if burn_flag is True and telem.velocity() > 120:
            heading += 180
        elif burn_flag is False:
            heading += 180
            
        if heading > 360:
                heading -= 360

        # Roll control implementation
        current_heading = vessel.flight().heading
        heading_error = steering.compute_heading_error(current_heading, heading)
        roll_input = -1*roll_controller.update(heading_error)

        vessel.auto_pilot.target_heading = heading
        vessel.auto_pilot.target_pitch = 90+pitch_input
        vessel.auto_pilot.target_roll = roll_input

        print(f"Mode 2 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {pitch_input:.2f}, {tb:.2f}")

        if (tb < 0.5 or telem.surface_altitude() < 1500) and burn_flag is False:
            vessel.control.throttle = slam_controller.update(tb)
            burn_flag = True
            vessel.auto_pilot.stopping_time = (0.8, 0.8, 0.8)

        if telem.surface_altitude() >= 50 and burn_flag is True:
                vessel.control.throttle = slam_controller.update(tb)

        if telem.surface_altitude() < 300:
            mode = 3
            vert_vel_controller.set_gains(0.25, 0.1, 0)
            alt_controller.set_point = 10.0
            alt_controller.set_max_output(10.0)
            alt_controller.set_min_output(-30.0)
            hvel_controller.set_max_output(15.0)
            hvel_controller.set_min_output(-15.0)
            hvel_controller.set_gains(0.1, 0.5, 0.1)
            dist_controller.set_gains(0.1, 0.0, 0.0)

        if telem.vertical_vel() > -30 and not single_engine_flag:
            single_engine_flag = True
            engine_2 = vessel.parts.with_tag("engine_2")[0]
            engine_2.engine.active = False
            engine_3 = vessel.parts.with_tag("engine_3")[0]
            engine_3.engine.active = False

    # Mode 4 is null horizontal velocity and slew to pad
    if mode == 3:
        if telem.vertical_vel() > -30 and not single_engine_flag:
            single_engine_flag = True
            engine_2 = vessel.parts.with_tag("engine_2")[0]
            engine_2.engine.active = False
            engine_3 = vessel.parts.with_tag("engine_3")[0]
            engine_3.engine.active = False
        if vessel.resources.amount("LiquidFuel") < starting_LF*0.12:
            mode = 4
            alt_controller.set_gains(0.5, 0.1, 0.05)
            vert_vel_controller.set_gains(0.25, 0.05, 0)
            hvel_controller.set_max_output(10.0)
            hvel_controller.set_min_output(-10.0)
            hvel_controller.set_gains(0.5, 0.15, 0.05)
            vessel.auto_pilot.stopping_time = (0.9, 0.9, 0.9)
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        v_vec = vessel.flight(ref_frame).velocity
        v_mag = np.linalg.norm(v_vec)

        if vessel.flight(ref_frame).horizontal_speed > 1:
            # Calculate the direction to apply thrust
            retro_vec = -np.array(v_vec) / v_mag
            # Use the PID controller to adjust the pointing vector
            pitch_input = hvel_controller.update(vessel.flight(ref_frame).horizontal_speed)
            # Calculate the heading using atan2 and convert to 0-360 degrees
            heading_raw = degrees(atan2(retro_vec[2], retro_vec[1]))
            if heading_raw < 0:
                heading = heading_raw + 360
            else:
                heading = heading_raw
            
            # Set the target direction for the autopilot
            vessel.auto_pilot.target_pitch = 90+pitch_input
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')
        else:
            pitch_input = 0
            heading = vessel.flight(ref_frame).heading
            vessel.auto_pilot.target_pitch = 90
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')
            mode = 4
            alt_controller.set_gains(0.5, 0.1, 0.05)
            vert_vel_controller.set_gains(0.25, 0.05, 0)
            hvel_controller.set_max_output(10.0)
            hvel_controller.set_min_output(-10.0)
            hvel_controller.set_gains(0.5, 0.15, 0.05)
            vessel.auto_pilot.stopping_time = (0.9, 0.9, 0.9)

        print(f"Mode 3 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity}, {pitch_input:.2f}")

    # constant rate of descent mode
    if mode == 4:
        if telem.vertical_vel() > -30 and not single_engine_flag:
            single_engine_flag = True
            engine_2 = vessel.parts.with_tag("engine_2")[0]
            engine_2.engine.active = False
            engine_3 = vessel.parts.with_tag("engine_3")[0]
            engine_3.engine.active = False
        if vessel.flight(ref_frame).horizontal_speed > 1:
            alt_controller.set_point = -3.0
            alt_controller.set_max_output(10.0)
            alt_controller.set_min_output(-10.0)
        else:
            alt_controller.set_point = -10.0
            alt_controller.set_max_output(-4.0)
            alt_controller.set_min_output(-10.0)
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_setpoint(vert_vel_setpoint)
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())

        # Get the vessel's velocity relative to the surface
        v_vec = vessel.flight(ref_frame).velocity
        v_mag = np.linalg.norm(v_vec)

        if vessel.flight(ref_frame).horizontal_speed > 1:
            # Calculate the direction to apply thrust
            retro_vec = -np.array(v_vec) / v_mag
            # Use the PID controller to adjust the pointing vector
            pitch = hvel_controller.update(vessel.flight(ref_frame).horizontal_speed)
            # Calculate the heading using atan2 and convert to 0-360 degrees
            heading_raw = degrees(atan2(retro_vec[2], retro_vec[1]))
            if heading_raw < 0:
                heading = heading_raw + 360
            else:
                heading = heading_raw
            
            # Set the target direction for the autopilot
            vessel.auto_pilot.target_pitch = 90+pitch
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')

        else:
            pitch = 0
            heading = vessel.flight(ref_frame).heading
            vessel.auto_pilot.target_pitch = 90
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')

        print(f"Mode 4 Outputs: {distance_to_pad:.2f}, {telem.horizontal_vel():2f}, {pitch:.2f}, {vert_vel_setpoint:.2}, {alt_controller.set_point:.2f}")

    if telem.surface_altitude() < 30:
        vessel.control.gear = True

    # if int(time.time()) - int(throttle_update) > 5:
    #     if telem.surface_altitude()-target_alt < 50 and telem.vertical_vel() < -10:
    #         continue
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.25)
    #     elif mode == 5:
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.5)
    #         vert_vel_controller.set_max_output(utils.throttle_from_twr(vessel, 1.1))
    #     else:
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        
    #     vert_vel_controller.set_min_output(new_throttle_limit)
    #     print("Current Machine State %i" % mode)
    #     throttle_update = time.time()

    # Plot states
    throttle_datastream.update_data_stream(elapsed_time, vessel.control.throttle)
    altitude_datastream.update_data_stream(elapsed_time, telem.altitude())
    vertvel_datastream.update_data_stream(elapsed_time, telem.vertical_vel())

    time.sleep(1/CLOCK_RATE)

vessel.control.throttle = 0.0
vessel.auto_pilot.disengage()
time.sleep(10/CLOCK_RATE)
vessel.control.rsc = False
time.sleep(10/CLOCK_RATE)
vessel.control.sas = True
