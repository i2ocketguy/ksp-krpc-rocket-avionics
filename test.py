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
import krpc
import proportional_navigation as PN



def test_part_with_tag(vessel):
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
        velocity=body.reference_frame
    )
    # ref_frame = body.reference_frame

    while True:
        velocity_vector = vessel.flight(ref_frame).velocity
        vmag = np.linalg.norm(velocity_vector)

        # Calculate the direction to null out horizontal velocity
        retrograde = -np.array(velocity_vector) / vmag
        correction_direction = np.arctan2(retrograde[2], retrograde[1])
        correction_direction_deg = np.degrees(correction_direction)
        if correction_direction_deg < 0:
            correction_direction_deg += 360

        # Convert correction direction to vessel's heading
        # Assuming the vessel's current heading is aligned with the y-axis direction
        # current_heading = vessel.flight(ref_frame).heading
        desired_heading = (correction_direction_deg)
            
        print(desired_heading, np.linalg.norm([velocity_vector[2], velocity_vector[1]]), vessel.flight(ref_frame).horizontal_speed)

def test_hoverslam(conn, vessel):
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
    vessel, telem = utils.check_active_vehicle(conn, vessel,
                                        mission_params.root_vessel)
    
    # landing_site = (-0.09720804982287173, -74.55761822488455) # LC-1
    # landing_site = (0.00248536258708449, -74.712536691836)   # LZ-1
    landing_site = (-0.122666607678452, -74.5432488018431)  # LZ-2

    # Create the hybrid reference frame
    body = vessel.orbit.body
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=body.reference_frame,
        rotation=vessel.surface_reference_frame,
        velocity=body.reference_frame
    )

    new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
    vessel.control.throttle = new_throttle_limit
    Tu = 300
    Ku = 2.5
    Kp = 0.2*Ku
    Ki = 2.0*Kp/Tu
    Kd = 2.0*Kp/Tu
    vert_vel_controller = controllers.PID(0.0, Kp, Ki, Kd, new_throttle_limit, 1.0, deadband=0.005)
    vert_vel_controller.set_point = -0.02  # vertical velocity target, m/s
    alt_controller = controllers.PID(0.0, .4, 0.005, 0.0, min_output=-20, max_output=10)
    alt_controller.set_point = telem.apoapsis()
    slam_controller = controllers.PID(0.0, 0.5, 0.01, 0.0, 0.0, 1.0, deadband=0.005)
    slam_controller.set_point = 0.3
    roll_controller = controllers.PID(0.0, 0.1, 0.0, 0.0, -20, 20, rate_limit=0.1)
    roll_controller.set_point = 180.0

    vessel.auto_pilot.engage()
    vessel.auto_pilot.auto_tune = True
    dist_controller = controllers.PID(0.0, 0.025, 0.0, 0.0, -20.0, 20.0, velocity_form=False)
    dist_controller.set_point = 0.0
    hvel_controller = controllers.PID(0.0, 0.5, 0.0, 0.0, -20.0, 20.0, velocity_form=False)
    mode = 2
    status = vessel.situation.landed
    starting_time = time.time()
    throttle_update = starting_time
    burn_flag = False
    burn_start_flag = False
    throttle_datastream = pu.data_stream_plot()
    altitude_datastream = pu.data_stream_plot()
    vertvel_datastream = pu.data_stream_plot()
    pitch_lpf = LPF(4,2,CLOCK_RATE)
    prev_dist = 0
    vel_sign = -1
    while vessel.situation != status:
        elapsed_time = time.time() - starting_time

        # Mode 2 is descent and landing burn start calculation
        if mode == 2:
            tb = steering.calculate_landing_burn_time(vessel)
            if telem.surface_altitude() < 3000:
                if vessel.thrust > 0:
                    burn_start_flag = True

                for thruster in vessel.parts.rcs:
                    thruster.enabled = True

                # Compute line of sight angle to landing pad
                current_position = (vessel.flight().latitude, vessel.flight().longitude)
                heading = steering.compass_heading(np.degrees(steering.compute_los_angle(current_position, landing_site)))

                # Compute pitch parameter inputs
                distance_to_pad = steering.haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
                distance_pitch = dist_controller.update(distance_to_pad)
                current_horizontal_velocity = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
                # Apply velocity sign based on whether moving toward/away from pad
                if distance_to_pad < prev_dist:
                    current_horizontal_velocity *= -1  # Moving away from pad
                prev_dist = distance_to_pad

                velocity_pitch = -hvel_controller.update(current_horizontal_velocity)
                pitch_input = distance_pitch + velocity_pitch

                # Update heading based on engine thrust status
                if burn_start_flag is False:
                    heading += 180
                    if heading > 360:
                        heading -= 360
                
                vessel.auto_pilot.target_heading = heading
                vessel.auto_pilot.target_pitch = 90-pitch_input
                vessel.auto_pilot.target_roll = float('NaN')

                if distance_to_pad > prev_dist:
                    vel_sign = -1
                else: # distance_to_pad < prev_dist:
                    vel_sign = 1

                prev_dist = distance_to_pad

                print(f"Mode 2b: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {distance_pitch:.2f}, {velocity_pitch:.2f}, {pitch_input:.2f}, {burn_start_flag}, {heading:.2f}")

            else:
                
                # Compute line of sight angle to landing pad
                current_position = (vessel.flight().latitude, vessel.flight().longitude)
                heading = steering.compass_heading(np.degrees(steering.compute_los_angle(current_position, landing_site)))

                # Compute pitch parameter inputs
                distance_to_pad = steering.haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
                distance_pitch = dist_controller.update(distance_to_pad)
                current_horizontal_velocity = vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed

                # Apply velocity sign based on whether moving toward/away from pad
                if distance_to_pad < prev_dist:
                    current_horizontal_velocity *= -1  # Moving away from pad
                prev_dist = distance_to_pad

                velocity_pitch = -hvel_controller.update(current_horizontal_velocity)
                pitch_input = distance_pitch + velocity_pitch

                # Roll control implementation
                current_heading = vessel.flight().heading
                heading_error = steering.compute_heading_error(current_heading, heading)
                roll_input = 1*roll_controller.update(heading_error)

                # Update control input
                vessel.auto_pilot.target_heading = heading
                vessel.auto_pilot.target_pitch = 90-pitch_input
                vessel.auto_pilot.target_roll = 180+roll_input

                print(f"Mode 2a: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {distance_pitch:.2f}, {velocity_pitch:.2f}, {pitch_input:.2f}, {roll_input:.2f}")

            if tb > -0.75 and telem.surface_altitude() < 2800 and burn_flag is False:
                vessel.control.throttle = slam_controller.update(tb)
                burn_flag = True
                vessel.auto_pilot.stopping_time = (1.2, 0.2, 0.2)

            if telem.surface_altitude() >= 50 and burn_flag is True:
                    vessel.control.throttle = slam_controller.update(tb)
            # if abs(telem.horizontal_vel()) > 20 and telem.surface_altitude() < 15000:
            #     for thruster in vessel.parts.rcs:
            #         thruster.enabled = True
            #     vessel.auto_pilot.target_pitch = 90
            if telem.surface_altitude() < 380 or telem.vertical_vel() > -25:
                mode = 3
                Tu = 225
                Ku = 2.5
                Kp = 0.2*Ku
                Ki = 2.0*Kp/Tu
                Kd = 0.0*Kp/Tu
                vert_vel_controller.set_gains(0.5, 0, 0)
                alt_controller.set_point = -5.0
                alt_controller.set_max_output(-3.0)
                alt_controller.set_min_output(-10.0)
                hvel_controller.set_max_output(10.0)
                hvel_controller.set_min_output(-10.0)
                hvel_controller.set_gains(0.5, 0.0, 0.0)

        # Mode 3 is constant descent rate at -5 m/s
        if mode == 3:
            vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
            vert_vel_controller.set_point = vert_vel_setpoint
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
                vessel.auto_pilot.target_roll = heading_raw
            else:
                pitch = 0
                heading = vessel.flight(ref_frame).heading
                vessel.auto_pilot.target_pitch = 90
                vessel.auto_pilot.target_heading = heading
                vessel.auto_pilot.target_roll = float('NaN')

            print(f"Mode 3: {distance_to_pad:.2f}, {v_mag:.2f}, {vessel.flight(ref_frame).horizontal_speed:.2f}, {pitch:.2f}")

        if int(time.time()) - int(throttle_update) > 5:  
            vert_vel_controller.set_min_output(new_throttle_limit)
            print("Current Machine State %i" % mode)
            throttle_update = time.time()

        # Plot states
        throttle_datastream.update_data_stream(elapsed_time, vessel.control.throttle)
        altitude_datastream.update_data_stream(elapsed_time, telem.altitude())
        vertvel_datastream.update_data_stream(elapsed_time, telem.vertical_vel())

        time.sleep(1/CLOCK_RATE)

    vessel.control.throttle = 0.0

def test_roll_control(conn, vessel):
    landing_site = (-0.09720804982287173, -74.55761822488455)
    status = vessel.situation.landed
    roll_controller = controllers.PID(1.0, 0.0, 0.1,-20, 20, 1.0)
    roll_controller.set_point = 0.0
    while vessel.situation != status:
        current_position = (vessel.flight().latitude, vessel.flight().longitude)
        heading = steering.compass_heading(np.degrees(steering.compute_los_angle(current_position, landing_site)))
        current_heading = vessel.flight().heading
        heading_error = steering.compute_heading_error(current_heading, heading)
        roll_input = -1*roll_controller.update(heading_error)

        print(f"{heading:.2f}, {heading_error:.2f}, {vessel.flight().roll:.2f}, {roll_input:.2f}")

def time_to_go(_, vessel):
    while True:
        tb = steering.calculate_landing_burn_time(vessel)
        print(tb)
    
def print_lat_lon(_, vessel):
    target_lat = vessel.flight().latitude
    target_lon = vessel.flight().longitude
    landing_site = (target_lat, target_lon)
    print(landing_site)

def altitude_controller(conn, vessel):
    Kp = 0.05
    Ki = 0.01
    Kd = 0.1
    alt_controller = controllers.PID(Kp, Ki, Kd, min_output=-5, max_output=5)
    alt_controller.set_point = 1000
    vert_vel_controller = controllers.PID(Kp, Ki, Kd, min_output=-5, max_output=5)
    vert_vel_controller.set_point = 0.0
    vertical_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
    while True:
        current_alt = vessel.flight().mean_altitude
        vert_vel_setpoint = alt_controller.update(current_alt)
        vert_vel_controller.set_point = vert_vel_setpoint
        pitch_cmd = vert_vel_controller.update(vertical_vel())
        vessel.control.pitch = pitch_cmd
        print(f"altitude_error:, {(current_alt-alt_controller.set_point):.2f}, vert: {vertical_vel():.2f},  pitch cmd: , {pitch_cmd:.2f}")
        time.sleep(0.1)

def test_guidance(conn, vessel):
    # Initialize connection
    target = conn.space_center.target_vessel
    
    # Create telemetry streams
    flight = vessel.flight(vessel.surface_reference_frame)
    target_flight = target.flight(vessel.surface_reference_frame)
    
    # Create guidance system
    guidance = PN.ProportionalNavigationGuidance(vessel, target, conn)
    
    try:
        guidance.start()
        
        while True:
            # Get current guidance vector
            vector = guidance.current_target_vector
            fpa, heading_err, magnitude = guidance.vector_to_local_directions(vector)
            
            # Clear screen (works in most terminals)
            print('\033[2J\033[H')
            
            # Print current state
            print(f"Current Flight State:")
            print(f"Altitude:     {flight.mean_altitude:8.1f} m")
            print(f"Ground Speed: {flight.speed:8.1f} m/s")
            print(f"Target Range: {guidance.get_target_range():8.1f} m")
            
            # Print guidance commands
            print(f"\nGuidance Commands:")
            print(f"{'↑' if fpa > 0 else '↓'} Pitch:    {abs(fpa):6.1f}°")
            print(f"{'→' if heading_err > 0 else '←'} Heading:  {abs(heading_err):6.1f}°")
            print(f"Magnitude: {magnitude:6.1f} m/s")
            
            # Print raw vector for debugging
            print(f"\nRaw Vector [surface frame]:")
            print(f"X: {vector[0]:8.2f}")
            print(f"Y: {vector[1]:8.2f}")
            print(f"Z: {vector[2]:8.2f}")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nTest ended by user")
    finally:
        guidance.stop()

def initialize_test():
    _conn, vessel = utils.initialize()
    # time_to_go(conn, vessel)
    test_hoverslam(_conn, vessel)

initialize_test()