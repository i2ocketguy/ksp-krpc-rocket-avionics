import spacecraft as sc
import launch_utils as utils
import mission
import pid
import time
import plotting_utils as pu
import matplotlib.pyplot as plt
import steering_logic as steering
from digitalfilter import low_pass_filter as LPF
import numpy as np
from math import radians, sin, cos, sqrt, atan2, degrees

# TODO: 
#       ADD Line-of-Sight navigation
#       ADD PID control for roll and yaw

def euler_step(vessel, h, v, dt):
    h = h + v * dt
    r = vessel.orbit.body.equatorial_radius + h 
    Ft = 0 # coasting, no thrust
    rho = 1.7185*np.exp(-2e-04*h)
    Cd_A = 3.5 # kg/m^3
    Fd =  0.5*rho*v**2*Cd_A
    Fg = vessel.orbit.body.gravitational_parameter * vessel.mass / r**2
    a = (Ft - Fd - Fg) / vessel.mass
    v = v + a * dt

    return h, v

def compute_los_angle(current_position, target_position):
    delta_x = target_position[0] - current_position[0]  # Change in latitude
    delta_y = target_position[1] - current_position[1]   # Change in longitude
    # print("target_position", target_position, "current_position:", current_position)
    # print("delta_x:", delta_x, "delta_y:", delta_y)
    los_angle = np.arctan2(-delta_y, delta_x)
    # print("los_angle (radians):", los_angle)
    return los_angle

def corrected_compute_los_angle(current_position, target_position):
    delta_y = target_position[0] - current_position[0]  # Change in latitude
    delta_x = target_position[1] - current_position[1]   # Change in longitude
    # print("target_position", target_position, "current_position:", current_position)
    # print("delta_x:", delta_x, "delta_y:", delta_y)
    los_angle = np.arctan2(delta_y, delta_x)
    # print("los_angle (radians):", los_angle)
    return los_angle

def compute_los_rate(current_velocity, current_position, target_position):
    los_angle = compute_los_angle(current_position, target_position)
    los_angle = normalize_angle(np.degrees(los_angle))
    vel_angle = np.arctan2(current_velocity[0], current_velocity[1])
    vel_angle = normalize_angle(np.degrees(vel_angle))

    # Compute shortest angle difference
    los_rate = vel_angle - los_angle
    # if los_rate >= 180: ******
    #     los_rate -= 360
    # elif los_rate < -180:
    #     los_rate += 360

    return los_rate

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

# constants
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "DCX"
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 0
payload_LF = 0

meco_condition_multiplier = 0.06  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 100000  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0,
                                           target_roll=180,
                                           altimeter_bias=71,
                                           grav_turn_end=85000,
                                           max_q=16000,
                                           max_g=3.0)
dcx = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  dcx.bref)
target_lat = vessel.flight().latitude
target_lon = vessel.flight().longitude
landing_site = (target_lat, target_lon)

# Create the hybrid reference frame
body = vessel.orbit.body
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=body.reference_frame,
    rotation=vessel.surface_reference_frame,
    velocity=body.reference_frame
)

# Pre-Launch
utils.launch_countdown(3)
vessel.control.activate_next_stage()  # Engine Ignition

if vessel.available_thrust < 10:
    for engine in vessel.parts.engines:
        engine.active = True

vessel.auto_pilot.engage()
vessel.auto_pilot.auto_tune = True
throttle_limit = utils.throttle_from_twr(vessel, 1.5)
vessel.control.throttle = throttle_limit
vessel.control.sas = True
vessel.control.rcs = True
time.sleep(1/dcx.CLOCK_RATE)
vessel.control.toggle_action_group(1)
vessel.auto_pilot.target_roll = vessel.flight().roll
vessel.auto_pilot.target_pitch_and_heading(88.50, mission_params.target_heading)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)

time.sleep(3)
vessel.control.gear = False

# Wait until target alititude is achieved
target_alt = 69000
dt = 0.5
while True:
    #predict future apoapsis
    h_future = vessel.flight().mean_altitude
    v_future = telem.vertical_vel() # np.linalg.norm(np.array(vessel.flight().velocity))
    for _ in range(int(vessel.orbit.time_to_apoapsis/dt)):
        h_old = h_future
        h_future, v_future = euler_step(vessel, h_future, v_future, dt)
        if h_future < h_old:
            break

    if h_future > target_alt:
        vessel.control.throttle = 0.0
        break

    # tweak = 0.90 # fudge factor for strength of drag correction, probably somewhere in the 0.9 - 1.0 range is most accurate
    # scale_h = 5600.0 * vessel.orbit.body.atmosphere_depth / 70000.0 # rough atmosphere 1/e scale height in m
    # drag = vessel.flight().drag
    # d_loss = tweak * scale_h * sqrt(drag[0]*drag[0] + drag[1]*drag[1] + drag[2]*drag[2]) # energy loss to drag in J
    # h_future = 1.0/(1.0/vessel.orbit.apoapsis + d_loss/(vessel.orbit.body.gravitational_parameter * vessel.mass)) - vessel.orbit.body.equatorial_radius

    # print(h_future)

while telem.vertical_vel() > 2:
    pass

print("Passed %i, entering vertical velocity hold ..." % target_alt)
new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
vessel.control.throttle = new_throttle_limit
Tu = 300
Ku = 2.5
Kp = 0.2*Ku
Ki = 2.0*Kp/Tu
Kd = 2.0*Kp/Tu
vert_vel_controller = pid.PID(Kp, Ki, Kd, new_throttle_limit, 1.0, deadband=0.005)
vert_vel_controller.set_point = -0.02  # vertical velocity target, m/s
alt_controller = pid.PID(.4, 0.005, 0.0, min_output=-20, max_output=10)
alt_controller.set_point = telem.apoapsis()
slam_controller = pid.PID(5, 0.0, 0.0, 0.0, 1.0, deadband=0.005)
slam_controller.set_point = 0.5

dist_controller = pid.PID(0.025, 0.0, 0.3, -20.0, 20.0, deadband=0.005)
dist_controller.set_point = 0.0
hvel_controller = pid.PID(0.1, 0.01, 0.01, -20.0, 20.0, deadband=0.005)
hvel_controller.set_point = 0.0

for thruster in vessel.parts.rcs:
    thruster.enabled = True

mode = 1
status = vessel.situation.landed
starting_time = time.time()
throttle_update = starting_time
burn_flag = False
burn_start_flag = False
throttle_datastream = pu.data_stream_plot()
altitude_datastream = pu.data_stream_plot()
vertvel_datastream = pu.data_stream_plot()
pitch_lpf = LPF(4,2,dcx.CLOCK_RATE)
prev_dist = 0
vel_sign = -1
while vessel.situation != status:
    elapsed_time = time.time() - starting_time

    # Mode 1 is hover at target altitude
    if mode == 1:
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        if int(elapsed_time) > 1:
            vessel.control.throttle = 0.0
            mode = 2
            vessel.auto_pilot.stopping_time = (0.3, 0.3, 0.3)
            vessel.control.toggle_action_group(1)
            vessel.auto_pilot.target_pitch = -10.0
            vessel.auto_pilot.target_roll = 0.0
            print("Exiting altitude hold ...")
            while telem.vertical_vel() > -10:
                pass
            vessel.auto_pilot.stopping_time = (0.5, 0.3, 0.5)
            # roll_control = pid.PID(1.0, 0.1, 0.5, -30, 30)
            # yaw_control = pid.PID(0.1, 0.05, 0.1, -180, 180)

        
    
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
            heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

            # Compute pitch parameter inputs
            distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
            distance_pitch = dist_controller.update(distance_to_pad)
            current_horizontal_velocity = vel_sign*vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
            velocity_pitch = -hvel_controller.update(current_horizontal_velocity)
            pitch_input = distance_pitch + velocity_pitch

            # Update heading based on engine thrust status
            if burn_start_flag is False or abs(vessel.flight(ref_frame).vertical_speed) > 50 :
                heading += 180
                if heading > 360:
                    heading -= 360
            
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_pitch = pitch_lpf(90+pitch_input)

            if distance_to_pad > prev_dist:
                vel_sign = -1
            else: # distance_to_pad < prev_dist:
                vel_sign = 1

            prev_dist = distance_to_pad

            print(f"{distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {distance_pitch:.2f}, {velocity_pitch:.2f}, {pitch_input:.2f}, {burn_start_flag}, {heading:.2f}")

        else:
            
            # Compute line of sight angle to landing pad
            current_position = (vessel.flight().latitude, vessel.flight().longitude)
            heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

            # Compute pitch parameter inputs
            distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
            distance_pitch = dist_controller.update(distance_to_pad)
            current_horizontal_velocity = vel_sign*vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed
            velocity_pitch = -hvel_controller.update(current_horizontal_velocity)
            pitch_input = distance_pitch + velocity_pitch
            # Update control input
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_pitch = pitch_lpf(pitch_input)

            if distance_to_pad > prev_dist:
                vel_sign = -1
            else: # distance_to_pad < prev_dist:
                vel_sign = 1

            prev_dist = distance_to_pad

            print(f"{distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {distance_pitch:.2f}, {velocity_pitch:.2f}, {pitch_input:.2f}")

        if tb > -0.75 and telem.surface_altitude() < 8000 and burn_flag is False:
            vessel.control.throttle = slam_controller.update(tb)
            burn_flag = True
            vessel.auto_pilot.stopping_time = (1.2, 0.2, 0.2)

        if telem.surface_altitude() >= 50 and burn_flag is True:
                vessel.control.throttle = slam_controller.update(tb)
        # if abs(telem.horizontal_vel()) > 20 and telem.surface_altitude() < 15000:
        #     for thruster in vessel.parts.rcs:
        #         thruster.enabled = True
        #     vessel.auto_pilot.target_pitch = 90
        if telem.surface_altitude() < 150 or telem.vertical_vel() > -5:
            mode = 3
            Tu = 225
            Ku = 2.5
            Kp = 0.2*Ku
            Ki = 2.0*Kp/Tu
            Kd = 0.0*Kp/Tu
            vert_vel_controller.update_gains(Kp, Ki, Kd)
            alt_controller.set_point = -5.0
            alt_controller.set_max_output(-3.0)
            alt_controller.set_min_output(-10.0)
            hvel_controller.set_max_output(1.0)
            hvel_controller.set_min_output(-1.0)
            hvel_controller.update_gains(0.05, 0.005, 0.01)

    # Mode 3 is constant descent rate at -5 m/s
    if mode == 3:
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())

        # Get the vessel's velocity relative to the surface
        v_vec = vessel.flight(ref_frame).velocity
        v_mag = np.linalg.norm(v_vec)

        if vessel.flight(ref_frame).horizontal_speed > 2:
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

        print(f"{distance_to_pad:.2f}, {v_mag:.2f}, {pitch:.2f}, {heading:.2f}")

    if telem.surface_altitude() < 30:
        vessel.control.gear = True

    if int(time.time()) - int(throttle_update) > 5:
        if telem.surface_altitude()-target_alt < 50 and telem.vertical_vel() < -10:
            continue
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.25)
        elif mode == 5:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.5)
            vert_vel_controller.set_max_output(utils.throttle_from_twr(vessel, 1.1))
        else:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        
        vert_vel_controller.set_min_output(new_throttle_limit)
        print("Current Machine State %i" % mode)
        throttle_update = time.time()

    # Plot states
    throttle_datastream.update_data_stream(elapsed_time, vessel.control.throttle)
    altitude_datastream.update_data_stream(elapsed_time, telem.altitude())
    vertvel_datastream.update_data_stream(elapsed_time, telem.vertical_vel())

    time.sleep(1/dcx.CLOCK_RATE)

vessel.control.throttle = 0.0
vessel.auto_pilot.disengage()
time.sleep(1/dcx.CLOCK_RATE)
vessel.control.rsc = False
time.sleep(1/dcx.CLOCK_RATE)
vessel.control.sas = True
# throttle_datastream.plot()
# altitude_datastream.plot()
# vertvel_datastream.plot()
# plt.show()