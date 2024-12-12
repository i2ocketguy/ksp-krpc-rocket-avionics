import spacecraft as sc
import launch_utils as utils
import mission
import pid
import time
import plotting_utils as pu
import matplotlib.pyplot as plt
import steering_logic as steering
import math
from digitalfilter import low_pass_filter as LPF

from launch_utils import DcxControlMode, enter_control_mode
from telemetry import KSPTelemetry

telem_viz = KSPTelemetry()
telem_viz.start_metrics_server()
telem_viz.register_enum_metric(utils.CONTROL_MODE, "The enumerated control mode of the flight computer",
                               [mode.name for mode in DcxControlMode])
telem_viz.register_gauge_metric('distance_to_pad', 'distance_to_pad')
telem_viz.register_gauge_metric('current_horizontal_velocity', 'current_horizontal_velocity')
telem_viz.register_gauge_metric('distance_pitch', 'distance_pitch')
telem_viz.register_gauge_metric('velocity_pitch', 'velocity_pitch')
telem_viz.register_gauge_metric('pitch_input', 'pitch_input')
telem_viz.register_gauge_metric('heading_error', 'heading_error')
telem_viz.register_gauge_metric('roll', 'roll')
telem_viz.register_gauge_metric('roll_input', 'roll_input')
telem_viz.register_gauge_metric('pitch', 'pitch (mode 3)')
telem_viz.register_gauge_metric('throttle', 'throttle')

telem_viz.register_counter_metric('gnc_frame_count', 'gnc_frame_count')
telem_viz.register_counter_metric('gnc_overrun_count', 'gnc_overrun_count')

telem_viz.register_histogram_metric('calculate_landing_burn_time', 'calculate_landing_burn_time')
telem_viz.register_histogram_metric('calculate_heading', 'calculate_heading')
telem_viz.register_histogram_metric('haversine_distance', 'haversine_distance')
telem_viz.register_histogram_metric('publish_gnc_metrics_b', 'publish_gnc_metrics_b')
telem_viz.register_histogram_metric('get_flight_path_angle', 'get_flight_path_angle')
telem_viz.register_histogram_metric('calc_lb_final_math', 'calc_lb_final_math')

enter_control_mode(DcxControlMode.PAD, telem_viz)

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
                                           target_inc=0.2,
                                           target_roll=180,
                                           altimeter_bias=95,
                                           grav_turn_end=85000,
                                           max_q=16000,
                                           max_g=3.0)
dcx = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  dcx.bref)

# Pre-Launch
utils.launch_countdown(1)
enter_control_mode(DcxControlMode.IGNITION, telem_viz)
vessel.control.activate_next_stage()  # Engine Ignition
vessel.auto_pilot.engage()
vessel.auto_pilot.auto_tune = True
throttle_limit = utils.throttle_from_twr(vessel, 3.0)
vessel.control.throttle = throttle_limit
telem_viz.publish_gauge_metric('throttle', throttle_limit, False)
vessel.control.sas = True
vessel.control.rcs = True
time.sleep(1/dcx.CLOCK_RATE)
vessel.control.toggle_action_group(1)
vessel.auto_pilot.target_roll = vessel.flight().roll
vessel.auto_pilot.target_pitch_and_heading(90, mission_params.target_heading)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)

telem_viz.start_telem_publish(telem)
time.sleep(3)
vessel.control.gear = False

# Wait until target alititude is achieved
target_alt = 2000 + telem.apoapsis()
hmax = 0
drag = 0
g = 0
enter_control_mode(DcxControlMode.BURN_TO_ALTITUDE, telem_viz)
while telem.apoapsis()-(telem.vertical_vel()*(drag/vessel.mass)) < target_alt: #telem.altitude()+hmax < target_alt:
    drag = vessel.flight().drag
    drag = math.sqrt(math.pow(drag[0],2) + math.pow(drag[1],2) + math.pow(drag[2],2))
    g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
    #hmax = math.pow(telem.velocity(),2)*math.pow(math.sin(vessel.flight().pitch),2)/(2*(g-(0.9*drag/vessel.mass)))
    telem_viz.increment_counter_metric('gnc_frame_count')
    time.sleep(10/dcx.CLOCK_RATE)

vessel.control.throttle = 0.0
telem_viz.publish_gauge_metric('throttle', 0.0, False)
enter_control_mode(DcxControlMode.COAST_TO_ALTITUDE, telem_viz)
while telem.surface_altitude() < target_alt and telem.vertical_vel() > 10:
    telem_viz.increment_counter_metric('gnc_frame_count')
    pass

'''
while telem.surface_altitude() < target_alt:
    drag = vessel.flight().drag
    drag = math.sqrt(math.pow(drag[0],2) + math.pow(drag[1],2) + math.pow(drag[2],2))
    hmax = math.pow(telem.velocity(),2)*math.pow(math.sin(vessel.flight().pitch),2)/(2*(vessel.orbit.body.surface_gravity))
    #print(telem.surface_altitude() + hmax)
    if telem.surface_altitude() + hmax <= target_alt:
        throttle_limit = utils.throttle_from_twr(vessel, 3.0)
        vessel.control.throttle = throttle_limit
    elif telem.surface_altitude() + hmax > target_alt:
        throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        vessel.control.throttle = throttle_limit
        break
    else:
        vessel.control.throttle = throttle_limit
    time.sleep(1/dcx.CLOCK_RATE)
'''

print("Passed %i, entering vertical velocity hold ..." % target_alt)
new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
vessel.control.throttle = new_throttle_limit
telem_viz.publish_gauge_metric('throttle',new_throttle_limit, False)
#vert_vel_controller = pid.PID(0.5, 0.0, 0.05, -0.5, 0.5, deadband=0.01) # 0.8, .1, .001
Tu = 300
Ku = 2.5
Kp = 0.2*Ku
Ki = 2.0*Kp/Tu
Kd = 0.0*Kp/Tu
vert_vel_controller = pid.PID(Kp, Ki, Kd, new_throttle_limit, 1.0, deadband=0.005)
vert_vel_controller.set_point = 0.02  # vertical velocity target, m/s
print(Kp,Ki,Kd)
alt_controller = pid.PID(.1, 0.005, 0.0, min_output=-20, max_output=10)
alt_controller.set_point = target_alt
throttle_lpf = LPF(4,3,dcx.CLOCK_RATE)

for thruster in vessel.parts.rcs:
    thruster.enabled = True

enter_control_mode(DcxControlMode.MODE_1, telem_viz)
mode = 1
status = vessel.situation
starting_time = time.time()
throttle_update = starting_time
count = 0
burn_flag = False
throttle_datastream = pu.data_stream_plot()
altitude_datastream = pu.data_stream_plot()
vertvel_datastream = pu.data_stream_plot()
expected_frame_time = (1/dcx.CLOCK_RATE) * 1e9
def dynamic_sleep(actual_frame_time: float):
    if actual_frame_time > expected_frame_time:
        if telem_viz.gnc_debug:
            print(f"Overrun! Time={actual_frame_time:.2f} seconds")
        telem_viz.increment_counter_metric('gnc_overrun_count')
    else:
        to_sleep = expected_frame_time - actual_frame_time
        time.sleep(to_sleep / 1e9)
while vessel.situation == status:
    telem_viz.increment_counter_metric('gnc_frame_count')
    frame_start_time = time.time_ns()
    count = count + 1
    elapsed_time = time.time() - starting_time

    # Mode 1 is hover at target altitude
    if mode == 1:
        enter_control_mode(DcxControlMode.MODE_1, telem_viz, False)
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
        if int(elapsed_time) > 10:
            enter_control_mode(DcxControlMode.MODE_2, telem_viz)
            mode = 2
            vessel.auto_pilot.stopping_time = (0.1,0.1,0.1)
            vessel.auto_pilot.deceleration_time = (18.0,18.0,18.0)
            vessel.control.throttle = 0.0
            telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
            vessel.auto_pilot.target_pitch = 0
            vessel.auto_pilot.target_roll = 0
            print("Exiting altitude hold ...")
        
    
    # Mode 2 is descent and landing burn start calculation
    if mode == 2:
        if telem.surface_altitude() < 3000:
            v_vec = vessel.velocity(vessel.surface_reference_frame)
            b_vec = (0,0,-1)
            pitch = steering.vang(v_vec,b_vec)
            print(pitch)
            #vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
            #vessel.auto_pilot.target_direction = (0.0, -1.0, 0.0)
            ##vessel.auto_pilot.deceleration_time = (10,10,10)
            #if abs(vessel.auto_pilot.target_pitch) > 5:
            #    vessel.auto_pilot.target_pitch = (vessel.auto_pilot.target_pitch/abs(vessel.auto_pilot.target_pitch))*5
            #if abs(vessel.auto_pilot.target_heading) > 5:
            #    vessel.auto_pilot.target_heading = (vessel.auto_pilot.target_heading/abs(vessel.auto_pilot.target_heading))*5
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        T = steering.calculate_landing_burn(vessel)
        if count % 10 == 0:
            #print(T, vessel.available_thrust)
            pass
        if T >= vessel.available_thrust and burn_flag is False:
            vessel.control.throttle = 1.0
            telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
            burn_flag = True
            for thruster in vessel.parts.rcs:
                thruster.enabled = False
        if telem.vertical_vel() > -50 and telem.vertical_vel() < 0 and burn_flag is True:
            vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
            vessel.auto_pilot.target_pitch = 90
        if telem.surface_altitude() < 300 and telem.vertical_vel() > -7 and burn_flag is True:
            vessel.control.gear = True
            Tu = 100
            Ku = 2
            Kp = 0.2*Ku
            Ki = 2.0*Kp/Tu
            Kd = 0.0*Kp/Tu
            vert_vel_controller.update_gains(Kp, Ki, Kd)
            vert_vel_controller.set_point = -5
            mode = 3
            for thruster in vessel.parts.rcs:
                thruster.enabled = True
        if telem.vertical_vel() > 0 and burn_flag is True:
            vessel.control.gear = True
            Tu = 100
            Ku = 1
            Kp = 0.2*Ku
            Ki = 2.0*Kp/Tu
            Kd = 0.0*Kp/Tu
            vert_vel_controller.update_gains(Kp, Ki, Kd)
            vert_vel_controller.set_point = -5
            mode = 3
            enter_control_mode(DcxControlMode.MODE_3, telem_viz)
            for thruster in vessel.parts.rcs:
                thruster.enabled = True
            vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
            vessel.auto_pilot.target_pitch = 90
            print("Panic! ...")

    # Mode 3 is constant descent rate at -5 m/s
    if mode == 3:
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        #vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        #vessel.auto_pilot.target_direction = (0.0, -0.01, 0.0)
        #steering.landing_gate(vessel, telem, vert_vel_controller)
        steering.landing_gate(vessel, telem, vert_vel_controller)

    if int(time.time()) - int(throttle_update) > 10:
        if telem.surface_altitude()-target_alt < 50 and telem.vertical_vel() < -10:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.25)
        elif mode == 3:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.9)
            vert_vel_controller.set_max_output(utils.throttle_from_twr(vessel, 1.05))
        else:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        
        vert_vel_controller.set_min_output(new_throttle_limit)
        print("Updating throttle limit: " + str(vert_vel_controller.min_output))
        print("Current Machine State %i" % mode)
        throttle_update = time.time()

    # Plot states
    telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
    throttle_datastream.update_data_stream(elapsed_time, vessel.control.throttle)
    altitude_datastream.update_data_stream(elapsed_time, telem.altitude())
    vertvel_datastream.update_data_stream(elapsed_time, telem.vertical_vel())

    frame_end_time = time.time_ns()
    frame_time = frame_end_time - frame_start_time
    dynamic_sleep(frame_time)

enter_control_mode(DcxControlMode.LANDED, telem_viz)
vessel.control.throttle = 0.0
telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
vessel.auto_pilot.disengage()
vessel.control.rsc = False
vessel.control.sas = True
throttle_datastream.plot()
altitude_datastream.plot()
vertvel_datastream.plot()
plt.show()
enter_control_mode(DcxControlMode.SHUTDOWN, telem_viz)
time.sleep(1)