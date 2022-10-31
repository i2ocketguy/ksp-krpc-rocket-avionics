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
                                           target_inc=-0.2,
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
vessel.auto_pilot.target_pitch_and_heading(88.75, mission_params.target_heading)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)

time.sleep(3)
vessel.control.gear = False

# Wait until target alititude is achieved
init_alt = telem.apoapsis()
target_alt = 5000
hmax = 0
drag = 0
g = 0
while telem.apoapsis()-(telem.vertical_vel()*(0.8*drag/vessel.mass)) < target_alt+init_alt: #telem.altitude()+hmax < target_alt:
    drag = vessel.flight().drag
    drag = math.sqrt(math.pow(drag[0],2)*0 + math.pow(drag[1],2)*0 + math.pow(drag[2],2))
    g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
    #hmax = math.pow(telem.velocity(),2)*math.pow(math.sin(vessel.flight().pitch),2)/(2*(g-(0.9*drag/vessel.mass)))
    time.sleep(10/dcx.CLOCK_RATE)

vessel.control.throttle = 0.0
while telem.surface_altitude() < target_alt and telem.vertical_vel() > 10:
    pass

print("Passed %i, entering vertical velocity hold ..." % target_alt)
new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
vessel.control.throttle = new_throttle_limit
#vert_vel_controller = pid.PID(0.5, 0.0, 0.05, -0.5, 0.5, deadband=0.01) # 0.8, .1, .001
Tu = 300
Ku = 2.5
Kp = 0.2*Ku
Ki = 2.0*Kp/Tu
Kd = 0.0*Kp/Tu
vert_vel_controller = pid.PID(Kp, Ki, Kd, new_throttle_limit, 1.0, deadband=0.005)
vert_vel_controller.set_point = 0.02  # vertical velocity target, m/s
alt_controller = pid.PID(.4, 0.005, 0.0, min_output=-20, max_output=10)
alt_controller.set_point = target_alt
slam_controller = pid.PID(5, 0.0, 0.0, 0.0, 1.0, deadband=0.005)
slam_controller.set_point = 0.5
throttle_lpf = LPF(4,3,dcx.CLOCK_RATE)

for thruster in vessel.parts.rcs:
    thruster.enabled = True

mode = 1
status = vessel.situation.landed
starting_time = time.time()
throttle_update = starting_time
count = 0
burn_flag = False
throttle_datastream = pu.data_stream_plot()
altitude_datastream = pu.data_stream_plot()
vertvel_datastream = pu.data_stream_plot()
while vessel.situation != status:
    count = count + 1
    elapsed_time = time.time() - starting_time

    # Mode 1 is hover at target altitude
    if mode == 1:
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        if int(elapsed_time) > 5:
            vessel.control.throttle = 0.0
            mode = 2
            vessel.control.toggle_action_group(1)
            vessel.auto_pilot.target_pitch = 90.0
            vessel.auto_pilot.target_roll = 0.0
            print("Exiting altitude hold ...")
            while telem.vertical_vel() > -20:
                pass
        
    
    # Mode 2 is descent and landing burn start calculation
    if mode == 2:
        tb = steering.calculate_landing_burn_time2(vessel, telem)
        if telem.surface_altitude() < 4000:
            for thruster in vessel.parts.rcs:
                thruster.enabled = True
            vessel.auto_pilot.target_pitch = 90
        if tb > -0.75 and burn_flag is False:
            vessel.control.throttle = slam_controller.update(tb)
            burn_flag = True
        if telem.surface_altitude() >= 50 and burn_flag is True:
                vessel.control.throttle = slam_controller.update(tb)
            #slam_controller.debug()
        #if T >= vessel.available_thrust and burn_flag is False:
        #    vessel.control.throttle = 1.0
        #    burn_flag = True
        #if telem.surface_altitude() < 100 or telem.vertical_vel() > -20 and burn_flag is True:
        if telem.surface_altitude() < 50 or telem.vertical_vel() > -5:
            mode = 3
            Tu = 225
            Ku = 2.5
            Kp = 0.2*Ku
            Ki = 2.0*Kp/Tu
            Kd = 0.0*Kp/Tu
            vert_vel_controller.update_gains(Kp, Ki, Kd)
            alt_controller.set_point = 0.0
            alt_controller.set_max_output(-3.0)
            alt_controller.set_min_output(-10.0)

    # Mode 3 is constant descent rate at -5 m/s
    if mode == 3:
        #steering.landing_gate(vessel, telem, vert_vel_controller)
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())

        if telem.surface_altitude() < 30:
            vessel.control.gear = True

    if int(time.time()) - int(throttle_update) > 1:
        if telem.surface_altitude()-target_alt < 50 and telem.vertical_vel() < -10:
            continue
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.25)
        elif mode == 5:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.5)
            vert_vel_controller.set_max_output(utils.throttle_from_twr(vessel, 1.1))
        else:
            new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        
        vert_vel_controller.set_min_output(new_throttle_limit)
        print("Updating throttle limit: " + str(vert_vel_controller.min_output))
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
throttle_datastream.plot()
altitude_datastream.plot()
vertvel_datastream.plot()
plt.show()