import krpc
import spacecraft as sc
import launch_utils as utils
import mission
import steering_logic as steering
import pid
import time

# constants
CLOCK_RATE = 10  # refresh rate [Hz]
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
maxq_thrust_control = pid.PID(0.001,
                              0.0001,
                              0.00003,
                              0.5,
                              1.0,
                              clamp=mission_params.max_q)
maxq_thrust_control.set_point = mission_params.max_q
max_accel_thrust_control = pid.PID(0.1,
                                   0.4,
                                   0.05,
                                   0.5,
                                   1.0,
                                   deadband=0.001,
                                   clamp=mission_params.max_g)
max_accel_thrust_control.set_point = mission_params.max_g

# Pre-Launch
vessel.control.sas = True
vessel.control.rcs = True
vessel.control.throttle = 1
utils.launch_countdown(5)
vessel.control.activate_next_stage()  # Engine Ignition
time.sleep(0.5)

# ignition abort: engines not started
if vessel.thrust == 0:
    vessel.control.throttle = 0.0
    print("Launch Abort: off-nominal engine ignition")
    quit()
vessel.auto_pilot.engage()
vessel.auto_pilot.auto_tune = True
vessel.auto_pilot.target_pitch_and_heading(90, vessel.flight().heading)

vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)
steering.roll_program(mission_params, telem, vessel, conn, dcx)
steering.pitch_maneuver(dcx.meco_condition_multiplier, mission_params, telem, vessel,
                        conn, dcx, maxq_thrust_control,
                        max_accel_thrust_control, apoapsis_limit=73000)
vessel.control.throttle = 0.0
while telem.surface_altitude() < 70000:
    time.sleep(0.5)
    pass

action_group_flag = False
while telem.surface_altitude() > 50000:

    if telem.surface_altitude() < 60000 and action_group_flag == False:
        action_group_flag = True
        vessel.control.toggle_action_group(1)
    time.sleep(0.5)


while telem.surface_altitude() > 20000:
    pass

steering.landing_burn(vessel)

while telem.vertical_vel() < -10:
    pass
throttle_control = pid.PID(0.1,
                            0.001,
                            0.003,
                            0,
                            1)
steering.landing_gate(vessel, telem, throttle_control)




