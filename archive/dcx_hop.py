import spacecraft as sc
import launch_utils as utils
import mission
import steering_logic as steering
import controllers
import time
import plotting_utils

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

utils.launch_countdown(1)
vessel.control.activate_next_stage()  # Engine Ignition
vessel.auto_pilot.engage()
vessel.auto_pilot.auto_tune = True
throttle_limit = utils.throttle_from_twr(vessel, 1.1)
vessel.control.throttle = throttle_limit
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)
vessel.control.sas = True
vessel.control.rcs = True
vessel.auto_pilot.target_pitch_and_heading(90, vessel.flight().heading)
vessel.control.toggle_action_group(1)

while telem.surface_altitude() < 80:
    pass

print("Passed 80 m, entering altitude hold ...")
throttle_limit = utils.throttle_from_twr(vessel, 0.7)
vessel.control.throttle = throttle_limit
print(throttle_limit)
status = vessel.situation
alt_controller = controllers.PID(0.7, 0.35, 0.001, throttle_limit, 1, deadband=0.01)
alt_controller.set_point = 0.02
start_time = time.time()
mode = 1
throttle_data = plotting_utils.data_stream_plot()
while vessel.situation == status:
    if mode == 1:
        vessel.control.throttle = alt_controller.update(telem.vertical_vel())

    #throttle_limit = utils.throttle_from_twr(vessel, 0.9)
    if int(time.time()) - int(start_time) > 10:
        new_throttle_limit = 0#utils.throttle_from_twr(vessel, 0.9)
        alt_controller.set_min_output(new_throttle_limit)
        print("Updated minimum throttle: " + str(alt_controller.min_output))
        start_time = time.time()

    throttle_data.update_data_stream(time.time(), vessel.control.throttle)
    time.sleep(1/dcx.CLOCK_RATE)

vessel.control.throttle = 0.0
vessel.control.sas = True
throttle_data.plot()
