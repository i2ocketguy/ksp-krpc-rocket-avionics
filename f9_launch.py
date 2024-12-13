import spacecraft as sc
import launch_utils as utils
import mission
import pid
import time
import steering_logic as sas
import numpy as np
import final_stage

from launch_utils import ControlMode, enter_control_mode
from telemetry import KSPTelemetry

telem_viz = KSPTelemetry()
telem_viz.start_metrics_server()
telem_viz.register_enum_metric(utils.CONTROL_MODE, "The enumerated control mode of the flight computer",
                               [mode.name for mode in utils.ControlMode])
enter_control_mode(ControlMode.PAD, telem_viz)

def pitch_maneuver(prop_meco_condition, mission_params, telem, vessel, conn, sc,
                   maxq_thrust_control, max_accel_thrust_control, apoapsis_limit=None):
    print("Entering Pitch Over Maneuver")
    meco = False
    booster_sep = False
    while not meco:
        utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
                           mission_params, conn, "F9")
        if not mission_params.maxq_exit:
            sas.maxQ(mission_params, telem, vessel, maxq_thrust_control)

        if vessel.flight(
        ).g_force >= mission_params.max_g and not mission_params.maxg_enter:
            mission_params.maxg_enter = True
            print("Throttling Down to Maintain Load")
        if mission_params.maxq_exit and mission_params.maxg_enter and not mission_params.maxg_exit:
            # this criteria will never be met AFTER maxQ exits
            #      so this will always be invoked in flight
            sas.max_accel(mission_params, telem, vessel, max_accel_thrust_control)

        tot_thrust = vessel.thrust
        if tot_thrust <= 0:
            meco = True
            break
        #TODO: possibly add a section to look-up fuel and enter constant pitch to avoid shuttle flipping on MECO
        if vessel.resources.amount(
                "LiquidFuel"
        ) <= prop_meco_condition + sc.upper_stage_LF + sc.payload_LF and prop_meco_condition != 0:
            print("Stabilizing Pitch for MECO")
            time.sleep(5)
            meco = True
            break
        if apoapsis_limit is not None and telem.apoapsis() > apoapsis_limit:
            meco = True
            print("Apoapsis limit reached. Staging...")
            break

        d_theta = np.arctan2(telem.velocity(),
                             mission_params.v_stage) * 180 / np.pi
        vessel.auto_pilot.target_pitch = 90 - d_theta
        time.sleep(1 / sc.CLOCK_RATE)

# constants
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "F9"
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 1440
payload_LF = 45

meco_condition_multiplier = 4000  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 950  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
print(vessel.flight().longitude, vessel.flight().latitude)
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0,
                                           target_roll=180,
                                           altimeter_bias=116,
                                           grav_turn_end=85000,
                                           max_q=15000,
                                           max_g=4.0)
f9 = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  f9.bref)
maxq_thrust_control = pid.PID(mission_params.max_q, 
                                  0.001,
                                  0.0001,
                                  0.00003,
                                  0.5,
                                  1.0,
                                  clamp=mission_params.max_q)
max_accel_thrust_control = pid.PID(mission_params.max_g,
                                       0.1,
                                       0.4,
                                       0.05,
                                       0.5,
                                       1.0,
                                       deadband=0.001,
                                       clamp=mission_params.max_g)

# Pre-Launch
vessel.control.sas = True
vessel.control.rcs = False
vessel.control.throttle = 1
utils.launch_countdown(10, 3, vessel) # Countdown + Engine Ignition
vessel.control.activate_next_stage()  # Pad separation
time.sleep(3)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)

telem_viz.start_telem_publish(telem)
vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time = (1.5, 1.3, 1.5)
vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch,
                                               vessel.flight().heading)
vessel.auto_pilot.auto_tune = True

sas.roll_program(mission_params, telem, vessel, conn, f9, telem_viz)
time.sleep(5*50 / CLOCK_RATE)
pitch_maneuver(meco_condition_multiplier, mission_params, telem, vessel, conn, f9,
                   maxq_thrust_control, max_accel_thrust_control)
sas.meco(vessel, f9, telem_viz)
vessel.control.set_action_group(1, True)
# vessel.control.activate_next_stage()  # 2nd stage separation
second, telem = utils.check_active_vehicle(conn, vessel,
                                                    mission_params.root_vessel)
time.sleep(2*50 / CLOCK_RATE)
second.control.throttle = 1
vessel.auto_pilot.stopping_time = (.5, .3, .5)
second.control.activate_next_stage()  # 2nd stage engine ignition
time.sleep(3*50 / CLOCK_RATE)
second.control.activate_next_stage()  # Fairing deployment
print("Enter closed loop guidance, second stage.")
mission_params.target_roll = 0
second.control.rcs = True
second = final_stage.close_loop_guidance(second, mission_params, telem, 110, mission_params.target_heading, telem_viz)
second.auto_pilot.disengage()
second.control.sas = True
second.control.rcs = True
enter_control_mode(ControlMode.COAST, telem_viz)
while True:
    enter_control_mode(ControlMode.COAST, telem_viz, False)
    time.sleep(1 / CLOCK_RATE)