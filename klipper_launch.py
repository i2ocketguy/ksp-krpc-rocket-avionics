import spacecraft as sc
import launch_utils as utils
import mission
import controllers
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

def pitch_maneuver(prop_sep_condition, mission_params, telem, vessel, conn, sc,
                   maxq_thrust_control, max_accel_thrust_control, apoapsis_limit=None):
    print("Entering Pitch Over Maneuver")
    meco = False
    while not meco:
        if not mission_params.maxq_exit:
            sas.maxQ(mission_params, telem, vessel, maxq_thrust_control, telem_viz)

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
        
        if vessel.resources.amount(
                "LiquidFuel"
        ) <= prop_sep_condition*vessel.resources.amount("LiquidFuel") + sc.upper_stage_LF + sc.payload_LF and prop_sep_condition != 0:
            print("Throttling down and stablizing for ET Separation")
            setting = utils.throttle_from_twr(vessel, 1.0)
            vessel.control.throttle = setting            
            time.sleep(4)
            vessel.control.set_action_group(1, True)
            time.sleep(10/CLOCK_RATE)
            vessel.control.activate_next_stage()  # ET separation
            vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)
            time.sleep(1)
            vessel.control.throttle = 1.0
            time.sleep(2)
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
root_vessel = "KLIPPER"
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 4*1125 + 4*270 + 2*900 + 2*6*810 + 2*540 + 2*2250 + 1125 + 2*24.30 + 3*9.9
payload_LF = 360

sep_condition_multiplier = 0.01  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 700  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0,
                                           target_roll=180,
                                           altimeter_bias=104,
                                           grav_turn_end=85000,
                                           max_q=12000,
                                           max_g=3.0)
klipper = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, sep_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  klipper.bref)
maxq_thrust_control = controllers.PID(0.001,
                                  0.0001,
                                  0.00003,
                                  0.5,
                                  1.0,
                                  clamp=mission_params.max_q)
maxq_thrust_control.set_point = mission_params.max_q
max_accel_thrust_control = controllers.PID(0.1,
                                       0.4,
                                       0.05,
                                       0.5,
                                       1.0,
                                       deadband=0.001,
                                       clamp=mission_params.max_g)
max_accel_thrust_control.set_point = mission_params.max_g

# Pre-Launch
vessel.control.sas = True
vessel.control.rcs = False
vessel.control.throttle = 1
utils.launch_countdown(10, 3, vessel) # Countdown + Engine Ignition
# utils.set_launch_ut(conn.space_center, "Y2, D245, 05:12:00")
vessel.control.activate_next_stage()  # Pad separation
time.sleep(3)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)

vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time = (0.2, 1.2, 0.2)
vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch,
                                               vessel.flight().heading)
vessel.auto_pilot.auto_tune = True

time.sleep(10)
sas.roll_program(mission_params, telem, vessel, conn, klipper, telem_viz)
time.sleep(5*50 / CLOCK_RATE)
vessel.auto_pilot.stopping_time = (0.3, 0.5, 0.3)
pitch_maneuver(sep_condition_multiplier, mission_params, telem, vessel, conn, klipper,
                   maxq_thrust_control, max_accel_thrust_control)
klipper, telem = utils.check_active_vehicle(conn, vessel,
                                                    mission_params.root_vessel)
print("Enter closed loop guidance, second stage.")
time.sleep(5*50 / CLOCK_RATE)
#mission_params.target_roll = 0
vessel.control.rcs = True
# pitch_control = pid.PID(0.1,
#                         0.0007,
#                         0.0003,
#                         -35,
#                         35,
#                         deadband=100)
klipper = final_stage.close_loop_guidance(klipper, mission_params, telem, 100, mission_params.target_heading, target_apo=120000, telem_viz=telem_viz)
vessel.auto_pilot.disengage()
vessel.control.sas = True
vessel.control.rcs = True
enter_control_mode(ControlMode.COAST, telem_viz)
while True:
    enter_control_mode(ControlMode.COAST, telem_viz, False)
    time.sleep(1 / CLOCK_RATE)