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

def pitch_maneuver(mission_params, telem, vessel, sc):
    print("Entering Pitch Over Maneuver")
    meco = False
    flag1 = False
    flag2 = False
    while not meco:
        met = vessel.met
        if telem.velocity() > 154 and flag1 is False:
            vessel.control.set_action_group(1, True)
            flag1 = True
        if met > 108 and flag2 is False:
            vessel.control.set_action_group(2, True)
            flag2 = True
        if met > 128:
            vessel.control.activate_next_stage()
            break
        d_theta = np.arctan2(telem.velocity(),
                             mission_params.v_stage) * 180 / np.pi
        vessel.auto_pilot.target_pitch = 90 - d_theta

        time.sleep(1 / sc.CLOCK_RATE)

# constants
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "STS"
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 0
payload_LF = 0

meco_condition_multiplier = 0  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 800  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=-0.2,
                                           target_roll=180,
                                           altimeter_bias=95,
                                           grav_turn_end=85000,
                                           max_q=16000,
                                           max_g=3.0)
sts = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  sts.bref)
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
vessel.control.activate_next_stage()  # Ignite ROFIs
utils.launch_countdown(10, 3, vessel) # Countdown + SSME Ignition
vessel.control.activate_next_stage()  # SRB Ignition + Pad separation
time.sleep(3)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)
telem_viz.start_telem_publish(telem)
vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time = (0.5, 0.3, 0.5)
vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch,
                                               vessel.flight().heading)
vessel.auto_pilot.auto_tune = True

sas.roll_program(mission_params, telem, vessel, conn, sts)
time.sleep(5*50 / CLOCK_RATE)
vessel.auto_pilot.stopping_time = (0.5, 0.1, 0.5)
pitch_maneuver(mission_params, telem, vessel,sts)
sts, telem = utils.check_active_vehicle(conn, vessel,
                                                    mission_params.root_vessel)
print("Enter closed loop guidance, second stage.")
time.sleep(5*50 / CLOCK_RATE)
mission_params.target_roll = 0
vessel.control.rcs = True
sts = final_stage.close_loop_guidance(sts, mission_params, telem, 110, mission_params.target_heading, target_periapsis=23000, telem_viz=telem_viz)
vessel.auto_pilot.disengage()
vessel.control.sas = True
vessel.control.rcs = True

enter_control_mode(ControlMode.COAST, telem_viz)
while True:
    enter_control_mode(ControlMode.COAST, telem_viz, False)
    time.sleep(1 / CLOCK_RATE)