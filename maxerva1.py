import spacecraft as sc
import launch_utils as utils
import mission
import controllers
import time
import steering_logic as sas
import numpy as np
import final_stage

def pitch_maneuver(prop_meco_condition, mission_params, telem, vessel, conn, sc,
                   maxq_thrust_control, max_accel_thrust_control, apoapsis_limit=None):
    print("Entering Pitch Over Maneuver")
    meco = False
    while not meco:
        utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
                           mission_params, conn, "CEV")
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
        ) <= prop_meco_condition + sc.upper_stage_LF + sc.payload_LF:
            print("Stabilizing Pitch for MECO")
            time.sleep(5)
            vessel.control.rcs = True
            vessel.control.sas = True
            meco = True
            break
        if apoapsis_limit is not None and telem.apoapsis() > apoapsis_limit:
            meco = True
            print("Apoapsis limit reached. Staging...")
            break

        d_theta = np.arctan2(telem.velocity(),
                             mission_params.v_stage) * 180 / np.pi
        vessel.auto_pilot.target_pitch = 90 - d_theta
        time.sleep(10 / sc.CLOCK_RATE)

# constants
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "Maxerva-I"
is_abort_installed = False
abort_criteria = 15  # maximum off-angle before automated abort triggered

upper_stage_LF = 1620*2+24.3*2
payload_LF = 24.3*9*2*4+8*24.3+72*2

meco_condition_multiplier = 680  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 900  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0.0,
                                           target_roll=180,
                                           altimeter_bias=103,
                                           grav_turn_end=85000,
                                           max_q=14000,
                                           max_g=4.0,
                                           v_stage=v_stage)
maxerva = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier, is_abort_installed=is_abort_installed, abort_criteria=abort_criteria)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  maxerva.bref)
maxq_thrust_control = controllers.PID(mission_params.max_q, 
                                  0.001,
                                  0.0001,
                                  0.00003,
                                  0.5,
                                  1.0) 
max_accel_thrust_control = controllers.PID(mission_params.max_g,
                                       0.1,
                                       0.4,
                                       0.05,
                                       0.5,
                                       1.0)

K1 = 0.1
K2 = 200
Kp = 0.2*K1
Ki = 2.0*K1/K2
Kd = Kp*K2/3.0
final_pitch_control = controllers.PID(0.0,
                        Kp,
                        Ki,
                        Kd,
                        -40,
                        40)

# Pre-Launch
vessel.control.sas = True
vessel.control.rcs = False
vessel.control.throttle = 1
utils.launch_countdown(10, 1, vessel) # Countdown + Engine Ignition
time.sleep(0.05)
vessel.control.activate_next_stage()  # Outboard Engine Ignition
time.sleep(0.5)
vessel.control.activate_next_stage()  # pad sep
time.sleep(3)
vessel = utils.check_active_vehicle_and_control(conn, root_vessel, control_tag="control_point")

# Acquire telemetry again, just in case
telem = mission.Telemetry(conn, vessel)

vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch,
                                               vessel.flight().heading)
vessel.auto_pilot.roll_pid_gains = (10.1, 0.0, 0.0)

sas.roll_program(mission_params, telem, vessel, conn, maxerva)
pitch_maneuver(meco_condition_multiplier, mission_params, telem, vessel, conn, maxerva,
                   maxq_thrust_control, max_accel_thrust_control)
sas.meco(vessel, maxerva)
vessel.control.activate_next_stage()  # 2nd stage separation
second = utils.check_active_vehicle_and_control(conn, root_vessel, "control_point")
telem = mission.Telemetry(conn, second)
vessel.control.sas = True
time.sleep(2*50 / CLOCK_RATE)
second.control.throttle = 1
vessel.auto_pilot.stopping_time = (.5, .8, .5)
vessel.auto_pilot.target_roll = float("NaN")
second.control.activate_next_stage()  # 2nd stage engine ignition
time.sleep(4*50 / CLOCK_RATE)
second.control.activate_next_stage()  # Fairing deployment
print("Enter closed loop guidance, second stage.")
second.control.rcs = True
second = final_stage.close_loop_guidance(second, mission_params, telem, 100, mission_params.target_heading, pid_input=final_pitch_control)
second.auto_pilot.disengage()
second.control.sas = True
second.control.rcs = True