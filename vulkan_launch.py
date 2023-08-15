import spacecraft as sc
import launch_utils as utils
import mission
import pid
import time
import steering_logic as sas
import numpy as np
import final_stage

def pitch_maneuver(prop_meco_condition, has_srms, mission_params, telem, vessel, conn, sc,
                   maxq_thrust_control, max_accel_thrust_control, apoapsis_limit=None):
    print("Entering Pitch Over Maneuver")
    meco = False
    srm_sep = False
    while not meco:
        utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
                           mission_params, conn)
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
        liquid_fuel = vessel.resources.amount("LiquidFuel")
        if liquid_fuel <= prop_meco_condition*liquid_fuel + sc.upper_stage_LF + sc.payload_LF and prop_meco_condition != 0:
            print("Stabilizing Pitch for MECO")
            time.sleep(5)
            meco = True
            break
        if has_srms and srm_sep is not True:
            if vessel.met >= 82.0:
                srm_sep = True
                vessel.control.activate_next_stage() # SRM separation
                vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)
        if apoapsis_limit is not None and telem.apoapsis() > apoapsis_limit:
            meco = True
            print("Apoapsis limit reached. Staging...")
            break

        altitude_ratio = vessel.flight().mean_altitude / mission_params.grav_turn_end
        # Quartic/Quintic Easing Out Equation
        altitude_ratio = altitude_ratio-1
        vessel.auto_pilot.target_pitch = -(-90*(altitude_ratio**4 - 1)) + 90

        #d_theta = np.arctan2(telem.velocity(),
        #                     mission_params.v_stage) * 180 / np.pi
        #vessel.auto_pilot.target_pitch = 90 - d_theta
        time.sleep(1 / sc.CLOCK_RATE)

def jettison_fairing():
    fairings = vessel.parts.with_tag("payload_fairing")
    fairings[0].fairing.jettison()

# constants
CLOCK_RATE = 50  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "VULKAN"
has_srms = False
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 4032*2+6*24.3+2*9.9
payload_LF = 0#5000+360

meco_condition_multiplier = 0.01  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 600  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0,
                                           target_roll=0,
                                           altimeter_bias=117,
                                           grav_turn_end=120000,
                                           max_q=12000,
                                           max_g=4.0,
                                           target_apoapsis=120000)
vulkan = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  vulkan.bref)
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
vessel.control.rcs = False
vessel.control.throttle = 1
# utils.launch_countdown(5, 2, vessel) # Countdown + Engine Ignition
utils.set_launch_ut(conn.space_center, "Y3, D293, 00:07:30", 2, vessel)
vessel.control.activate_next_stage()  # Pad separation + SRM Ignition
time.sleep(1)
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)
vessel.auto_pilot.engage()
vessel.auto_pilot.stopping_time = (0.2, 0.7, 0.2)
vessel.auto_pilot.target_pitch_and_heading(89,
                                               vessel.flight().heading)
vessel.auto_pilot.auto_tune = True
time.sleep(7)
sas.roll_program(mission_params, telem, vessel, conn, vulkan)
time.sleep(5*50 / CLOCK_RATE)
pitch_maneuver(meco_condition_multiplier, has_srms, mission_params, telem, vessel, conn, vulkan,
                   maxq_thrust_control, max_accel_thrust_control)
sas.meco(vessel, vulkan)
vessel.control.activate_next_stage()  # 2nd stage separation
secondstage, telem = utils.check_active_vehicle(conn, vessel,
                                                    mission_params.root_vessel)
secondstage.auto_pilot.engage()
secondstage.auto_pilot.target_pitch = secondstage.flight().pitch
secondstage.auto_pilot.target_roll = secondstage.flight().roll
secondstage.auto_pilot.target_heading = secondstage.flight().heading
time.sleep(2*50 / CLOCK_RATE)
secondstage.control.throttle = 1
secondstage.auto_pilot.stopping_time = (.9, .3, .9)
secondstage.control.activate_next_stage()  # 2nd stage engine ignition
while secondstage.available_thrust <= 0:
    secondstage, telem = utils.check_active_vehicle(
        conn, secondstage, mission_params.root_vessel)
    time.sleep(10 / CLOCK_RATE)
    secondstage.control.activate_next_stage()
    print("2nd Stage Engine Ignition")
secondstage.control.rcs = True
time.sleep(3*50 / CLOCK_RATE)
#jettison_fairing()
print("Enter closed loop guidance, secondstage stage.")
pitch_control = pid.PID(1.51,
                        0.01,
                        0.3,
                        -30,
                        30,
                        deadband=1, rate_limit=2.0)
secondstage = final_stage.close_loop_guidance(secondstage, mission_params, telem, 120, mission_params.target_heading, target_apo=120000, pid_input=pitch_control)
secondstage.auto_pilot.disengage()
secondstage.control.sas = True
secondstage.control.rcs = True