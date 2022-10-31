import numpy as np
import time
import mission
import launch_utils as utils
import pid
import final_stage

# constants
CLOCK_RATE = 10  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "MRS-1"
is_abort_installed = False
abort_criteria = 20  # maximum off-angle before automated abort triggered

upper_stage_LF = 2 * 405  # LF in Benhken 4 US
payload_LF = 6*24.3+6*9.9+2*24.3 #45+2*27+18 # LF in KRV shuttle + abort

meco_condition_multiplier = 0.06  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 1000  # velocity target for 45 degree pitch over


def main():
    conn, vessel = utils.initialize()
    mission_params = mission.MissionParameters(root_vessel,
                                               state="init",
                                               target_inc=0.2,
                                               target_roll=180,
                                               altimeter_bias=95,
                                               grav_turn_end=85000,
                                               max_q=14000,
                                               max_g=4.0)

    vessel = launch(conn, vessel, mission_params)
    vessel = second_stage(conn, vessel, mission_params)


def launch(conn, vessel, mission_params):
    # initial vehicle setup
    bref = vessel.orbit.body.reference_frame
    mission_params.target_heading = utils.set_azimuth(
        vessel, mission_params.target_inc, bref)
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
    vessel.control.throttle = 1
    utils.launch_countdown()

    print("Engine Ignition")
    vessel.control.activate_next_stage()  # Engine Ignition
    time.sleep(0.5)

    # ignition abort: engines not started
    if vessel.thrust == 0:
        vessel.control.throttle = 0.0
        print("Launch Abort: off-nominal engine ignition")
        quit()

    # vessel.control.activate_next_stage()  # Attempted pad disconnet
    utils.pad_separation(vessel)
    time.sleep(3)
    vessel, telem = utils.check_active_vehicle(conn, vessel,
                                               mission_params.root_vessel)

    first_stage_liquid_fuel = vessel.resources.amount("LiquidFuel") - (
        upper_stage_LF + payload_LF)
    prop_meco_condition = meco_condition_multiplier * (first_stage_liquid_fuel)
    print("First stage LF: ", first_stage_liquid_fuel)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch,
                                               vessel.flight().heading)
    vessel.auto_pilot.auto_tune = True

    roll_program(mission_params, telem, vessel, conn)
    time.sleep(1 / CLOCK_RATE)
    pitch_maneuver(prop_meco_condition, mission_params, telem, vessel, conn,
                   maxq_thrust_control, max_accel_thrust_control)
    meco(vessel)

    return vessel


def roll_program(mission_params, telem, vessel, conn):
    # roll program after velocity reached based on TWR
    # heading angle
    print("Entering Roll Program")
    vessel.auto_pilot.target_pitch_and_heading(89,
                                               mission_params.target_heading)
    vessel.auto_pilot.target_roll = mission_params.target_roll
    while telem.velocity() < 20 or telem.surface_altitude() < (
            220 + mission_params.altimeter_bias):
        utils.abort_system(is_abort_installed, abort_criteria, vessel,
                           mission_params, conn)
        pass

    # time.sleep(0.5)
    # vessel.auto_pilot.target_roll = mission_params.target_roll
    # print(vessel.auto_pilot.roll_error,vessel.auto_pilot.target_roll)
    while telem.velocity() < 40 or telem.altitude() < 350 + \
            mission_params.altimeter_bias:
        utils.abort_system(is_abort_installed, abort_criteria, vessel,
                           mission_params, conn)
        pass
    time.sleep(1 / CLOCK_RATE)


def pitch_maneuver(prop_meco_condition, mission_params, telem, vessel, conn,
                   maxq_thrust_control, max_accel_thrust_control):
    print("Entering Pitch Over Maneuver")
    # inertial_ref = vessel.orbit.body.non_rotating_reference_frame
    meco = False
    while not meco:
        utils.abort_system(is_abort_installed, abort_criteria, vessel,
                           mission_params, conn)
        if not mission_params.maxq_exit:
            maxQ(mission_params, telem, vessel, maxq_thrust_control)

        if vessel.flight(
        ).g_force >= mission_params.max_g and not mission_params.maxg_enter:
            mission_params.maxg_enter = True
            print("Throttling Down to Maintain Load")
        if mission_params.maxq_exit and mission_params.maxg_enter and not mission_params.maxg_exit:
            # this criteria will never be met AFTER maxQ exits
            #      so this will always be invoked in flight
            max_accel(mission_params, telem, vessel, max_accel_thrust_control)

        tot_thrust = vessel.thrust
        if tot_thrust <= 0:
            meco = True
            break
        #TODO: possibly add a section to look-up fuel and enter constant pitch to avoid shuttle flipping on MECO
        if vessel.resources.amount(
                "LiquidFuel"
        ) <= prop_meco_condition + upper_stage_LF + payload_LF and prop_meco_condition != 0:
            print("Stabilizing Pitch for MECO")
            time.sleep(5)
            meco = True
            break


        d_theta = np.arctan2(telem.velocity(),
                             mission_params.v_stage) * 180 / np.pi
        # vessel.auto_pilot.target_pitch_and_heading(90-d_theta, mission_params.target_heading)
        vessel.auto_pilot.target_pitch = 90 - d_theta

        # altitude_ratio = vessel.flight(
        #     inertial_ref).mean_altitude / mission_params.grav_turn_end

        # Quartic Easing Out Equation
        # altitude_ratio = altitude_ratio-1
        # vessel.auto_pilot.target_pitch = -(-90*(altitude_ratio**4 - 1)) + 90

        # Quadratic Eeasing Out Equation
        # vessel.auto_pilot.target_pitch = - \
        #     (-90 * altitude_ratio * (altitude_ratio - 2)) + 90

        time.sleep(1 / CLOCK_RATE)


def maxQ(mission_params, telem, vessel, maxq_thrust_control):
    if vessel.flight().dynamic_pressure >= 10000 and telem.altitude() < 12000:
        vessel.control.throttle = maxq_thrust_control.update(
            vessel.flight().dynamic_pressure)
        if not mission_params.maxq_enter:
            print("MaxQ - Entering Throttle Bucket")
            mission_params.maxq_enter = True
    elif telem.altitude() >= 12000:
        vessel.control.throttle = 1
        if not mission_params.maxq_exit:
            print("Exiting MaxQ - Throttle Up 1st Stage")
            mission_params.maxq_exit = True
    else:
        vessel.control.throttle = 1


def max_accel(mission_params, telem, vessel, max_accel_thrust_control):
    print(vessel.flight().g_force,
          max_accel_thrust_control.update(vessel.flight().g_force))
    #if vessel.flight().g_force >= mission_params.max_g:
    vessel.control.throttle = max_accel_thrust_control.update(
        vessel.flight().g_force)


def meco(vessel):
    vessel.control.throttle = 0
    vessel.auto_pilot.disengage()
    print("MECO")
    time.sleep(1 / CLOCK_RATE)


def second_stage(conn, vessel, mission_params):
    vessel.control.toggle_action_group(1)
    #vessel.control.activate_next_stage()
    #part = vessel.parts.with_tag("stage_separator")[0]
    #part.docking_port.undock()
    print("2nd Stage Separation Confirmed")
    secondstage, telem = utils.check_active_vehicle(conn, vessel,
                                                    mission_params.root_vessel)
    #secondstage.control.activate_next_stage()
    secondstage.control.toggle_action_group(2)
    secondstage.control.sas = True
    secondstage.control.rcs = True
    secondstage.control.throttle = 1.0

    print(secondstage.available_thrust)
    while secondstage.available_thrust <= 0:
        secondstage, telem = utils.check_active_vehicle(
            conn, vessel, mission_params.root_vessel)
        time.sleep(10 / CLOCK_RATE)
        secondstage.control.activate_next_stage()
        print("2nd Stage Engine Ignition")

    print("Enter closed loop guidance, second stage.")
    secondstage = final_stage.close_loop_guidance(secondstage, mission_params, telem, 100, mission_params.target_heading)

    return secondstage


if __name__ == "__main__":
    main()
    print("DONE")
    quit()
