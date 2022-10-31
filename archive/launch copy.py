import numpy as np
import time
import mission
import launch_utils as utils
import pid

#constants
CLOCK_RATE = 10 # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]
root_vessel = "ALV-15"
# v_stage = 1000  # velocity target for 45 degree pitch over
# altimeter_bias = 92

def main():
    conn, vessel = utils.initialize()
    mission_params = mission.MissionParameters(root_vessel, target_inc=-0.1, grav_turn_end=85000, max_q=14000)
    telem = mission.Telemetry(conn, vessel)
    launch(conn,vessel, mission_params, telem)
    second_stage(conn,vessel, mission_params)

def launch(conn, vessel, mission_params, telem):
    #initial vehicle setup
    bref = vessel.orbit.body.reference_frame
    mission_params.target_heading = utils.set_azimuth(vessel, mission_params.target_inc, bref)
    thrust_control = pid.PID(0.001, 0.0001, 0.0, clamp=mission_params.max_q)
    thrust_control.set_point = mission_params.max_q

    #Pre-Launch
    vessel.control.sas = True
    vessel.control.throttle = 1
    utils.launch_countdown()

    print("Engine Ignition")
    vessel.control.activate_next_stage() #Engine Ignition
    time.sleep(0.5)

    #ignition abort: engines not started
    if vessel.thrust == 0:
        vessel.control.throttle = 0.0
        print("Launch Abort: off-nominal engine ignition")
        quit()

    print("Launch")
    vessel.control.activate_next_stage() #Attempted pad disconnet
    time.sleep(3)
    vessel = utils.check_active_vehicle(conn, vessel, mission_params.root_vessel)

    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch, vessel.flight().heading)
    vessel.auto_pilot.auto_tune = True

    roll_program(mission_params, telem, vessel)
    time.sleep(1/CLOCK_RATE)
    pitch_maneuver(mission_params, telem, vessel, thrust_control)
    meco(vessel)

def roll_program(mission_params, telem, vessel):
    # roll program after velocity reached based on TWR
    # heading angle
    velocity = telem.velocity()
    print("Entering Roll Program")
    while telem.velocity() < 20 and telem.altitude() < 100 + mission_params.altimeter_bias:
        pass
    vessel.auto_pilot.target_pitch_and_heading(90, mission_params.target_heading)
    vessel.auto_pilot.target_roll = mission_params.target_roll
    # print(vessel.auto_pilot.roll_error,vessel.auto_pilot.target_roll)
    while telem.velocity() < 40 or telem.altitude() < 250 + mission_params.altimeter_bias:
        pass
    time.sleep(1/CLOCK_RATE)

def pitch_maneuver(mission_params, telem, vessel, thrust_control):
    print("Entering Pitch Over Maneuver")
    inertial_ref = vessel.orbit.body.non_rotating_reference_frame
    while telem.apoapsis() < 1000000:
        # NOTE: Use this for gathering LiquidFuel Stage information
        #       Did not work for Avionics Test Rocket (due to root node?)
        # mf = vessel.resources_in_decouple_stage(2, False).amount("LiquidFuel")
        # if mf <= 0:
        #     break
        
        tot_thrust = vessel.thrust
        if tot_thrust <= 0:
            break

        # d_theta = np.arctan2(telem.velocity(), mission_params.v_stage)*180/np.pi
        # vessel.auto_pilot.target_pitch_and_heading(90-d_theta, mission_params.target_heading)
        altitude_ratio = vessel.flight(inertial_ref).mean_altitude / mission_params.grav_turn_end

        # Quartic Easing Out Equation
        # altitude_ratio = altitude_ratio-1
        # vessel.auto_pilot.target_pitch = -(-90*(altitude_ratio**4 - 1)) + 90

        # Quadratic Eeasing Out Equation
        vessel.auto_pilot.target_pitch = -(-90*altitude_ratio*(altitude_ratio-2)) + 90
        
        maxQ(mission_params, telem, vessel, thrust_control)
        time.sleep(1/CLOCK_RATE)

def maxQ(mission_params, telem, vessel, thrust_control):
    if vessel.flight().dynamic_pressure >= 10000 and telem.altitude() < 10000:
        vessel.control.throttle = thrust_control.update(vessel.flight().dynamic_pressure)
        if mission_params.maxq_enter == False:
            print("MaxQ - Entering Throttle Bucket")
            mission_params.maxq_enter = True
    elif telem.altitude() >= 10000:
        vessel.control.throttle = 1
        if mission_params.maxq_exit == False:
            print("Exiting MaxQ - Throttle Up 1st Stage")
            mission_params.maxq_exit = True
    else:
        vessel.control.throttle = 1

def meco(vessel):
    vessel.control.throttle = 0
    vessel.auto_pilot.disengage()
    time.sleep(1/CLOCK_RATE)

def second_stage(conn, vessel, mission_params):
    vessel.control.toggle_action_group(1)
    time.sleep(1/CLOCK_RATE)
    vessel.control.toggle_action_group(2)
    time.sleep(1/CLOCK_RATE)
    vessel.control.activate_next_stage()
    secondstage = utils.check_active_vehicle(conn,vessel, mission_params.root_vessel)
    secondstage.control.sas = True
    secondstage.control.rcs = True
    print(secondstage.available_thrust)
    time.sleep(1)
    if secondstage.available_thrust <= 0:
        secondstage.control.activate_next_stage()

    secondstage.control.throttle = 1

if __name__ == "__main__" :
    main()
    print("DONE")
    quit()

