import controllers
import time

# constants
CLOCK_RATE = 10  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]

def close_loop_guidance(vessel, mission, telem, init_apo, heading, pid_input=None,target_apo=None, target_periapsis=None):

    if target_apo == None:
        target_apo = mission.target_apoapsis

    if controllers is not None:
        pitch_control = controllers.PID(0.001,
                                0.0001,
                                0.0003,
                                -vessel.flight().pitch,
                                vessel.flight().pitch,
                                deadband=100)
    else:
        pitch_control = pid_input
    #NOTE: init_apo in km
    pitch_control.set_point = init_apo*1000

    vessel.auto_pilot.engage()
    vessel.auto_pilot.auto_tune = True
    vessel.auto_pilot.target_roll = mission.target_roll
    vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch, heading)
    time.sleep(5)
    
    rate_control = controllers.PID(0.06,
                            0.00001,
                            0.0,
                            -60,
                            60,
                            deadband=0.01)
    rate_control.set_point = 1.0

    '''
    First pitch control loop used to manage apoapsis altitude wrt vertical velocity
    '''
    print("Entering first control loop.")
    apo_achieved = False
    pitch_control.set_point = 10.0
    while telem.apoapsis() < target_apo or telem.periapsis() < 0.0:
        pitch_angle = pitch_control.update(telem.vertical_vel())
        vessel.auto_pilot.target_pitch = pitch_angle
        if telem.periapsis() >= 0.5*target_apo:
            vessel.auto_pilot.target_pitch = 0
            break
        time.sleep(1 / CLOCK_RATE)

    '''
    Second pitch control loop used to manage vertical velocity
    '''
    print("Entering second control loop.")
    while telem.periapsis() < target_apo*.95:
        pitch_control.set_point = -0.1 # target veritcal velocity
        pitch_control.deadband = 0.3
        pitch_control.Kp = 1
        pitch_angle = pitch_control.update(telem.vertical_vel())
        vessel.auto_pilot.target_pitch = pitch_angle
        if telem.apoapsis() > target_apo*1.05:
            vessel.auto_pilot.target_pitch = 0
            break
        if target_periapsis is not None:
            if telem.periapsis() > 0.95*target_periapsis:
                break
        time.sleep(1 / CLOCK_RATE)

    '''
    Third pitch control used to manage periapsis to target apoapsis
    '''
    print("Entering third control loop.")
    while telem.periapsis() < target_apo:
        vessel.auto_pilot.target_pitch = 0
        if telem.apoapsis() > target_apo*1.05:
            vessel.auto_pilot.target_pitch = 0
            break
        if target_periapsis is not None:
            if telem.periapsis() > target_periapsis:
                break
        time.sleep(1 / CLOCK_RATE)

    vessel.control.throttle = 0.0

    return vessel
