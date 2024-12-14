import controllers
import time

# constants
CLOCK_RATE = 10  # refresh rate [Hz]
TELEM_RATE = 1  # refresh rate for telemetry aquistion [Hz]

def close_loop_guidance(vessel, mission, telem, init_apo, heading, pid_input=None,target_apo=None, target_periapsis=None):

    if target_apo == None:
        target_apo = mission.target_apoapsis

    K1 = 1.02
    K2 = 25
    Kp = 0.2*K1
    Ki = 2.0*K1/K2
    Kd = Kp*K2/3.0
    if pid_input is None:
        pitch_control = controllers.PID(Kp,
                                Ki,
                                Kd,
                                -40,
                                40,
                                deadband=100)
    else:
        pitch_control = pid_input
    #NOTE: init_apo in km
    pitch_control.set_point = init_apo*1000
    # pitch_control.set_point = vessel.orbit.time_to_apoapsis


    R1 = 0.3
    R2 = 20
    RKp = 0.2*K1
    RKi = 2.0*K1/K2
    RKd = Kp*K2/3.0
    rate_control = controllers.PID(RKp,
                            RKi,
                            RKd,
                            -40,
                            40,
                            deadband=0.5)
    rate_control.set_point = -0.2 # target veritcal velocity

    vessel.auto_pilot.engage()
    vessel.auto_pilot.auto_tune = True
    vessel.auto_pilot.target_roll = mission.target_roll
    vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch, heading)
    time.sleep(5)
    

    '''
    First pitch control loop used to manage apoapsis altitude wrt vertical velocity
    '''
    counter = 0
    init_time = vessel.orbit.time_to_apoapsis
    print("Entering first control loop.")
    while telem.apoapsis() < target_apo:
        pitch_control.set_point = init_time-counter
        pitch_angle = pitch_control.update(vessel.orbit.time_to_apoapsis)
        if telem.apoapsis() > 0.9* target_apo:
            pitch_control.set_point = 10

        vessel.auto_pilot.target_pitch = pitch_angle
        if telem.periapsis() >= 0.5*target_apo:
            vessel.auto_pilot.target_pitch = 0
            break
        if telem.apoapsis() > 1.5*target_apo:
            break

        print(counter, int(counter), init_time-counter)
        if int(counter) < init_time-10:
            counter += 0.25/CLOCK_RATE
        time.sleep(1 / CLOCK_RATE)

    '''
    Second pitch control loop used to manage vertical velocity
    '''
    print("Entering second control loop.")
    while telem.periapsis() < 90*1000:
        rate_control.deadband = 0.5
        pitch_angle = rate_control.update(telem.vertical_vel())
        vessel.auto_pilot.target_pitch = pitch_angle
        if telem.apoapsis() > target_apo*1.3:
            vessel.auto_pilot.target_pitch = 0
            break
        # if target_periapsis is not None:
        #     if telem.periapsis() > 0.95*target_periapsis:
        #         break
        time.sleep(1 / CLOCK_RATE)

    '''
    Third pitch control used to manage periapsis to target apoapsis
    '''
    print("Entering third control loop.")
    while telem.apoapsis() < target_apo:
        vessel.auto_pilot.target_pitch = 0
        # if telem.apoapsis() > target_apo*1.2:
        #     vessel.auto_pilot.target_pitch = 0
        #     break
        # if target_periapsis is not None:
        #     if telem.periapsis() > 0.95*target_periapsis:
        #         break
        time.sleep(1 / CLOCK_RATE)

    vessel.control.throttle = 0.0

    return vessel
