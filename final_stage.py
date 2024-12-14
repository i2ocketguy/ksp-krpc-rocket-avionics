import controllers
import time

# constants
CLOCK_RATE = 20  # refresh rate [Hz]

def close_loop_guidance(vessel, mission, telem, init_apo, heading, pid_input=None,target_apo=None, target_periapsis=None):

    if target_apo == None:
        target_apo = mission.target_apoapsis

    K1 = 1.02
    K2 = 25
    Kp = 0.2*K1
    Ki = 2.0*K1/K2
    Kd = Kp*K2/3.0
    if pid_input is None:
        pitch_control = controllers.PID(0.0,
                                Kp,
                                Ki,
                                Kd,
                                -40,
                                40,
                                deadband=300)
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
    rate_control = controllers.PID(0.0,
                            RKp,
                            RKi,
                            RKd,
                            -40,
                            40,
                            deadband=0.5)
    rate_control.set_point = -0.5 # target veritcal velocity

    vessel.auto_pilot.engage()
    vessel.auto_pilot.auto_tune = True
    vessel.auto_pilot.target_roll = mission.target_roll
    vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch, heading)
    time.sleep(5)
    

    start_time = time.time()
    '''
    First pitch control loop used to manage apoapsis altitude wrt vertical velocity
    '''
    print("Entering first control loop.")
    while telem.vertical_vel() > 0:
        pitch_cmd = pitch_control.update(telem.apoapsis())
        vessel.auto_pilot.target_pitch = pitch_cmd
        vessel.auto_pilot.target_heading = heading
        if telem.periapsis() > 20*1000:
            break
        time.sleep(1 / CLOCK_RATE)
    '''
    Second pitch control loop used to manage vertical velocity
    '''
    print("Entering second control loop.")
    while telem.periapsis() < 70*1000:
        rate_control.deadband = 0.5
        pitch_angle = rate_control.update(telem.vertical_vel())
        vessel.auto_pilot.target_pitch = pitch_angle
        vessel.auto_pilot.target_heading = heading

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
