import krpc
import numpy as np
import time

# constants
desired_heading = 90
v_stage = 1200
root_vessel = "MOD-8"
altimeter_bias = 92

def roll_program(desired_heading):
    # roll program after velocity reached based on TWR
    # heading angle
    print("Entering Roll Program")
    while velocity() < 20 and altitude() < 100+altimeter_bias:
        pass
    vessel.auto_pilot.target_pitch_and_heading(90, desired_heading)
    vessel.auto_pilot.target_roll = 180
    print(vessel.auto_pilot.roll_error,vessel.auto_pilot.target_roll)
    while velocity() < 40 or altitude() < 250+altimeter_bias:
        pass
    time.sleep(0.1)
    

def pitch_maneuver():
    print("Entering Pitch Over Maneuver")
    # print(vessel.resources_in_decouple_stage(2, False).amount("LiquidFuel"))
    while apoapsis() < 1000000:
        # NOTE: Use this for gathering LiquidFuel Stage information
        #       Did not work for Avionics Test Rocket (due to root node?)
        # mf = vessel.resources_in_decouple_stage(2, False).amount("LiquidFuel")
        # if mf <= 0:
        #     break
        
        tot_thrust = vessel.thrust
        if tot_thrust <= 0:
            break
        d_theta = np.arctan2(velocity(), v_stage)*180/np.pi
        vessel.auto_pilot.target_pitch_and_heading(90-d_theta, desired_heading)
        maxQ()
        time.sleep(0.1)

def maxQ():
    if vessel.flight().dynamic_pressure >= 8000 and altitude() < 10000:
        vessel.control.throttle = 0.85
    else:
        vessel.control.throttle = 1

def check_active_vehicle(vessel):
    print("Checking active vehicle...")
    checked = conn.space_center.active_vessel
    print("Current active vehicle is " + checked.name)
    if checked.name != root_vessel:
        vessels = conn.space_center.vessels
        for vessel1 in vessels:
            print(vessel1.name)
        for target_vessel in vessels:
            if target_vessel.name == root_vessel: 
                # print(target_vessel.name)
                conn.space_center.active_vessel = target_vessel
                vessel = conn.space_center.active_vessel
                print("New active vehicle: " + vessel.name)
                continue
    else:
        vessel = checked
        print("Vessel not changed, active vessel: " + vessel.name)
    check_control(vessel)
    return vessel

def check_control(vessel):
    # initialize control systems
    print(conn.space_center.active_vessel.name)
    control_point = vessel.parts.controlling
    print("Current control point name is " + vessel.parts.controlling.name)
    # if control_point.with_module('ModuleCommand') == False:
    if 'ModuleCommand' not in [x.name for x in control_point.modules]:
        vessel.parts.controlling = vessel.parts.with_module('ModuleCommand')[0]
        print("Changed control point to " + vessel.parts.controlling.name)
        print("Current control point name is " + vessel.parts.controlling.name)
    
    return vessel


# # TODO Reactivate this line
# vessel.control.activate_next_stage()
# vessels = conn.space_center.vessels
# # for vessel1 in vessels:
# #     print(vessel1.name)
# print(vessel.name)
# for target_vessel in vessels:
#     if target_vessel.name == 'Avionics Rocket Test Article': 
#         # print(target_vessel.name)
#         conn.space_center.active_vessel = target_vessel
#         vessel = conn.space_center.active_vessel
#         break

conn = krpc.connect(name='Launch')
vessel = conn.space_center.active_vessel
print(vessel.name)
vessel.control.sas = True
vessel.control.throttle = 1

vessel.control.activate_next_stage() #Engine Ignition
time.sleep(0.25)
vessel.control.activate_next_stage() #Attempted pad disconnet
time.sleep(1)
vessel = check_active_vehicle(vessel)

vessel.auto_pilot.engage()
ref = vessel.flight(vessel.orbit.body.reference_frame)
vessel.auto_pilot.target_pitch_and_heading(vessel.flight().pitch, vessel.flight().heading)
vessel.auto_pilot.auto_tune = True
# vessel.auto_pilot.roll_pid_gains = (.5,0.01,25.5)
# vessel.auto_pilot.pitch_pid_gains = (1.1,0.1,1.5)
# vessel.auto_pilot.yaw_pid_gains = (1.1,0.1,1.5)

# set up data streams
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
velocity = conn.add_stream(getattr, ref, 'speed')

roll_program(desired_heading)
time.sleep(1)
pitch_maneuver()

# MECO
vessel.control.throttle = 0
vessel.auto_pilot.disengage()
time.sleep(0.1)

# 2nd Stage Separation
vessel.control.toggle_action_group(1)
time.sleep(0.1)
vessel.control.toggle_action_group(2)
time.sleep(0.1)
vessel.control.activate_next_stage()
secondstage = check_active_vehicle(vessel)
secondstage.control.sas = True
secondstage.control.rcs = True
print(secondstage.available_thrust)
time.sleep(1)
if secondstage.available_thrust <= 0:
    secondstage.control.activate_next_stage()

secondstage.control.throttle = 1


print("DONE\n")

