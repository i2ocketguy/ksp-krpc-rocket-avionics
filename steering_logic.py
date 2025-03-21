import launch_utils as utils
import time
import numpy as np
import math
import controllers

# Consists of functions for steering and controlling a generic launch vehicle.
# - Pitch Maneuver
# - Roll Program
# - MaxQ throttle
# - constant acceleration

def roll_program(mission_params, telem, vessel, conn, sc):
    # roll program after velocity reached based on TWR
    # heading angle
    print("Entering Roll Program")
    vessel.auto_pilot.target_pitch_and_heading(89,
                                               mission_params.target_heading)
    vessel.auto_pilot.target_roll = mission_params.target_roll
    # vessel.auto_pilot.wait()
    while telem.velocity() < 20 or telem.surface_altitude() < (
            220 + mission_params.altimeter_bias):
        utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
                           mission_params, conn, "KRV Shuttle")
        pass

    # while telem.velocity() < 40 or telem.altitude() < 350 + \
    #         mission_params.altimeter_bias:
    #     utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
    #                        mission_params, conn)
    #     pass
    time.sleep(1 / sc.CLOCK_RATE)


def pitch_maneuver(prop_meco_condition, mission_params, telem, vessel, conn, sc,
                   maxq_thrust_control, max_accel_thrust_control, apoapsis_limit=None):
    print("Entering Pitch Over Maneuver")
    # inertial_ref = vessel.orbit.body.non_rotating_reference_frame
    meco = False
    while not meco:
        utils.abort_system(sc.is_abort_installed, sc.abort_criteria, vessel,
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
        ) <= prop_meco_condition + sc.upper_stage_LF + sc.payload_LF and prop_meco_condition != 0:
            print("Stabilizing Pitch for MECO")
            time.sleep(5)
            meco = True
            break
        if apoapsis_limit is not None and telem.apoapsis() > apoapsis_limit:
            meco = True
            print("Apoapsis limit reached. Staging...")
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

        time.sleep(1 / sc.CLOCK_RATE)

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
    #print(vessel.flight().g_force,max_accel_thrust_control.update(vessel.flight().g_force))
    #if vessel.flight().g_force >= mission_params.max_g:
    vessel.control.throttle = max_accel_thrust_control.update(
        vessel.flight().g_force)


def meco(vessel, sc):
    vessel.control.throttle = 0
    vessel.auto_pilot.disengage()
    print("MECO")

def calculate_landing_burn(vessel):

    def get_flight_path_angle(vessel):
        r = vessel.position(srf_frame)
        v = vessel.velocity(srf_frame)
        r_mag = np.linalg.norm(r)
        v_mag = np.linalg.norm(v)
        return math.asin(np.dot(r,v)/(r_mag*v_mag))

    def get_drag(vessel):
        drag = vessel.flight().drag
        return math.sqrt(math.pow(drag[0],2) + math.pow(drag[1],2) + math.pow(drag[2],2))

    srf_frame = vessel.orbit.body.reference_frame
    g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
    engine_offset = (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
    drag_scale = 0.42

    #while not landing_burn_flag:
    m0 = vessel.mass
    v_inf = vessel.flight(srf_frame).speed
    h_inf = vessel.flight().surface_altitude + engine_offset
    ht = 0 # target stopping altitude

    fpa = get_flight_path_angle(vessel)
    a = 1
    b = math.sin(fpa)*(math.pow(v_inf,2)/(2*(h_inf-ht)*g))
    c = -(((math.pow(v_inf,2)*(1+math.pow(math.sin(fpa),2)))/(4*(h_inf-ht)*g))+1)
    dis = (b*b) - (4*a*c)
    aT_g = (-b + math.sqrt(dis))/(2*a)
    T = aT_g*g*m0 #- get_drag(vessel)*drag_scale
    T = T/(vessel.max_thrust)
    #print(T)

    return T

def _calculate_landing_burn_time(vessel):

    def get_flight_path_angle(vessel,):
        r = vessel.position(srf_frame)
        v = vessel.velocity(srf_frame)
        r_mag = np.linalg.norm(r)
        v_mag = np.linalg.norm(v)
        return math.asin(np.dot(r,v)/(r_mag*v_mag))

    def get_drag(vessel):
        drag = vessel.flight().drag
        return math.sqrt(math.pow(drag[0],2) + math.pow(drag[1],2) + math.pow(drag[2],2))

    srf_frame = vessel.orbit.body.reference_frame
    g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
    engine_offset = (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]

    #while not landing_burn_flag:
    m0 = vessel.mass
    v_inf = vessel.flight(srf_frame).speed
    h_inf = vessel.flight().surface_altitude + engine_offset
    ht = 20 # target stopping altitude

    fpa = get_flight_path_angle(vessel)
    a = 1
    b = math.sin(fpa)*(math.pow(v_inf,2)/(2*(h_inf-ht)*g))
    c = -(((math.pow(v_inf,2)*(1+math.pow(math.sin(fpa),2)))/(4*(h_inf-ht)*g))+1)
    dis = (b*b) - (4*a*c)
    aT_g = (-b + math.sqrt(dis))/(2*a)
    a = aT_g*g
    tb = 0.95*(v_inf/a)-(v_inf/(vessel.available_thrust/vessel.mass))
    return tb

def get_flight_path_angle(r, v):

    r_mag = np.linalg.norm(r)
    v_mag = np.linalg.norm(v)
    return math.asin(np.dot(r,v)/(r_mag*v_mag))

def calculate_landing_burn_time(vessel, g, engine_offset):

    srf_frame = vessel.orbit.body.reference_frame
    g = vessel.orbit.body.gravitational_parameter/(vessel.orbit.body.equatorial_radius*vessel.orbit.body.equatorial_radius)
    #while not landing_burn_flag:
    v_inf = vessel.flight(srf_frame).speed
    h_inf = vessel.flight().surface_altitude + engine_offset
    ht = 20 # target stopping altitude

    r = vessel.position(srf_frame)
    v = vessel.velocity(srf_frame)
    fpa = get_flight_path_angle(r, v)
    a = 1
    b = math.sin(fpa)*(math.pow(v_inf,2)/(2*(h_inf-ht)*g))
    c = -(((math.pow(v_inf,2)*(1+math.pow(math.sin(fpa),2)))/(4*(h_inf-ht)*g))+1)
    dis = (b*b) - (4*a*c)
    aT_g = (-b + math.sqrt(dis))/(2*a)
    a = aT_g*g
    tb = 0.95*(v_inf/a)-(v_inf/(vessel.available_thrust/vessel.mass))
    return tb
            
def landing_gate(vessel, telem, pid):
    pid.set_point = -5.0
    vessel.control.throttle = pid.update(telem.vertical_vel())

def vang(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    print(v1, v2)
    dotted = v1.dot(v2)
    print(dotted)
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    angle = np.arccos(dotted/(v1_norm*v2_norm))*180/np.pi
    print(angle)
    return angle

def calc_pitch_and_roll(v1):
    v1 = np.asarray(v1) # [x,y,z]
    rot_x = np.arctan2(v1[1], v1[2])
    rot_y = np.arctan2(v1[0], np.hypot(v1[1], v1[2]))
    pitch, heading = rot_x*180/np.pi, rot_y*180/np.pi
    return pitch, heading

def compute_los_angle(current_position, target_position):
    delta_y = target_position[0] - current_position[0]  # Change in latitude
    delta_x = target_position[1] - current_position[1]   # Change in longitude
    los_angle = np.arctan2(delta_y, delta_x)
    return los_angle

def compass_heading(angle):
    offset = 90
    if angle > 180:
        heading = normalize_angle(angle-offset)
    else:
        heading = normalize_angle(offset-angle)
    
    return heading

def normalize_angle(angle):
    return angle % 360

def haversine_distance(lat1, lon1, lat2, lon2):
    r = 600000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2-lat1)
    delta_lambda = math.radians(lon2-lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = r * c
    return distance

def compute_heading_error(current_heading, target_heading):
    error = target_heading - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

