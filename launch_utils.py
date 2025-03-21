import krpc
import numpy as np
import time
import controllers
import mission


def initialize():
    conn = create_instance()
    vessel = initialize_active_vessel(conn)
    return conn, vessel


def create_instance():
    return krpc.connect(name='Launch')


def initialize_active_vessel(conn):
    return conn.space_center.active_vessel


def check_active_vehicle(conn, desired_name):
    """
    Force the active vessel to be the one with the given name (e.g., "Minerva-II-R FR"),
    if it is not already. Returns the vessel object. This version tries to do so quickly.
    """
    current_vessel = conn.space_center.active_vessel
    if current_vessel.name == desired_name:
        return current_vessel  # No change needed

    # Force set if the active vessel is not correct
    for v in conn.space_center.vessels:
        if v.name == desired_name:
            conn.space_center.active_vessel = v
            return v
    
    # If not found at all, return the current anyway
    return current_vessel


def check_vehicle_control(conn, vessel, control_tag="control_point"):
    """
    Ensure the vessel is controlled from a part that has the given tag (default "control_point").
    If not found, it logs a warning. This is performed quickly without extra sleeps.
    """
    control_part = vessel.parts.controlling
    if control_part and control_part.tag == control_tag:
        # Already controlling from the correct part
        # print("Control point maintained:", control_part.name)
        return

    # Otherwise, try to set
    try:
        tagged_part = vessel.parts.with_tag(control_tag)[0]
        vessel.parts.controlling = tagged_part
        # print("New control point set:", tagged_part.name)
    except IndexError:
        # print("WARNING: No control point part found with tag:", control_tag)
        pass

def check_active_vehicle_and_control(conn, root_vessel_name, control_tag="control_point"):
    """
    Wrapper that ensures the active vessel is the rocket (by root_vessel_name)
    and that it is controlled from the correct control point. Returns the corrected vessel.
    """
    vessel = check_active_vehicle(conn, root_vessel_name)
    # print(vessel.name)
    check_vehicle_control(conn, vessel, control_tag=control_tag)
    return vessel

def set_azimuth(vessel, target_incl, bref):

    if target_incl == 0:
        return 90.0

    rd2, d2r = math_conversion()
    azimuth = np.arcsin(
        np.cos(
            target_incl *
            d2r) /
        np.cos(
            vessel.flight(bref).latitude *
            d2r))
    if np.isnan(azimuth):
        print("Flight Azimuth is NaN, inc: %f, lat: %f" %
              (target_incl, vessel.flight(bref).latitude))
        print("Revise flight plan, aborting program")
        quit()

    if target_incl < 0:
        azimuth = np.pi - azimuth

    azimuth = azimuth * rd2
    if azimuth < 0:
        azimuth = azimuth + 360
    print("Launch Azimuth: %f" % azimuth)
    print("Desired Inclination: %f" % target_incl)

    time.sleep(1)

    return azimuth


def math_conversion():
    r2d = 180 / np.pi
    d2r = np.pi / 180
    return r2d, d2r


def launch_countdown(countdown_start=10, engine_start=None, vessel = None):
    for x in range(countdown_start, 0, -1):
        if x % 10 == 0:
            print("T - %d" % int(x))
        elif x < 10:
            print("T - %d" % int(x))
        if vessel is not None and engine_start is not None:
            if int(x) == engine_start:
                vessel.control.activate_next_stage()
        time.sleep(1)

def set_launch_ut(sc, target_time_str, engine_start=None, vessel = None):
    YEAR_SECONDS = 9203545 - 1945
    DAY_SECONDS = 21600

    # Parse input string
    parts = target_time_str.split(', ')
    years = int(parts[0][1:]) - 1  # Subtract 1 because the game starts from year 1
    days = int(parts[1][1:]) - 1   # Subtract 1 because the game starts from day 1
    hms = list(map(int, parts[2].split(':')))
    
    # Convert to UT
    target_ut = years * YEAR_SECONDS + days * DAY_SECONDS
    target_ut += hms[0] * 3600 + hms[1] * 60 + hms[2]
    
    # Wait until the target time
    while sc.ut < target_ut:

        time_left = target_ut - sc.ut
        x = int(time_left)
        if x % 10 == 0:
            print("T - %d" % int(x))
        elif x < 10:
            print("T - %d" % int(x))
        if vessel is not None and engine_start is not None:
            if int(x) == engine_start:
                vessel.control.activate_next_stage()
        time.sleep(1)  # adjust as needed

def pad_separation(vessel):
    part = vessel.parts.with_tag("pad_separator")[0]
    part.docking_port.undock()
    print("Lift Off!")
    time.sleep(0.05)

def abort_system(is_abort_installed, abort_criteria, vessel, mission_params, conn, crew_vehicle):
    if is_abort_installed:
        abort_trigger_check(abort_criteria, vessel, mission_params, conn, crew_vehicle)
    else:
        pass

def abort_trigger_check(abort_criteria, vessel, mission_params, conn, crew_vehicle):
    # TODO: check abort activation
    #  check crtieria surpassed
    #  if crtieria surpassed, abort
    if vessel.auto_pilot.pitch_error > abort_criteria or vessel.control.abort is True:
        vessel.control.abort = True
        vessel.control.throttle = 1
        print("Launch Abort: Abort criteria exceeded")
        abort_steering(vessel, mission_params, conn, crew_vehicle)

def abort_steering(vessel, mission_params, conn, crew_vehicle):
    vessel.auto_pilot.disengage()
    vessel, telem = check_active_vehicle(conn, vessel, crew_vehicle)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch = 65
    vessel.control.rcs = True
    time.sleep(0.5)
    while vessel.thrust > 0:
        pass

    if vessel.thrust <= 0:
        for port in vessel.parts.docking_ports:
            if port.state == conn.space_center.DockingPortState.docked:
                vessel.auto_pilot.disengage()
                vessel = port.undock()
                vessel, _ = check_active_vehicle(conn, vessel, crew_vehicle)
                vessel.control.rcs = True
                vessel.control.sas = True
                time.sleep(0.5)
                quit()
    else:
        quit()

def throttle_from_twr(vessel, twr_desired):
    m0 =  vessel.mass
    g = vessel.orbit.body.surface_gravity
    # twr = thrust/(m0*g)
    # thrust = twr*m0*g
    return twr_desired*m0*g/(vessel.max_thrust)
        
