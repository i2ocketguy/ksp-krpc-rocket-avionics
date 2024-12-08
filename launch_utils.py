import os
from enum import Enum

import krpc
import numpy as np
import time
import pid
import mission
from telemetry import KSPTelemetry

CONTROL_MODE = "control_mode"
class ControlMode(str, Enum):
    PAD = "Pad Flight Computer Boot"
    ROLL = "Roll Program"
    PITCH_OVER = "Pitch Over Maneuver"
    MAX_Q = "MaxQ Throttle Bucket"
    THROTTLE_BUCKET_EXIT = "Throttle Bucket Exit"
    MECO_PREP = "Stabilizing Pitch for MECO"
    MECO = "MECO"
    S2_INIT = "Stage 2 Init"
    S2_LOOP1 = "Second Stage Control Loop 1"
    S2_LOOP2 = "Second Stage Control Loop 2"
    S2_LOOP3 = "Second Stage Control Loop 3"
    COAST = "Coast"
    ABORT = "Abort"


def enter_control_mode(mode: ControlMode, telem_viz: KSPTelemetry, console_out: bool = True):
    telem_viz.publish_enum_metric(CONTROL_MODE, mode.name, mode.value, console_out)


def initialize():
    conn = create_instance()
    vessel = initialize_active_vessel(conn)
    return conn, vessel


def create_instance():
    return krpc.connect(name='Launch', address=os.environ.get('KRPC_ADDRESS', '127.0.0.1')) # todo add this to all instances like this


def initialize_active_vessel(conn):
    return conn.space_center.active_vessel


def check_active_vehicle(conn, vessel, root_vessel):
    print("Checking active vehicle...")
    checked = conn.space_center.active_vessel
    print("Current active vehicle is " + checked.name)
    if checked.name != root_vessel:
        vessels = conn.space_center.vessels
        for target_vessel in vessels:
            if target_vessel.name == root_vessel:
                # print(target_vessel.name)
                conn.space_center.active_vessel = target_vessel
                vessel = conn.space_center.active_vessel
                print("New active vehicle: " + vessel.name)
                break
    else:
        vessel = checked
        print("Vessel not changed, active vessel: " + vessel.name)

    check_control(conn, vessel, root_vessel)
    telem = mission.Telemetry(conn, vessel)
    return vessel, telem


def check_control(conn, vessel, root_vessel):
    # initialize control systems
    control_point = vessel.parts.controlling
    # print("Current control point name is " + vessel.parts.controlling.name)
    # if control_point.with_module('ModuleCommand') == False:
    # if 'ModuleCommand' not in [x.name for x in control_point.modules]:
    #     vessel.parts.controlling = vessel.parts.with_module('ModuleCommand')[0]
    #     print("Changed control point to " + vessel.parts.controlling.name)
    #     print("Current control point name is " + vessel.parts.controlling.name)
    if control_point.tag == "control_point":
        print("Control point maintained")
    else:
        try:
            part = vessel.parts.with_tag("control_point")[0]
            vessel.parts.controlling = part
            print("New control point: %s" % part.name)
        except:
            print("PANIC - Control point does not exist")
            checked = conn.space_center.active_vessel
            print("Current active vehicle is " + checked.name)
            if checked.name != root_vessel:
                vessels = conn.space_center.vessels
                for target_vessel in vessels:
                    if target_vessel.name == root_vessel:
                        # print(target_vessel.name)
                        conn.space_center.active_vessel = target_vessel
                        vessel = conn.space_center.active_vessel
                        print("New active vehicle: " + vessel.name)
                        break

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

def abort_system(is_abort_installed, abort_criteria, vessel, mission_params, conn, crew_vehicle, telem_viz: KSPTelemetry):
    if is_abort_installed:
        abort_trigger_check(abort_criteria, vessel, mission_params, conn, crew_vehicle, telem_viz)
    else:
        pass

def abort_trigger_check(abort_criteria, vessel, mission_params, conn, crew_vehicle, telem_viz: KSPTelemetry):
    # TODO: check abort activation
    #  check crtieria surpassed
    #  if crtieria surpassed, abort
    if vessel.auto_pilot.pitch_error > abort_criteria or vessel.control.abort is True:
        vessel.control.abort = True
        vessel.control.throttle = 1
        enter_control_mode(ControlMode.ABORT, telem_viz)
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
        
