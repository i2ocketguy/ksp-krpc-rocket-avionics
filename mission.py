class MissionParameters():
    def __init__(self, root_vessel, state="init", target_apoapsis=100000, grav_turn_end=60000, target_inc=0, target_roll=0, max_q=8000, max_g=3, v_stage=1000, altimeter_bias=90):
        self.root_vessel = root_vessel
        self.state = state,
        self.target_apoapsis = target_apoapsis
        self.grav_turn_end = grav_turn_end
        self.target_inc = target_inc
        self.target_roll = target_roll
        self.max_q = max_q
        self.max_g = max_g
        self.v_stage = v_stage
        self.altimeter_bias = altimeter_bias
        self.target_heading = 0
        self.maxq_enter = False
        self.maxq_exit = False
        self.maxg_enter = False
        self.maxg_exit = False

class Telemetry():
    def __init__(self, conn, vessel):
        self.surface_altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'surface_altitude')
        self.altitude = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'mean_altitude')
        self.apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        self.velocity = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
        self.vertical_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
        self.horizontal_vel = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'horizontal_speed')
        self.periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')