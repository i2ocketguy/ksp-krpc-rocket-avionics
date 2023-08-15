import krpc

# Class that maintains a generic launch vehicle or spacecraft configuration
class launch_vehicle():
    def __init__(self, vessel, CLOCK_RATE, root_vessel, altimeter_bias, v_stage, upper_stage_LF, payload_LF, meco_condition_multiplier, TELEM_RATE = None, is_abort_installed = False, abort_criteria = 0):
        self.is_abort_installed = is_abort_installed
        self.abort_criteria = abort_criteria
        self.upper_stage_LF = upper_stage_LF
        self.payload_LF = payload_LF
        self.meco_condition_multiplier = meco_condition_multiplier
        self.CLOCK_RATE = CLOCK_RATE
        if TELEM_RATE is None:
            self.TELEM_RATE = CLOCK_RATE
        else:
            self.TELEM_RATE = TELEM_RATE
        self.root_vessel = root_vessel
        self.altimeter_bias = altimeter_bias
        self.v_stage =  v_stage
        self.bref = vessel.orbit.body.reference_frame
