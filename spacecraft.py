import krpc

# Class that maintains a generic launch vehicle or spacecraft configuration
class launch_vehicle():
    def __init__(self, vessel, CLOCK_RATE, root_vessel, altimeter_bias, v_stage, upper_stage_LF, payload_LF = 0, meco_condition_multiplier = 0, TELEM_RATE = None, is_abort_installed = False, abort_criteria = 0):
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
        
class LaunchVehicle:
    def __init__(self, vessel, CLOCK_RATE=1, root_vessel=None, altimeter_bias=0, v_stage=1, upper_stage_LF=1000, payload_LF=0, meco_condition_multiplier=1, TELEM_RATE=None, is_abort_installed=False, abort_criteria=0):
        self.vessel = vessel
        self.CLOCK_RATE = max(CLOCK_RATE, 1)  # Ensure CLOCK_RATE is at least 1
        self.root_vessel = root_vessel if root_vessel else vessel
        self.altimeter_bias = altimeter_bias
        self.v_stage = v_stage
        self.upper_stage_LF = upper_stage_LF
        self.payload_LF = payload_LF
        self.meco_condition_multiplier = max(meco_condition_multiplier, 1)  # Ensure multiplier is at least 1
        self.TELEM_RATE = TELEM_RATE if TELEM_RATE else CLOCK_RATE
        self.is_abort_installed = is_abort_installed
        self.abort_criteria = abort_criteria
        self.bref = vessel.orbit.body.reference_frame

    @classmethod
    def default(cls, vessel):
        """Factory method for a standard launch vehicle configuration."""
        return cls(vessel)

    @classmethod
    def custom_configuration(cls, vessel, **kwargs):
        """Factory method for a custom launch vehicle configuration."""
        return cls(vessel, **kwargs)

    def set_abort_installed(self, is_abort_installed=True):
        """Set whether an abort system is installed."""
        self.is_abort_installed = is_abort_installed
        return self

    def set_abort_criteria(self, abort_criteria):
        """Set the abort criteria."""
        self.abort_criteria = abort_criteria
        return self

    def set_upper_stage_LF(self, upper_stage_LF):
        """Set the upper stage liquid fuel amount."""
        self.upper_stage_LF = upper_stage_LF
        return self

    def set_payload_LF(self, payload_LF):
        """Set the payload liquid fuel amount."""
        self.payload_LF = payload_LF
        return self

    # Additional methods for other attributes can be added here following the same pattern.

    def __str__(self):
        """String debug output for a launch vehicle instance"""
        return f"LaunchVehicle Configuration: CLOCK_RATE={self.CLOCK_RATE}, Upper Stage LF={self.upper_stage_LF}, Payload LF={self.payload_LF}, Abort Installed={self.is_abort_installed}"

if __name__ == "__main__":
    # Assuming 'vessel' is a predefined object from the kRPC library representing the spacecraft.
    vessel = None  # Placeholder for the actual vessel object

    # Create a standard configuration and customize it further.
    lv = LaunchVehicle.default(vessel).set_abort_installed().set_abort_criteria(10).set_upper_stage_LF(1500)

    print(lv)

