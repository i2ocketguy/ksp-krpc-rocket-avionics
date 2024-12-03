import krpc
import time
import numpy as np
import threading
from dataclasses import dataclass
from typing import Tuple, List, Optional, Literal
from pid import PID
import launch_utils as utils
import mission
import spacecraft as sc
import numpy as np

class ReferenceMode:
    """Defines available reference frame modes"""
    SURFACE = 1  # Surface velocity reference frame
    INERTIAL = 2  # Inertial reference frame

@dataclass
class SteeringConfig:
    """Configuration parameters for steering control"""
    rot_kp: float = 1.0
    rot_ki: float = 0.1
    rot_kd: float = 7.0
    max_angle: float = 10.0 * np.pi / 180.0  # 10 degrees in radians
    
    # Torque control parameters
    tor_tu: float = 8.0
    tor_ku: float = 5.0
    
    # Reference vectors
    FORE: Tuple[int, int, int] = (0, 1, 0)
    TOP: Tuple[int, int, int] = (0, 0, -1)
    STAR: Tuple[int, int, int] = (1, 0, 0)

class FlightController:
    def __init__(self):
        """Initialize the vessel controller with KRPC connection and default parameters"""
        # Initialize KRPC connection
        self.conn = krpc.connect(name='Launch Controller')
        self.vessel = self.conn.space_center.active_vessel
        
        # Set up telemetry streams
        self.altitude = self.conn.add_stream(getattr, 
            self.vessel.flight(self.vessel.orbit.body.reference_frame), 
            'surface_altitude')
        self.vertical_vel = self.conn.add_stream(getattr, 
            self.vessel.flight(self.vessel.orbit.body.reference_frame), 
            'vertical_speed')
        
        # Initialize steering parameters
        self.config = SteeringConfig()
        self.seek_dir = np.array([0.0, np.pi * 0.5, 0.0])
        self.current_dir = np.zeros(3)
        self.current_rot_vel = np.zeros(3)
        
        # Initialize PID controllers
        self.setup_pid_controllers()

    def setup_pid_controllers(self):
        """Initialize rotation and torque PID controllers"""
        min_val = -self.config.max_angle
        max_val = self.config.max_angle
        
        # Rotation PID controllers
        self.rot_pitch = PID(0.0, self.config.rot_kp, self.config.rot_ki, self.config.rot_kd, min_val, max_val)
        self.rot_heading = PID(0.0, self.config.rot_kp, self.config.rot_ki, self.config.rot_kd, min_val, max_val)
        self.rot_roll = PID(0.0, self.config.rot_kp, self.config.rot_ki, self.config.rot_kd, min_val, max_val)

    def get_reference_frame(self, mode: int):
        """Get the appropriate reference frame based on mode"""
        if mode == ReferenceMode.SURFACE:
            return self.vessel.surface_velocity_reference_frame
        else:
            return self.vessel.surface_reference_frame

    def get_torque_parameters(self, mode: int) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate torque parameters based on mode"""
        tor_kp = np.array([0.45 * self.config.tor_ku] * 3)
        
        if mode == ReferenceMode.SURFACE:
            tor_kp[2] *= 0.25  # Standard surface mode
        else:
            tor_kp[2] *= 0.25 * 40  # Increased roll control for inertial mode
            
        tor_kd = tor_kp * 10.0
        return tor_kp, tor_kd

    def calculate_steering_commands(self, seek_dir1: np.ndarray, mode: int) -> Tuple[float, float, float]:
        """Calculate steering commands based on desired direction and mode"""
        # Transform desired direction based on mode
        ref_frame = self.get_reference_frame(mode)
        seek_dir1 = self.conn.space_center.transform_direction(
            seek_dir1, 
            ref_frame,
            self.vessel.surface_reference_frame
        )
        
        # Get torque parameters for current mode
        tor_kp, tor_kd = self.get_torque_parameters(mode)
        
        # Calculate direction angles
        self.calculate_direction_angles(seek_dir1)
                
        # Update current rotational velocity
        self.update_rotation_velocity()
        
        # Calculate desired rotational velocities
        out_pitch_vel = self.rot_pitch.update(-self.current_dir[0])
        out_heading_vel = self.rot_heading.update(-self.current_dir[1])
        out_roll_vel = 0.0
        
        # Create torque PID controllers
        torque_controllers = [
            PID(vel, kp, 0.0, kd, -4.0, 4.0)
            for kp, kd, vel in zip(tor_kp, tor_kd, 
                [out_pitch_vel, out_heading_vel, out_roll_vel])
        ]
        
        # Calculate final commands
        commands = [
            controller.update(rot_vel)
            for controller, rot_vel in zip(torque_controllers, self.current_rot_vel)
        ]
        
        return tuple(commands)

    def calculate_direction_angles(self, seek_dir1: np.ndarray):
        """Calculate current direction angles from a desired direction vector.
        
        Args:
            seek_dir1: Normalized direction vector in vessel's reference frame
            
        Updates self.current_dir with [pitch, heading, roll] in radians
        """
        # Normalize input vector
        seek_dir1 = seek_dir1 / np.linalg.norm(seek_dir1)
        
        # Safely calculate pitch angle (rotation around X-axis)
        x_component = np.clip(seek_dir1[0], -1.0, 1.0)  # Ensure value is in [-1, 1]
        self.seek_dir[0] = 0.5 * np.pi - np.arccos(x_component)
        
        # Calculate heading angle (rotation around Y-axis)
        self.seek_dir[1] = np.arctan2(seek_dir1[2], seek_dir1[1])
        if self.seek_dir[1] < 0.0:
            self.seek_dir[1] += 2.0 * np.pi
            
        # Initialize roll angle
        self.seek_dir[2] = 0.0
        
        # Calculate top vector for roll reference
        seek_dir1_top = np.array([0.0, 0.0, 0.0])
        seek_dir1_top[0] = np.cos(-self.seek_dir[0])
        
        if self.seek_dir[0] <= 0.0:
            seek_dir1_top[1] = np.sin(self.seek_dir[0]) * np.cos(self.seek_dir[1])
            seek_dir1_top[2] = np.sin(self.seek_dir[0]) * np.sin(self.seek_dir[1])
        else:
            seek_dir1_top[1] = -np.sin(self.seek_dir[0]) * np.cos(self.seek_dir[1])
            seek_dir1_top[2] = -np.sin(self.seek_dir[0]) * np.sin(self.seek_dir[1])
        
        # Transform vectors to vessel reference frame
        seek_dir1 = self.conn.space_center.transform_direction(
            seek_dir1, 
            self.vessel.surface_reference_frame,
            self.vessel.reference_frame
        )
        seek_dir1_top = self.conn.space_center.transform_direction(
            seek_dir1_top,
            self.vessel.surface_reference_frame,
            self.vessel.reference_frame
        )
        
        # Calculate current direction angles using vector operations
        current_dir_mag = self.vector_angle(self.config.FORE, seek_dir1)
        
        # Pitch calculation
        self.current_dir[0] = self.vector_angle(
            self.config.FORE,
            self.vector_exclude(self.config.STAR, seek_dir1)
        ) * 180.0 / np.pi
        
        if self.vector_angle(self.config.TOP, 
                            self.vector_exclude(self.config.STAR, seek_dir1)) * 180.0 / np.pi > 90:
            self.current_dir[0] *= -1.0
        
        # Heading calculation
        self.current_dir[1] = self.vector_angle(
            self.config.FORE,
            self.vector_exclude(self.config.TOP, seek_dir1)
        ) * 180.0 / np.pi
        
        if self.vector_angle(self.config.STAR,
                            self.vector_exclude(self.config.TOP, seek_dir1)) * 180.0 / np.pi > 90:
            self.current_dir[1] *= -1.0
        
        # Roll calculation
        self.current_dir[2] = self.vector_angle(
            self.config.TOP,
            self.vector_exclude(self.config.FORE, seek_dir1_top)
        ) * 180.0 / np.pi
        
        if self.vector_angle(self.config.STAR,
                            self.vector_exclude(self.config.FORE, seek_dir1_top)) * 180.0 / np.pi > 90.0:
            self.current_dir[2] *= -1.0
        
        # Convert angles to radians
        self.current_dir = np.multiply(self.current_dir, np.pi/180.0)


    def update_rotation_velocity(self):
        """Update current rotation velocity"""
        self.current_rot_vel = self.vessel.angular_velocity(self.vessel.surface_reference_frame)
        self.current_rot_vel = self.conn.space_center.transform_direction(
            self.current_rot_vel,
            self.vessel.surface_reference_frame,
            self.vessel.reference_frame
        )
        self.current_rot_vel = np.asarray(self.current_rot_vel)
        
        # Swap Y and Z components
        self.current_rot_vel[1], self.current_rot_vel[2] = (
            self.current_rot_vel[2],
            self.current_rot_vel[1]
        )
        self.current_rot_vel *= -1

    def control_loop(self, in_q: List[bool], in_e: List[np.ndarray], in_mode: List[int]):
        """Main control loop for vessel steering"""
        while True:
            if not in_q[0]:
                self.disable_control()
                if self.should_exit():
                    break
                time.sleep(0.01)
                continue
                
            self.vessel.control.sas = False
            pitch, yaw, roll = self.calculate_steering_commands(in_e[0], in_mode[0])
            
            # Apply controls
            self.vessel.control.pitch = pitch
            self.vessel.control.yaw = yaw
            self.vessel.control.roll = roll
            
            time.sleep(0.01)

    def disable_control(self):
        """Disable all controls and enable SAS"""
        self.vessel.control.pitch = 0.0
        self.vessel.control.yaw = 0.0
        self.vessel.control.roll = 0.0
        self.vessel.control.sas = True

    def should_exit(self) -> bool:
        """Check if control loop should exit"""
        sign = np.sign(self.vertical_vel())
        h = self.get_vessel_height()
        return (sign * h > 0 and 
                self.vessel.flight(self.vessel.orbit.body.reference_frame).mean_altitude < 70000)

    def get_vessel_height(self) -> float:
        """Calculate vessel's current height"""
        return (self.altitude() + 
                self.vessel.parts.with_name(self.vessel.parts.engines[0].part.name)[0]
                .position(self.vessel.reference_frame)[1])
    
    @staticmethod
    def vector_angle(v1: np.ndarray, v2: np.ndarray) -> float:
        """Calculate angle between two vectors in radians.
        
        Args:
            v1: First vector
            v2: Second vector
            
        Returns:
            float: Angle between vectors in radians
        """
        dot_product = np.vdot(v1, v2)
        # Handle numerical precision issues
        if dot_product > 1.0:
            dot_product = 1.0
        elif dot_product < -1.0:
            dot_product = -1.0
        return np.arccos(dot_product)

    @staticmethod
    def vector_exclude(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        """Project v2 onto plane perpendicular to v1.
        
        Args:
            v1: Vector defining normal of exclusion plane
            v2: Vector to be projected
            
        Returns:
            np.ndarray: Projected vector
        """
        # Calculate cross product and normalize
        cross = np.cross(v1, v2)
        cross_mag = np.linalg.norm(cross)
        if cross_mag < 1e-10:  # Handle near-parallel vectors
            # Return a perpendicular vector in this case
            if abs(v1[0]) < abs(v1[1]):
                return np.array([1.0, 0.0, 0.0])
            else:
                return np.array([0.0, 1.0, 0.0])
            
        cross = cross / cross_mag
        
        # Calculate excluded vector
        n = np.cross(cross, v1)
        n_mag = np.linalg.norm(n)
        if n_mag < 1e-10:  # Handle numerical issues
            return cross
        return n / n_mag

    @staticmethod
    def r2d(x: float) -> float:
        """Convert radians to degrees.
        
        Args:
            x: Angle in radians
            
        Returns:
            float: Angle in degrees
        """
        return x * 180.0 / np.pi

    @staticmethod
    def d2r(x: float) -> float:
        """Convert degrees to radians.
        
        Args:
            x: Angle in degrees
            
        Returns:
            float: Angle in radians
        """
        return x * np.pi / 180.0

    @staticmethod
    def normalize_vector(v: np.ndarray) -> np.ndarray:
        """Normalize a vector to unit length.
        
        Args:
            v: Input vector
            
        Returns:
            np.ndarray: Normalized vector
        """
        mag = np.linalg.norm(v)
        if mag < 1e-10:  # Handle zero vector
            return np.zeros_like(v)
        return v / mag

    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp a value between min and max.
        
        Args:
            value: Value to clamp
            min_val: Minimum allowed value
            max_val: Maximum allowed value
            
        Returns:
            float: Clamped value
        """
        return max(min_val, min(value, max_val))

    def compute_error_vector(self, current: np.ndarray, desired: np.ndarray) -> np.ndarray:
        """Compute error vector between current and desired directions.
        
        Args:
            current: Current direction vector
            desired: Desired direction vector
            
        Returns:
            np.ndarray: Error vector
        """
        current_norm = self.normalize_vector(current)
        desired_norm = self.normalize_vector(desired)
        
        # Calculate cross product for rotation axis
        rotation_axis = np.cross(current_norm, desired_norm)
        rotation_angle = self.vector_angle(current_norm, desired_norm)
        
        # Return scaled error vector
        if np.linalg.norm(rotation_axis) < 1e-10:
            return np.zeros(3)
        return rotation_axis * rotation_angle

    def transform_vector(self, vector: np.ndarray, from_frame, to_frame) -> np.ndarray:
        """Transform a vector between reference frames.
        
        Args:
            vector: Vector to transform
            from_frame: Source reference frame
            to_frame: Target reference frame
            
        Returns:
            np.ndarray: Transformed vector
        """
        transformed = self.conn.space_center.transform_direction(
            tuple(vector),
            from_frame,
            to_frame
        )
        return np.array(transformed)

    def get_vessel_direction(self) -> np.ndarray:
        """Get current vessel forward direction vector.
        
        Returns:
            np.ndarray: Vessel's forward direction vector
        """
        return np.array(self.vessel.direction(self.vessel.surface_reference_frame))


def launch_sequence():
    # constants
    CLOCK_RATE = 50.0  # refresh rate [Hz]
    TELEM_RATE = 1.0  # refresh rate for telemetry aquistion [Hz]
    root_vessel = "DCX"

    upper_stage_LF = 0
    payload_LF = 0

    meco_condition_multiplier = 0.06  # 0 to ignore condition, otherwise set to desired
    #    1st stage liquid fuel percentage at MECO
    v_stage = 100000  # velocity target for 45 degree pitch over

    conn, vessel = utils.initialize()
    mission_params = mission.MissionParameters(root_vessel,
                                            state="init",
                                            target_inc=0,
                                            target_roll=180,
                                            altimeter_bias=71,
                                            grav_turn_end=85000,
                                            max_q=16000,
                                            max_g=3.0)
    dcx = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                            mission_params.altimeter_bias, v_stage, upper_stage_LF,
                            payload_LF, meco_condition_multiplier)
    mission_params.target_heading = utils.set_azimuth(vessel,
                                                    mission_params.target_inc,
                                                    dcx.bref)
    # Initialize controller
    controller = FlightController()

    # Create shared variables for control
    control_active = [True]
    desired_direction = [np.array([0.0, 1.0, 0.0])]  # Initial vertical direction
    control_mode = [ReferenceMode.SURFACE]  # Start in surface mode

    # Start control thread
    control_thread = threading.Thread(
        target=controller.control_loop,
        args=(control_active, desired_direction, control_mode)
    )
    control_thread.start()

    try:
        # Example of changing modes during flight
        
        # Initial vertical ascent in surface mode
        print(f"First target direction: {desired_direction[0]}")
        time.sleep(10)
        
        # Switch to gravity turn in surface mode
        desired_direction[0] = np.array([0.05, 0.95, 0.0])  # Slight pitch over
        print(f"Second target direction: {desired_direction[0]}")
        time.sleep(10)
        
        # Switch to inertial mode for orbital insertion
        control_mode[0] = ReferenceMode.SURFACE
        desired_direction[0] = np.array([0.5, 1.0, 0.0])  # More aggressive pitch
        print(f"Third target direction: {desired_direction[0]}")
        time.sleep(10)
        
    finally:
        # Cleanup
        control_active[0] = False
        control_thread.join()

# Run the launch sequence
if __name__ == "__main__":
    launch_sequence()