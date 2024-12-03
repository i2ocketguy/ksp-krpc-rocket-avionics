import numpy as np
import threading
import time
import krpc
from typing import Tuple

class ProportionalNavigationGuidance:
    """Implements proportional navigation guidance for target interception"""
    
    def __init__(self, vessel, target, conn, update_rate: float = 10.0):
        self.vessel = vessel
        self.target = target
        self.conn = conn
        self.update_rate = update_rate
        
        # Threading control
        self.stop_event = threading.Event()
        self.guidance_thread = None
        
        # Guidance parameters
        self.mode = 0
        self._lock = threading.Lock()
        self._current_target_vector = np.zeros(3)

    def start(self):
        """Start the guidance calculation thread"""
        if self.guidance_thread is None or not self.guidance_thread.is_alive():
            self.stop_event.clear()
            self.guidance_thread = threading.Thread(
                target=self._guidance_loop,
                daemon=True
            )
            self.guidance_thread.start()

    def stop(self):
        """Stop the guidance calculation thread"""
        self.stop_event.set()
        if self.guidance_thread:
            self.guidance_thread.join(timeout=2.0)
            if self.guidance_thread.is_alive():
                print("Warning: Guidance thread did not terminate cleanly")

    def calculate_guidance(self, mode: int) -> np.ndarray:
        """Calculate guidance vector using proportional navigation"""
        # Get and transform target position
        target_position = self._get_transformed_target_position()
        
        # Calculate relative position and velocity
        relative_position = self._calculate_relative_position(target_position)
        relative_velocity = self._calculate_relative_velocity()
        
        # Calculate navigation parameters
        omega = self._calculate_omega(relative_position, relative_velocity)
        acceleration_vector = self._calculate_acceleration_vector(relative_velocity, omega)
        
        # Calculate final guidance vector based on mode
        return self._calculate_target_vector(acceleration_vector, relative_velocity, mode)

    def _get_transformed_target_position(self) -> np.ndarray:
        """Get and transform target position with altitude compensation"""
        target_position = np.asarray(self.target.position(
            self.target.orbit.body.reference_frame))
        
        # Transform to surface reference frame
        target_position = np.asarray(self.conn.space_center.transform_direction(
            target_position,
            self.target.orbit.body.reference_frame,
            self.target.surface_reference_frame
        ))
        
        # Apply altitude-based compensation
        altitude = self.vessel.flight(
            self.vessel.orbit.body.reference_frame).mean_altitude
        compensation = 100.0 * (1.0 - np.exp(-altitude/300))
        target_position[0] += compensation
        
        # Transform back to body reference frame
        return np.asarray(self.conn.space_center.transform_direction(
            tuple(target_position),
            self.target.surface_reference_frame,
            self.target.orbit.body.reference_frame
        ))

    def _calculate_relative_position(self, target_position: np.ndarray) -> np.ndarray:
        """Calculate relative position vector"""
        vessel_position = np.asarray(
            self.vessel.position(self.vessel.orbit.body.reference_frame))
        relative_pos = target_position - vessel_position
        
        return np.asarray(self.conn.space_center.transform_direction(
            tuple(relative_pos),
            self.target.orbit.body.reference_frame,
            self.vessel.reference_frame
        ))

    def _calculate_relative_velocity(self) -> np.ndarray:
        """Calculate relative velocity vector"""
        velocity = self.vessel.velocity(self.vessel.orbit.body.reference_frame)
        transformed_velocity = self.conn.space_center.transform_direction(
            velocity,
            self.vessel.orbit.body.reference_frame,
            self.vessel.reference_frame
        )
        return -np.asarray(transformed_velocity)

    def _calculate_omega(self, pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        """Calculate angular velocity vector"""
        return np.cross(pos, vel) / np.dot(pos, pos)

    def _calculate_acceleration_vector(self, vel: np.ndarray, omega: np.ndarray) -> np.ndarray:
        """Calculate commanded acceleration vector"""
        return 5.0 * np.cross(vel, omega)

    def _calculate_target_vector(self, acc_vec: np.ndarray, rel_vel: np.ndarray, mode: int) -> np.ndarray:
        """Calculate final target vector based on mode"""
        mode_multipliers = {
            0: -50.0,
            1: -25.0,
            2: 5.0,
            3: 0.0,  # Pure pursuit
            4: -5.0
        }
        
        multiplier = mode_multipliers.get(mode, 0.0)
        if mode == 3:
            return rel_vel
        return rel_vel + multiplier * acc_vec

    def _guidance_loop(self):
        """Main guidance loop running in separate thread"""
        try:
            while not self.stop_event.is_set():
                target_vector = self.calculate_guidance(self.mode)
                
                with self._lock:
                    self._current_target_vector = target_vector
                
                time.sleep(1.0 / self.update_rate)
        except Exception as e:
            print(f"Error in guidance loop: {e}")

    @property
    def current_target_vector(self) -> np.ndarray:
        """Thread-safe access to current target vector"""
        with self._lock:
            return np.copy(self._current_target_vector)

    def set_mode(self, new_mode: int):
        """Thread-safe mode update"""
        if 0 <= new_mode <= 4:
            self.mode = new_mode

    def vector_to_local_directions(self, vector: np.ndarray) -> Tuple[float, float, float]:
        """Convert a guidance vector in surface reference frame to flight directions.
        
        In surface reference frame:
        - y-axis: aligned with velocity vector
        - z-axis: in horizontal plane
        - x-axis: orthogonal to both
        
        Args:
            vector: 3D vector in surface reference frame [x, y, z]
            
        Returns:
            Tuple[float, float, float]: (flight_path_angle, heading_error, magnitude)
            - flight_path_angle: angle from horizontal plane (positive up) in degrees
            - heading_error: deviation from velocity vector in degrees
            - magnitude: magnitude of the guidance vector
        """
        # Get vector magnitude
        magnitude = np.linalg.norm(vector)
        if magnitude < 1e-6:
            return 0.0, 0.0, 0.0
        
        # Normalize the vector
        norm_vector = vector / magnitude
        
        # Flight path angle (angle from horizontal plane)
        # Using x and y components because y is velocity direction
        flight_path_angle = np.degrees(np.arctan2(norm_vector[0], norm_vector[1]))
        
        # Heading error (deviation from velocity vector in horizontal plane)
        # Using z component because it's in the horizontal plane
        heading_error = np.degrees(np.arctan2(norm_vector[2], norm_vector[1]))
        
        return flight_path_angle, heading_error, magnitude
    
    def get_target_range(self) -> float:
        """Calculate the current range to target in meters.
        
        Returns:
            float: Distance to target in meters
        """
        # Get positions in body reference frame
        vessel_pos = np.array(self.vessel.position(self.vessel.orbit.body.reference_frame))
        target_pos = np.array(self.target.position(self.target.orbit.body.reference_frame))
        
        # Calculate range vector and magnitude
        range_vector = target_pos - vessel_pos
        return np.linalg.norm(range_vector)
    
    def get_closing_velocity(self) -> float:
        """Calculate the closing velocity with target in m/s.
        
        Returns:
            float: Closing velocity in m/s (positive means closing with target)
        """
        # Get velocities in body reference frame
        vessel_vel = np.array(self.vessel.velocity(self.vessel.orbit.body.reference_frame))
        target_vel = np.array(self.target.velocity(self.vessel.orbit.body.reference_frame))
        
        # Calculate relative velocity
        rel_vel = target_vel - vessel_vel
        
        # Get positions for range vector
        vessel_pos = np.array(self.vessel.position(self.vessel.orbit.body.reference_frame))
        target_pos = np.array(self.target.position(self.vessel.orbit.body.reference_frame))
        range_vector = target_pos - vessel_pos
        
        # Project relative velocity onto range vector to get closing velocity
        range_unit = range_vector / np.linalg.norm(range_vector)
        return np.dot(rel_vel, range_unit)

    def print_guidance_info(self):
        """Print human-readable guidance information"""
        vector = self.current_target_vector
        fpa, heading_err, magnitude = self.vector_to_local_directions(vector)
        
        print(f"\nGuidance Information:")
        print(f"Flight Path Angle: {fpa:6.1f}° {'UP' if fpa > 0 else 'DOWN'}")
        print(f"Heading Error:    {heading_err:6.1f}° {'RIGHT' if heading_err > 0 else 'LEFT'}")
        print(f"Command Magnitude:{magnitude:6.1f} m/s")

def main():
    # Initialize connection and get vessel/target
    conn = krpc.connect(name='Proportional Navigation')
    vessel = conn.space_center.active_vessel
    target = conn.space_center.target_vessel  # Or however you get your target

    # Create guidance instance
    guidance = ProportionalNavigationGuidance(vessel, target, conn)

    try:
        # Start guidance thread
        guidance.start()

        while True:
            # Get current guidance vector
            target_vector = guidance.current_target_vector
            
            # Use target vector for control...
            
            # Update mode if needed
            guidance.set_mode(1)  # Example mode change
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        guidance.stop()

if __name__ == "__main__":
    main()