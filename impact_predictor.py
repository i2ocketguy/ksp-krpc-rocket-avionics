from dataclasses import dataclass
from typing import Tuple
import numpy as np
import threading
import time
from queue import Queue
import krpc

@dataclass
class OrbitalElements:
    """Stores orbital elements for a spacecraft"""
    semi_major_axis: float
    eccentricity: float
    inclination: float
    arg_periapsis: float
    raan: float
    true_anomaly: float

class OrbitalCalculator:
    """Handles orbital calculations and coordinate transformations with threading"""
    
    def __init__(self, vessel, update_rate: float = 10.0):
        self.vessel = vessel
        self.body_mu = vessel.orbit.body.gravitational_parameter
        self.body_radius = vessel.orbit.body.equatorial_radius
        self.update_rate = update_rate  # Hz
        
        # Threading control
        self.stop_event = threading.Event()
        self.calculation_thread = None
        
        # Shared data storage with thread safety
        self._lock = threading.Lock()
        self._current_position = np.zeros(3)
        self._current_elements = None
        self._target_position = np.zeros(3)
        
        # Command queue for thread communication
        self.command_queue = Queue()

    @property
    def current_position(self) -> np.ndarray:
        """Thread-safe access to current position"""
        with self._lock:
            return np.copy(self._current_position)

    @property
    def target_position(self) -> np.ndarray:
        """Thread-safe access to target position"""
        with self._lock:
            return np.copy(self._target_position)

    def start(self):
        """Start the orbital calculation thread"""
        if self.calculation_thread is None or not self.calculation_thread.is_alive():
            self.stop_event.clear()
            self.calculation_thread = threading.Thread(
                target=self._calculation_loop,
                daemon=True  # Make thread daemon so it dies with main program
            )
            self.calculation_thread.start()

    def stop(self):
        """Stop the orbital calculation thread"""
        self.stop_event.set()
        if self.calculation_thread:
            self.calculation_thread.join(timeout=2.0)
            if self.calculation_thread.is_alive():
                print("Warning: Orbital calculation thread did not terminate cleanly")

    def _calculation_loop(self):
        """Main calculation loop running in separate thread"""
        try:
            while not self.stop_event.is_set():
                # Process any pending commands
                while not self.command_queue.empty():
                    cmd, args = self.command_queue.get_nowait()
                    self._handle_command(cmd, args)

                # Get current state vectors
                position = np.array(self.vessel.position(self.vessel.orbit.body.reference_frame))
                velocity = np.array(self.vessel.velocity(self.vessel.orbit.body.reference_frame))

                # Calculate orbital elements
                elements = self.calculate_orbital_elements(position, velocity)
                
                # Update shared data thread-safely
                with self._lock:
                    self._current_position = position
                    self._current_elements = elements
                    
                # Calculate target position if needed
                if np.any(self._target_position):
                    new_position = self.elements_to_state(elements)
                    with self._lock:
                        self._target_position = new_position

                # Sleep for update rate
                time.sleep(1.0 / self.update_rate)

        except Exception as e:
            print(f"Error in orbital calculation thread: {e}")
        finally:
            self.cleanup()

    def _handle_command(self, cmd: str, args: tuple):
        """Handle commands received from main thread"""
        if cmd == "set_target":
            lat, lon = args
            # Convert lat/lon to position vector and store
            # Implementation depends on your coordinate system
            pass
        elif cmd == "update_rate":
            self.update_rate = args[0]

    def cleanup(self):
        """Clean up resources"""
        with self._lock:
            self._current_position = np.zeros(3)
            self._current_elements = None
            self._target_position = np.zeros(3)

    @staticmethod
    def normalize_vector(vector: np.ndarray) -> np.ndarray:
        """Normalize a vector to unit length"""
        return vector / np.linalg.norm(vector)

    @staticmethod
    def rad_to_deg(rad: float) -> float:
        """Convert radians to degrees"""
        return np.degrees(rad)

    @staticmethod
    def deg_to_rad(deg: float) -> float:
        """Convert degrees to radians"""
        return np.radians(deg)

    def haversine_between_coords(self, coord1: Tuple[float, float], 
                                coord2: Tuple[float, float]) -> float:
        """
        Calculate great circle distance between two lat/lon coordinates
        using the haversine formula
        """
        lat1, lon1 = self.deg_to_rad(coord1[0]), self.deg_to_rad(coord1[1])
        lat2, lon2 = self.deg_to_rad(coord2[0]), self.deg_to_rad(coord2[1])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = (np.sin(dlat/2)**2 + 
             np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2)
        c = 2 * np.arcsin(np.clip(np.sqrt(a), 0, 1))
        
        return self.body_radius * c

    def state_to_latlon(self, state_vector: np.ndarray) -> Tuple[float, float]:
        """Convert cartesian state vector to latitude and longitude"""
        lon = np.arctan2(state_vector[1], state_vector[0])
        projection = np.sqrt(state_vector[0]**2 + state_vector[1]**2)
        lat = np.arctan2(state_vector[2], projection)
        
        return (self.rad_to_deg(lat), self.rad_to_deg(lon))

    def calculate_orbital_elements(self, position: np.ndarray, 
                                 velocity: np.ndarray) -> OrbitalElements:
        """Calculate orbital elements from state vectors"""
        # Rearrange vectors for calculation
        r = np.array([position[0], position[2], position[1]])
        v = np.array([velocity[0], velocity[2], velocity[1]])
        
        # Calculate angular momentum
        h = np.cross(r, v)
        h_mag = np.linalg.norm(h)
        
        # Calculate eccentricity vector
        e = (np.cross(v, h) / self.body_mu - 
             r / np.linalg.norm(r))
        ecc = np.linalg.norm(e)
        
        # Calculate node vector
        n = np.array([-h[1], h[0], 0])
        n_mag = np.linalg.norm(n)
        
        # Calculate true anomaly
        nu = (np.arccos(np.dot(e, r)/(ecc * np.linalg.norm(r)))
              if np.dot(r, v) >= 0 
              else 2*np.pi - np.arccos(np.dot(e, r)/(ecc * np.linalg.norm(r))))
        
        # Calculate inclination
        inc = np.arccos(h[2]/h_mag)
        
        # Calculate RAAN
        raan = (np.arccos(n[0]/n_mag) 
               if n[1] >= 0 
               else 2*np.pi - np.arccos(n[0]/n_mag))
        
        # Calculate argument of periapsis
        argp = (np.arccos(np.dot(n, e)/(n_mag * ecc))
                if e[2] >= 0
                else 2*np.pi - np.arccos(np.dot(n, e)/(n_mag * ecc)))
        
        # Calculate semi-major axis
        sma = 1.0 / (2.0/np.linalg.norm(r) - np.linalg.norm(v)**2/self.body_mu)
        
        return OrbitalElements(sma, ecc, inc, argp, raan, nu)

    def elements_to_state(self, elements: OrbitalElements) -> np.ndarray:
        """Convert orbital elements back to state vector"""
        # Calculate new true anomaly at target radius
        new_nu = -np.arccos(
            ((1.0 - elements.eccentricity**2) * 
             (elements.semi_major_axis/self.body_radius) - 1.0) / 
            elements.eccentricity
        )
        
        # Calculate position components
        angle_sum = elements.arg_periapsis + new_nu
        cos_raan = np.cos(elements.raan)
        sin_raan = np.sin(elements.raan)
        cos_sum = np.cos(angle_sum)
        sin_sum = np.sin(angle_sum)
        cos_inc = np.cos(elements.inclination)
        
        x = self.body_radius * (cos_raan * cos_sum - 
                               sin_raan * sin_sum * cos_inc)
        y = self.body_radius * (sin_raan * cos_sum + 
                               cos_raan * sin_sum * cos_inc)
        z = self.body_radius * np.sin(elements.inclination) * sin_sum
        
        return np.array([x, y, z])

    def set_target(self, lat: float, lon: float):
        """Thread-safe method to set new target coordinates"""
        self.command_queue.put(("set_target", (lat, lon)))

    def update_calculation_rate(self, new_rate: float):
        """Thread-safe method to update calculation rate"""
        self.command_queue.put(("update_rate", (new_rate,)))

def main():
    # Initialize connection and vessel
    conn = krpc.connect(name='Orbital Calculator')
    vessel = conn.space_center.active_vessel

    # Create calculator instance
    calculator = OrbitalCalculator(vessel, update_rate=10.0)

    try:
        # Start calculation thread
        calculator.start()

        # Example usage in main thread
        while True:
            # Get current position (thread-safe)
            current_pos = calculator.current_position
            
            # Set new target if needed
            calculator.set_target(lat=0.0, lon=45.0)
            
            # Get target position (thread-safe)
            target_pos = calculator.target_position
            
            print(current_pos + " " + target_pos)
            
            time.sleep(0.1)  # Main loop rate

    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        # Clean shutdown
        calculator.stop()

if __name__ == "__main__":
    main()