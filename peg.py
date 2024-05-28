import numpy as np
import krpc

def compute_launch_parameters(apoapsis_target, periapsis_target, inclination_target, lan_target, slip_offset):
    """Compute essential launch parameters for a desired orbit."""
    
    # Constants and Reference Frames
    orbital_reference = vessel.orbit.body.non_rotating_reference_frame
    local_axes = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}
    
    # Adjust target apoapsis and periapsis based on the celestial body's radius
    apoapsis_target = (apoapsis_target * 1000) + vessel.orbit.body.equatorial_radius
    periapsis_target = (periapsis_target * 1000) + vessel.orbit.body.equatorial_radius
    
    # Compute orbital parameters
    semi_major = (apoapsis_target + periapsis_target) / 2
    velocity_at_periapsis = np.sqrt((mu * apoapsis_target) / (periapsis_target * semi_major))
    
    # Calculate launch azimuth
    current_latitude = vessel.flight().latitude
    if np.abs(inclination_target) < current_latitude:
        azimuth = 90
    elif inclination_target == 0: 
        azimuth = 90
    else:
        beta = np.degrees(np.arcsin(np.cos(np.radians(inclination_target)) / np.cos(np.radians(current_latitude))))
        
        if inclination_target < 0:
            beta = 180 - beta if beta <= 90 else 540 - beta
        
        rotational_velocity = vessel.orbit.body.rotational_speed * vessel.orbit.body.equatorial_radius
        velocity_components = {
            "x": velocity_at_periapsis * np.sin(np.radians(beta)) - rotational_velocity * np.cos(np.radians(current_latitude)),
            "y": velocity_at_periapsis * np.cos(np.radians(beta))
        }
        
        azimuth = np.degrees(np.arctan2(velocity_components["x"], velocity_components["y"]))
    
    # Compute the time to launch based on target LAN
    relative_longitude = np.degrees(np.arcsin(np.tan(np.radians(current_latitude)) / np.tan(np.radians(inclination_target))))
    relative_longitude += 180 if inclination_target < 0 else 0
    prime_meridian_angle = np.degrees(np.arctan2(np.dot(local_axes["z"], vessel.orbit.body.msl_position(0, 0, orbital_reference)),
                                                 np.dot(local_axes["x"], vessel.orbit.body.msl_position(0, 0, orbital_reference))))
    prime_meridian_angle += 360.0 if prime_meridian_angle < 0.0 else 0.0
    geo_longitude = (lan_target + relative_longitude - prime_meridian_angle) % 360.0
    node_angle = (geo_longitude - vessel.flight().longitude + 360.0 + slip_offset) % 360.0
    optimal_launch_time = (node_angle / 360.0) * vessel.orbit.body.rotational_period
    
    return np.array([velocity_at_periapsis, periapsis_target, azimuth, optimal_launch_time])


conn = krpc.connect(name='test')
vessel = conn.space_center.active_vessel
body=vessel.orbit.body
reference_frame=body.non_rotating_reference_frame
mu = vessel.orbit.body.gravitational_parameter

targets = compute_launch_parameters(100,100,0.1,0,0)