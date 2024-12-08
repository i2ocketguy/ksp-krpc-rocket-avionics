import random
import time
import threading

from prometheus_client import start_http_server, Gauge

from mission import Telemetry


class KSPTelemetry:
    """
    A class to represent and expose Kerbal Space Program telemetry metrics.
    """

    def __init__(self):
        # Define your metrics here
        self.altitude_gauge = Gauge('rocket_altitude_meters', 'Current altitude of the rocket in meters')
        self.velocity_gauge = Gauge('rocket_velocity_m_s', 'Current velocity of the rocket in meters per second')
        self.throttle_gauge = Gauge('rocket_throttle', 'Current throttle setting (0.0 to 1.0)')
        # You can add more metrics as needed, like fuel mass, stage number, pitch, yaw, roll, etc.


    def start_metrics_server(self, port: int = 8012):
        # todo some kind of lock or better error handling here if this class is instantiated > once
        start_http_server(port)

    def start_telem_publish(self, telemetry: Telemetry):
        updater_thread = threading.Thread(target=self.telem_publish_thread, args=(telemetry,), daemon=True)
        updater_thread.start()


    def telem_publish_thread(self, telemetry: Telemetry):
        while True:
            # todo real telem in next iteration
            # In a real application, these would come from the rocket's control system logic
            altitude = random.uniform(0, 100000)  # altitude in meters
            velocity = random.uniform(0, 2000)  # velocity in m/s
            throttle = random.uniform(0, 1)  # throttle setting

            self.update_metrics(altitude, velocity, throttle)

            time.sleep(1)

    def update_metrics(self, altitude: float, velocity: float, throttle: float):
            self.altitude_gauge.set(altitude)
            self.velocity_gauge.set(velocity)
            self.throttle_gauge.set(throttle)