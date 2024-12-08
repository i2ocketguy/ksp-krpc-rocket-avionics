import os
import random
import time
import threading

from prometheus_client import start_http_server, Gauge

from mission import Telemetry


class KSPTelemetry:
    """
    A class to handle publishing metrics periodically
    """

    def __init__(self):
        self.publish_rate_hz = int(os.environ.get('TELEM_PUBLISH_RATE', 0))

        self.surface_altitude = Gauge('surface_altitude', 'surface_altitude in meters')
        self.altitude = Gauge('altitude', 'altitude in meters')
        self.apoapsis = Gauge('apoapsis', 'apoapsis in meters')
        self.velocity = Gauge('velocity', 'velocity in meters per second')
        self.vertical_vel = Gauge('vertical_vel', 'vertical_vel in meters per second')
        self.horizontal_vel = Gauge('horizontal_vel', 'horizontal_vel in meters per second')
        self.periapsis = Gauge('periapsis', 'periapsis in meters')



    def start_metrics_server(self, port: int = 8012):
        # todo some kind of lock or better error handling here if this class is instantiated > once
        start_http_server(port)


    def start_telem_publish(self, telemetry: Telemetry) -> bool:
        if self.publish_rate_hz == 0:
            print('not starting telemetry publish thread')
            return False

        updater_thread = threading.Thread(target=self._telem_publish_thread, args=(telemetry,), daemon=True)
        updater_thread.start()
        return True


    def _telem_publish_thread(self, telemetry: Telemetry):
        while True:
            self.surface_altitude.set(telemetry.surface_altitude())
            self.altitude.set(telemetry.altitude())
            self.apoapsis.set(telemetry.apoapsis())
            self.velocity.set(telemetry.velocity())
            self.vertical_vel.set(telemetry.vertical_vel())
            self.horizontal_vel.set(telemetry.horizontal_vel())
            self.periapsis.set(telemetry.periapsis())

            time.sleep(1/self.publish_rate_hz)
