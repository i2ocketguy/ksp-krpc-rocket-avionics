import os
import random
import time
from datetime import datetime, UTC
import threading

from prometheus_client import start_http_server, Gauge, Enum

from mission import Telemetry


class KSPTelemetry:
    """
    A class to handle publishing metrics periodically
    """

    def __init__(self):
        self.publish_rate_hz = int(os.environ.get('TELEM_PUBLISH_RATE', 0))

        # all the metrics which come from krpc stream and mission.Telemetry class
        self.surface_altitude = Gauge('surface_altitude', 'surface_altitude in meters')
        self.altitude = Gauge('altitude', 'altitude in meters')
        self.apoapsis = Gauge('apoapsis', 'apoapsis in meters')
        self.velocity = Gauge('velocity', 'velocity in meters per second')
        self.vertical_vel = Gauge('vertical_vel', 'vertical_vel in meters per second')
        self.horizontal_vel = Gauge('horizontal_vel', 'horizontal_vel in meters per second')
        self.periapsis = Gauge('periapsis', 'periapsis in meters')

        # metrics which come from control system calculations, not krpc streams
        self.gauge_metrics = {}
        self.enum_metrics = {}



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


    def register_gauge_metric(self, name: str, description: str):
        self.gauge_metrics[name] = Gauge(name, description)


    def publish_gauge_metric(self, name: str, state: str, console_out: bool = False):
        if console_out:
            print(
                f'{str(datetime.now(UTC).isoformat())} [{name}] {state}')

        if name not in self.enum_metrics:
            print("WARNING: skipping publish of unknown metric. remember to register it first with register_gauge_metric!")
            return
        try:
            self.gauge_metrics[name].set(state)
        except Exception as e:
            print(f"ERROR: could not publish gauge metric for reason: {str(e)}")


    def register_enum_metric(self, name: str, description: str, states: [str]):
        self.enum_metrics[name] = Enum(name, description, states=states)


    def publish_enum_metric(self, name:str, state: str, display_name: str = None, console_out: bool = True):
        if console_out:
            print(f'{str(datetime.now(UTC).isoformat())} [{name}] {state} {f"\n   {display_name}" if display_name else ""}')

        if name not in self.enum_metrics:
            print("WARNING: skipping publish of unknown enum. remember to register it first with register_enum_metric!")
            return
        try:
            self.enum_metrics[name].state(state)
        except Exception as e:
            print(f"ERROR: could not publish enum metric for reason: {str(e)}")

