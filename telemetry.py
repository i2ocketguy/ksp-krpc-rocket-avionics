import os
import time
from datetime import datetime, UTC
import threading
from queue import Queue

from prometheus_client import start_http_server, Gauge, Enum, Counter, Histogram
import psutil

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

        # host metrics
        self.system_cpu_percent = Gauge('system_cpu_percent', 'total host CPU usage in percent')

        # metrics which come from control system calculations, not krpc streams
        self.gauge_metrics = {}
        self.counter_metrics = {}
        self.histogram_metrics = {}
        self.enum_metrics = {}

        self.gnc_debug = os.environ.get('GNC_DEBUG', None) is not None



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

            self.system_cpu_percent.set(psutil.cpu_percent())

            time.sleep(1/self.publish_rate_hz)

    # -------------------------------------------------------------
    # Worker Thread Function
    # -------------------------------------------------------------
    def metric_publisher(self, queue: Queue):
        """
        Continuously consume metrics from the queue and update Prometheus metrics.
        """
        while True:
            # Block until an item is available
            metric_data = queue.get()
            if metric_data is None:
                # This can be a signal to gracefully stop if you ever need it
                break

            name = metric_data["name"]
            type = metric_data["type"]
            value = metric_data["value"]

            if type == "gauge":
                self.gauge_metrics[name].set(value)
            else:
                print(f"warning: unsupported metric type {type}")

            # Mark this task as done, helpful if you're using join() on the queue
            queue.task_done()


    def register_gauge_metric(self, name: str, description: str):
        self.gauge_metrics[name] = Gauge(name, description)


    def publish_gauge_metric(self, name: str, state: float, console_out: bool = False):
        if console_out:
            print(
                f'{str(datetime.now(UTC).isoformat())} [{name}] {state}')

        if name not in self.gauge_metrics:
            print(f"WARNING: skipping publish of unknown metric '{name}'. remember to register it first with register_gauge_metric!")
            return
        try:
            self.gauge_metrics[name].set(state)
        except Exception as e:
            print(f"ERROR: could not publish gauge metric for reason: {str(e)}")


    def register_counter_metric(self, name: str, description: str):
        self.counter_metrics[name] = Counter(name, description)


    def increment_counter_metric(self, name: str):
        if name not in self.counter_metrics:
            print(f"WARNING: skipping publish of unknown counter metric '{name}'. remember to register it first with register_counter_metric!")
            return
        try:
            self.counter_metrics[name].inc()
        except Exception as e:
            print(f"ERROR: could not publish counter metric for reason: {str(e)}")

    def register_histogram_metric(self, name: str, description: str):
        custom_buckets = (.001, .005, .01, .02, .025, .033, .05, .075, .1, .25, .5, .75, 1.0, 2.5, 5.0, 7.5, 10.0, float("inf"))
        self.histogram_metrics[name] = Histogram(name, description, buckets=custom_buckets)


    def get_histogram_metric(self, name: str):
        if name not in self.histogram_metrics:
            print(f"WARNING: skipping publish of unknown histogram metric '{name}'. remember to register it first with register_histogram_metric!")
            return
        try:
            return self.histogram_metrics[name]
        except Exception as e:
            print(f"ERROR: could not publish histogram metric for reason: {str(e)}")


    def register_enum_metric(self, name: str, description: str, states: [str]):
        self.enum_metrics[name] = Enum(name, description, states=states)


    def publish_enum_metric(self, name:str, state: str, display_name: str = None, console_out: bool = True):

        if name not in self.enum_metrics:
            print(f"WARNING: skipping publish of unknown enum '{name}'. remember to register it first with register_enum_metric!")
            return
        try:
            self.enum_metrics[name].state(state)
        except Exception as e:
            print(f"ERROR: could not publish enum metric for reason: {str(e)}")

