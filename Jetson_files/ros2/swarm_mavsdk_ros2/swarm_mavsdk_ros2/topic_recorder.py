# Record discovered ROS topics and local NED positions in a unique run directory

import csv
import math
import warnings
import json
import threading
import re
from datetime import datetime, timezone
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def flatten_fields(prefix, value):
    """Expose every nested scalar, including array elements, as a CSV column."""
    if isinstance(value, dict):
        for key, item in value.items():
            yield from flatten_fields(f"{prefix}.{key}", item)
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            yield from flatten_fields(f"{prefix}.{index}", item)
    else:
        yield prefix, value


class BoundedOutput:
    """Enforce byte budgets before writing complete UTF-8 records."""

    def __init__(self, handle, run, budget):
        self.handle, self.run, self.budget = handle, run, budget

    def write(self, text):
        size = len(text.encode('utf-8'))
        with self.run.lock:
            if size > self.run.budgets[self.budget]:
                self.run.budgets[self.budget] = 0
                self.run.dropped += 1
                if self.budget not in self.run.exhausted:
                    self.run.exhausted.add(self.budget)
                    warnings.warn(f'Recorder {self.budget} size limit reached; further records in this budget are dropped. Control continues.', RuntimeWarning)
                    self.run.write_summary()
                return 0
            self.run.budgets[self.budget] -= size
            return self.handle.write(text)

    def __getattr__(self, name):
        return getattr(self.handle, name)


class CsvRun:
    # Write complete messages as JSON cells and positions as numeric columns

    def __init__(self, output_dir, vehicle_namespaces=None, *, record_hz=10.0,
                 detailed_timeline=False, max_run_bytes=100_000_000):
        if not math.isfinite(record_hz) or record_hz <= 0:
            raise ValueError("record_hz must be finite and positive")
        if max_run_bytes < 16384:
            raise ValueError("max_run_bytes must be at least 16384")
        self.period_ns = max(1, int(1e9 / record_hz))
        self.detailed_timeline = detailed_timeline
        self.max_run_bytes = int(max_run_bytes)
        usable = self.max_run_bytes - 4096
        self.budgets = {"data": usable * 40 // 100, "events": usable * 10 // 100}
        self.export_bytes = usable * 25 // 100
        self.exhausted = set()
        self.dropped = 0
        self.last_topic_ns = {}
        self.transitions = {}
        root = Path(output_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.path = Path(tempfile.mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=root))
        self.files = []
        self.vehicle_namespaces = vehicle_namespaces
        self.latest = {}
        self.columns = set()
        self.lock = threading.RLock()
        self.export_lock = threading.Lock()
        self.timeline_path = None
        self.last_sim_ns = None
        self.spool_path = self.path / 'timeline.jsonl'
        self.spool = BoundedOutput(self.spool_path.open('w+', encoding='utf-8'), self, 'data')
        self.events = self._open('events.csv', ['received_unix_ns', 'kind', 'data_json'], 'events')
        self.write_summary()
        try:
            self.topics = self._open("topics.csv", ["received_unix_ns", "received_ros_ns", "topic", "type", "data_json"])
            self.positions = self._open("positions.csv", ["received_unix_ns", "received_ros_ns", "topic", "drone_id", "frame_id", "north_m", "east_m", "down_m"])
            self.inventory = self._open("inventory.csv", ["topic", "type", "status", "detail"], "events")
        except Exception:
            self.close()
            raise

    def _open(self, name, fields, budget="data"):
        handle = (self.path / name).open("w", newline="", encoding="utf-8")
        handle = BoundedOutput(handle, self, budget)
        self.files.append(handle)
        writer = csv.writer(handle)
        writer.writerow(fields)
        return writer

    def write_summary(self):
        # Reserved space remains available even after telemetry reaches its cap.
        summary = {"max_run_bytes": self.max_run_bytes, "exhausted_budgets": sorted(self.exhausted),
                   "dropped_records": self.dropped, "test": self.latest.get('test'),
                   "test_status": self.latest.get('test_status'), "detailed_timeline": self.detailed_timeline}
        text = json.dumps(summary, ensure_ascii=True)
        if len(text.encode('utf-8')) <= 4096:
            (self.path / 'summary.json').write_text(text, encoding='utf-8')

    def event(self, kind, data):
        self.events.writerow([time.time_ns(), kind, json.dumps(data, ensure_ascii=False)])
        self.files[0].flush()

    def record(self, topic, type_name, data, ros_ns):
        with self.lock:
            if topic.endswith('/test_event'):
                if self.transitions.get(topic) != data:
                    self.transitions[topic] = data
                    self.event('test_event', data)
            elif type_name == 'rcl_interfaces/msg/Log':
                self.event('ros_log', data)
            elif topic.endswith('/control_state'):
                state = json.loads(data['data'])
                state = {k: v for k, v in state.items() if k in (
                    'mode', 'architecture', 'started', 'armed', 'landing', 'takeoff',
                    'takeoff_complete', 'landing_complete', 'takeoff_pending',
                    'takeoff_climb_active', 'offboard_requested')}
                if self.transitions.get(topic) != state:
                    self.transitions[topic] = state
                    self.event(topic, state)
            elif type_name.startswith('px4_msgs/msg/VehicleStatus'):
                state = {k: data[k] for k in ('arming_state', 'nav_state', 'failsafe',
                         'pre_flight_checks_pass') if k in data}
                if self.transitions.get(topic) != state:
                    self.transitions[topic] = state
                    self.event(topic, state)
            if topic == "/clock" or self.latest.get("test_status") not in (None, "running"):
                return
            now = time.monotonic_ns()
            previous = self.last_topic_ns.get(topic)
            if previous is not None and now - previous < self.period_ns:
                return
            self.last_topic_ns[topic] = now
            parts = topic.strip("/")
            if self.vehicle_namespaces is not None:
                for namespace, drone_id in self.vehicle_namespaces.items():
                    prefix = namespace.strip('/') + '/fmu/' if namespace else 'fmu/'
                    if parts.startswith(prefix):
                        parts = f"drone_{drone_id}/px4/{parts[len(prefix):]}"
                        break
            else:
                match = re.match(r"(?:px4_(\d+)/)?fmu/(.*)", parts)
                if match:
                    parts = f"drone_{match.group(1) or '0'}/px4/{match.group(2)}"
            column = parts.replace("/", ".")
            self.latest[column] = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
            self.latest[column + ".received_ros_ns"] = ros_ns
            self.columns.update((column, column + ".received_ros_ns"))
            # Clear old indexed values when a variable-length array shrinks.
            for key in list(self.latest):
                if key.startswith(column + ".") and key != column + ".received_ros_ns":
                    del self.latest[key]
            fields = data
            if topic.endswith("/control_state"):
                fields = json.loads(data["data"])
            for key, value in flatten_fields(column, fields):
                self.latest[key] = value
                self.columns.add(key)
            if type_name == "nav_msgs/msg/Odometry":
                drone = parts.split("/")[0]
                position = data["pose"]["pose"]["position"]
                for axis, name in zip(("x", "y", "z"), ("north", "east", "down")):
                    key = f"{drone}.positions_ned.{name}"
                    self.latest[key] = position[axis]
                    self.columns.add(key)
                velocity = data.get("twist", {}).get("twist", {}).get("linear", {})
                for axis, value in velocity.items():
                    key = f"{drone}.velocity_ned.{axis}"
                    self.latest[key] = value
                    self.columns.add(key)
        stamp = [time.time_ns(), ros_ns, topic]
        self.topics.writerow([*stamp, type_name, json.dumps(data, ensure_ascii=False, separators=(",", ":"))])
        frame = data.get("header", {}).get("frame_id", "")
        if type_name == "nav_msgs/msg/Odometry":
            parts = topic.strip("/").split("/")
            drone_id = next((p[6:] for p in parts if p.startswith("drone_") and p[6:].isdigit()), "")
            poses = [(drone_id, data["pose"]["pose"])]
        elif type_name == "geometry_msgs/msg/PoseArray" and topic.endswith(("/positions_ned", "/peer_states_ned")):
            poses = enumerate(data["poses"])
        else:
            poses = []
        for drone_id, pose in poses:
            p = pose["position"]
            self.positions.writerow([*stamp, drone_id, frame, p["x"], p["y"], p["z"]])

    def begin_test(self, test, status="running"):
        with self.lock:
            if (self.latest.get("test"), self.latest.get("test_status")) == (test, status):
                return
            self.event("test_transition", {"test": test, "status": status})
            if self.detailed_timeline and self.timeline_path is None:
                timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
                self.timeline_path = self.path / (timestamp + ".csv")
            self.latest["test"] = test
            self.latest["test_status"] = status
            self.columns.update(("test", "test_status"))
            self.last_sim_ns = None
            self.write_summary()

    def physics_step(self, sim_ns):
        with self.lock:
            if (not self.detailed_timeline or self.latest.get("test_status") != "running"
                    or 'data' in self.exhausted
                    or self.timeline_path is None or (self.last_sim_ns is not None
                    and 0 <= sim_ns - self.last_sim_ns < self.period_ns)):
                return
            self.last_sim_ns = sim_ns
            row = {"simulation_time_ns": sim_ns, "received_unix_ns": time.time_ns(), **self.latest}
            self.spool.write(json.dumps(row) + "\n")

    def export_timeline(self):
        # Stream the spool into a wide CSV once discovery has established all
        # columns. Atomic replacement also allows inspecting an active run.
        with self.export_lock:
            with self.lock:
                if self.timeline_path is None:
                    return
                fields = ["simulation_time_ns", "received_unix_ns", *sorted(self.columns)]
                temporary = self.timeline_path.with_suffix(".csv.tmp")
                self.spool.flush()
                limit = self.spool.tell()
            self.budgets["export"] = self.export_bytes
            with temporary.open("w", newline="", encoding="utf-8") as raw:
                handle = BoundedOutput(raw, self, "export")
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                # Use a separate reader and a fixed snapshot boundary so native
                # clock callbacks can keep appending while disk export runs.
                with self.spool_path.open(encoding='utf-8') as source:
                    while source.tell() < limit:
                        writer.writerow(json.loads(source.readline()))
            temporary.replace(self.timeline_path)

    def flush(self):
        with self.lock:
            if not self.spool.closed:
                self.spool.flush()
        for handle in self.files:
            handle.flush()

    def close(self):
        if not self.spool.closed:
            self.export_timeline()
            self.spool.close()
        for handle in self.files:
            handle.close()
        self.write_summary()


class TopicRecorder(Node):
    # Discover new topics every second; keep disk I/O outside flight control

    def __init__(self):
        super().__init__("topic_recorder")
        self.declare_parameter("output_dir", "results/ros2")
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter('vehicle_config').value
        namespaces = None
        if config_path:
            from .vehicle_config import load_vehicles
            count = len(json.loads(Path(config_path).read_text())['vehicles'])
            vehicles = load_vehicles(config_path, count)
            namespaces = {v['px4_namespace']: v['drone_id'] for v in vehicles}
        self.declare_parameter("record_hz", 10.0)
        self.declare_parameter("detailed_timeline", False)
        self.declare_parameter("max_run_bytes", 100_000_000)
        self.run = CsvRun(self.get_parameter("output_dir").value, namespaces,
                          record_hz=self.get_parameter("record_hz").value,
                          detailed_timeline=self.get_parameter("detailed_timeline").value,
                          max_run_bytes=self.get_parameter("max_run_bytes").value)
        self.run.begin_test("session", "idle")
        self.declare_parameter("world_name", "")
        self.gz_node = None
        self.native_clock_active = False
        world = self.get_parameter("world_name").value
        if world:
            try:
                from gz.transport13 import Node as GzNode
                from gz.msgs10.clock_pb2 import Clock
                self.gz_node = GzNode()
                self.native_clock_active = bool(self.gz_node.subscribe(
                    Clock, f"/world/{world}/clock", self._gazebo_clock))
            except (ImportError, RuntimeError) as exc:
                self.get_logger().warning(f"Native physics clock unavailable: {exc}; use ROS /clock bridge.")
        from rosgraph_msgs.msg import Clock as RosClock
        self.create_subscription(RosClock, "/clock", self._ros_clock, 1000)
        self.export_thread = None
        self.create_timer(10.0, self._export_timeline)
        from std_msgs.msg import String
        self.test_subscription = self.create_subscription(
            String, "/testing/test_event",
            lambda msg: self._record("/testing/test_event", "std_msgs/msg/String", msg), 100)
        self.recorded = {("/testing/test_event", "std_msgs/msg/String"): self.test_subscription}
        self.failures = {}
        self.qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_timer(1.0, self.discover)
        self.create_timer(1.0, self.run.flush)
        self.get_logger().info(f"Recording topic messages and positions: {self.run.path}")

    def _gazebo_clock(self, msg):
        self.run.physics_step(msg.sim.sec * 1_000_000_000 + msg.sim.nsec)

    def _export_timeline(self):
        if self.export_thread is None or not self.export_thread.is_alive():
            self.export_thread = threading.Thread(target=self.run.export_timeline, daemon=True)
            self.export_thread.start()

    def _ros_clock(self, msg):
        # A delayed ROS bridge clock must not look like a native clock reset.
        if not self.native_clock_active:
            self.run.physics_step(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec)

    def _record(self, topic, type_name, msg):
        data = message_to_ordereddict(msg)
        if topic.endswith("/test_event"):
            event = json.loads(data["data"])
            self.run.begin_test(event["test"], event["status"])
        self.run.record(topic, type_name, data, self.get_clock().now().nanoseconds)

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            for type_name in types:
                key = (topic, type_name)
                if key in self.recorded:
                    continue
                try:
                    message_type = get_message(type_name)
                    subscription = self.create_subscription(
                        message_type, topic,
                        lambda msg, t=topic, n=type_name: self._record(t, n, msg),
                        self.qos,
                    )
                except (ImportError, AttributeError, ValueError, RuntimeError) as exc:
                    detail = str(exc)
                    if self.failures.get(key) != detail:
                        self.run.inventory.writerow([topic, type_name, "error", detail])
                        self.get_logger().warning(f"Cannot record {topic} ({type_name}): {detail}")
                        self.failures[key] = detail
                    continue
                self.recorded[key] = subscription
                self.failures.pop(key, None)
                self.run.inventory.writerow([topic, type_name, "subscribed", ""])
        self.run.flush()

    def destroy_node(self):
        try:
            if self.gz_node is not None:
                self.gz_node.unsubscribe(f"/world/{self.get_parameter('world_name').value}/clock")
            if self.export_thread is not None:
                self.export_thread.join()
            self.run.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TopicRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
