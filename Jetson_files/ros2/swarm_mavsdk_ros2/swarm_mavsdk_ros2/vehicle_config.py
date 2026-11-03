"""Shared logical vehicle identities. JSON keeps the launcher dependency-free."""
import json
import re
from pathlib import Path


def default_vehicles(count):
    return [dict(drone_id=i, bench_label=f"bench_{i+1}",
                 px4_namespace="" if i == 0 else f"/px4_{i}",
                 system_id=i+1, dds_key=i+1,
                 tracking_topic=f"/tracking/drone_{i}/pose",
                 tracking_message="PoseStamped",
                 tracking_frame="lab", mocap_body="", gazebo_model=f"x500_{i}")
            for i in range(count)]


def load_vehicles(path, count):
    if type(count) is not int or not 1 <= count <= 250:
        raise ValueError('Vehicle count must be between 1 and 250')
    vehicles = default_vehicles(count)
    if path:
        document = json.loads(Path(path).expanduser().read_text())
        if set(document) != {"vehicles"} or len(document["vehicles"]) != count:
            raise ValueError("Vehicle config must contain exactly num_drones entries under 'vehicles'.")
        for i, entry in enumerate(document["vehicles"]):
            if entry.get("drone_id") != i:
                raise ValueError("Vehicle entries must be ordered with contiguous zero-based drone_id values.")
            unknown = set(entry) - set(vehicles[i]) - {"odometry"}
            if unknown:
                raise ValueError(f"Unknown vehicle keys: {sorted(unknown)}")
            vehicles[i].update(entry)
    for v in vehicles:
        ns = v["px4_namespace"]
        if not isinstance(ns, str):
            raise ValueError("px4_namespace must be a string")
        ns = ns.strip("/")
        if ns and not all(re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", p) for p in ns.split("/")):
            raise ValueError(f"Invalid PX4 namespace: {ns}")
        v["px4_namespace"] = "/" + ns if ns else ""
        for key, maximum in (("system_id", 250), ("dds_key", 4294967295)):
            if type(v[key]) is not int or not 1 <= v[key] <= maximum:
                raise ValueError(f"{key} must be an integer between 1 and {maximum}")
        if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z_0-9]*/)*[A-Za-z_][A-Za-z_0-9]*", v["tracking_topic"]):
            raise ValueError("tracking_topic must be an absolute ROS topic")
        if not v["tracking_frame"]:
            raise ValueError("tracking_frame must be explicit")
        if v['tracking_message'] not in ('PoseStamped', 'TransformStamped'):
            raise ValueError('tracking_message must be PoseStamped or TransformStamped')
    for key in ("px4_namespace", "system_id", "dds_key", "tracking_topic", "gazebo_model"):
        if len({v[key] for v in vehicles}) != count:
            raise ValueError(f"Each vehicle must have a unique {key}")
    bodies = [v["mocap_body"] for v in vehicles if v["mocap_body"]]
    if len(set(bodies)) != len(bodies):
        raise ValueError("Each configured mocap_body must be unique")
    return vehicles
