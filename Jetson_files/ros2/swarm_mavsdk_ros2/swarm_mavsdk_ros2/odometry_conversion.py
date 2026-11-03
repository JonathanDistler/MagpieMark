"""Pose transforms, independent of ROS. Quaternions use Hamilton w,x,y,z."""
import math


def vector(value, size):
    if len(value) != size or not all(math.isfinite(x) for x in value):
        raise ValueError(f"Expected {size} finite values")
    return tuple(float(x) for x in value)


def quaternion(value):
    value = vector(value, 4)
    norm = math.hypot(*value)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("Zero or unnormalizable quaternion")
    return tuple(x/norm for x in value)


def multiply(a, b):
    w,x,y,z = a
    s,u,v,t = b
    return (w*s-x*u-y*v-z*t, w*u+x*s+y*t-z*v,
            w*v-x*t+y*s+z*u, w*t+x*v-y*u+z*s)


def rotate(q, v):
    return multiply(multiply(q, (0., *v)), (q[0], -q[1], -q[2], -q[3]))[1:]


DEFAULTS = dict(
    # ENU lab -> NED navigation; body FRD -> tracker FLU.
    world_rotation=[0., math.sqrt(.5), math.sqrt(.5), 0.],
    world_translation=[0., 0., 0.], body_rotation=[0., 1., 0., 0.],
    marker_offset_body=[0., 0., 0.], pose_frame=1,
    position_variance=[.0025, .0025, .0025],
    orientation_variance=[.0025, .0025, .0025],
    max_age_s=.2, future_tolerance_s=.02)


def settings(overrides=None):
    overrides = overrides or {}
    if set(overrides) - set(DEFAULTS):
        raise ValueError(f"Unknown odometry options: {set(overrides) - set(DEFAULTS)}")
    cfg = {**DEFAULTS, **overrides}
    for name in ("world_rotation", "body_rotation"):
        cfg[name] = quaternion(cfg[name])
    for name in ("world_translation", "marker_offset_body", "position_variance", "orientation_variance"):
        cfg[name] = vector(cfg[name], 3)
    for name in ("position_variance", "orientation_variance"):
        if min(cfg[name]) < 1.401298464324817e-45 or max(cfg[name]) > 3.4028234663852886e38:
            raise ValueError("Measurement variances must be positive and representable as float32")
    if cfg["pose_frame"] not in (1, 2):
        raise ValueError("pose_frame must be NED (1) or fixed-heading FRD (2)")
    if not math.isfinite(cfg["max_age_s"]) or cfg["max_age_s"] <= 0:
        raise ValueError("max_age_s must be positive")
    if not math.isfinite(cfg["future_tolerance_s"]) or cfg["future_tolerance_s"] < 0:
        raise ValueError("future_tolerance_s must be nonnegative")
    return cfg


def transform(position, orientation, cfg):
    q = quaternion(multiply(multiply(cfg["world_rotation"], quaternion(orientation)), cfg["body_rotation"]))
    p = rotate(cfg["world_rotation"], vector(position, 3))
    offset = rotate(q, cfg["marker_offset_body"])
    return tuple(p[i] + cfg["world_translation"][i] - offset[i] for i in range(3)), q


def valid_sample(stamp_ns, now_ns, previous_ns, cfg):
    return (stamp_ns > 0 and stamp_ns > previous_ns and
            -cfg["future_tolerance_s"] <= (now_ns-stamp_ns)*1e-9 <= cfg["max_age_s"])
