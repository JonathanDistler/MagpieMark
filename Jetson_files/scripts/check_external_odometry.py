#!/usr/bin/env python3
"""Read-only check of pose-only ROS input and active PX4 external-vision fusion."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition, VehicleOdometry
from swarm_mavsdk_ros2.vehicle_config import load_vehicles


def samples_fresh(i, samples, last, now, status_max_age):
    # EKF2 publishes unchanged status flags once per simulated second, whereas
    # pose/state streams are much faster. Slow SITL must not require 10 status
    # updates with the same one-second wall-time deadline as odometry.
    for kind, minimum, age in (('input', 10, 1.), ('local', 10, 1.),
                               ('flags', 3, status_max_age)):
        if samples.get((i, kind), 0) < minimum or (i, kind) not in last:
            return False
        if now - last[i, kind][0] > age:
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=1)
    parser.add_argument('--vehicle-config', default='')
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--status-max-age', type=float, default=5.,
                        help='Wall-clock age limit for slow EKF status flags (seconds).')
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in (args.timeout, args.status_max_age)):
        parser.error('Timeout and status age must be finite and positive')
    vehicles = load_vehicles(args.vehicle_config, args.n)
    rclpy.init()
    node = Node('external_odometry_check')
    samples, last = {}, {}

    def receive(i, kind, msg):
        samples[i, kind] = samples.get((i, kind), 0) + 1
        last[i, kind] = (time.monotonic(), msg)

    for v in vehicles:
        i, prefix = v['drone_id'], v['px4_namespace']
        for kind, cls, suffix in (
            ('input', VehicleOdometry, '/fmu/in/vehicle_visual_odometry'),
            ('flags', EstimatorStatusFlags, '/fmu/out/estimator_status_flags'),
            ('local', VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1')):
            node.create_subscription(cls, prefix + suffix,
                lambda msg, i=i, kind=kind: receive(i, kind, msg), qos_profile_sensor_data)

    def ready(i):
        if not samples_fresh(i, samples, last, time.monotonic(), args.status_max_age):
            return False
        inp, flags, local = (last[i,k][1] for k in ('input', 'flags', 'local'))
        return (flags.cs_ev_pos and flags.cs_ev_hgt and flags.cs_ev_yaw and not flags.cs_ev_vel
                and local.xy_valid and local.z_valid and local.v_xy_valid and local.v_z_valid
                and local.heading_good_for_control
                and all(math.isfinite(x) for x in (*inp.position, *inp.q, local.x, local.y, local.z))
                and all(math.isnan(x) for x in (*inp.velocity, *inp.angular_velocity))
                and inp.velocity_frame == VehicleOdometry.VELOCITY_FRAME_UNKNOWN)

    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(ready(v['drone_id']) for v in vehicles):
                for v in vehicles:
                    i = v['drone_id']
                    print(f"PASS drone {i} {v['px4_namespace'] or '/'}: fresh pose-only input, EV position/height/yaw fusion, valid local state; samples={samples[i,'input']}")
                return
        for v in vehicles:
            i = v['drone_id']
            print(f"drone {i}: samples=" + str({k: samples.get((i,k), 0) for k in ('input','flags','local')}))
            if (i, 'flags') in last:
                f = last[i,'flags'][1]
                print(f"  EV position={f.cs_ev_pos} height={f.cs_ev_hgt} yaw={f.cs_ev_yaw} velocity={f.cs_ev_vel}")
            if (i, 'local') in last:
                state = last[i, 'local'][1]
                print(f"  local xy={state.xy_valid} z={state.z_valid} velocity_xy={state.v_xy_valid} velocity_z={state.v_z_valid} heading={state.heading_good_for_control}")
            print('  ages_s=' + str({k: round(time.monotonic()-last[i,k][0], 3)
                  for k in ('input', 'flags', 'local') if (i,k) in last}))
        raise SystemExit('External odometry/fusion check failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
