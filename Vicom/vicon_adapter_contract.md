# HVL external Vicon adapter handoff

The teammate's Vicon adapter is not present in this checkout. No Vicon driver,
raw topic names, capture-clock mapping, or tracking-validity interface could be
identified. The existing `external_odometry` is a generic ROS pose bridge, not
that adapter. This handoff adds configuration only; it does not implement or
launch the missing Vicon acquisition path.

`config/hvl_external.example.json` is the shared three-vehicle mapping. Pass its
absolute path to the teammate's adapter once it supports this schema, and to
the existing controller:

```bash
ros2 launch swarm_mavsdk_ros2 swarm.launch.py num_drones:=3 vehicle_config:="$PWD/config/hvl_external.example.json"
```

| Vicon body | Logical interface | VehicleOdometry destination | DDS key | MAV_SYS_ID |
| --- | --- | --- | --- | --- |
| magpie_01/body | /drone_0 | /fmu/in/vehicle_visual_odometry | 1 | 1 |
| magpie_02/body | /drone_1 | /px4_1/fmu/in/vehicle_visual_odometry | 2 | 2 |
| magpie_03/body | /drone_2 | /px4_2/fmu/in/vehicle_visual_odometry | 3 | 3 |

JSON does not configure aircraft identities. Check each aircraft's actual
namespace, `UXRCE_DDS_KEY`, and `MAV_SYS_ID`. VehicleOdometry has no target ID;
the topic selects the aircraft. The controller launch starts no odometry bridge.

## One publisher and one extrinsic correction

Preferred integration: the teammate's adapter publishes VehicleOdometry directly
to the destinations above. Do not also launch `odometry.launch.py`, the Gazebo
tracker, or another EV publisher for those aircraft.

Alternatively, if the teammate supplies normalized PoseStamped instead, the
proposed `/tracking/drone_i/pose` topics in this example are an explicit handoff
interface, **not discovered Vicon topics**. They must already express the IMU
point in `hvl_world_frd`, with body FRD attitude. Only in this mode run:

```bash
ros2 launch swarm_mavsdk_ros2 odometry.launch.py num_drones:=3 vehicle_config:="$PWD/config/hvl_external.example.json"
```

The example uses identity world/body transforms and zero marker offsets because
the external adapter owns normalization and marker-to-IMU compensation. These
zeros do not assert physical marker/IMU coincidence. Do not feed raw marker poses
to this profile. PoseStamped has no body ID or validity flag: upstream must
verify the body association and suppress invalid tracking before publishing.

## Frame and reference contract

Use one shared origin and heading for all three bodies: `hvl_world_frd`, metres,
Z down. Publish `POSE_FRAME_FRD=2`; arbitrary lab heading is not geographic NED.
Body orientations map body forward/right/down into that world. PX4 quaternion
arrays are Hamilton **[w,x,y,z]**; ROS fields are accessed by name (x,y,z,w).
Validate finite values and a nonzero normalizable quaternion before publication.

Measure the Vicon world-to-HVL rotation/translation and marker-to-body rotation.
With `q = q_HVL_Vicon * q_Vicon_marker * q_marker_body` and body-FRD vector
`r = IMU-to-marker`, compute `p_IMU = R_HVL_Vicon*p_marker + t - R(q)*r`.
Apply this once in the external adapter. Record any driver-applied transform.
All three aircraft must share this world calibration; do not independently zero
each body at startup. A +90-degree Z-down yaw maps forward +X to +Y and has
quaternion `[sqrt(0.5),0,0,sqrt(0.5)]`.

IMU is the provisional position reference. In local PX4 1.17 source,
`ev_pos_control.cpp` and `ev_height_control.cpp` subtract the rotated vector
`ev_pos_body - imu_pos_body`. For an already IMU-corrected measurement, set
`EKF2_EV_POS_X/Y/Z` equal to the corresponding `EKF2_IMU_POS_X/Y/Z`, so this
relative lever arm is zero. All EV_POS values may be zero when IMU_POS is also
zero in the provisional reference convention. Do not enter marker-to-IMU
offsets again in EV_POS; do not blindly zero EV_POS with nonzero IMU_POS.
Reconcile the actual aircraft reference parameters before use.

PX4 1.17's arbitrary-heading EV yaw path may fuse EV while leaving
`heading_good_for_control` false. The existing controller requires that flag.
See [the measured limitation](hardware_integration.md#v1170-arbitrary-heading-limitation).
Keep that controller check; FRD must not be relabelled NED without surveying and
transforming to North. This remains an integration blocker for offboard use.

## Freshness, timing, and QoS

Reject missing/untracked bodies, wrong parent/body frames, invalid numbers,
zero/nonincreasing capture stamps, stale stamps, and excessive future stamps.
The scaffold's existing bridge accepts age at most 0.2 s and future skew at most
0.02 s. Stationary poses with new valid capture stamps remain valid. Never
infer tracking loss solely from an unchanged pose, and never refresh the stamp
on cached data. If Vicon exposes frame numbers/validity, use them upstream;
freshly restamped cached poses cannot be detected by PoseStamped alone.
Stop publishing on loss, including timer-based replay. Restart disarmed after
clock rollback or frame calibration changes unless explicit resets are supported.

Use capture timestamps mapped to the ROS host wall-clock domain; use wall time
on hardware. `timestamp_sample` preserves capture time in microseconds and
`timestamp` is publication time. Local PX4 XRCE deserialization corrects both
using its synchronized clock offset. Do not also subtract that offset upstream.
Verify `UXRCE_DDS_SYNCT=1`, converged `uxrce_dds_client status`, ROS/agent host
clock alignment, and boot-time sample age with `listener vehicle_visual_odometry`
on **each** aircraft. Measure capture latency/drift; do not compensate latency
twice in timestamps and `EKF2_EV_DELAY`.

The existing bridge publishes reliable/volatile/depth 10, compatible with PX4's
best-effort reader, and receives tracker poses using sensor-data QoS. Match the
external adapter endpoints using `ros2 topic info -v`; use best-effort readers
for PX4 outputs. No transient-local cached poses. Aim for 50 Hz; the bridge
warns below 20 Hz. Pose-only outputs use NaN velocity/angular velocity, unknown
velocity frame, finite positive calibrated variances, and quality 0 (unused in
the checked message). Do not enable EV velocity fusion for pose-only data.

## Required from the teammate / lab

- Adapter package/executable, launch arguments, and configuration schema (or a
  mapping from this JSON); choose direct VehicleOdometry or normalized pose mode.
- Actual Vicon topics/types, exact parent/child identifiers, capture clock and
  frame counter/validity semantics, and behavior during occlusion/reconnection.
- Measured common world transform, per-body marker-to-FRD rotation and IMU
  lever arm, upstream compensation ownership, and calibrated measurement noise.
- Actual aircraft namespace/identity and firmware/message match, time-sync and
  sample-age evidence, EKF reference/fusion settings, and FRD heading resolution.

Before flight, with propellers removed, move each body separately along X/Y/Z,
rotate yaw by 90 degrees, hold it stationary, and occlude it. Verify isolation,
quaternion axes, stationary updates, and complete publication cessation during
loss on ROS and PX4. Check estimator fusion and heading validity. Synthetic
tests validate the existing bridge contract, not the missing external adapter.
