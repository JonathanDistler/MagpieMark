# Vehicle identities and external odometry

Bulk recording is now opt-in: add `--record` for SITL ULog/MAVLink/ROS
recordings, or `record:=true` for the ROS recorder in a direct swarm launch.
Small startup diagnostics remain available. See the [recovery report](hvl_recovery_validation.md).

The existing launch commands still select `gz_x500_vision` and its built-in
vision measurements. The optional `--odometry-source external` path uses a base
`gz_x500` with no odometry plugin, then routes simulated tracker poses through
the ROS adapter used for hardware. Controller commands and logical drone IDs
are the same in both modes.

## Quick start

Existing three-drone simulation (unchanged):

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --headless
```

Hardware-style measurement path:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --headless --odometry-source external
```

Two logical drones with explicit bench mappings:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 2 --world swarm --headless --odometry-source external --vehicle-config config/sitl_external.json
```

Add `--architecture edge:drone_0` (or another valid member) to select drone-led
coordination. All previous console, Python, RL, position, velocity, stop, landing,
and obstacle commands keep their syntax. `--tests 1 2 3` still requires a
single-drone obstacle world. Old shell launchers retain their original behavior;
the new options belong to `scripts/start_simulation.py`.

The launcher builds the overlay unless `--skip-build` is supplied. Rebuild after
adding or changing installed entry points/launch files. Native Gazebo Python
bindings `gz.transport13` and `gz.msgs10` are required for the simulated tracker;
they are installed with the supported Gazebo Harmonic environment. Their
availability in a Conda environment must be checked separately.

## Identity configuration

Default logical drone `i` uses MAVLink system ID `i+1`, DDS client key `i+1`,
and an empty PX4 namespace for zero or `/px4_i` for other drones. Public control
interfaces remain `/drone_i` regardless of hardware naming.

`config/bench.example.json` is an example, **not an inventory of the HVL hardware**.
It maps bench drone 1 to logical drone 0 and bench drone 2 to logical drone 1,
with namespaces `/uav_0`, `/uav_1`, system IDs 1, 2, and DDS keys 101, 102.
The real Vicon names, axes, offsets, and calibrated noise are still unknown.
The empty `mocap_body` fields intentionally reflect this.

The JSON contains one ordered entry per vehicle under `vehicles`; `drone_id`
must run from zero through `--n - 1`. Omitted fields take these defaults:

| Field | Meaning / default |
| --- | --- |
| `bench_label` | Human-readable bench association, `bench_<i+1>` |
| `px4_namespace` | Prefix before `/fmu`, not including `/fmu` itself |
| `system_id` | PX4 `MAV_SYS_ID`; also `VehicleCommand.target_system` |
| `dds_key` | PX4 `UXRCE_DDS_KEY`; unique client identity on the agent |
| `mocap_body` | Recorded body association; empty until identified in the lab |
| `tracking_topic` | Absolute ROS input topic, `/tracking/drone_i/pose` |
| `tracking_message` | `PoseStamped` (default) or `TransformStamped` |
| `tracking_frame` | Required input `header.frame_id`, default `lab` |
| `gazebo_model` | Simulated tracker model name, `x500_i` |
| `odometry` | Transform, uncertainty, and sample-validity settings below |

Namespaces, system IDs, keys, tracker topics, model names, and nonempty mocap body
associations must be unique. Config counts and numeric identity ranges are
validated before simulation processes start. The resolved mapping is saved to
`results/startup/vehicles.json`. The recorder maps custom PX4 prefixes back to
logical `drone_i` columns. Raw topics retain their original names.

PX4's local SITL startup hardcodes instance-based system IDs and keys. For custom
IDs, the launcher writes a private `rcS-identities` under that instance's startup
directory, replacing only those two assignments. It fails on an unrecognized
startup format rather than silently using the wrong IDs. It never edits the
PX4 checkout. Hardware startup must be configured on the aircraft separately.

`VehicleOdometry` has no target-system field. These are different destinations:

```text
/fmu/in/vehicle_visual_odometry
/px4_1/fmu/in/vehicle_visual_odometry
/uav_0/fmu/in/vehicle_visual_odometry
```

`/fmu/in/in/vehicle_visual_odometry` is not the configured PX4 topic. A DDS key
does not route an odometry message; the namespace does. Command routing also
requires the correct autopilot system/component fields.

## Measurement path and adapter contract

```text
Simulation: Gazebo world poses -> gazebo_tracker -> ROS tracking topic
Hardware:  cameras -> Vicon/tracker ROS driver -> ROS tracking topic
                                            |
                                  external_odometry
                                            |
                         /<namespace>/fmu/in/vehicle_visual_odometry
                                            |
                                  XRCE-DDS -> PX4 EKF2
                                            |
                             VehicleLocalPosition -> DroneNode
                                            |
                TrajectorySetpoint + OffboardControlMode + VehicleCommand
                                            |
                                     PX4 motor control
```

The hardware adapter consumes standard ROS messages; it does not connect to the
Vicon DataStream SDK or configure the vendor tracker driver. Configure that
driver's real object/segment selection and remap its output to `tracking_topic`,
or put its existing absolute topic in the JSON. `mocap_body` documents and logs the association; for `TransformStamped`
it also requires an exact `child_frame_id` match when nonempty. It does not
select objects inside a vendor driver. `PoseStamped` has no body identifier,
so its dedicated per-body topic and upstream association must be verified.
For a Vicon driver publishing `TransformStamped`, set `tracking_message` to
`TransformStamped`; the translation and rotation are interpreted as that tracked
rigid body's pose in `tracking_frame`.

Each valid input produces one PX4 pose sample. Linear velocity, angular velocity,
and velocity variance are NaN, with `VELOCITY_FRAME_UNKNOWN`. Position and
orientation variances are configured positive values; they are not inferred
from a pose-only input. External simulation enables EV position, height, and yaw
fusion (`EKF2_EV_CTRL=11`) and does not enable EV velocity fusion. IMU-driven
EKF2 velocity estimates still feed velocity control.

The adapter rejects nonfinite positions, zero/invalid quaternions, wrong frames,
zero timestamps, stale samples, future samples beyond tolerance, and duplicate
or backward timestamps. It normalizes valid quaternions. It never repeats the
last pose when tracking stops. Tracking loss produces throttled warnings and
allows PX4's configured estimator/failsafe handling to act. A driver must stop
publishing invalid/occluded poses; a fresh timestamp on an invalid cached pose
cannot be distinguished from a real stationary observation.

This initial bridge does not estimate velocities, propagate input covariance,
or support changing the world origin while running. Coordinate resets require
stopping and reinitializing the session while disarmed; reset_counter remains
zero. It does not use Vicon confidence fields that are absent from the standard
input messages.

## Frames, offsets, and time

All configuration quaternions are Hamilton **[w, x, y, z]**. Incoming ROS message
fields use their named x/y/z/w fields. Define:

- `world_rotation`: rotation from tracker/lab coordinates into PX4 navigation coordinates.
- `world_translation`: translation of that world transform, in navigation metres.
- `body_rotation`: rotation from vehicle body FRD into the tracked rigid-body axes.
- `marker_offset_body`: vector from the desired published reference point to
  the tracked rigid-body origin, expressed in vehicle body FRD metres.

The conversion is:

```text
R_navigation_body = R_navigation_lab * R_lab_rigid * R_rigid_body
p_navigation_reference = R_navigation_lab * p_lab_rigid
                         + translation_navigation
                         - R_navigation_body * marker_offset_body
```

Defaults convert ENU positions and FLU body orientation to NED/FRD:
`world_rotation=[0,sqrt(0.5),sqrt(0.5),0]`, `body_rotation=[0,1,0,0]`,
with zero translation/offset. **These defaults are simulation conventions, not
measured HVL conventions.** Configure the actual rigid-body axes in the lab.

`pose_frame=1` means true-North-aligned NED; `pose_frame=2` means a fixed-heading,
down-positive navigation frame with arbitrary heading. The bench example uses 2.
Choose one common lab origin and heading for the fleet. Verify that the resulting
PX4 local estimates share consistent axes/origins before coordinated movement;
the controller does not automatically calibrate or reconcile per-drone frames.
Its historical North/East/Down command labels then refer to the aligned local
axes, not necessarily geographic North in an arbitrary-heading lab setup.

Apply each marker lever arm **once**:

1. Publish the tracked marker origin with `marker_offset_body=[0,0,0]` and
   configure `EKF2_EV_POS_X/Y/Z` for that origin; or
2. Transform to a chosen reference point with `marker_offset_body` and configure
   `EKF2_EV_POS_X/Y/Z` for that resulting point.

Your local firmware defines EV offsets relative to vehicle centre of gravity.
Account consistently for `EKF2_IMU_POS_X/Y/Z`; transforming to the IMU does not
automatically imply a zero EV offset in that convention. Confirm the installed
firmware's definitions. The body origin need not be the geometric centroid.

Sample timestamps must be in the ROS host clock domain. The adapter preserves
acquisition time in `timestamp_sample` and uses publication time in `timestamp`,
both in microseconds. The PX4 DDS time synchronization translates these to PX4
time. Synchronize tracker/host clocks or translate the tracker's clock upstream;
do not put raw Vicon frame numbers, unsynchronized device time, or Gazebo time
into hardware messages. Defaults: `max_age_s=0.2`, `future_tolerance_s=0.02`.

The Gazebo tracker timestamps poses when received on the host, publishes at up
to 50 Hz, and never republishes without a new Gazebo sample. This exercises the
ROS/DDS/estimator path but does not reproduce camera latency, measurement noise,
occlusion, or hardware clock drift. Its pose coordinates use the Gazebo model
origin. Hardware rigid-body-to-reference calibration remains a lab task.

Default `position_variance` and `orientation_variance` are `[0.0025]*3`, in m²
and rad² respectively. They are illustrative values. Calibrate them, the EKF
noise policy, and `EKF2_EV_DELAY` for the actual tracking system.

## Hardware bench bring-up

Do not run `start_simulation.py` against physical aircraft. It launches SITL and
uses a simulation-only failsafe/arming profile. Keep the aircraft's normal
hardware safety and sensor settings.

1. Record firmware/message versions, actual `MAV_SYS_ID`, `UXRCE_DDS_KEY`, DDS
   namespace, Vicon object/segment, tracking topic/type, and frame axes per drone.
2. Copy the bench example, replace its illustrative settings with the measured
   ones, and choose/measure the lab world transform and body lever arms.
3. Configure unique `MAV_SYS_ID` and `UXRCE_DDS_KEY` on the aircraft. Configure
   each PX4 DDS client with its namespace (for example client `-n uav_0`),
   transport/agent address, and matching ROS domain. Namespace startup mechanisms
   depend on firmware; the simulation environment variable is not a hardware
   setup procedure. Use px4_msgs compatible with that firmware.
4. Start the tracker ROS driver with the actual rigid-body associations. Check
   timestamps, tracking validity, and frame IDs before starting this adapter.
5. Start the adapter in a sourced, built ROS workspace:

   ```bash
   ros2 launch swarm_mavsdk_ros2 odometry.launch.py num_drones:=2 vehicle_config:=/absolute/path/to/bench.json
   ```

   Hardware defaults to `gazebo:=false`. This launch does not start an agent,
   configure PX4 parameters, arm, or launch the offboard controller.
6. Configure EV position/height/yaw fusion, height reference, delays, noise,
   lever arms, and other sensor fusion for that aircraft and lab. Do not blindly
   copy the SITL profile. Verify the internal PX4 `vehicle_visual_odometry`
   using `listener vehicle_visual_odometry` and inspect estimator fusion/status.
7. With motors inactive, translate and rotate each body separately. Confirm the
   matching vehicle's local estimate follows with correct signs, scale, heading,
   and offset. Confirm the other vehicle is unaffected. Stop tracking and confirm
   fresh odometry stops and estimator tracking-loss behavior is appropriate.
8. Only after bench validation, launch the existing controller interfaces with
   the same mapping:

   ```bash
   ros2 launch swarm_mavsdk_ros2 swarm.launch.py num_drones:=2 vehicle_config:=/absolute/path/to/bench.json architecture:=central
   ```

   Leave `world_name` empty on hardware. This starts controller interfaces with
   bulk recording disabled; startup, arm, and takeoff remain explicit commands.
   Add `record:=true` only when ROS telemetry recording is required.

## Validation and maintenance

In a separate terminal using the same ROS overlay/domain as simulation:

```bash
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
python3 scripts/check_external_odometry.py --n 3
python3 scripts/validate_flight_controls.py --n 3 --controls-only
```

The read-only odometry check requires fresh pose-only input, EV position/height/yaw
fusion, no EV velocity fusion, and valid local position/velocity. It is intended
for the configured pose-only profile, not every possible hardware fusion setup.
Flight validation commands arm and move **simulated** vehicles. Omit
`--controls-only` to include all three swarm benchmarks in both control modes
(requires at least three drones). Pass the launch's `--architecture` to the
validator. Pass `--vehicle-config config/bench.example.json` to validators when
using custom namespaces, including `swarm_validity_test.py`.

Run regressions on a different ROS domain from the running simulation:

```bash
ROS_DOMAIN_ID=91 python3 -m pytest ros2/swarm_mavsdk_ros2/test -q
```

Use one ROS environment per terminal. If Conda is active, open a clean system
terminal before sourcing the system overlay; mixing their Python/library paths
can produce misleading test failures.

Relevant source locations:

| File | Responsibility |
| --- | --- |
| `swarm_mavsdk_ros2/vehicle_config.py` | Identity defaults and JSON validation |
| `swarm_mavsdk_ros2/odometry_conversion.py` | Pure pose math and timestamp checks |
| `swarm_mavsdk_ros2/external_odometry.py` | ROS tracker inputs -> PX4 messages |
| `swarm_mavsdk_ros2/gazebo_tracker.py` | Gazebo world poses -> standard ROS input |
| `launch/odometry.launch.py` | Independent bridge launch |
| `launch/swarm.launch.py` | Logical drone/controller routing |
| `scripts/start_simulation.py` | Builds and owns simulated processes |
| `scripts/check_external_odometry.py` | Read-only estimator-path validation |

Python/launch paths above are relative to `ros2/swarm_mavsdk_ros2/`.
The [recovery and validation report](hvl_recovery_validation.md) records measured results separately from this
setup guide. Hardware flight readiness cannot be established from SITL results.

## Installed contract and build

The recovered local firmware is **PX4 v1.17.0**; system ROS is **Jazzy** on
Python 3.12. `px4_msgs/package.xml` is 2.0.1 (that package version alone does
not establish firmware compatibility). All seven messages used by this path
match the local firmware's fields, constants, array sizes, and ordering:

```bash
/usr/bin/python3 scripts/verify_px4_contract.py --px4-dir ~/PX4-Autopilot
```

Run that check against the firmware source actually flashed to each aircraft.
The firmware YAML declares `/fmu/out/vehicle_local_position`, but message
version 1 adds the runtime suffix `_v1`; the controller already subscribes to
`/uav_0/fmu/out/vehicle_local_position_v1` (and similarly for vehicle 1).
`VehicleStatus` also has `_v1`; `VehicleOdometry` version 0 has no suffix.
The input is exactly `/uav_0/fmu/in/vehicle_visual_odometry`, not a versioned
input or `vehicle_mocap_odometry`. The latter is a different uORB input.

Use a clean system terminal, without Conda ROS paths:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log-system build --build-base build-system --install-base install-system \
  --paths px4_msgs ros2/swarm_mavsdk_ros2 --symlink-install
source install-system/local_setup.bash
```

The bridge uses reliable, volatile, depth-10 publication, compatible with PX4's
best-effort reader; tracker subscriptions use the sensor-data QoS (best effort,
volatile). Use sensor-data QoS to read PX4 outputs. There are no latched poses.
The bridge checks accepted message rate every five seconds and warns below
`minimum_rate_hz` (default 20 Hz); input/output rates should normally be near
50 Hz. Warnings per vehicle are limited to one per two seconds. A rate warning
is diagnostic; stale/invalid data is suppressed independently.

## Lab frame and offset calibration decision

Use **Approach B**, transforming marker poses to the vehicle centre of gravity
before publication. This keeps marker-specific calibration in the shared JSON
and is testable without PX4. PX4 v1.17.0 describes `EKF2_EV_POS_*` relative to
centre of gravity, so set all three to zero only when that is the point you
actually publish. Approach A is supported by leaving `marker_offset_body=0`
and configuring the physical marker's FRD displacement in PX4 instead. Never
subtract a lever arm in both places. If publishing the IMU point, use its
CG-relative displacement for EV position, rather than assuming zero.

Measure these per vehicle while disarmed:

1. Define a reproducible HVL origin (floor fiducial) and horizontal heading
   (a second fiducial). Record the tracker-frame coordinates of both. Determine
   vertical direction and units. The bench config uses fixed-heading FRD,
   `pose_frame=2`, because geographic North has not been surveyed.
2. Fit `world_rotation` from the measured axes, and choose
   `world_translation = -R_navigation_lab * p_lab_origin`. Keep Z vertical;
   the arbitrary-heading FRD representation permits heading offset, not an
   arbitrary tilted navigation frame. For ENU use the documented conversion
   quaternion; compose the measured lab heading with it as necessary.
3. Align or measure the marker axes relative to drone forward/right/down.
   `body_rotation` maps body FRD vectors into the tracked rigid-body axes.
   Determine it by fixture/axis measurements; do not infer it from marker names.
4. Measure the vector **CG to marker origin**, in body FRD metres, and put that
   in `marker_offset_body`. Record CG-to-IMU separately and reconcile
   `EKF2_IMU_POS_*`. Account for any tracker-driver transform already applied.
5. Hold the reference point stationary while rotating the drone. The corrected
   reference position should remain stationary. Translate along all three lab
   axes and rotate yaw ±90 degrees; compare quaternion-derived body axes.
   Repeat separately for both drones and record residual RMS error.

`position_variance` is expressed along the published navigation axes;
`orientation_variance` along published body axes. Values are variances, not
standard deviations. The example assumes 5 cm / 0.05 rad standard deviations
(0.0025 variances); these are conservative placeholders, not measured accuracy.
Measure stationary scatter and moving residuals and account for calibration
uncertainty, including lever-arm error. This pose-only adapter does not consume
input covariance or claim tracking confidence (`quality=0`, unused in the
checked message). Leave `EKF2_EV_QMIN=0` for this source.

## Proposed PX4 parameter worksheet (operator applies)

These are v1.17.0 bench starting assumptions, not commands executed by this
software. Inspect the actual aircraft parameters and other aiding sources first.

| Parameter | Proposed setting / interpretation |
| --- | --- |
| `MAV_SYS_ID` | 1 / 2 for this example; replace with actual unique IDs |
| `UXRCE_DDS_KEY` | 101 / 102; unique on the shared agent |
| `UXRCE_DDS_SYNCT` | 1; confirm client reports converged synchronization |
| `EKF2_EV_CTRL` | 11 = horizontal position + height + yaw; velocity bit 2 clear |
| `EKF2_HGT_REF` | 3 (vision), if EV is the intended lab height reference |
| `EKF2_EV_POS_X/Y/Z` | 0 only for CG-corrected Approach B |
| `EKF2_EV_DELAY` | Measured residual delay relative to IMU, ms; 0 initially with correct capture timestamps; reboot required |
| `EKF2_EV_NOISE_MD` | 0: message variance, bounded below by EKF noise parameters |
| `EKF2_EVP_NOISE` | Default 0.1 m floor; calibrate using measured residuals |
| `EKF2_EVA_NOISE` | Default 0.1 rad floor; calibrate heading residuals |
| `EKF2_EVP_GATE` | Default 5 standard deviations; inspect innovations before adjustment |
| `EKF2_EV_QMIN` | 0; standard pose inputs do not provide confidence |

`EKF2_EVV_NOISE/GATE` do not justify enabling unavailable external velocity.
Decide GPS, magnetometer, barometer and range aiding policies for the actual
lab; the simulation parameter script disables/reconfigures sensors for SITL
and must not be copied to hardware. Yaw fusion assumes measured rigid-body
orientation accurately represents body heading. Inspect magnetic interference
and yaw innovations instead of disabling physical sensors automatically.

## Time verification

In local v1.17.0, `src/modules/uxrce_dds_client/uxrce_dds_client.cpp` estimates
agent/client clock offset; `Tools/msg/templates/ucdr/msg.h.em` subtracts that
offset from both `timestamp` and `timestamp_sample` on reception (clamped to
PX4 current time). Do not subtract the PX4 offset again in the bridge.

Input `header.stamp` is capture time already mapped to the ROS host's epoch
clock. Callback reception time is used for freshness validation; publication
time is taken separately for `timestamp`. `timestamp_sample` retains capture
time at microsecond resolution. The agent's host and ROS host must share a
clock (same machine or measured PTP/NTP synchronization). Hardware runs must
use wall time, not `use_sim_time`. A driver timestamped only on reception needs
an upstream capture-clock/known-latency correction; do not relabel arrival as
accurate capture time.

Record capture-to-arrival and capture-to-publish distributions, clock offset
and drift, and PX4 timesync status. Align deliberately oscillated bench motion
with IMU/ULog data to estimate residual EV delay after timestamp correction;
look for position/yaw innovation lag and verify the delay sign using replay.
Do not compensate the same latency in both timestamps and `EKF2_EV_DELAY`.
Restart disarmed after clock rollback, tracker origin changes, or calibration
changes: the bridge rejects nonincreasing timestamps and does not automatically
invent a frame reset (`reset_counter=0`). Loss of fresh messages produces no
new odometry. PX4's separate timesync convergence must be checked before fusion.

## Five-stage propeller-free verification

Prerequisites: secured aircraft with propellers removed, appropriate power,
firmware matching the message contract, reachable XRCE agent, one valid tracker
rigid body per aircraft, calibrated frames/offsets, and operator-reviewed EKF
settings. No stage here arms or takes off.

Example agent (one agent can serve both unique clients):

```bash
MicroXRCEAgent udp4 -p 8888
```

On each PX4 console inspect `uxrce_dds_client status`. If manually starting an
otherwise stopped client, the UDP pattern is
`uxrce_dds_client start -t udp -h <HOST_IP> -p 8888 -n uav_0`, using `uav_1` on
vehicle 2. Configure startup persistence/serial transport and ROS domain for
the actual board. Do not start a second competing client. JSON cannot change
a physical aircraft's DDS namespace or MAVLink ID.

**Stage 1 — tracker:** Confirm driver-selected bodies and dedicated topics;
`ros2 topic info -v /tracking/drone_0/pose`,
`ros2 topic echo /tracking/drone_0/pose --once`, and
`ros2 topic hz /tracking/drone_0/pose`. Repeat for drone 1. Confirm metres,
frame ID, body orientation, synchronized capture stamps, and no fresh cached
poses during occlusion. Use `TransformStamped` with exact `mocap_body` matching
when possible; for `PoseStamped` verify upstream topic association explicitly.

**Stage 2 — bridge:** Launch `odometry.launch.py` with the bench config. Inspect:

```bash
ros2 topic echo /uav_0/fmu/in/vehicle_visual_odometry --once
ros2 topic hz /uav_0/fmu/in/vehicle_visual_odometry
ros2 topic echo /uav_1/fmu/in/vehicle_visual_odometry --once
```

Move only one body at a time. Check destination isolation, ENU/FRD signs,
quaternion norm, NaN velocity/angular velocity, nonzero variance, capture age,
and expected frame constant. Cover/disable one body; its publication must stop.

**Stage 3 — PX4 reception:** On the corresponding aircraft use
`listener vehicle_visual_odometry 5`, `uorb top`, and `uxrce_dds_client status`.
Confirm rate, sample age in PX4 boot microseconds, transformed coordinates,
NaNs and variances. Repeat independently for both aircraft.

**Stage 4 — EKF2:** Use `ekf2 status`, `listener estimator_status_flags 5`,
`listener vehicle_local_position 5`, and ULog estimator innovations/test ratios
(`estimator_aid_src_ev_pos`, `estimator_aid_src_ev_hgt`, and
`estimator_aid_src_ev_yaw` where available). Confirm `cs_ev_pos`, `cs_ev_hgt`,
`cs_ev_yaw` true, `cs_ev_vel` false, `xy_valid`, `z_valid`, velocity validity,
and `heading_good_for_control`. Inspect ROS output with
`ros2 topic echo /uav_0/fmu/out/vehicle_local_position_v1 --once --qos-reliability best_effort`.
The read-only checker can validate both aircraft:

```bash
python3 scripts/check_external_odometry.py --n 2 --vehicle-config config/bench.example.json
```

The checker uses a five-second status-flag age limit (`--status-max-age`) for
slow SITL, and one second for pose/local-state streams; status messages are
slower than pose messages. It requires multiple samples from all three streams.
This checker verifies the specified pose-only fusion profile, not calibration
accuracy. Compare position scale, axis direction, height and heading across
the full measurement volume and verify common fleet origins; EKF validity alone
is insufficient. Inspect estimator resets and rejected observations in ULog.

**Stage 5 — offboard interface:** Launch `swarm.launch.py` with the same mapping
and empty `world_name`. Confirm `/drone_0/state` and `/drone_1/state` reflect the
correct PX4 estimates. Default construction does not stream setpoints or arm.
Inspect command generation in the synthetic regression tests and any recorded
SITL output. For physical command-generation inspection, disconnect the aircraft
DDS clients and use an isolated ROS domain with synthetic estimated-state
messages before issuing controller commands. Verify `VehicleCommand` has the
configured target system, component 1, source system equal to configured system,
source component 1, and `from_external=true`. Inspect setpoints and offboard
heartbeat topics under `/uav_0` and `/uav_1`. Never use flight validation scripts
against aircraft. Powered testing is a separate operator-approved procedure.

Common faults:

| Symptom | Checks |
| --- | --- |
| No ROS input | Tracker body/topic/type/frame; occlusion; domain/network |
| Bridge rejects input | Capture clock, age, duplicate stamps, finite pose, transform child ID |
| No PX4 reception | Agent transport, namespace, unique keys, topic type/version, QoS |
| Huge/wrong sample age | SYNCT convergence, host clocks, use_sim_time, double time correction |
| EV received but not fused | EV_CTRL bits, QMIN, noise, innovations, sensor policies, EKF status |
| Rotating moves estimated position | Lever-arm sign, body rotation, double offset correction |
| Wrong heading or mirrored motion | ENU/NED and FLU/FRD conversion; world/body transform directions |
| Both vehicles move together | Wrong upstream rigid-body association or duplicated DDS identities |
| Only state topics absent | Runtime `_v1` suffix, message compatibility and best-effort subscription |

## Synthetic tracker and isolated SITL replay

For a bridge-only test, use a ROS domain with no physical clients and source the
built overlay in both terminals:

```bash
export ROS_DOMAIN_ID=91
ros2 launch swarm_mavsdk_ros2 odometry.launch.py num_drones:=2 vehicle_config:="$PWD/config/bench.example.json"
```

In the second terminal:

```bash
export ROS_DOMAIN_ID=91
ros2 run swarm_mavsdk_ros2 mock_tracker --ros-args \
  -p num_drones:=2 -p vehicle_config:="$PWD/config/bench.example.json" \
  -p axis:=x -p rate_hz:=50.0 -p dropout_after_s:=10.0
```

`axis` supports `stationary`, `x`, `y`, `z`, and `yaw`; `amplitude` and
`period_s` control sinusoidal motion (yaw amplitude in radians). Set
`capture_latency_s:=0.5` to exercise stale rejection with default bridge
limits. This models sample age, not realistic tracking noise. The mock emits
standard pose/transform messages only and never sends control commands.

For stationary SITL fusion through the same bridge, use a new log/state directory:

```bash
ROS_DOMAIN_ID=92 /usr/bin/python3 scripts/start_simulation.py --ros system \
  --n 2 --world blank --headless --odometry-source external \
  --vehicle-config config/sitl_external.json --run-dir results/hvl_sitl_new
```

In a sourced terminal with `ROS_DOMAIN_ID=92`, run the read-only fusion checker.
Do not run the mock alongside the Gazebo tracker on the same topics. This mode
uses base `gz_x500` without the built-in vision plugin, preventing competing EV
sources. Default `--odometry-source builtin` retains `gz_x500_vision` behavior.

### v1.17.0 arbitrary-heading limitation

Source inspection and stationary SITL showed that `LOCAL_FRAME_FRD` EV yaw
fusion explicitly leaves `cs_yaw_align=false` in
`EKF/aid_sources/external_vision/ev_yaw_control.cpp`. In this firmware,
`heading_good_for_control` requires that flag (`EKF/ekf.h`). Therefore the
uncalibrated bench profile (`pose_frame=2`) can demonstrate EV fusion but does
**not** meet the existing controller's heading-validity requirement. Do not
bypass that controller check. For a usable estimate with this firmware, survey
true North and use `pose_frame=1` with the corresponding calibrated world
rotation, or separately validate a firmware solution for arbitrary-heading
operation. Merely changing the enum without aligning the axes is incorrect.

`config/sitl_external.json` supplies the simulation-only NED convention with the
same `/uav_0` and `/uav_1` routing. Use that config for the SITL fusion checker
as well. `config/bench.example.json` deliberately retains FRD until physical
alignment is known. Successful EV fusion flags by themselves do not establish
a usable offboard heading or geographic frame alignment.
