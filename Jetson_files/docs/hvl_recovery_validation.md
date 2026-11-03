# Recovery and HVL software validation — 2026-10-30

## Recovered state

The interrupted work was present and was continued in place. The project and
`px4_msgs` have no usable Git metadata in this workspace: the project `.git`
directory is empty, and status/branch/diff/history commands report “not a git
repository.” No reset, checkout, clean, or source replacement was performed.
Consequently a complete Git-based uncommitted-change inventory is unavailable.
PX4 has working Git metadata: tag `v1.17.0`, commit `d6f12ad1c4`. Its unrelated
untracked files and submodule changes were left intact.

Already implemented on recovery:

- `vehicle_config.py` centralizes namespaces, system IDs, DDS keys and tracker associations.
- `external_odometry.py`, `odometry_conversion.py`, `gazebo_tracker.py`, and
  `mock_tracker.py` implement standard pose/transform input, frame/marker correction,
  capture timestamps, NaN velocities, suppression and diagnostics.
- Configurations map bench 1/2 to logical 0/1 and PX4 `/uav_0`/`/uav_1`.
  Simulation defaults remain empty namespace/`/px4_1`, system IDs 1/2; the
  logical `/drone_i` controller API is preserved in both cases.
- The shared launcher selects built-in `gz_x500_vision` or base `gz_x500` plus
  the ROS external-odometry bridge, preventing competing visual odometry sources.
- Controller/recorder routing, launch files, contract checker, tests, and the
  HVL calibration/bring-up guide already existed. Historical logs read before
  deletion contained a 261-test pass and a two-vehicle NED SITL fusion pass;
  those are historical observations, separate from this session's checks.

Incomplete on recovery: the linked recovery report was missing; bulk logging
could still consume disk; the bridge needed float32 serialization guards;
physical tracking associations/calibration and lab verification were outstanding.

## Changes completed in this continuation

| Files | Result |
| --- | --- |
| `scripts/start_simulation.py` | Existing command syntax retained; bulk recording requires `--record`; normal SITL sets `SDLOG_BACKEND=0` before boot and MAVProxy uses `--no-state` |
| `ros2/swarm_mavsdk_ros2/launch/swarm.launch.py` | ROS recorder conditional on `record:=true`, default false |
| `swarm_mavsdk_ros2/external_odometry.py` | Reject float32 overflow/nonfinite transformed measurements before serialization; other vehicle remains functional |
| `swarm_mavsdk_ros2/odometry_conversion.py` | Reject variances that would serialize as zero/infinity |
| `scripts/check_external_odometry.py` | Separate freshness policy for slow status flags; detailed failure diagnostics; no relaxation of fusion/heading requirements |
| `scripts/validate_hvl_sitl.py` | Repeatable unarmed two-vehicle built-in/external SITL smoke check with owned-process cleanup and no bulk recording |
| `test/test_external_odometry.py`, `test/test_odometry_checker.py` | Serialization and independent stream-freshness regression coverage |
| `ReadMe.md`, `docs/hardware_integration.md`, `docs/run_and_share.md`, `docs/swarm_validity.md`, this report | Updated commands, recording policy, recovered status and lab limitations |

Python and test paths in the table are relative to `ros2/swarm_mavsdk_ros2/`.

## Installed contract and verification

System ROS 2 is Jazzy, Python 3.12; `px4_msgs` package version is 2.0.1.
The contract script confirmed fields, constants, dimensions and order for
VehicleOdometry, VehicleLocalPosition, VehicleCommand, OffboardControlMode,
TrajectorySetpoint, VehicleStatus and EstimatorStatusFlags against local PX4.
The DDS declarations match; local position uses runtime `_v1`, odometry input
has no version suffix. Aircraft firmware must be checked independently.

Tests cover stationary pose, all world axes, yaw rotations, normalized
quaternions, rigid-body offsets, timestamps, stale/invalid/duplicate tracking,
wrong body/frame, dropout without replay, two-topic routing with live ROS,
all odometry fields, NaN velocity, VehicleCommand destination fields, controller
behavior, and simulation launch configuration. The first restricted run had
three AF_UNIX socket permission failures; all passed outside the sandbox.
The final fresh regression result is **280 passed in 11.81 seconds**, including
the added float32 and checker-cadence tests.
Fresh two-vehicle SITL checks passed:

| Mode | Observed result |
| --- | --- |
| Default built-in vision | Two valid estimators, GCS links and controller interfaces ready |
| External ROS odometry, `/uav_0` and `/uav_1` | Both reported EV position/height/yaw fusion, EV velocity fusion disabled, valid local position/velocity and heading; checker received 279 input samples per vehicle |
| Disk use | No `.ulg` or `.tlog` recordings generated; both smoke-run diagnostic directories total under 1 MB |

Neither check armed or took off. This is stationary SITL reception/fusion evidence,
not physical-aircraft or powered-flight validation.

The status checker requires at least ten fresh input/local-state observations
and three status observations. Input and state have a one-second wall-time
freshness limit; status has a configurable five-second limit, since EKF2 emits
unchanged flags once per simulated second and slow SITL stretches that interval.
The first external run observed EV fusion on both drones but timed out with
9/10 status samples; this exposed the checker cadence bug. Fusion flags, local
position/velocity validity, finite state, and heading validity remain mandatory.
A passing sample check does not establish sustained flight performance.

Reproduce from a sourced system overlay, with other simulator sessions stopped:

```bash
/usr/bin/python3 scripts/verify_px4_contract.py
ROS_DOMAIN_ID=91 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider ros2/swarm_mavsdk_ros2/test
/usr/bin/python3 scripts/validate_hvl_sitl.py
```

The SITL harness uses domain 92, existing built overlays and two instances,
checks readiness and external fusion, and shuts down. It never arms or takes
off. New diagnostics are under `results/hvl_smoke`; no large recording/rebuild
is needed. Conda and powered flight were not revalidated in this continuation.

## Disk cleanup and future use

Approximately **32.2 GiB** of historical data was deleted: project flight/ROS
recordings and colcon logs, PX4 rootfs and numbered-instance ULogs, ROS home
logs, Gazebo recordings, and colcon logs in the other ROS workspaces. Source,
CAD, worlds/models, dependencies, existing builds/install overlays, configuration,
parameters and small JSON/Markdown reports were retained. Project usage fell
from approximately 18 GB to under 1 GB. Old documentation referencing deleted
flight logs describes historical checks; those raw logs are no longer available.

Normal shared-launcher sessions now keep only small process diagnostics and
SITL state, with no ULog/MAVLink/topic recorder output. Explicit test/benchmark
commands can still write their own CSV results. `--record` enables ULog/MAVLink
files (which can grow large) and the existing bounded ROS recorder (100 MB/run).
Direct `swarm.launch.py` accepts `record:=true` for ROS recording only. Older
shell launchers retain their historical behavior; use the documented shared
Python launcher for this disk-saving policy. No physical aircraft parameters
were changed. Simulator binaries and working overlays were intentionally kept
so your previous commands do not trigger a full native rebuild.

### WSL host space

Linux free-space figures do not measure Windows free space or prove that a
WSL VHDX has shrunk. Deletion frees blocks inside Linux immediately; Windows
may need trim and offline VHD compaction. The existing
`scripts/reclaim_wsl_disk.ps1` has an inspection mode and a `-Compact` mode.
Review its hard-coded distro registration against your installation first;
compaction terminates that WSL distribution. Run it separately from Windows
PowerShell after stopping your work, if required. This session did not terminate
WSL, compact its disk, or allocate zero-filled files. Windows reclaimed space
has not been measured.

## Remaining physical HVL work

Follow [the staged HVL guide](hardware_integration.md) with propellers removed.

1. Verify the actual flashed firmware/messages, distinct namespaces, DDS keys,
   MAVLink IDs, transport and agent/host clock synchronization for both aircraft.
2. Select the actual tracking driver and rigid bodies; configure dedicated topics
   and verified tracking validity/occlusion handling. No proprietary API was invented.
3. Survey world origin/heading; measure marker-to-body rotation, CG/IMU lever
   arms and measurement uncertainty. Apply marker correction exactly once.
4. Resolve PX4 v1.17.0 arbitrary-heading FRD yaw behavior: the bench profile can
   fuse EV but leave `heading_good_for_control=false`. Survey true North for
   calibrated NED or separately validate a firmware solution; never relabel
   arbitrary axes as true North or bypass controller validity checks.
5. Operator-review/apply EKF parameters and measure capture latency/residual
   EV delay. Verify PX4 reception, EV fusion/innovations, axes, estimates, resets,
   dropouts and cross-vehicle isolation by moving each airframe independently.
6. Verify passive state consumption and disconnected command routing before
   any separately authorized powered test. No hardware integration or physical
   EKF fusion has been verified by this software-only work.
