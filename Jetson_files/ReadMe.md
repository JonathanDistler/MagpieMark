# Drone obstacle course

For **GitHub packaging and Jetson deployment**, start with the
[Jetson guide](docs/jetson.md). It includes a native ARM64 container build,
hardware configuration and deployment commands. Flash the Jetson with the
NVIDIA image for your board first, then deploy this application.

The project files are distributed in `MagpieMark_shareable.zip`. This project
is a ROS 2 / PX4 / Gazebo benchmark for individual drones and drone swarms,
with static obstacle routes, swarm behaviors, and Python interfaces for RL.
The research goal is to compare a physical test bed with its simulation.

## Hardware-style odometry and bench mappings

**Previous simulation and control commands still work unchanged.** The default
continues to use Gazebo's built-in x500 vision measurements. To exercise the
new pose-only ROS external-odometry path, add `--odometry-source external`:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --headless --odometry-source external
```

Add `--vehicle-config config/bench.example.json --n 2` for the example mapping
of bench drones 1/2 to logical drones 0/1. The example does not contain measured
HVL/Vicon settings. Controller commands still address logical `/drone_0`, etc.

See the [hardware integration guide](docs/hardware_integration.md) for the full
pipeline, configuration fields, frames, marker offsets, timestamps, NaN velocity
convention, hardware bring-up, validation, and source ownership.

## Disk use and recovered integration

Your existing launch, console, swarm, and Python control commands keep their syntax.
Normal launches now disable PX4 ULog, MAVProxy telemetry files, and the ROS topic
recorder. Small process diagnostics remain in `results/startup`; explicit obstacle
and benchmark commands can still produce their own result CSVs. Add `--record`
to the launcher to enable flight recordings (ULog/MAVLink files can grow large).
Direct ROS launches accept `record:=true` to enable the bounded ROS recorder.
No physical flight-controller parameters are changed by this recording option.

Historical ROS/PX4/Gazebo logs and flight recordings were removed on 2026-10-30;
built overlays, simulator binaries, models, source, calibration/configuration,
and small reports were retained. See the [recovery and validation report](docs/hvl_recovery_validation.md)
for tested behavior and the exact remaining lab work.

For a short **unarmed** two-drone check of both simulation estimation paths:

```bash
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
/usr/bin/python3 scripts/validate_hvl_sitl.py
```

Stop existing simulator sessions first. This checks the installed overlay without
rebuilding, uses ROS domain 92, stops its processes, and disables bulk recording.
It starts no physical hardware and sends no arm/takeoff commands.

The two-vehicle bench configuration routes PX4 through `/uav_0` and `/uav_1`;
public controller commands still use logical `--drone 0` and `--drone 1`.
Physical tracker names, offsets, clocks, and heading alignment require lab calibration.
In PX4 v1.17.0 the arbitrary-heading bench profile can fuse EV while still failing
`heading_good_for_control`; follow the guide before enabling offboard operation.

Deleting Linux files does not necessarily shrink the Windows WSL virtual disk.
If Windows still reports little space, see [WSL disk reclamation](docs/hvl_recovery_validation.md#wsl-host-space).
Do not fill free space with zero files; that would make your immediate disk problem worse.

## Start here

Extract `MagpieMark_shareable.zip` and open the extracted project folder in
VS Code, then use **Terminal → New Terminal**. If you received the updated
archive named `drone_obstacle_course_updated_2026-10-30.zip`, extract that archive
instead. A previous version in `prev_files`, when supplied separately, offers
an older workflow with less dashboard overhead; follow that version's own
instructions because its features and commands may differ.

Commands assume
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, a built PX4 checkout, and a DDS agent.
A ZIP contains the project files; it does not install those dependencies.
For a new machine, complete [installation and sharing](docs/run_and_share.md)
first. For RoboStack, use [Conda setup](docs/miniconda_setup.md) and its matching
overlay instead of the system commands below. Replace `~/drone_obstacle_course`
with your extracted project path when different.

The source ZIP excludes virtual environments, generated build and install
folders, and historical results. Build outputs are files generated from the
source code, such as compiled programs, libraries, and ROS package setup files.
The launcher builds the ROS packages and recreates the required overlay
(`install-system/` for the system ROS commands below). On a new machine, install
the prerequisites first and adjust any local PX4, DDS agent, or vehicle settings
as described in the installation guide. Archive integrity has been checked;
a fresh installation from the updated ZIP has not yet been validated.

## Single drone

Terminal 1:

```bash
cd ~/drone_obstacle_course
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1 --world blank
```

Wait for `Ready: 1 drones`. The launcher builds the ROS packages and starts
Gazebo, PX4, the DDS agent, MAVProxy, and ROS interfaces. Bulk recording is off
by default to conserve disk space; add `--record` when you need flight recordings. It does not
open a control console. Keep this terminal running.

Terminal 2:

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Enter one command at a time and wait for completion:

```text
startup
arm
takeoff
forward 0.5
backward 0.5
land
disarm
quit
```

Wait for touchdown before disarming. Quitting a console does not stop the
simulation or cancel active targets. After landing, press Ctrl-C in Terminal 1.

## Single-drone obstacle tests

Stop the previous session, then launch:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1 --world obstacles --tests 1 2 3
```

This prepares flight and runs the three routes automatically. On success the
drone holds its last destination; use the sourced Terminal 2 to land and disarm:

```bash
ros2 run swarm_mavsdk_ros2 drone_command land --drone 0
ros2 run swarm_mavsdk_ros2 drone_command disarm --drone 0
```

For interactive tests, omit `--tests` and enter `test 1`, `test 2`, or `test 3`
in the control console. See [test execution and recording](docs/swarm_validity.md).

## Swarm and drone-led control

Restart Terminal 1 with:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --world swarm
```

After `Ready: 3 drones`, source the system overlay in Terminal 2 as above:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3
```

Directional swarm moves wait until every drone reaches its target and settles.
Slow simulation can take longer than 45 seconds: each drone must make at least
2 cm of progress toward its target within 45 seconds, or the command fails with
telemetry diagnostics. Readiness has its own 45-second limit. Python callers
can pass `timeout_s` to `move_all` for a strict overall deadline.

For benchmarks, after startup/arm/takeoff run:

```bash
ros2 run swarm_mavsdk_ros2 swarm_tests --n 3 --test all --seed 42
```

Then land and disarm. For a drone-led fleet, add `--architecture edge:drone_0`
to the launcher and every fleet CLI invocation. Counts and architectures must
match the running session. See [benchmarks and Python behaviors](docs/swarm_benchmarks.md).

## Guides

- [Installation, two-terminal workflow, ZIP export, and recipient setup](docs/run_and_share.md)
- [Conda environment and native simulator dependencies](docs/miniconda_setup.md)
- [Coordinates, position/velocity control, Python, and RL](docs/flight_controls.md)
- [Obstacle tests, validation, CSV recording, and MATLAB plots](docs/swarm_validity.md)
- [Swarm benchmarks and behaviors](docs/swarm_benchmarks.md)
- [Dated control validation results](docs/control_validation.md)
- [Recorded environment versions](environment-locks/README.md)

The active package is `ros2/swarm_mavsdk_ros2`; its name does not imply a
runtime MAVSDK dependency. Root-level MAVSDK tests, `run_drone_swarm.bash`,
`scripts/start_swarm_ros2.sh`, and the other ROS packages are older workflows.
Use the shared Python launcher for the process documented here. The single-drone
shell wrapper is supported, but defaults to the blank world and system ROS.
