# GitHub and Jetson deployment

This source package includes a CPU-only ROS 2 Jazzy companion-computer
container for Linux ARM64 Jetsons. Gazebo and PX4 SITL stay on your workstation;
the Jetson runs the DDS agent, external odometry bridge and drone controller.
Camera/Vicon drivers run separately and publish the configured ROS topics.
CUDA, TensorRT, GPU passthrough and camera drivers are not included.

## Flash the operating system first

The archive is application source, not a bootable disk image or PX4 firmware.
Use NVIDIA's image and flashing procedure for your exact module, carrier board
and storage device. See [NVIDIA SDK Manager](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html).
Flashing can erase the target storage. Do not write this ZIP to an SD card.

JetPack 6 provides an Ubuntu 22.04 host on supported Orin devices
([NVIDIA](https://developer.nvidia.com/embedded/jetpack-sdk-62)).
ROS Jazzy packages target Ubuntu 24.04, including ARM64
([ROS installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
The container supplies that userspace without replacing the Jetson's host OS.
This does not establish support for every Jetson: Nano, TX and Xavier boards
need a separate review of their JetPack, kernel and container compatibility.
There is no board-specific flash image in this release.

## Put the package on GitHub

Extract `exports/drone_obstacle_course-github-jetson.zip` into a new folder.
The curated archive excludes local credentials, agent settings, environments,
compiled workstation binaries, recordings and old release archives. It keeps
the active ROS package, PX4 messages, simulation assets and documentation.
Legacy ROS packages and CAD are outside this runtime package.

Create an empty GitHub repository, then run inside the extracted folder:

```bash
git init
git add .
git commit -m "Package drone obstacle course with Jetson deployment"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
git push -u origin main
```

Review files before publishing. The project's package metadata declares
Apache-2.0; bundled `px4_msgs` has its own LICENSE. Confirm you have permission
to publish the source and assets; this packaging does not change their licenses.

## Build on the Jetson

Install Docker Engine and the Compose v2 plugin for your host using
[Docker's Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/).
The commands below assume your account can run Docker; use `sudo docker` if needed.
An internet connection and sufficient disk space for ROS and native builds are required.

```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
cd YOUR_REPOSITORY
docker compose -f compose.jetson.yaml build
docker compose -f compose.jetson.yaml run --rm --no-deps controller \
  python3 -c 'import rclpy; from px4_msgs.msg import VehicleOdometry; import swarm_mavsdk_ros2.drone_node; print("Imports OK")'
```

Build on the Jetson to generate ARM64 binaries. Do not copy `install-system`
from an x86 workstation. Default build parallelism is two; use `BUILD_JOBS=1`
before the build command if memory is limited. No GPU runtime is needed.
The image uses ROS Jazzy and agent v2.4.3; upstream base images, apt packages
and agent dependencies are not fully pinned, so builds are not byte-reproducible.
Jetson image build and hardware operation still require validation on your board.

## Configure hardware and start manually

```bash
cp config/bench.example.json config/jetson.local.json
```

Edit `jetson.local.json` for the actual vehicle count, namespaces, system IDs,
DDS keys, tracker topics/bodies, frames and calibration. This local file is
ignored by Git. Read [hardware integration](hardware_integration.md), including
the PX4 v1.17 arbitrary-heading limitation and five-stage propeller-free checks.
The included `px4_msgs` must match your flight-controller firmware and its
versioned DDS topics. The agent version also must match the firmware's XRCE
client. This package does not flash or configure the flight controller.

Set the same ROS domain on the Jetson, tracker host and PX4 DDS client
(`UXRCE_DDS_DOM_ID` where available). Compose's `ROS_DOMAIN_ID` configures ROS
nodes; it does not override the domain requested by a PX4 client. Networked
tracker publishers should use compatible ROS Jazzy message definitions.

```bash
export NUM_DRONES=2 ROS_DOMAIN_ID=0
docker compose -f compose.jetson.yaml up -d agent
docker compose -f compose.jetson.yaml logs -f agent
```

Point each PX4 UDP client to the Jetson IP on port 8888 with its own namespace
and unique DDS key. Host networking permits DDS discovery on Linux. Configure
the network/firewall to permit the ROS DDS traffic as well as UDP 8888.
The [agent CLI](https://micro-xrce-dds.docs.eprosima.com/en/v2.4.3/agent.html)
also supports serial; serial needs a separate Compose device mapping and command,
so this template uses UDP only. Do not run two agents on the same port.

Start your calibrated tracker driver separately, then in another terminal:

```bash
export NUM_DRONES=2 ROS_DOMAIN_ID=0
docker compose -f compose.jetson.yaml --profile hardware up -d
docker compose -f compose.jetson.yaml logs -f odometry controller
```

These services do not automatically issue arm/takeoff commands, and no boot
autostart is configured. Do not run the simulation flight-validation scripts
against physical aircraft. For the read-only fusion check:

```bash
docker compose -f compose.jetson.yaml exec controller \
  /drone-entrypoint.sh python3 scripts/check_external_odometry.py \
  --n 2 --vehicle-config config/jetson.local.json
```

Replace `--n 2` with your configured count. `docker compose exec` needs the
entrypoint explicitly to source the ROS overlay. To inspect available topics:

```bash
docker compose -f compose.jetson.yaml exec controller /drone-entrypoint.sh ros2 topic list
```

To stop all services:

```bash
docker compose -f compose.jetson.yaml --profile hardware down
```

## Regenerate the source archive

```bash
python3 scripts/export_project.py --output exports/drone_obstacle_course-github-jetson.zip
```

ZIP and tar.gz exports include a top-level `drone_obstacle_course` folder.
Use a new output name for each export; existing archives are preserved.
