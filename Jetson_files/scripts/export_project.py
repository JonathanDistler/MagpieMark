#!/usr/bin/env python3
# Export the active simulator workspace without host environments or logs
import argparse
from pathlib import Path
import tarfile
import zipfile
import hashlib


PROJECT = Path(__file__).resolve().parents[1]
SOURCES = [
    ".gitignore", ".dockerignore", "compose.jetson.yaml", "deploy/jetson",
    "ReadMe.md", "scripts/start_simulation.py", "scripts/export_project.py",
    "scripts/start_single_drone_ros2.sh", "scripts/validate_flight_controls.py",
    "scripts/plot_drone_positions.m",
    "scripts/px4_sitl_parameters.sh", "scripts/px4_vision_sitl_parameters.sh",
    "scripts/wait_for_sitl_ready.py", "scripts/swarm_validity_test.py",
    "scripts/check_external_odometry.py",
    "ros2/swarm_mavsdk_ros2", "px4_msgs", "worlds", "models", "docs", "config",
    "environment.yml", "requirements-venv.txt", "environment-locks",
]
EXCLUDED_DIRECTORIES = {".git", ".venv", ".aws", ".agents", ".codex",
                        "__pycache__", ".pytest_cache", "results"}


def include(member):
    path = Path(member.name)
    if any(part in EXCLUDED_DIRECTORIES or part.endswith(".egg-info") for part in path.parts):
        return None
    if path.name == ".env" or path.name.endswith(".local.json"):
        return None
    if path.name.startswith("mav.") or path.name.endswith((".pyc", ".backup", ".log", ".csv", ".ulg")):
        return None
    # Source symlinks outside this workspace would not be portable
    if member.issym() or member.islnk():
        raise ValueError(f"Export requires regular source files: {member.name}")
    return member


def main():
    parser = argparse.ArgumentParser(description='Export the active simulator workspace without host environments or logs')
    parser.add_argument("--output", type=Path,
                        default=PROJECT / "exports/drone_obstacle_course-source.tar.gz")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents accidentally replacing a previous export
    sources = SOURCES.copy()
    if (PROJECT / "requirements-venv.lock.txt").is_file():
        sources.append("requirements-venv.lock.txt")
    # Build one filtered manifest for both formats; reject links before writing.
    members = []
    for relative in sources:
        root = PROJECT / relative
        paths = [root, *sorted(root.rglob("*"))] if root.is_dir() else [root]
        for path in paths:
            name = f"drone_obstacle_course/{path.relative_to(PROJECT).as_posix()}"
            info = tarfile.TarInfo(name)
            if path.is_symlink():
                info.type = tarfile.SYMTYPE
            if include(info) is not None and path.is_file():
                if path.stat().st_size >= 100 * 1024 * 1024:
                    raise ValueError(f"File exceeds GitHub's regular file limit: {name}")
                members.append((path, name))
    if args.output.suffix == ".zip":
        with zipfile.ZipFile(args.output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for path, name in members:
                archive.write(path, name)
    else:
        with tarfile.open(args.output, "x:gz") as archive:
            for path, name in members:
                archive.add(path, arcname=name, recursive=False, filter=include)
    digest = hashlib.sha256()
    with args.output.open("rb") as exported:
        for chunk in iter(lambda: exported.read(1024 * 1024), b""):
            digest.update(chunk)
    checksum = args.output.with_name(args.output.name + ".sha256")
    checksum.write_text(f"{digest.hexdigest()}  {args.output.name}\n")
    print(f"Exported {args.output.resolve()} ({args.output.stat().st_size:,} bytes)")
    print(f"SHA256: {digest.hexdigest()}")
    print("Recipients: follow docs/jetson.md for Jetson or docs/run_and_share.md for simulation")


if __name__ == "__main__":
    main()
