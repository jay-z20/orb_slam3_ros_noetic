#!/usr/bin/env bash
# Headless Noetic smoke build+run (requires Docker and local bags).
# Usage:
#   BAG_MI=/path/MH_01_easy.bag BAG_RGBD=/path/rgbd_dataset_freiburg1_xyz.bag \
#     ./scripts/noetic_smoke_docker.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG_MI="${BAG_MI:-}"
BAG_RGBD="${BAG_RGBD:-}"
if [[ -z "$BAG_MI" || -z "$BAG_RGBD" ]]; then
  echo "Set BAG_MI and BAG_RGBD to EuRoC MH_01_easy and TUM freiburg1_xyz bags." >&2
  exit 1
fi
OUT="${OUT:-$ROOT/.verify/smoke}"
mkdir -p "$OUT"
docker run --rm \
  -v "$ROOT:/src" \
  -v "$BAG_MI:/data/MH_01_easy.bag:ro" \
  -v "$BAG_RGBD:/data/rgbd_dataset_freiburg1_xyz.bag:ro" \
  -v "$OUT:/out" \
  ros:noetic-ros-base \
  bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq build-essential cmake git pkg-config \
      libeigen3-dev libepoxy-dev libgl1-mesa-dev libglew-dev libboost-all-dev \
      python3-catkin-tools \
      ros-noetic-cv-bridge ros-noetic-image-transport \
      ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
      ros-noetic-message-filters ros-noetic-nav-msgs ros-noetic-visualization-msgs \
      ros-noetic-rosbag
    if [[ ! -f /usr/local/lib/libpangolin.so && ! -f /usr/local/lib/libpangolin.so.0 ]]; then
      git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin
      cmake -S /opt/Pangolin -B /opt/Pangolin/build -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
      cmake --build /opt/Pangolin/build -j"$(nproc)"
      cmake --install /opt/Pangolin/build
      ldconfig
    fi
    mkdir -p /ws/src && ln -sfn /src /ws/src/orb_slam3_ros
    source /opt/ros/noetic/setup.bash
    cd /ws && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Keep concurrency low for modest machines (override with CATKIN_JOBS).
    catkin build orb_slam3_ros -j"${CATKIN_JOBS:-1}" --no-status
    source /ws/devel/setup.bash
    echo "BUILD_OK" | tee /out/result.txt
  '
echo "Build smoke finished; see $OUT/result.txt"
