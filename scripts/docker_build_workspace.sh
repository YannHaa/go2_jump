#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${GO2_JUMP_IMAGE:-go2-jump-humble:latest}"

"${ROOT_DIR}/scripts/bootstrap_workspace_repo.sh"

docker run --rm --net host \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "${IMAGE_TAG}" \
  bash -lc '
    set -euo pipefail

    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

    cd /workspace/src/unitree_ros2/cyclonedds_ws/src
    [ -d rmw_cyclonedds ] || git clone --depth 1 -b humble https://github.com/ros2/rmw_cyclonedds.git
    [ -d cyclonedds ] || git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git

    cd /workspace/src/unitree_ros2/cyclonedds_ws
    export LD_LIBRARY_PATH=/opt/ros/humble/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    colcon build --symlink-install --packages-select cyclonedds
    set +u
    source /workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash
    set -u
    colcon build --symlink-install --packages-select unitree_go unitree_hg unitree_api
    set +u
    source /workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash
    set -u

    cd /workspace/src/unitree_ros2/example
    colcon build --symlink-install --packages-select unitree_ros2_example

    cd /workspace
    source /workspace/scripts/container_source_env.sh
    colcon build --symlink-install \
      --packages-ignore unitree_api unitree_go unitree_hg unitree_ros2_example \
      --packages-select go2_jump_planner go2_jump_controller go2_jump_bringup stand_go2

    cd /workspace/src/unitree_mujoco/simulate
    ln -snf /root/.mujoco/mujoco-3.3.6 mujoco
    rm -rf build
    mkdir -p build
    cd build
    cmake ..
    make -j"$(nproc)"

    cd /workspace/tools
    rm -rf build
    mkdir -p build
    cd build
    cmake ..
    make -j"$(nproc)"
  '
