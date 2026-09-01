#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REBOTARM_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK_DIR="${REBOTARM_WS}/third_party/reBotArm_control_py"
VR_WS="${REBOTARM_WS}/vr_ws"
VR_REPO="${VR_WS}/src/openarmx_teleop_vr"

SDK_URL="https://github.com/Seeed-Projects/reBotArm_control_py.git"
SDK_COMMIT="5ba28acef46237eb6a7560658bbc43b06cf8a259"
VR_URL="https://github.com/openarmx/openarmx_teleop_vr.git"
VR_COMMIT="a3da7411b3d6ecaa7f94df859e07fb642aec859b"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

mkdir -p "${REBOTARM_WS}/third_party" "${VR_WS}/src"

if [[ ! -d "${SDK_DIR}/.git" ]]; then
  git clone "${SDK_URL}" "${SDK_DIR}"
fi
if [[ "$(git -C "${SDK_DIR}" rev-parse HEAD)" != "${SDK_COMMIT}" ]]; then
  if [[ -n "$(git -C "${SDK_DIR}" status --porcelain)" ]]; then
    echo "SDK has local changes; refusing to change revision: ${SDK_DIR}" >&2
    exit 1
  fi
  git -C "${SDK_DIR}" fetch origin main
  if ! git -C "${SDK_DIR}" cat-file -e "${SDK_COMMIT}^{commit}"; then
    echo "Pinned SDK commit is unavailable: ${SDK_COMMIT}" >&2
    exit 1
  fi
  git -C "${SDK_DIR}" checkout --detach "${SDK_COMMIT}"
fi

if [[ ! -d "${VR_REPO}/.git" ]]; then
  git clone --depth 1 --branch 6.0_basic "${VR_URL}" "${VR_REPO}"
fi
if [[ "$(git -C "${VR_REPO}" rev-parse HEAD)" != "${VR_COMMIT}" ]]; then
  if [[ -n "$(git -C "${VR_REPO}" status --porcelain)" ]]; then
    echo "VR bridge has local changes; refusing to change revision: ${VR_REPO}" >&2
    exit 1
  fi
  git -C "${VR_REPO}" fetch origin 6.0_basic
  if ! git -C "${VR_REPO}" cat-file -e "${VR_COMMIT}^{commit}"; then
    echo "Pinned VR bridge commit is unavailable: ${VR_COMMIT}" >&2
    exit 1
  fi
  git -C "${VR_REPO}" checkout --detach "${VR_COMMIT}"
fi

source /opt/ros/humble/setup.bash

python3 -m pip install --user --index-url https://pypi.org/simple \
  "${SDK_DIR}" "motorbridge==0.5.1" "transforms3d>=0.4.2"

rosdep install --from-paths "${REBOTARM_WS}/src" "${VR_WS}/src" \
  --ignore-src -r -y

cd "${REBOTARM_WS}"
colcon build --symlink-install

cd "${VR_WS}"
colcon build --symlink-install

echo
echo "Setup complete."
echo "reBotArm workspace: ${REBOTARM_WS}"
echo "PICO VR workspace:  ${VR_WS}"
