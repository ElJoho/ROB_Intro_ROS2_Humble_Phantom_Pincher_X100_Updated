#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `phantomx_pincher_description` package.
# It is ROS-distro aware:
#   - On ROS 2 Humble  -> prefers `ign`
#   - On ROS 2 Jazzy   -> prefers `gz`
#   - Otherwise        -> chooses whichever is available

set -e

# Require at least one argument: the input .xacro file
if [ $# -lt 1 ]; then
  echo "Usage: $0 <robot.urdf.xacro> [xacro arguments...]" >&2
  exit 1
fi

TMP_URDF_PATH=$(mktemp /tmp/phantomx_pincher_XXXXXX.urdf)

# Decide which Gazebo command to use, based on ROS_DISTRO and availability
GAZEBO_CMD=""

case "${ROS_DISTRO}" in
  jazzy)
    # Jazzy typically uses the new Gazebo (ros-gz) with `gz` CLI
    if command -v gz >/dev/null 2>&1; then
      GAZEBO_CMD="gz"
    elif command -v ign >/dev/null 2>&1; then
      GAZEBO_CMD="ign"
    fi
    ;;
  humble)
    # Humble is more likely to use Ignition Gazebo with `ign` CLI
    if command -v ign >/dev/null 2>&1; then
      GAZEBO_CMD="ign"
    elif command -v gz >/dev/null 2>&1; then
      GAZEBO_CMD="gz"
    fi
    ;;
  *)
    # Unknown or not set: pick whatever is available
    if command -v gz >/dev/null 2>&1; then
      GAZEBO_CMD="gz"
    elif command -v ign >/dev/null 2>&1; then
      GAZEBO_CMD="ign"
    fi
    ;;
esac

if [ -z "${GAZEBO_CMD}" ]; then
  echo "Error: neither 'gz' nor 'ign' command found in PATH." >&2
  echo "Make sure Gazebo is installed and that you've sourced the proper ROS 2 setup.bash." >&2
  exit 1
fi

# Process xacro into URDF
xacro "$1" "${@:2}" -o "${TMP_URDF_PATH}"

# Convert URDF to SDF using Gazebo CLI and adjust mesh paths
SDF_XML=$("${GAZEBO_CMD}" sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/phantomx_pincher_description\///g")

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null || true

# Return SDF as XML string
echo "${SDF_XML}"
