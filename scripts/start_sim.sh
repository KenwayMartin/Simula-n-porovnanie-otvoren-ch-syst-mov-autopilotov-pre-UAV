


    
  
#!/usr/bin/env bash
# Run inside the ROS 2 Jazzy container.
set -eo pipefail

BP_DIR="${BP_DIR:-/home/user/Projects/BP}"
CAMERA='/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'

[[ -d "$BP_DIR/Systems/PX4-Autopilot" ]] || { echo "Missing: $BP_DIR/Systems/PX4-Autopilot" >&2; exit 1; }
[[ -f "$BP_DIR/.venv/bin/activate" ]] || { echo "Missing: $BP_DIR/.venv/bin/activate" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
command -v setsid >/dev/null
command -v ros2 >/dev/null

pids=()
cleanup() {
  trap '' INT TERM
  for pid in "${pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Separate process groups let Ctrl+C also stop child processes.
setsid bash -c '
  source "$1/.venv/bin/activate"
  cd "$1/Systems/PX4-Autopilot" || exit 1
  export PX4_GZ_WORLD=baylands
  exec make px4_sitl gz_x500_depth
' bash "$BP_DIR" < /dev/null &
pids+=("$!")

setsid ros2 run ros_gz_bridge parameter_bridge \
  "${CAMERA}@sensor_msgs/msg/Image[gz.msgs.Image" < /dev/null &
pids+=("$!")

setsid ros2 run ros_gz_bridge parameter_bridge   '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' < /dev/null &
pids+=("$!")

setsid ros2 run rqt_image_view rqt_image_view < /dev/null &
pids+=("$!")

setsid ros2 launch mavros px4.launch   fcu_url:='udp://:14540@' < /dev/null &
pids+=("$!")

printf '\nSelect this topic in rqt_image_view:\n%s\nStop: Ctrl+C\n\n' "$CAMERA"
# Closing the viewer or any component exiting stops the rest.
wait -n "${pids[@]}"

