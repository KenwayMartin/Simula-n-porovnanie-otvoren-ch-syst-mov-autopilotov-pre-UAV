
## Current implementation

Bachelor project: simulation comparison of PX4 and ArduPilot UAV autopilots.

Environment:
- Arch Linux host
- Docker container with ROS 2 Jazzy
- Gazebo simulation
- PX4 SITL, x500_depth vehicle
- Default world: baylands
- MAVROS interface

### Implemented scenario

Automatic takeoff, forward movement, obstacle-triggered position hold,
and landing. Obstacle detection uses depth-camera measurements in ROS 2.
The external Python node makes the stop decision; PX4 executes flight control.
Native autopilot obstacle avoidance is not evaluated by this scenario.

ArduPilot comparison is planned.

### Files

- start_sim.sh: starts PX4/Gazebo, the RGB camera bridge and image viewer.
- obstacle_stop.py: flight experiment with automatic CSV recording.
- depth_debug.py: read-only depth viewer with ROI and minimum-depth marker.
- record_telemetry.py: telemetry recording script.
- wall.sdf: static wall for repeatable experiments.
- logs/: experiment CSV files.
- dependencies.txt: source URLs and exact Git commits of dependencies.
- simulation/worlds/: archived custom-world files.
- kosice.osm: OpenStreetMap export for future environment work.

### Running inside the ROS 2 container

The scripts currently expect the project at /home/user/Projects/BP.

Terminal 1:

```bash
cd /home/user/Projects/BP
bash start_sim.sh

Terminal 2:
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:='udp://:14540@'

Terminal 3:
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'

Terminal 4, monitoring:
cd /home/user/Projects/BP
/usr/bin/python3 obstacle_stop.py

Flight experiment in Gazebo:
/usr/bin/python3 obstacle_stop.py --fly

Depth diagnostics, in a separate terminal:
/usr/bin/python3 depth_debug.py

Place a broad wall ahead of the vehicle and keep the vertical takeoff
space clear. Defaults: height 2.5 m, stop threshold 4 m, maximum forward
travel 5 m. CSV files are created automatically in logs/.
Data interpretation
CSV rows contain latest available values from asynchronous ROS topics.
Freshness flags and sensor timestamps must be checked before analysis.
PX4 estimated position is not Gazebo ground truth.
External dependencies
PX4, ArduPilot, Micro-XRCE-DDS-Agent and the terrain generator are kept
outside this repository. See dependencies.txt for their source revisions.

The Košice OSM data is from OpenStreetMap contributors:
https://www.openstreetmap.org/copyright

## Current implementation

Bachelor project: simulation comparison of PX4 and ArduPilot UAV autopilots.

Environment:
- Arch Linux host
- Docker container with ROS 2 Jazzy
- Gazebo simulation
- PX4 SITL, x500_depth vehicle
- Default world: baylands
- MAVROS interface

### Implemented scenario

Automatic takeoff, forward movement, obstacle-triggered position hold,
and landing. Obstacle detection uses depth-camera measurements in ROS 2.
The external Python node makes the stop decision; PX4 executes flight control.
Native autopilot obstacle avoidance is not evaluated by this scenario.

ArduPilot comparison is planned.

### Files

- start_sim.sh: starts PX4/Gazebo, the RGB camera bridge and image viewer.
- obstacle_stop.py: flight experiment with automatic CSV recording.
- depth_debug.py: read-only depth viewer with ROI and minimum-depth marker.
- record_telemetry.py: telemetry recording script.
- wall.sdf: static wall for repeatable experiments.
- logs/: experiment CSV files.
- dependencies.txt: source URLs and exact Git commits of dependencies.
- simulation/worlds/: archived custom-world files.
- kosice.osm: OpenStreetMap export for future environment work.

### Running inside the ROS 2 container

The scripts currently expect the project at /home/user/Projects/BP.

Terminal 1:

```bash
cd /home/user/Projects/BP
bash start_sim.sh

Terminal 2:
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch fcu_url:='udp://:14540@'

Terminal 3:
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'

Terminal 4, monitoring:
cd /home/user/Projects/BP
/usr/bin/python3 obstacle_stop.py

Flight experiment in Gazebo:
/usr/bin/python3 obstacle_stop.py --fly

Depth diagnostics, in a separate terminal:
/usr/bin/python3 depth_debug.py

Place a broad wall ahead of the vehicle and keep the vertical takeoff
space clear. Defaults: height 2.5 m, stop threshold 4 m, maximum forward
travel 5 m. CSV files are created automatically in logs/.
Data interpretation
CSV rows contain latest available values from asynchronous ROS topics.
Freshness flags and sensor timestamps must be checked before analysis.
PX4 estimated position is not Gazebo ground truth.
External dependencies
PX4, ArduPilot, Micro-XRCE-DDS-Agent and the terrain generator are kept
outside this repository. See dependencies.txt for their source revisions.

The Košice OSM data is from OpenStreetMap contributors:
https://www.openstreetmap.org/copyright
