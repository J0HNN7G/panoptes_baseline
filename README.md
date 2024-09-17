# Panoptes Baseline

## Overview

The `panoptes_baseline` package integrates `apriltag_ros` for AprilTag detection and `depthai_ros` for spatial human detection with the OAK-D Lite camera. It provides functionality for publishing AprilTag-to-camera and camera-to-human transforms. This setup is intended for use with static cameras, each assigned a unique namespace.

## Installation

Install ROS2 Humble.

Clone the repositories and build your workspace:

```sh
mkdir -p <workspace>/src
cd <workspace>
git clone https://github.com/J0HNN7G/panoptes_baseline src/panoptes_baseline
git clone https://github.com/J0HNN7G/apriltag_ros.git src/apriltag_ros
git clone https://github.com/J0HNN7G/depthai-ros.git src/depthai-ros
colcon build
```

## Usage

### Launch the Nodes

To launch the setup for a specific namespace, use the following command:

```sh
ros2 launch panoptes_baseline panoptes_baseline.launch.py \
     robomaster_name:=<namespace>
```

Replace `<namespace>` with the desired namespace for your camera.

## World to AprilTag ROS2 tf2 Static Broadcaster

### Overview

The `world_to_apriltag_tf2.launch.py` uses `tf2` static broadcasters to publish transforms from map to AprilTags, based on configurations provided in CSV files. This package is designed to run on a single computer within your setup.

### Configuration

Transformations are specified in a CSV file with the following format:

```csv
tag_family,tag_id,trans_x,trans_y,trans_z,rot_x,rot_y,rot_z,rot_w
tag36h11,0,1.0,1.0,0.0,0.0,0.0,0.0,1.0
tag36h11,1,-1.0,1.0,0.0,0.0,0.0,0.0,1.0
tag36h11,2,1.0,-1.0,0.0,0.0,0.0,0.0,1.0
tag36h11,3,-1.0,-1.0,0.0,0.0,0.0,0.0,1.0
```

- **`tag_family`**: The AprilTag family used by the detector.
- **`tag_id`**: Unique identifier for each tag.
- **`trans_x, trans_y, trans_z`**: Translation values (meters).
- **`rot_x, rot_y, rot_z, rot_w`**: Rotation values (quaternion).

The default frame name will be formatted as `<tag_family>:<tag_id>`.

### Launch the Broadcasters

To start broadcasting transforms, use:

```sh
ros2 launch world_to_apriltag_tf2 world_to_apriltag_tf2.launch.py \
     transforms_file:=`ros2 pkg prefix panoptes_baseline`/share/panoptes_baseline/cfg/<filename>.csv
```

Replace `<filename>` with the name of your CSV file containing the transforms.

## Additional Resources

- [AprilTag Documentation](https://april.eecs.umich.edu/software/apriltag.html)
- [apriltag_ros GitHub Repository](https://github.com/J0HNN7G/apriltag_ros)
- [depthai_ros GitHub Repository](https://github.com/J0HNN7G/depthai-ros)