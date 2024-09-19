FROM dustynv/ros:humble-pytorch-l4t-r35.3.1

RUN apt-get update && apt-get install -y git vim

WORKDIR /opt/jonathan

# oak-d dependencies
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash

# ROS2 workspace
RUN mkdir -p panoptes_baseline/src
WORKDIR /opt/jonathan/panoptes_baseline/src

# depthai packages
RUN git clone https://github.com/J0HNN7G/depthai-ros.git

RUN git clone --no-checkout https://github.com/ros/diagnostics.git
WORKDIR /opt/jonathan/panoptes_baseline/src/diagnostics
RUN git sparse-checkout init --cone && \
    git sparse-checkout set diagnostic_updater && \
    git checkout ros2-humble
WORKDIR /opt/jonathan/panoptes_baseline/src

RUN git clone --no-checkout https://github.com/ros-perception/image_pipeline.git
WORKDIR /opt/jonathan/panoptes_baseline/src/image_pipeline
RUN git sparse-checkout init --cone && \
    git sparse-checkout set image_proc tracetools_image_pipeline && \
    git checkout humble
WORKDIR /opt/jonathan/panoptes_baseline/src

RUN git clone -b ros2 https://github.com/ros/xacro.git
RUN git clone -b humble https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs.git

RUN git clone --no-checkout https://github.com/foxglove/schemas.git
WORKDIR /opt/jonathan/panoptes_baseline/src/schemas
RUN git sparse-checkout init --cone && \
    git sparse-checkout set ros_foxglove_msgs && \
    git checkout main
WORKDIR /opt/jonathan/panoptes_baseline/src

# apriltag packages
RUN git clone https://github.com/J0HNN7G/apriltag_ros.git

RUN git clone https://github.com/AprilRobotics/apriltag.git
RUN git clone https://github.com/christianrauch/apriltag_msgs.git

RUN git clone --no-checkout https://github.com/ament/ament_lint.git
WORKDIR /opt/jonathan/panoptes_baseline/src/ament_lint
RUN git sparse-checkout init --cone && \
    git sparse-checkout set ament_cmake_clang_format ament_clang_format && \
    git checkout humble
WORKDIR /opt/jonathan/panoptes_baseline/src

# panoptes baseline packages
RUN git clone https://github.com/J0HNN7G/panoptes_baseline.git

RUN git clone --no-checkout https://github.com/ros2/common_interfaces.git
WORKDIR /opt/jonathan/panoptes_baseline/src/common_interfaces
RUN git sparse-checkout init --cone && \
    git sparse-checkout set geometry_msgs && \
    git checkout humble
WORKDIR /opt/jonathan/panoptes_baseline/src

RUN git clone --no-checkout https://github.com/ros2/geometry2.git
WORKDIR /opt/jonathan/panoptes_baseline/src/geometry2
RUN git sparse-checkout init --cone && \
    git sparse-checkout set tf2 tf2_py tf2_ros_py && \
    git checkout humble
WORKDIR /opt/jonathan/panoptes_baseline/src

# build workspace
WORKDIR /opt/jonathan/panoptes_baseline 
RUN source /opt/ros/humble/install/setup.bash && \
    rosdep install --from-paths src -y -r --ignore-src --skip-keys libopencv-dev && \
    colcon build

WORKDIR /opt/jonathan
