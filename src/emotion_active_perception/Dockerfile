FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/emotion_active_perception
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

CMD ["bash", "-lc", ". install/setup.bash && ros2 run emotion_active_perception ap_action_server"]

